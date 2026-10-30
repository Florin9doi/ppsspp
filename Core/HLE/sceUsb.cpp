// Copyright (c) 2012- PPSSPP Project.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 2.0 or later versions.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License 2.0 for more details.

// A copy of the GPL 2.0 should have been included with the program.
// If not, see http://www.gnu.org/licenses/

// Official git repository and contact information can be found at
// https://github.com/hrydgard/ppsspp and http://www.ppsspp.org/.

#include "Common/Serialize/Serializer.h"
#include "Common/Serialize/SerializeFuncs.h"
#include "Core/CoreTiming.h"
#include "Core/HLE/HLE.h"
#include "Core/HLE/ErrorCodes.h"
#include "Core/HLE/FunctionWrappers.h"
#include "Core/HLE/KernelWaitHelpers.h"
#include "Core/HLE/sceKernelMemory.h"
#include "Core/HLE/sceKernelModule.h"
#include "Core/HLE/sceKernelThread.h"
#include "Core/HLE/pspcm_manager.h"
#include "Core/HLE/sceUsb.h"
#include "Core/MemMapHelpers.h"
#include "Core/MIPS/MIPS.h"
#include "Core/Reporting.h"
#include <Thread/ThreadUtil.h>
#include <TimeUtil.h>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <File/FileDescriptor.h>

// TODO: Map by driver name
static bool usbStarted = false;
// TODO: Check actual status
static bool usbConnected = true;
// TODO: Activation by product id
static bool usbActivated = false;

static int usbWaitTimer = -1;
static std::vector<SceUID> waitingThreads;
static Usbd::Config sceUsbConfig; // TODO: move in namespace

// The link to the host (the PS3 side of usbpspcm.prx), see pspcm_manager.h. Its socket thread only
// receives and queues. Everything that touches PSP memory or calls into the driver runs on the CPU
// thread, from UsbLinkPoll.
static void UsbLinkOnConnect();
static void UsbLinkOnDisconnect();
static void UsbLinkOnPacket(PspCmPacketType type, int endpoint, std::vector<u8> &data);

static std::unique_ptr<PspCmManager> usbLink;
static PspCmManager &UsbLink() {
	if (!usbLink) {
		usbLink = std::make_unique<PspCmManager>(
			[]() { UsbLinkOnConnect(); },
			[]() { UsbLinkOnDisconnect(); },
			[](PspCmPacketType type, int endpoint, std::vector<u8> &data) { UsbLinkOnPacket(type, endpoint, data); });
	}
	return *usbLink;
}
static bool UsbLinkRunning() {
	return usbLink && usbLink->IsRunning();
}

// The last control packet from the host, header included. A reply to it reuses the header.
static char gRecvBuffer[512];
static int gRecvLen = 0;

static const int USB_LINK_POLL_US = 1000;
static const int USB_LINK_MAX_ENDPOINTS = 16;
static int usbLinkEvent = -1;
// CPU thread only from here on.
static bool usbLinkConnected = false;
static u32 usbControlReqAddr = 0;
// Bulk data from the host that no sceUsbbdReqRecv has asked for yet, and the requests waiting for data.
static std::deque<std::vector<u8>> usbRxQueue[USB_LINK_MAX_ENDPOINTS];
static std::deque<u32> usbPendingRecv[USB_LINK_MAX_ENDPOINTS];
// Device to host data waits until the host asks for it on that endpoint, like on a real bus. The host
// asks once per endpoint and waits for the answer, usbHostWaiting collapses the repeated asks.
static bool usbHostWaiting[USB_LINK_MAX_ENDPOINTS];
// What waits to be sent to the host, one queue per IN endpoint (1: bulk IN 0x81, 3: interrupt IN 0x83).
// An entry goes out when the host asks for data on its endpoint, in the order they were queued.
struct UsbInEntry {
	u32 reqAddr = 0;       // a sceUsbbdReqSend request of the driver, or 0:
	std::vector<u8> data;  // data the app wrote to "usbpspcm0:"
};
static std::deque<UsbInEntry> usbInQueue[USB_LINK_MAX_ENDPOINTS];
static const size_t USB_IN_QUEUE_MAX = 256; // of the writes of the app

enum UsbStatus {
	USB_STATUS_STOPPED      = 0x001,
	USB_STATUS_STARTED      = 0x002,
	USB_STATUS_DISCONNECTED = 0x010,
	USB_STATUS_CONNECTED    = 0x020,
	USB_STATUS_DEACTIVATED  = 0x100,
	USB_STATUS_ACTIVATED    = 0x200,
};

static int UsbCurrentState() {
	int state = 0;
	if (usbStarted) {
		state = USB_STATUS_STARTED
			| (usbConnected ? USB_STATUS_CONNECTED : USB_STATUS_DISCONNECTED)
			| (usbActivated ? USB_STATUS_ACTIVATED : USB_STATUS_DEACTIVATED);
	}
	return state;
}

static bool UsbMatchState(int state, u32 mode) {
	int match = state & UsbCurrentState();
	if (mode == 0) {
		return match == state;
	}
	return match != 0;
}

static void UsbSetTimeout(PSPPointer<int> timeout) {
	if (!timeout.IsValid() || usbWaitTimer == -1)
		return;

	// This should call __UsbWaitTimeout() later, unless we cancel it.
	CoreTiming::ScheduleEvent(usToCycles(*timeout), usbWaitTimer, __KernelGetCurThread());
}

static void UsbWaitExecTimeout(u64 userdata, int cycleslate) {
	u32 error;
	SceUID threadID = (SceUID)userdata;

	// The thread may have stopped waiting on USB since (for example, released), keeping the event.
	if (__KernelGetWaitID(threadID, WAITTYPE_USB, error) == 0 || error != 0) {
		HLEKernel::RemoveWaitingThread(waitingThreads, threadID);
		return;
	}

	PSPPointer<int> timeout = PSPPointer<int>::Create(__KernelGetWaitTimeoutPtr(threadID, error));
	if (timeout.IsValid())
		*timeout = 0;

	HLEKernel::RemoveWaitingThread(waitingThreads, threadID);
	__KernelResumeThreadFromWait(threadID, SCE_ERROR_USB_WAIT_TIMEOUT);
	__KernelReSchedule("wait timed out");
}

static void UsbUpdateState() {
	u32 error;
	bool wokeThreads = false;
	for (size_t i = 0; i < waitingThreads.size(); ++i) {
		SceUID threadID = waitingThreads[i];
		int state = __KernelGetWaitID(threadID, WAITTYPE_USB, error);
		// 0 means it's not in a USB wait anymore. Waits always have a nonzero state.
		if (error != 0 || state == 0)
			continue;

		u32 mode = __KernelGetWaitValue(threadID, error);
		if (UsbMatchState(state, mode)) {
			waitingThreads.erase(waitingThreads.begin() + i);
			--i;

			PSPPointer<int> timeout = PSPPointer<int>::Create(__KernelGetWaitTimeoutPtr(threadID, error));
			if (timeout.IsValid() && usbWaitTimer != -1) {
				// Remove any event for this thread.
				s64 cyclesLeft = CoreTiming::UnscheduleEvent(usbWaitTimer, threadID);
				if (cyclesLeft < 0)
					cyclesLeft = 0;
				*timeout = (int)cyclesToUs(cyclesLeft);
			}

			__KernelResumeThreadFromWait(threadID, UsbCurrentState());
			wokeThreads = true;
		}
	}

	if (wokeThreads)
		hleReSchedule("usb state change");
}

static void send_to_ps3(const char *buf, int len) {
	UsbLink().Send(buf, len);

	// TODO: HACK: transfer the bulk IN req from host to psp and start this thread based on that:
	//if (buf[4] == 0x41 && buf[5] == 0x02) {
	//if (len == 4) {
	//	ERROR_LOG(Log::HLE, "pspcm_manager: sceKernelStartThread(0x184)");
	//	sceKernelStartThread(0x184, 0, 0);
	//}
}

static void UsbLinkSendToHost(int endpoint, const u8 *data, u32 size) {
	UsbLink().SendToHost(endpoint, data, size);
}

// Everything below runs on the CPU thread.

static void UsbLinkCallDriver(u32 func, u32 arg0, u32 arg1, u32 arg2) {
	if (func == 0) {
		return;
	}
	u32 args[] = { arg0, arg1, arg2 };
	// Calls made one after the other run in that order: a later one waits until the earlier returns.
	__KernelDirectMipsCall(func, nullptr, args, ARRAY_SIZE(args), false);
}

static void UsbLinkResetEndpoints() {
	for (int i = 0; i < USB_LINK_MAX_ENDPOINTS; ++i) {
		usbRxQueue[i].clear();
		usbPendingRecv[i].clear();
		usbInQueue[i].clear();
		usbHostWaiting[i] = false;
	}
}

// Copies queued bulk data into a sceUsbbdReqRecv request. Returns false if there's none yet.
static bool UsbFillRecv(u32 reqAddr, int endpoint) {
	auto &queue = usbRxQueue[endpoint];
	auto req = PSPPointer<UsbdDeviceRequest>::Create(reqAddr);
	if (queue.empty() || !req.IsValid()) {
		return false;
	}

	std::vector<u8> &front = queue.front();
	u32 size = std::min<u32>(req->size, (u32)front.size());
	req->returnCode = 0;
	if (size > 0 && !Memory::IsValidRange(req->data, size)) {
		req->returnCode = SCE_KERNEL_ERROR_ILLEGAL_ADDR;
		size = 0;
	} else if (size > 0) {
		Memory::Memcpy(req->data, front.data(), size, "UsbbdReqRecv");
	}
	// A request smaller than the packet leaves the rest for the next one.
	if (size < front.size()) {
		front.erase(front.begin(), front.begin() + size);
	} else {
		queue.pop_front();
	}
	req->transmitted = size;
	req.NotifyWrite("sceUsbbdReqRecv");
	return true;
}

// Sends the oldest entry of the endpoint's queue if the host asked for it.
// inHle: called from an HLE function, where the completion is queued instead of called directly.
static void UsbLinkServeIn(int endpoint, bool inHle) {
	if (!usbHostWaiting[endpoint] || usbInQueue[endpoint].empty()) {
		return;
	}
	UsbInEntry entry = std::move(usbInQueue[endpoint].front());
	usbInQueue[endpoint].pop_front();

	if (entry.reqAddr == 0) {
		// Written by the app.
		usbHostWaiting[endpoint] = false;
		INFO_LOG(Log::HLE, "pspcm: %d bytes written by the app sent on endpoint 0x%02x", (int)entry.data.size(), endpoint | 0x80);
		UsbLinkSendToHost(endpoint | 0x80, entry.data.data(), (u32)entry.data.size());
		return;
	}

	const u32 reqAddr = entry.reqAddr;
	auto req = PSPPointer<UsbdDeviceRequest>::Create(reqAddr);
	if (!req.IsValid()) {
		return;
	}
	usbHostWaiting[endpoint] = false;

	const u32 size = req->size;
	static const u8 noData = 0;
	const bool validData = size > 0 && Memory::IsValidRange(req->data, size);
	const u8 *data = validData ? Memory::GetPointerUnchecked(req->data) : &noData;
	// The USB address the host knows: the endpoint number plus the direction.
	UsbLinkSendToHost(endpoint | 0x80, data, validData ? size : 0);

	req->transmitted = validData ? size : 0;
	req->returnCode = (size > 0 && !validData) ? SCE_KERNEL_ERROR_ILLEGAL_ADDR : 0;
	req.NotifyWrite("sceUsbbdReqSend");

	if (req->onComplete_func != 0) {
		if (inHle) {
			u32 args[] = { reqAddr, 0, 0 };
			hleEnqueueCall(req->onComplete_func, ARRAY_SIZE(args), args);
		} else {
			UsbLinkCallDriver(req->onComplete_func, reqAddr, 0, 0);
		}
	}
}

// Reply to a control request: its header, with the length of the data it carries. The host's wLength
// is only the most it takes. At most 0x40 bytes of data fit a packet of the protocol.
static void UsbLinkSendControlReply(const u8 *data, u32 size) {
	const u32 replySize = std::min<u32>(size, 0x40);
	char reply[12 + 0x40];
	memcpy(reply, gRecvBuffer, 12);
	if (replySize > 0) {
		memcpy(reply + 12, data, replySize);
	}
	reply[2] = (char)(12 + replySize);
	reply[10] = (char)(replySize & 0xff);
	reply[11] = (char)(replySize >> 8);
	send_to_ps3(reply, 12 + replySize);
}

// Acknowledges a host to device control request, with or without data: the header only.
static void UsbLinkSendAck() {
	const char ack[] = { (char)0xf0, 0x0f, 0x04, 0x00 };
	send_to_ps3(ack, sizeof(ack));
}

// The standard requests are answered by the usbd stack, not by the driver: the descriptors come from
// the tables the driver registered, if it registered some. The host asks for them when the connection is made.
//
// Layout of what a driver registers (the PSPSDK structs, little endian, pointers are guest addresses):
//   device descriptor: the 18 bytes of the USB descriptor.
//   config: 9 bytes of the USB descriptor, padding, u32 settings -> array of { u32 interfaces, u32 count }.
//   interface: 9 bytes of the USB descriptor, padding, u32 endpoints, u32 extra, u32 extraLength (24 bytes).
//   endpoint: 7 bytes of the USB descriptor, padding, u32 extra, u32 extraLength (16 bytes).
static const u32 USB_CONFIG_SETTINGS_OFFSET = 12;
static const u32 USB_INTERFACE_STRIDE = 24;
static const u32 USB_ENDPOINT_STRIDE = 16;
static const u32 USB_MAX_EXTRA = 0x40;

static bool UsbReadGuest(u32 addr, void *out, u32 size) {
	if (addr == 0 || !Memory::IsValidRange(addr, size)) {
		return false;
	}
	Memory::Memcpy(out, addr, size, "UsbDescriptor");
	return true;
}

static void UsbAppendExtra(std::vector<u8> &out, u32 extraAddr, u32 extraLen) {
	if (extraAddr == 0 || extraLen == 0 || extraLen > USB_MAX_EXTRA) {
		return;
	}
	std::vector<u8> extra(extraLen);
	if (UsbReadGuest(extraAddr, extra.data(), extraLen)) {
		out.insert(out.end(), extra.begin(), extra.end());
	}
}

static bool UsbBuildDeviceDescriptor(u32 addr, std::vector<u8> &out) {
	u8 desc[18];
	if (!UsbReadGuest(addr, desc, sizeof(desc)) || desc[0] != 18 || desc[1] != 1) {
		return false;
	}
	out.assign(desc, desc + sizeof(desc));
	return true;
}

// Flattens the config, its interfaces and endpoints into the bytes a host reads.
static bool UsbBuildConfigDescriptor(u32 addr, std::vector<u8> &out) {
	u8 cfg[9];
	if (!UsbReadGuest(addr, cfg, sizeof(cfg)) || cfg[0] != 9 || cfg[1] != 2) {
		return false;
	}
	const u32 numInterfaces = cfg[4];
	u32 settings = 0;
	UsbReadGuest(addr + USB_CONFIG_SETTINGS_OFFSET, &settings, 4);
	if (numInterfaces == 0 || numInterfaces > 8 || settings == 0) {
		return false;
	}

	out.assign(cfg, cfg + sizeof(cfg));
	for (u32 i = 0; i < numInterfaces; ++i) {
		u32 ifaces = 0, count = 0;
		if (!UsbReadGuest(settings + i * 8, &ifaces, 4) || !UsbReadGuest(settings + i * 8 + 4, &count, 4) || count == 0 || count > 8) {
			return false;
		}
		for (u32 j = 0; j < count; ++j) {
			const u32 ifaceAddr = ifaces + j * USB_INTERFACE_STRIDE;
			u8 iface[9];
			u32 endpoints = 0, extra = 0, extraLen = 0;
			if (!UsbReadGuest(ifaceAddr, iface, sizeof(iface)) || iface[0] != 9 || iface[1] != 4 || iface[4] > 16) {
				return false;
			}
			UsbReadGuest(ifaceAddr + 12, &endpoints, 4);
			UsbReadGuest(ifaceAddr + 16, &extra, 4);
			UsbReadGuest(ifaceAddr + 20, &extraLen, 4);
			out.insert(out.end(), iface, iface + sizeof(iface));
			UsbAppendExtra(out, extra, extraLen);

			for (u32 k = 0; k < iface[4]; ++k) {
				const u32 epAddr = endpoints + k * USB_ENDPOINT_STRIDE;
				u8 ep[7];
				u32 epExtra = 0, epExtraLen = 0;
				if (!UsbReadGuest(epAddr, ep, sizeof(ep)) || ep[0] != 7 || ep[1] != 5) {
					return false;
				}
				UsbReadGuest(epAddr + 8, &epExtra, 4);
				UsbReadGuest(epAddr + 12, &epExtraLen, 4);
				out.insert(out.end(), ep, ep + sizeof(ep));
				UsbAppendExtra(out, epExtra, epExtraLen);
			}
		}
	}
	// The total length covers what was flattened.
	out[2] = (u8)(out.size() & 0xff);
	out[3] = (u8)(out.size() >> 8);
	return true;
}

// usbpspcm.prx registers no descriptor tables (devp and confp are 0, only the interface string is
// there): the bus describes the device it was activated for. These are the descriptors of the
// product id 0x01cb, "PSP Type D" by Sony, with 3 endpoints: bulk IN 1, bulk OUT 2, interrupt IN 3.
static const u8 usbPspcmDevice[] = {
	18, 1, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x4c, 0x05, 0xcb, 0x01, 0x00, 0x01, 1, 2, 0, 1,
};
static const u8 usbPspcmConfig[] = {
	9, 2, 0x27, 0x00, 1, 1, 0, 0xc0, 0x01,                      // configuration
	9, 4, 0, 0, 3, 0xff, 0x01, 0xff, 3,                         // interface, string 3 is the driver's
	7, 5, 0x81, 0x02, 0x40, 0x00, 0x00,                         // bulk IN
	7, 5, 0x02, 0x02, 0x40, 0x00, 0x00,                         // bulk OUT
	7, 5, 0x83, 0x03, 0x08, 0x00, 0x08,                         // interrupt IN
};

// What the driver registered, or what the bus knows when it registered nothing.
static bool UsbGetDeviceDescriptor(const PspUsbDriver *drv, std::vector<u8> &out) {
	const u32 addr = drv->devp_hi ? drv->devp_hi : drv->devp;
	if (addr != 0 && UsbBuildDeviceDescriptor(addr, out)) {
		return true;
	}
	out.assign(usbPspcmDevice, usbPspcmDevice + sizeof(usbPspcmDevice));
	return true;
}

static bool UsbGetConfigDescriptor(const PspUsbDriver *drv, std::vector<u8> &out) {
	const u32 addr = drv->confp_hi ? drv->confp_hi : drv->confp;
	if (addr != 0 && UsbBuildConfigDescriptor(addr, out)) {
		return true;
	}
	out.assign(usbPspcmConfig, usbPspcmConfig + sizeof(usbPspcmConfig));
	return true;
}

// Strings: 0 is the list of languages, the default string of the driver is the one of its interface.
// The manufacturer and product strings aren't in the driver, the bus gives them.
static bool UsbBuildStringDescriptor(const PspUsbDriver *drv, u32 index, std::vector<u8> &out) {
	if (index == 0) {
		out = { 4, 3, 0x09, 0x04 }; // English (United States)
		return true;
	}

	std::vector<u8> cfg;
	u8 ifaceString = 0;
	if (UsbGetConfigDescriptor(drv, cfg)) {
		for (size_t pos = 9; pos + 9 <= cfg.size() && cfg[pos] >= 2; pos += cfg[pos]) {
			if (cfg[pos + 1] == 4) {
				ifaceString = cfg[pos + 8];
				break;
			}
		}
	}

	if (index == ifaceString && drv->str != 0) {
		u8 head[2];
		if (UsbReadGuest(drv->str, head, sizeof(head)) && head[1] == 3 && head[0] >= 2 && head[0] <= 0x40) {
			out.resize(head[0]);
			return UsbReadGuest(drv->str, out.data(), head[0]);
		}
		return false;
	}

	const char *text = nullptr;
	if (index == 1) {
		text = "Sony";
	} else if (index == 2) {
		text = "PSP Type D";
	}
	if (!text) {
		return false;
	}
	out = { 0, 3 };
	for (const char *c = text; *c; ++c) {
		out.push_back((u8)*c);
		out.push_back(0);
	}
	out[0] = (u8)out.size();
	return true;
}

// A control request of the standard type. Whatever isn't answered gets an empty reply.
static void UsbLinkHandleStandardRequest(const PspUsbDriver *drv, const std::vector<u8> &packet) {
	const u8 requestType = packet[4];
	const u8 request = packet[5];
	const u16 value = packet[6] | (packet[7] << 8);
	const u16 length = packet[10] | (packet[11] << 8);

	if (requestType == 0x80 && request == 6) { // GET_DESCRIPTOR
		const u8 type = value >> 8;
		const u8 index = value & 0xff;
		std::vector<u8> desc;
		bool ok = false;
		switch (type) {
		case 1: // device desc
			ok = UsbGetDeviceDescriptor(drv, desc);
			break;
		case 2: // config desc
			ok = UsbGetConfigDescriptor(drv, desc);
			break;
		case 3: // string desc
			ok = UsbBuildStringDescriptor(drv, index, desc);
			break;
		}
		INFO_LOG(Log::HLE, "pspcm_manager: GET_DESCRIPTOR type=%d index=%d -> %s %d bytes", type, index, ok ? "ok" : "none", ok ? (int)desc.size() : 0);
		if (ok) {
			if (desc.size() > 0x40) {
				WARN_LOG(Log::HLE, "pspcm_manager: descriptor of %d bytes, only 0x40 fit a packet", (int)desc.size());
			}
			UsbLinkSendControlReply(desc.data(), std::min<u32>((u32)desc.size(), length));
		} else {
			UsbLinkSendControlReply(nullptr, 0);
		}
		return;
	}

	WARN_LOG(Log::HLE, "pspcm_manager: standard request %02x/%02x not handled", requestType, request);
	if (requestType & 0x80) {
		UsbLinkSendControlReply(nullptr, 0);
	} else {
		UsbLinkSendAck();
	}
}

int __UsbPspcmWrite(int endpoint, const u8 *data, u32 size) {
	if (endpoint != 1 && endpoint != 3) {
		ERROR_LOG(Log::HLE, "pspcm: endpoint %d isn't an IN endpoint (1 bulk, 3 interrupt)", endpoint);
		return 0;
	}
	if (!data || size == 0) {
		return 0;
	}
	auto &queue = usbInQueue[endpoint];
	if (queue.size() >= USB_IN_QUEUE_MAX) {
		WARN_LOG(Log::HLE, "pspcm: the queue of endpoint %d is full, dropping the oldest", endpoint);
		queue.pop_front();
	}
	UsbInEntry entry;
	entry.data.assign(data, data + size);
	queue.push_back(std::move(entry));
	// The host may be waiting already.
	UsbLinkServeIn(endpoint, true);
	return (int)size;
}

static void UsbLinkOnConnect() {
	PspUsbDriver *drv = Usbd::getUsbDriver();
	if (usbLinkConnected) {
		// The host came back without the old one being seen leaving.
		INFO_LOG(Log::HLE, "pspcm_manager: detaching the driver before the new connection");
		UsbLinkResetEndpoints();
		UsbLinkCallDriver(drv->detach_func, 0, 0, 0);
	}
	INFO_LOG(Log::HLE, "pspcm_manager: host connected, attaching the driver");
	usbLinkConnected = true;
	UsbLinkResetEndpoints();
	const u32 speed = 2; // usb_version : speed 1=full, 2=hi
	UsbLinkCallDriver(drv->attach_func, speed, 0, 0);
	UsbLinkCallDriver(drv->configure_func, speed, 0, 0);
	UsbLinkCallDriver(drv->intf_chang_func, 0, 0, 0); // interfaceNumber, alternateSetting, unk
}

static void UsbLinkOnDisconnect() {
	PspUsbDriver *drv = Usbd::getUsbDriver();
	if (usbLinkConnected) {
		INFO_LOG(Log::HLE, "pspcm_manager: host disconnected, detaching the driver");
		usbLinkConnected = false;
		UsbLinkResetEndpoints();
		UsbLinkCallDriver(drv->detach_func, 0, 0, 0);
	}
}

static void UsbLinkOnControl(const std::vector<u8> &packet) {
	PspUsbDriver *drv = Usbd::getUsbDriver();
	if (packet.size() < 12 || packet.size() > sizeof(gRecvBuffer)) {
		return;
	}
	if (usbControlReqAddr == 0) {
		u32 size = sizeof(DeviceRequest);
		u32 addr = kernelMemory.Alloc(size, false, "sceUsb");
		if (addr == 0 || addr == (u32)-1) {
			ERROR_LOG(Log::HLE, "pspcm_manager: no memory for the control request");
			return;
		}
		usbControlReqAddr = addr;
	}

	const u8 requestType = packet[4];
	if ((requestType & 0x60) == 0) {
		// Standard: the reply reuses the header of the request.
		memcpy(gRecvBuffer, packet.data(), 12);
		gRecvLen = 12;
		UsbLinkHandleStandardRequest(drv, packet);
		return;
	}
	// A device to host request may come padded to wLength, that isn't data for sceUsbbdReqRecv.
	gRecvLen = (requestType & 0x80) ? 12 : (int)packet.size();
	memcpy(gRecvBuffer, packet.data(), packet.size());
	Memory::Memcpy(usbControlReqAddr, &packet[4], sizeof(DeviceRequest), "UsbControlReq");

	const u16 length = packet[10] | (packet[11] << 8);
	UsbLinkCallDriver(drv->recvctl_func, requestType & 0x1f, 0, usbControlReqAddr);

	// Host Out and no data -> sceUsbbdReqRecv isn't called, send an empty reply here
	if ((requestType & 0x80) == 0 && length == 0) {
		UsbLinkSendAck();
	}
}

static void UsbLinkOnPacket(PspCmPacketType type, int rawEndpoint, std::vector<u8> &data) {
	const int endpoint = rawEndpoint & 0xf;
	switch (type) {
	case PspCmPacketType::CONTROL:
		UsbLinkOnControl(data);
		break;

	case PspCmPacketType::IN_REQUEST:
		if (endpoint == 0 || endpoint >= USB_LINK_MAX_ENDPOINTS) {
			break;
		}
		if (usbHostWaiting[endpoint]) {
			// Already asked, and not answered yet.
			DEBUG_LOG(Log::HLE, "pspcm_manager: duplicate IN request on endpoint %d dropped", endpoint);
			break;
		}
		usbHostWaiting[endpoint] = true;
		UsbLinkServeIn(endpoint, false);
		break;

	case PspCmPacketType::BULK:
		usbRxQueue[endpoint].push_back(std::move(data));
		// Requests that were waiting for data.
		while (!usbPendingRecv[endpoint].empty() && !usbRxQueue[endpoint].empty()) {
			u32 reqAddr = usbPendingRecv[endpoint].front();
			usbPendingRecv[endpoint].pop_front();
			if (UsbFillRecv(reqAddr, endpoint)) {
				auto req = PSPPointer<UsbdDeviceRequest>::Create(reqAddr);
				UsbLinkCallDriver(req->onComplete_func, reqAddr, 0, 0);
			}
		}
		break;
	}
}

static void UsbLinkPoll(u64 userdata, int cyclesLate) {
	if (!usbLink) {
		return;
	}
	usbLink->Poll();

	if (usbLink->IsRunning()) {
		CoreTiming::ScheduleEvent(std::max<s64>(usToCycles(USB_LINK_POLL_US) - cyclesLate, 1), usbLinkEvent, 0);
	}
}

static bool UsbLinkStart(bool scheduleEvent = true) {
	if (!UsbLink().Start()) {
		return false;
	}
	if (scheduleEvent && usbLinkEvent != -1) {
		CoreTiming::ScheduleEvent(usToCycles(USB_LINK_POLL_US), usbLinkEvent, 0);
	}
	return true;
}

static void UsbLinkStop() {
	if (usbLink) {
		usbLink->Stop();
	}
	usbLinkConnected = false;
	UsbLinkResetEndpoints();
}


// usbpspcm.prx registers a thread (sceIoDevctl 0x03415001) that is started once the USB is
// activated. It is started with 8 bytes of arguments: 0, then flags saying the USB is connected.
static int usbRegisteredThread = -1;
static bool usbRegisteredThreadStarted = false;
static u32 usbStartThreadParams = 0;

static void UsbStartRegisteredThread() {
	if (!usbActivated || usbRegisteredThreadStarted || usbRegisteredThread < 0) {
		return;
	}
	if (usbStartThreadParams == 0) {
		u32 size = 8;
		u32 addr = kernelMemory.Alloc(size, false, "sceUsbPspcm thread parameters");
		if (addr == 0 || addr == (u32)-1) {
			ERROR_LOG(Log::HLE, "sceUsbPspcm: no memory for the thread parameters");
			return;
		}
		usbStartThreadParams = addr;
	}

	const u32 params[2] = { 0, 0x80 | 0x01 }; // the second one: the USB is in connected state
	Memory::Memcpy(usbStartThreadParams, params, sizeof(params), "UsbThreadParams");

	usbRegisteredThreadStarted = true;
	int ret = __KernelStartThread(usbRegisteredThread, 8, usbStartThreadParams);
	INFO_LOG(Log::HLE, "sceUsbPspcm: started the registered thread 0x%x = %08x", usbRegisteredThread, ret);
}

void __UsbRegisterThread(int threadId) {
	usbRegisteredThread = threadId;
	usbRegisteredThreadStarted = false;
	//UsbStartRegisteredThread();
}

void __UsbUnregisterThread(int threadId) {
	if (usbRegisteredThread == threadId) {
		usbRegisteredThread = -1;
		usbRegisteredThreadStarted = false;
	}
}

void __UsbInit() {
	usbStarted = false;
	usbConnected = true;
	usbActivated = false;
	usbRegisteredThread = -1;
	usbRegisteredThreadStarted = false;
	usbStartThreadParams = 0;
	waitingThreads.clear();

	usbWaitTimer = CoreTiming::RegisterEvent("UsbWaitTimeout", UsbWaitExecTimeout);
	usbLinkEvent = CoreTiming::RegisterEvent("UsbLinkPoll", UsbLinkPoll);
	memset(&sceUsbConfig, 0, sizeof(Usbd::Config));
	usbControlReqAddr = 0;
	UsbLinkStop();
}
void __UsbShutdown() {
	UsbLinkStop();
}

void __UsbDoState(PointerWrap &p) {
	auto s = p.Section("sceUsb", 1, 6);
	if (!s)
		return;

	if (s >= 2) {
		Do(p, usbStarted);
		Do(p, usbConnected);
	} else {
		usbStarted = false;
		usbConnected = true;
	}
	Do(p, usbActivated);
	if (s >= 3) {
		Do(p, waitingThreads);
		Do(p, usbWaitTimer);
	} else {
		waitingThreads.clear();
		usbWaitTimer = -1;
	}
	CoreTiming::RestoreRegisterEvent(usbWaitTimer, "UsbWaitTimeout", UsbWaitExecTimeout);
	if (s >= 4) {
		Do(p, usbLinkEvent);
	} else {
		usbLinkEvent = -1;
	}
	CoreTiming::RestoreRegisterEvent(usbLinkEvent, "UsbLinkPoll", UsbLinkPoll);

	// The link itself (socket, queued packets) isn't part of a savestate, only what the guest was told.
	bool linkRunning = UsbLinkRunning();
	bool guestAttached = usbLinkConnected;
	if (s >= 5) {
		Do(p, linkRunning);
		Do(p, guestAttached);
	} else {
		linkRunning = false;
		guestAttached = false;
	}

	if (s >= 6) {
		Do(p, usbRegisteredThread);
		Do(p, usbRegisteredThreadStarted);
		Do(p, usbStartThreadParams);
	} else {
		usbRegisteredThread = -1;
		usbRegisteredThreadStarted = false;
		usbStartThreadParams = 0;
	}

	if (p.mode == PointerWrap::MODE_READ) {
		// The guest is back to the state it was saved in, but the host connection is from now.
		// Close it: the host connects again, and attach/configure/intf_chang run for the new one.
		if (UsbLinkRunning() && !linkRunning) {
			UsbLinkStop();
			if (usbLinkEvent != -1) {
				CoreTiming::RemoveEvent(usbLinkEvent);
			}
		} else if (UsbLinkRunning()) {
			UsbLink().DropClient();
		} else if (linkRunning && !UsbLinkStart(false)) {
			ERROR_LOG(Log::HLE, "pspcm_manager: can't listen for the host after loading");
		}
		UsbLink().ClearInbox();
		UsbLinkResetEndpoints();
		// The request buffer may not exist in the loaded memory.
		usbControlReqAddr = 0;
		// If the guest was attached, the next connection detaches it before attaching again.
		usbLinkConnected = guestAttached;
	}
}

static int sceUsbStart(const char* driverName, u32 argsSize, u32 argsPtr) {
	INFO_LOG(Log::HLE, "sceUsbStart(%s, size=%i, args=%08x)", driverName, argsSize, argsPtr);
	usbStarted = true;
	UsbUpdateState();

	if (Usbd::getUsbDriver()->name != NULL &&
			strcmp(driverName, (const char*)Memory::GetPointerOrException(Usbd::getUsbDriver()->name)) == 0) {
		if (Usbd::getUsbDriver()->start_func != NULL) {
			u32 args[] = { argsSize, argsPtr };
			hleEnqueueCall(Usbd::getUsbDriver()->start_func, ARRAY_SIZE(args), args);
		}
	}

	return hleLogInfo(Log::HLE, 0);
}

static int sceUsbStop(const char* driverName, u32 argsSize, u32 argsPtr) {
	INFO_LOG(Log::HLE, "sceUsbStop(%s, size=%i, args=%08x)", driverName, argsSize, argsPtr);
	usbStarted = false;
	UsbUpdateState();

	if (Usbd::getUsbDriver()->name != NULL &&
		strcmp(driverName, (const char*)Memory::GetPointerOrException(Usbd::getUsbDriver()->name)) == 0) {
		if (Usbd::getUsbDriver()->stop_func != NULL) {
			u32 args[] = { argsSize, argsPtr };
			hleEnqueueCall(Usbd::getUsbDriver()->stop_func, ARRAY_SIZE(args), args);
		}
	}

	return hleLogInfo(Log::HLE, 0);
}

static int sceUsbGetState() {
	int state = 0;
	if (!usbStarted) {
		state = 0x80243007;
	} else {
		state = UsbCurrentState();
	}
	return hleLogDebug(Log::HLE, state);
}

static int sceUsbActivate(u32 pid) {
	INFO_LOG(Log::HLE, "sceUsbActivate(0x%04x)", pid);
	usbActivated = true;

	// usbpspcm.prx: the driver is attached when the host connects, see UsbLinkOnConnect.
	const bool isPspcm = pid == 0x01cb;
	if (isPspcm && !UsbLinkRunning() && !UsbLinkStart()) {
		ERROR_LOG(Log::HLE, "pspcm_manager: can't listen for the host");
	}
	if (pid == 0x039e) { // T-DMB tunner
		int modId = sceKernelLoadModule(currentMIPS, "flash0:/kd/usbdmb.prx", 0, 0);
		INFO_LOG(Log::sceUtility, "sceKernelLoadModule flash0:/kd/usbdmb.prx : modid=%d", modId);
		int ret = __KernelStartModule(modId, 0, 0, 0, nullptr, nullptr);
		INFO_LOG(Log::sceUtility, "sceKernelStartModule flash0:/kd/usbdmb.prx = %d", ret);
	}

	UsbUpdateState();
	//UsbStartRegisteredThread();

	return hleLogDebug(Log::HLE, 0);
}

static int sceUsbDeactivate() {
	INFO_LOG(Log::HLE, "sceUsbDeactivate()");
	usbActivated = false;
	// Activating again starts the registered thread again (if it ended).
	usbRegisteredThreadStarted = false;
	UsbUpdateState();

	if (UsbLinkRunning()) {
		if (usbLinkConnected && Usbd::getUsbDriver()->detach_func != NULL) {
			u32 args[] = { 0, 0, 0 };
			hleEnqueueCall(Usbd::getUsbDriver()->detach_func, ARRAY_SIZE(args), args);
		}
		UsbLinkStop();
		if (usbLinkEvent != -1) {
			CoreTiming::RemoveEvent(usbLinkEvent);
		}
	}

	return hleLogDebug(Log::HLE, 0);
}

static int sceUsbWaitState(int state, u32 waitMode, u32 timeoutPtr) {
	hleEatCycles(10000);

	if (waitMode >= 2)
		return hleLogError(Log::HLE, SCE_KERNEL_ERROR_ILLEGAL_MODE, "invalid mode");
	if (state == 0)
		return hleLogError(Log::HLE, SCE_KERNEL_ERROR_EVF_ILPAT, "bad state");

	if (UsbMatchState(state, waitMode)) {
		return hleLogDebug(Log::HLE, UsbCurrentState());
	}

	// We'll have to wait as long as it takes.  Cleanup first, just in case.
	HLEKernel::RemoveWaitingThread(waitingThreads, __KernelGetCurThread());
	waitingThreads.push_back(__KernelGetCurThread());

	UsbSetTimeout(PSPPointer<int>::Create(timeoutPtr));
	__KernelWaitCurThread(WAITTYPE_USB, state, waitMode, timeoutPtr, false, "usb state waited");
	return hleLogDebug(Log::HLE, 0, "waiting");
}

static int sceUsbWaitStateCB(int state, u32 waitMode, u32 timeoutPtr) {
	ERROR_LOG_REPORT(Log::HLE, "UNIMPL sceUsbWaitStateCB(%i, %i, %08x)", state, waitMode, timeoutPtr);
	return 0;
}

static int sceUsbstorBootSetCapacity(u32 capacity) {
	return hleReportError(Log::HLE, 0, "unimplemented");
}

// Purpose unknown - JPCSP names it after its NID and returns 0.
static int sceUsb_8BFC3DE8(u32 arg) {
	return hleLogWarning(Log::sceMisc, 0, "UNIMPL");
}

const HLEFunction sceUsb[] =
{
	{0X8BFC3DE8, &WrapI_U<sceUsb_8BFC3DE8>,          "sceUsb_8BFC3DE8",                         'i', "x"  },
	{0XAE5DE6AF, &WrapI_CUU<sceUsbStart>,            "sceUsbStart",                             'i', "sxx"},
	{0XC2464FA0, &WrapI_CUU<sceUsbStop>,             "sceUsbStop",                              'i', "sxx"},
	{0XC21645A4, &WrapI_V<sceUsbGetState>,           "sceUsbGetState",                          'i', ""   },
	{0X4E537366, nullptr,                            "sceUsbGetDrvList",                        '?', ""   },
	{0X112CC951, nullptr,                            "sceUsbGetDrvState",                       '?', ""   },
	{0X586DB82C, &WrapI_U<sceUsbActivate>,           "sceUsbActivate",                          'i', "x"  },
	{0XC572A9C8, &WrapI_V<sceUsbDeactivate>,         "sceUsbDeactivate",                        'i', ""  },
	{0X5BE0E002, &WrapI_IUU<sceUsbWaitState>,        "sceUsbWaitState",                         'x', "xip"},
	{0X616F2B61, &WrapI_IUU<sceUsbWaitStateCB>,      "sceUsbWaitStateCB",                       'x', "xip"},
	{0X1C360735, nullptr,                            "sceUsbWaitCancel",                        '?', ""   },
};

const HLEFunction sceUsbstor[] =
{
	{0X60066CFE, nullptr,                            "sceUsbstorGetStatus",                     '?', ""   },
};

const HLEFunction sceUsbstorBoot[] =
{
	{0XE58818A8, &WrapI_U<sceUsbstorBootSetCapacity>,"sceUsbstorBootSetCapacity",               'i', "x"  },
	{0X594BBF95, nullptr,                            "sceUsbstorBootSetLoadAddr",               '?', ""   },
	{0X6D865ECD, nullptr,                            "sceUsbstorBootGetDataSize",               '?', ""   },
	{0XA1119F0D, nullptr,                            "sceUsbstorBootSetStatus",                 '?', ""   },
	{0X1F080078, nullptr,                            "sceUsbstorBootRegisterNotify",            '?', ""   },
	{0XA55C9E16, nullptr,                            "sceUsbstorBootUnregisterNotify",          '?', ""   },
};

PspUsbDriver* Usbd::getUsbDriver() {
	return &sceUsbConfig.pspUsbDriver;
}

static int sceUsbbdReqSend(u32 usbDeviceReqAddr) {
	// Requests can be chained through nextRequest. They're sent in order, and each one completes.
	u32 reqAddr = usbDeviceReqAddr;
	for (int count = 0; reqAddr != 0 && count < 16; ++count) {
		auto req = PSPPointer<UsbdDeviceRequest>::Create(reqAddr);
		if (!req.IsValid() || !Memory::IsValidRange(req->endpointPtr, sizeof(UsbEndpoint))) {
			return hleLogError(Log::HLE, SCE_KERNEL_ERROR_ILLEGAL_ADDR, "bad request");
		}
		req.NotifyRead("sceUsbbdReqSend");
		
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend: endpointPtr=0x%02x", req->endpointPtr);
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend:        data=0x%02x", req->data);
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend:        size=0x%02x", req->size);
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend:   isCtrlReq=0x%02x", req->isControlRequest);
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend:   onComplCb=0x%02x", req->onComplete_func);
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend: transmitted=0x%02x", req->transmitted);
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend:  returnCode=0x%02x", req->returnCode);
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend: nextRequest=0x%02x", req->nextRequest);
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend:         arg=0x%02x", req->arg);
		//INFO_LOG(Log::HLE, "        sceUsbbdReqSend:        link=0x%02x", req->link);

		const UsbEndpoint *ep = (const UsbEndpoint *)Memory::GetPointerUnchecked(req->endpointPtr);
		const int endpoint = ep->endpointAddres & 0xf;
		const u32 size = req->size;
		INFO_LOG(Log::HLE, "sceUsbbdReqSend: ep=0x%02x, sz=0x%x", ep->endpointAddres, size);
		if (size > 0 && !Memory::IsValidRange(req->data, size)) {
			return hleLogError(Log::HLE, SCE_KERNEL_ERROR_ILLEGAL_ADDR, "bad data");
		}

		static const u8 noData = 0;
		const u8 *data = size > 0 ? Memory::GetPointerUnchecked(req->data) : &noData;
		if (endpoint == 0) {
			// The reply to a control request keeps the header of the request.
			// It says how much it carries (the host's wLength is only the most it takes).
			UsbLinkSendControlReply(data, size);
		} else {
			// Waits for the host to ask for it. It completes when it's sent.
			if (endpoint < USB_LINK_MAX_ENDPOINTS) {
				UsbInEntry entry;
				entry.reqAddr = reqAddr;
				usbInQueue[endpoint].push_back(std::move(entry));
				const u32 next = req->nextRequest;
				UsbLinkServeIn(endpoint, true);
				reqAddr = next;
				continue;
			}
		}

		req->transmitted = size;
		req->returnCode = 0;
		req.NotifyWrite("sceUsbbdReqSend");

		if (req->onComplete_func != 0) {
			u32 args[] = { reqAddr, 0, 0 };
			hleEnqueueCall(req->onComplete_func, ARRAY_SIZE(args), args);
		}
		reqAddr = req->nextRequest;
	}
	return 0;
}

static int sceUsbbdReqRecv(u32 usbDeviceReqAddr) {
	auto req = PSPPointer<UsbdDeviceRequest>::Create(usbDeviceReqAddr);
	if (!req.IsValid() || !Memory::IsValidRange(usbDeviceReqAddr, sizeof(UsbEndpoint))) {
		return hleLogError(Log::HLE, SCE_KERNEL_ERROR_ILLEGAL_ADDR, "bad request");
	}

	UsbStartRegisteredThread();
	req.NotifyRead("sceUsbbdReqRecv");

	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv: endpointPtr=0x%02x", req->endpointPtr);
	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv:        data=0x%02x", req->data);
	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv:        size=0x%02x", req->size);
	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv:   isCtrlReq=0x%02x", req->isControlRequest);
	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv:   onComplCb=0x%02x", req->onComplete_func);
	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv: transmitted=0x%02x", req->transmitted);
	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv:  returnCode=0x%02x", req->returnCode);
	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv: nextRequest=0x%02x", req->nextRequest);
	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv:         arg=0x%02x", req->arg);
	//INFO_LOG(Log::HLE, "    sceUsbbdReqRecv:        link=0x%02x", req->link);

	const UsbEndpoint *ep = (const UsbEndpoint *)Memory::GetPointerUnchecked(req->endpointPtr);
	const int endpoint = ep->endpointAddres & 0xf;
	//INFO_LOG(Log::HLE, "sceUsbbdReqRecv: ep=0x%02x/0x%02x/0x%02x, sz=0x%x", ep->endpointAddres, ep->unk1, ep->unk2, req->size);

	if (endpoint != 0) {
		// Data the host sent already, or the request waits for it (see UsbLinkOnConnect).
		if (!UsbFillRecv(usbDeviceReqAddr, endpoint)) {
			usbPendingRecv[endpoint].push_back(usbDeviceReqAddr);
		} else if (req->onComplete_func != 0) {
			u32 args[] = { usbDeviceReqAddr, 0, 0 };
			hleEnqueueCall(req->onComplete_func, ARRAY_SIZE(args), args);
		}
		return 0;
	}

	u8* dataPtr = Memory::GetPointerWriteRangeOrException(req->data, req->size);
	if (!dataPtr) {
		ERROR_LOG(Log::HLE, "sceUsbbdReqRecv dataPtr null");
		return 0;
	}

	// The data stage of the control request that was just received.
	if (gRecvLen > 12) {
		const u32 size = std::min<u32>(req->size, gRecvLen - 12);
		memcpy(dataPtr, gRecvBuffer + 12, size);
		req->transmitted = size;
		req->returnCode = 0;
		req.NotifyWrite("sceUsbbdReqRecv");

		// Host out: the data is consumed (a later request mustn't see it again), and acknowledged with
		// the empty packet, whatever the request carried.
		gRecvLen = 12;
		UsbLinkSendAck();

		if (req->onComplete_func != 0) {
			INFO_LOG(Log::HLE, "sceUsbbdReqRecv: cb done");
			u32 args[] = { usbDeviceReqAddr , 0, 0 };
			hleEnqueueCall(req->onComplete_func, ARRAY_SIZE(args), args);
		}
	}
	return 0;
}

static int sceUsbbdRegister(u32 usbDrvAddr) {
	INFO_LOG(Log::HLE, "sceUsbbdRegister(drv=%08x)", usbDrvAddr);
	auto usbDrv = PSPPointer<PspUsbDriver>::Create(usbDrvAddr);
	if (usbDrv.IsValid()) {
		sceUsbConfig.pspUsbDriver = *usbDrv;
		usbDrv.NotifyRead("sceUsbbdRegister");
	}
	INFO_LOG(Log::HLE, "sceUsbbdRegister name : %s", Memory::GetPointerOrException(Usbd::getUsbDriver()->name));
	//INFO_LOG(Log::HLE, "sceUsbbdRegister endpoints : %d", Usbd::getUsbDriver()->endpoints);
	//for (int i = 0; i < Usbd::getUsbDriver()->endpoints; i++) {
	//	auto& ep = PSPPointer<UsbEndpoint>::Create(Usbd::getUsbDriver()->endp)[i];
	//	INFO_LOG(Log::HLE, "       endp[%d] : %02x %02x %02x", i, ep.endpointAddres, ep.unk1, ep.unk2);
	//}
	//INFO_LOG(Log::HLE, "sceUsbbdRegister devp_hi : %x", Usbd::getUsbDriver()->devp_hi);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister confp_hi : %x", Usbd::getUsbDriver()->confp_hi);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister devp : %x", Usbd::getUsbDriver()->devp);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister confp : %x", Usbd::getUsbDriver()->confp);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister str : %x", Usbd::getUsbDriver()->str);
	//auto& strp = PSPPointer<u8>::Create(Usbd::getUsbDriver()->str);
	//for (int i = 0; i < 0x1a; i++) {
	//	INFO_LOG(Log::HLE, "       str[%d] : %02x", i, strp[i]);
	//}
	//INFO_LOG(Log::HLE, "sceUsbbdRegister recvctl : %x", Usbd::getUsbDriver()->recvctl_func);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister intf_chang : %x", Usbd::getUsbDriver()->intf_chang_func);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister attach : %x", Usbd::getUsbDriver()->attach_func);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister detach : %x", Usbd::getUsbDriver()->detach_func);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister configure : %x", Usbd::getUsbDriver()->configure_func);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister start_func : %x", Usbd::getUsbDriver()->start_func);
	//INFO_LOG(Log::HLE, "sceUsbbdRegister stop_func : %x", Usbd::getUsbDriver()->stop_func);
	return 0;
}

static int sceUsbbdUnregister(u32 usbDrvAddr) {
	INFO_LOG(Log::HLE, "sceUsbbdUnregister(drv=%08x)", usbDrvAddr);
	auto usbDrv = PSPPointer<PspUsbDriver>::Create(usbDrvAddr);
	if (usbDrv.IsValid()) {
		memset(&sceUsbConfig.pspUsbDriver, 0, sizeof(PspUsbDriver));
		usbDrv.NotifyRead("sceUsbbdUnregister");
	}
	return 0;
}


const HLEFunction sceUsbBus_driver[] =
{
	{0x23E51D8F, &WrapI_U<sceUsbbdReqSend>,          "sceUsbbdReqSend",                         'i', "x"  },
	{0x913EC15D, &WrapI_U<sceUsbbdReqRecv>,          "sceUsbbdReqRecv",                         'i', "x"  },
	{0x951A24CC, nullptr,                            "sceUsbbdClearFIFO",                       '?', ""   },
	{0xB1644BE7, &WrapI_U<sceUsbbdRegister>,         "sceUsbbdRegister",                        'i', "x"  },
	{0xC1E2A540, &WrapI_U<sceUsbbdUnregister>,       "sceUsbbdUnregister",                      'i', "x"  },
	{0xC5E53685, nullptr,                            "sceUsbbdReqCancelAll",                    '?', ""   },
	{0xCC57EC9D, nullptr,                            "sceUsbbdReqCancel",                       '?', ""   },
	{0xE65441C1, nullptr,                            "sceUsbbdStall",                           '?', ""   },
};

const HLEFunction sceUsb1Seg[] =
{
	{}
};

static u32 sceUsbDmbConstruct(u32 arg1, u32 freq, u32 arg3) {
	INFO_LOG(Log::HLE, "sceUsbDmbConstruct(%d, %d, %d)", arg1, freq, arg3);
	return 0;
}
typedef struct {
	u32 nextPtr;
	u32 val1;
	u32 val2;
	u32 val3;
	u32 val4;
	u32 val5;
} DMBResult;
static u32 sceUsbDmbConstructResult(u32 arg1, u32 arg2, u32 arg3) {
	INFO_LOG(Log::HLE, "sceUsbDmbConstructResult(%x, %x, %x)", arg1, arg2, arg3);
	auto dmbResult = PSPPointer<DMBResult>::Create(arg1);
	if (dmbResult.IsValid()) {
		dmbResult->val1 = 0x1234;
		dmbResult->val2 = 0x3456;
		dmbResult->val3 = 0x5678;
		dmbResult->val4 = 0x7891;
		dmbResult->val5 = 0x9abc;
		dmbResult.NotifyWrite("DmbConstructResult");
	}
	return 0;
}
static u32 sceUsbDmbDestruct() {
	INFO_LOG(Log::HLE, "sceUsbDmbDestruct");
	return 0;
}

typedef struct {
	u8 major;
	u8 minor;
} DMBVersion;
static u32 sceUsbDmbGetVersion(u32 argptr) {
	INFO_LOG(Log::HLE, "sceUsbDmbGetVersion(%d)", argptr);
	auto dmbVersion = PSPPointer<DMBVersion>::Create(argptr);
	if (dmbVersion.IsValid()) {
		dmbVersion->major = 0x01;
		dmbVersion->minor = 0x02;
		dmbVersion.NotifyWrite("DmbGetVersion");
	}
	return 0;
}

const HLEFunction sceUsbDmb[] =
{
	/*
	{0x247da6b0, &WrapU_UUU<sceUsbDmbConstruct>,     "sceUsbDmbConstruct",                   'x', "xxx" },
	{0x48fc3564, &WrapU_V<sceUsbDmbDestruct>,        "sceUsbDmbDestruct",                    'x', "" },
	{0x5e5e8e1f, &WrapU_U<sceUsbDmbGetVersion>,      "sceUsbDmbGetVersion",                  'x', "x" },
	{0x62ebf14a, nullptr,                            "sceUsbDmb4",                           '?', "" },
	{0x9e658527, nullptr,                            "sceUsbDmb5",                           '?', "" },
	{0xd40f38fc, &WrapU_UUU<sceUsbDmbConstructResult>, "sceUsbDmbConstructResult",             'x', "xxx" },
	{0x04BE645D, nullptr,                            "sceUsbDmb8",                           '?', "" },
	{0x1127DF8E, nullptr,                            "sceUsbDmb9",                           '?', "" },
	{0x1806F585, nullptr,                            "sceUsbDmb10",                          '?', "" },
	{0x19B8E864, nullptr,                            "sceUsbDmb11",                          '?', "" },
	{0x3F8DBB02, nullptr,                            "sceUsbDmb12",                          '?', "" },
	{0x49C720C7, nullptr,                            "sceUsbDmb13",                          '?', "" },
	{0x4FA5500C, nullptr,                            "sceUsbDmb14",                          '?', "" },
	{0x5D17EDA7, nullptr,                            "sceUsbDmb15",                          '?', "" },
	{0x74151936, nullptr,                            "sceUsbDmb16",                          '?', "" },
	{0x7AC04917, nullptr,                            "sceUsbDmb17",                          '?', "" },
	{0x89A28C61, nullptr,                            "sceUsbDmb18",                          '?', "" },
	{0x9858B28F, nullptr,                            "sceUsbDmb19",                          '?', "" },
	{0x9E84294B, nullptr,                            "sceUsbDmb20",                          '?', "" },
	{0xA0B31495, nullptr,                            "sceUsbDmb21",                          '?', "" },
	{0xA3A3E9A5, nullptr,                            "sceUsbDmb22",                          '?', "" },
	{0xA578EA4F, nullptr,                            "sceUsbDmb23",                          '?', "" },
	{0xA7D6F145, nullptr,                            "sceUsbDmb24",                          '?', "" },
	{0xB354DB59, nullptr,                            "sceUsbDmb25",                          '?', "" },
	{0xD1345A20, nullptr,                            "sceUsbDmb26",                          '?', "" },
	{0xD3DBD48B, nullptr,                            "sceUsbDmb27",                          '?', "" },
	{0xF5E1D17A, nullptr,                            "sceUsbDmb28",                          '?', "" },
	/*/
	{}
	//*/
};

void Register_sceUsb()
{
	RegisterHLEModule("sceUsbstor", ARRAY_SIZE(sceUsbstor), sceUsbstor);
	RegisterHLEModule("sceUsbstorBoot", ARRAY_SIZE(sceUsbstorBoot), sceUsbstorBoot);
	RegisterHLEModule("sceUsb", ARRAY_SIZE(sceUsb), sceUsb);
	RegisterHLEModule("sceUsb_driver", ARRAY_SIZE(sceUsb), sceUsb);
	RegisterHLEModule("sceUsbBus_driver", ARRAY_SIZE(sceUsbBus_driver), sceUsbBus_driver);
	RegisterHLEModule("sceUsb1Seg", ARRAY_SIZE(sceUsb1Seg), sceUsb1Seg);
	RegisterHLEModule("sceUsbDmb", ARRAY_SIZE(sceUsbDmb), sceUsbDmb);
}
