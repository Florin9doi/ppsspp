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
#include "Core/HLE/FunctionWrappers.h"
#include "Core/HLE/KernelWaitHelpers.h"
#include "Core/HLE/sceKernelMemory.h"
#include "Core/HLE/sceKernelThread.h"
#include "Core/HLE/sceUsb.h"
#include "Core/MemMapHelpers.h"
#include "Core/MIPS/MIPS.h"
#include "Core/Reporting.h"
#include <Thread/ThreadUtil.h>
#include <TimeUtil.h>

#ifdef _WIN32
#include <WinSock2.h>
#include <Ws2tcpip.h>
#ifndef AI_ADDRCONFIG
#define AI_ADDRCONFIG 0x0400
#endif
#undef min
#undef max
#else
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <unistd.h>
#endif
#include <File/FileDescriptor.h>

static constexpr uint32_t ERROR_USB_WAIT_TIMEOUT = 0x80243008;

// TODO: Map by driver name
static bool usbStarted = false;
// TODO: Check actual status
static bool usbConnected = true;
// TODO: Activation by product id
static bool usbActivated = false;

std::thread ps3_thread;

static int usbWaitTimer = -1;
static std::vector<SceUID> waitingThreads;
static Usbd::Config sceUsbConfig; // TODO: move in namespace

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

	PSPPointer<int> timeout = PSPPointer<int>::Create(__KernelGetWaitTimeoutPtr(threadID, error));
	if (timeout.IsValid())
		*timeout = 0;

	HLEKernel::RemoveWaitingThread(waitingThreads, threadID);
	__KernelResumeThreadFromWait(threadID, ERROR_USB_WAIT_TIMEOUT);
	__KernelReSchedule("wait timed out");
}

static void UsbUpdateState() {
	u32 error;
	bool wokeThreads = false;
	for (size_t i = 0; i < waitingThreads.size(); ++i) {
		SceUID threadID = waitingThreads[i];
		int state = __KernelGetWaitID(threadID, WAITTYPE_USB, error);
		if (error != 0)
			continue;

		u32 mode = __KernelGetWaitValue(threadID, error);
		if (UsbMatchState(state, mode)) {
			waitingThreads.erase(waitingThreads.begin() + i);
			--i;

			PSPPointer<int> timeout = PSPPointer<int>::Create(__KernelGetWaitTimeoutPtr(threadID, error));
			if (timeout.IsValid() && usbWaitTimer != -1) {
				// Remove any event for this thread.
				s64 cyclesLeft = CoreTiming::UnscheduleEvent(usbWaitTimer, threadID);
				*timeout = (int)cyclesToUs(cyclesLeft);
			}

			__KernelResumeThreadFromWait(threadID, UsbCurrentState());
		}
	}

	if (wokeThreads)
		hleReSchedule("usb state change");
}

static int StartServer() {
	struct addrinfo hints {};
	struct addrinfo* info;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	if (getaddrinfo("127.0.0.1", "27015", &hints, &info) != 0)
	{
		ERROR_LOG(HLE, "pspcm_manager: getaddrinfo error");
		return -1;
	}

	int server_socket = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
	if (server_socket == -1)
	{
		ERROR_LOG(HLE, "pspcm_manager: Error creating socket");
		freeaddrinfo(info);
		return -2;
	}

	if (bind(server_socket, info->ai_addr, info->ai_addrlen) != 0)
	{
		ERROR_LOG(HLE, "pspcm_manager: Error binding");
		freeaddrinfo(info);
		closesocket(server_socket);
		return -3;
	}
	freeaddrinfo(info);

	if (listen(server_socket, 10) != 0)
	{
		ERROR_LOG(HLE, "pspcm_manager: Error listening");
		closesocket(server_socket);
		return -3;
	}

	return server_socket;
}

class DataPk {
public:
	union {
		u8 pk_buf[0x50];
		struct {
			u16 magic;
			u8 totalLen;
			u8 endpoint;

			union {
				u8 buf[8];
				struct {
					u8 bmRequestType;
					u8 bRequest;
					u16 wValue;
					u16 wIndex;
					u16 wLength;
				};
			} req;
			u8 data[0x40];
		};
	};

	int getPkLen() {
		return 12 + req.wLength;
	}

	int read8(unsigned char* buf, int size, int* offset) {
		if (*offset < size) {
			return buf[(*offset)++];
		}
		return -1;
	}

	int read16(unsigned char* buf, int size, int* offset) {
		if (*offset < size - 1) {
			int ret = buf[(*offset)++];
			ret += buf[(*offset)++] << 8;
			return ret;
		}
		return -1;
	}

public:
	int read(unsigned char* ptr, int size) {
		int offset = 0;
		if (size >= 12) {
			magic = read16(ptr, size, &offset);
			ERROR_LOG(HLE, "pspcm_manager : magic %x", magic);
			if (magic != 0x0ff0) {
				return 0;
			}
			totalLen = read8(ptr, size, &offset);
			endpoint = read8(ptr, size, &offset);

			req.bmRequestType = read8(ptr, size, &offset);
			req.bRequest = read8(ptr, size, &offset);
			req.wValue = read16(ptr, size, &offset);
			req.wIndex = read16(ptr, size, &offset);
			req.wLength = read16(ptr, size, &offset);
		}
		if (req.wLength) {
			if (offset + req.wLength <= size) {
				memcpy(data, ptr + offset, req.wLength);
			}
			else {
				return 0;
			}
		}
		return getPkLen();
	}
};

int client;
char gRecvBuffer[512];
int gRecvLen = 0;

static void PS3Thread() {
	SetCurrentThreadName("PS3Thread");

	int server = StartServer();

	while (usbActivated) {
		client = accept(server, NULL, NULL);
		if (client < 0) {
			ERROR_LOG(HLE, "pspcm_manager: Error accepting");
			return;
		}

		int iSendResult;
		unsigned char recvbuf[255];
		int recvLen;

		DataPk pk = DataPk();

		do {

			recvLen = recv(client, (char*) recvbuf, sizeof(recvbuf), 0);
			if (recvLen <= 0) {
				ERROR_LOG(HLE, "pspcm_manager: Connection closed (%d)", recvLen);
				break;
			}

			char arr[1024];
			int pos = sprintf(arr, "  >> received: %d [", recvLen);
			for (int i = 0; i < recvLen; i++) {
				pos += sprintf(arr + pos, "%02x ", (unsigned char)recvbuf[i]);
			}
			pos += sprintf(arr + pos, "]");
			ERROR_LOG(HLE, "pspcm_manager: %s", arr);

			int pklen = pk.read((unsigned char*)recvbuf, recvLen);
			if (pklen == 0) {
				continue;
			}
			ERROR_LOG(HLE, "pspcm_manager : getPacketLen %d", pklen);

			u32 structSize = sizeof(DeviceRequest);
			u32 dataBufAddr = userMemory.Alloc(structSize, false, "sceUsb"); // TODO: allocate only once


			if (pk.endpoint == 0) {
				DeviceRequest *req = (DeviceRequest*) Memory::GetPointer(dataBufAddr);
				std::memcpy(req, pk.req.buf, sizeof(DeviceRequest));
				ERROR_LOG(HLE, "pspcm_manager : req->bmRequestType %x", req->bmRequestType);
				ERROR_LOG(HLE, "pspcm_manager : req->bRequest %x", req->bRequest);
				ERROR_LOG(HLE, "pspcm_manager : req->wValue %x", req->wValue);
				ERROR_LOG(HLE, "pspcm_manager : req->wIndex %x", req->wIndex);
				ERROR_LOG(HLE, "pspcm_manager : req->wLength %x", req->wLength);


				gRecvLen = req->wLength;
				if (gRecvLen) {
					memcpy(gRecvBuffer, pk.data, req->wLength);
				}
				
				
				while (pk.req.bRequest == 2);


				u32 args[] = { req->bmRequestType, 0, dataBufAddr };
				hleEnqueueCall(Usbd::getUsbDriver()->recvctl_func, ARRAY_SIZE(args), args);
			}
		} while (true);
	}
}

void send_to_ps3(const char *buf, int len) {
	int ret = send(client, buf, len, 0);
	ERROR_LOG(HLE, "pspcm_manager : send_to_ps3 %d", ret);
}


void __UsbInit() {
	usbStarted = false;
	usbConnected = true;
	usbActivated = false;
	waitingThreads.clear();

	usbWaitTimer = CoreTiming::RegisterEvent("UsbWaitTimeout", UsbWaitExecTimeout);
	memset(&sceUsbConfig, 0, sizeof(Usbd::Config));
}

void __UsbDoState(PointerWrap &p) {
	auto s = p.Section("sceUsb", 1, 3);
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
}

static int sceUsbStart(const char* driverName, u32 argsSize, u32 argsPtr) {
	INFO_LOG(HLE, "sceUsbStart(%s, size=%i, args=%08x)", driverName, argsSize, argsPtr);
	usbStarted = true;
	UsbUpdateState();

	if (Usbd::getUsbDriver()->name != NULL &&
			strcmp(driverName, (const char*)Memory::GetPointer(Usbd::getUsbDriver()->name)) == 0) {
		if (Usbd::getUsbDriver()->start_func != NULL) {
			u32 args[] = { argsSize, argsPtr };
			hleEnqueueCall(Usbd::getUsbDriver()->start_func, ARRAY_SIZE(args), args);
		}
	}

	return 0;
}

static int sceUsbStop(const char* driverName, u32 argsSize, u32 argsPtr) {
	INFO_LOG(HLE, "sceUsbStop(%s, size=%i, args=%08x)", driverName, argsSize, argsPtr);
	usbStarted = false;
	UsbUpdateState();

	if (Usbd::getUsbDriver()->name != NULL &&
		strcmp(driverName, (const char*)Memory::GetPointer(Usbd::getUsbDriver()->name)) == 0) {
		if (Usbd::getUsbDriver()->stop_func != NULL) {
			u32 args[] = { argsSize, argsPtr };
			hleEnqueueCall(Usbd::getUsbDriver()->stop_func, ARRAY_SIZE(args), args);
		}
	}

	return 0;
}

static int sceUsbGetState() {
	int state = 0;
	if (!usbStarted) {
		state = 0x80243007;
	} else {
		state = UsbCurrentState();
	}
	DEBUG_LOG(HLE, "sceUsbGetState: 0x%x", state);
	return state;
}

static int sceUsbActivate(u32 pid) {
	INFO_LOG(HLE, "sceUsbActivate(0x%04x)", pid);
	usbActivated = true;

	if (pid == 0x01cb) {
		ps3_thread = std::thread(&PS3Thread);
	}

	UsbUpdateState();

	if (Usbd::getUsbDriver()->attach_func != NULL) {
		u32 speed = 2; // usb_version : speed 1=full, 2=hi
		u32 args[] = { speed, 0, 0 };
		hleEnqueueCall(Usbd::getUsbDriver()->attach_func, ARRAY_SIZE(args), args);
	}

	if (Usbd::getUsbDriver()->configure_func != NULL) {
		u32 speed = 2; // usb_version : speed 1=full, 2=hi
		u32 args[] = { speed, 0, 0 }; // usb_version
		hleEnqueueCall(Usbd::getUsbDriver()->configure_func, ARRAY_SIZE(args), args);
	}

	if (Usbd::getUsbDriver()->intf_chang_func != NULL) {
		u32 args[] = { 0, 0, 0 }; // interfaceNumber, alternateSetting, unk
		hleEnqueueCall(Usbd::getUsbDriver()->intf_chang_func, ARRAY_SIZE(args), args);
	}

	return 0;
}

static int sceUsbDeactivate(u32 pid) {
	INFO_LOG(HLE, "sceUsbDeactivate(0x%04x)", pid);
	usbActivated = false;
	UsbUpdateState();
	if (pid == 0x01cb) {
		// TODO: stop thread
	}
	return 0;
}

static int sceUsbWaitState(int state, u32 waitMode, u32 timeoutPtr) {
	hleEatCycles(10000);

	if (waitMode >= 2)
		return hleLogError(HLE, SCE_KERNEL_ERROR_ILLEGAL_MODE, "invalid mode");
	if (state == 0)
		return hleLogError(HLE, SCE_KERNEL_ERROR_EVF_ILPAT, "bad state");

	if (UsbMatchState(state, waitMode)) {
		return hleLogSuccessX(HLE, UsbCurrentState());
	}

	// We'll have to wait as long as it takes.  Cleanup first, just in case.
	HLEKernel::RemoveWaitingThread(waitingThreads, __KernelGetCurThread());
	waitingThreads.push_back(__KernelGetCurThread());

	UsbSetTimeout(PSPPointer<int>::Create(timeoutPtr));
	__KernelWaitCurThread(WAITTYPE_USB, state, waitMode, timeoutPtr, false, "usb state waited");
	return hleLogSuccessI(HLE, 0, "waiting");
}

static int sceUsbWaitStateCB(int state, u32 waitMode, u32 timeoutPtr) {
	ERROR_LOG_REPORT(HLE, "UNIMPL sceUsbWaitStateCB(%i, %i, %08x)", state, waitMode, timeoutPtr);
	return 0;
}

static int sceUsbstorBootSetCapacity(u32 capacity) {
	return hleReportError(HLE, 0, "unimplemented");
}

const HLEFunction sceUsb[] =
{
	{0XAE5DE6AF, &WrapI_CUU<sceUsbStart>,            "sceUsbStart",                             'i', "sxx"},
	{0XC2464FA0, &WrapI_CUU<sceUsbStop>,             "sceUsbStop",                              'i', "sxx"},
	{0XC21645A4, &WrapI_V<sceUsbGetState>,           "sceUsbGetState",                          'i', ""   },
	{0X4E537366, nullptr,                            "sceUsbGetDrvList",                        '?', ""   },
	{0X112CC951, nullptr,                            "sceUsbGetDrvState",                       '?', ""   },
	{0X586DB82C, &WrapI_U<sceUsbActivate>,           "sceUsbActivate",                          'i', "x"  },
	{0XC572A9C8, &WrapI_U<sceUsbDeactivate>,         "sceUsbDeactivate",                        'i', "x"  },
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

//UsbdDeviceRequest* Usbd::getUsbDevReq() {
//	return &sceUsbConfig.usbDevReq;
//}

static int sceUsbbdReqSend(u32 usbDeviceReqAddr) {
	auto usbDeviceReq = PSPPointer<UsbdDeviceRequest>::Create(usbDeviceReqAddr);
	if (usbDeviceReq.IsValid()) {
		//sceUsbConfig.usbDevReq = *usbDeviceReq;
		usbDeviceReq.NotifyRead("sceUsbbdReqSend");
	}
	INFO_LOG(HLE, "sceUsbbdReqSend: sz=0x%x", usbDeviceReq->size);
	const char* data = (char*) Memory::GetPointer(usbDeviceReq->data);
	for (int i = 0; i < usbDeviceReq->size; i++) {
		INFO_LOG(HLE, "    %02x ", data[i]);
	}

	send_to_ps3(data, usbDeviceReq->size);

	if (usbDeviceReq->onComplete_func != NULL) {
		u32 args[] = { usbDeviceReqAddr, 0, 0 };
		hleEnqueueCall(usbDeviceReq->onComplete_func, ARRAY_SIZE(args), args);
	}
	return 0;
}

static int sceUsbbdReqRecv(u32 usbDeviceReqAddr) {
	auto usbDeviceReq = PSPPointer<UsbdDeviceRequest>::Create(usbDeviceReqAddr);
	if (usbDeviceReq.IsValid()) {
		//sceUsbConfig.usbDevReq = *usbDeviceReq;
		usbDeviceReq.NotifyRead("sceUsbbdReqRecv");
	}
	INFO_LOG(HLE, "sceUsbbdReqRecv: sz=0x%x", usbDeviceReq->size);
	int ret = 0;

	UsbEndpoint* ep = (UsbEndpoint*) Memory::GetPointer(usbDeviceReq->endpointPtr);
	INFO_LOG(HLE, "        endpointPtr 0x%02x: %02x %02x %02x",  usbDeviceReq->endpointPtr, ep->endpointAddres, ep->unk1, ep->unk2);
	INFO_LOG(HLE, "        data: 0x%x", usbDeviceReq->data);
	INFO_LOG(HLE, "        size: 0x%x", usbDeviceReq->size);
	INFO_LOG(HLE, "        isControlRequest: 0x%x", usbDeviceReq->isControlRequest);
	INFO_LOG(HLE, "        onComplete_func: 0x%x", usbDeviceReq->onComplete_func);
	INFO_LOG(HLE, "        transmitted: 0x%x", usbDeviceReq->transmitted);
	INFO_LOG(HLE, "        returnCode: 0x%x", usbDeviceReq->returnCode);
	INFO_LOG(HLE, "        nextRequest: 0x%x", usbDeviceReq->nextRequest);
	INFO_LOG(HLE, "        arg: 0x%x", usbDeviceReq->arg);
	INFO_LOG(HLE, "        link: 0x%x", usbDeviceReq->link);

	u8* dataPtr = Memory::GetPointerWriteRange(usbDeviceReq->data, usbDeviceReq->size);
	if (!dataPtr) {
		ERROR_LOG(HLE, "sceUsbbdReqRecv dataPtr null");
		return 0;
	}

	if (gRecvLen > 0) {
		memcpy(dataPtr, gRecvBuffer, gRecvLen);
		usbDeviceReq->transmitted = gRecvLen;
		send_to_ps3(gRecvBuffer, gRecvLen);
		ret = 1;
	}

	char arr[1024];
	int pos = sprintf(arr, "  >> sceUsbbdReqRecv: %d [", usbDeviceReq->size);
	for (int i = 0; i < usbDeviceReq->size; i++) {
		pos += sprintf(arr + pos, "%02x ", (unsigned char)dataPtr[i]);
	}
	pos += sprintf(arr + pos, "]");
	ERROR_LOG(HLE, "pspcm_manager: %s", arr);

	if (usbDeviceReq->onComplete_func != NULL && ret > 0) {
		u32 args[] = { usbDeviceReqAddr , 0, 0 };
		hleEnqueueCall(usbDeviceReq->onComplete_func, ARRAY_SIZE(args), args);
	}
	return 0;
}

static int sceUsbbdRegister(u32 usbDrvAddr) {
	INFO_LOG(HLE, "sceUsbbdRegister(drv=%08x)", usbDrvAddr);
	auto& usbDrv = PSPPointer<PspUsbDriver>::Create(usbDrvAddr);
	if (usbDrv.IsValid()) {
		sceUsbConfig.pspUsbDriver = *usbDrv;
		usbDrv.NotifyRead("sceUsbbdRegister");
	}
	INFO_LOG(HLE, "sceUsbbdRegister name : %s", Memory::GetPointer(sceUsbConfig.pspUsbDriver.name));
	INFO_LOG(HLE, "sceUsbbdRegister endpoints : %d", sceUsbConfig.pspUsbDriver.endpoints);
	for (int i = 0; i < sceUsbConfig.pspUsbDriver.endpoints; i++) {
		auto& ep = PSPPointer<UsbEndpoint>::Create(sceUsbConfig.pspUsbDriver.endp)[i];
		INFO_LOG(HLE, "       endp[%d] : %02x %02x %02x", i, ep.endpointAddres, ep.unk1, ep.unk2);
	}
	INFO_LOG(HLE, "sceUsbbdRegister recvctl : %x", Usbd::getUsbDriver()->recvctl_func);
	INFO_LOG(HLE, "sceUsbbdRegister intf_chang : %x", Usbd::getUsbDriver()->intf_chang_func);
	INFO_LOG(HLE, "sceUsbbdRegister attach : %x", Usbd::getUsbDriver()->attach_func);
	INFO_LOG(HLE, "sceUsbbdRegister detach : %x", Usbd::getUsbDriver()->detach_func);
	INFO_LOG(HLE, "sceUsbbdRegister configure : %x", Usbd::getUsbDriver()->configure_func);
	INFO_LOG(HLE, "sceUsbbdRegister start_func : %x", Usbd::getUsbDriver()->start_func);
	INFO_LOG(HLE, "sceUsbbdRegister stop_func : %x", Usbd::getUsbDriver()->stop_func);
	return 0;
}

static int sceUsbbdUnregister(u32 usbDrvAddr) {
	INFO_LOG(HLE, "sceUsbbdUnregister(drv=%08x)", usbDrvAddr);
	auto& usbDrv = PSPPointer<PspUsbDriver>::Create(usbDrvAddr);
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

void Register_sceUsb()
{
	RegisterModule("sceUsbstor", ARRAY_SIZE(sceUsbstor), sceUsbstor);
	RegisterModule("sceUsbstorBoot", ARRAY_SIZE(sceUsbstorBoot), sceUsbstorBoot);
	RegisterModule("sceUsb", ARRAY_SIZE(sceUsb), sceUsb);
	RegisterModule("sceUsb_driver", ARRAY_SIZE(sceUsb), sceUsb);
	RegisterModule("sceUsbBus_driver", ARRAY_SIZE(sceUsbBus_driver), sceUsbBus_driver);
}
