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
#include "Core/HLE/sceKernelThread.h"
#include "Core/HLE/sceUsb.h"
#include "Core/MemMapHelpers.h"
#include "Core/MIPS/MIPS.h"
#include "Core/Reporting.h"

static constexpr uint32_t ERROR_USB_WAIT_TIMEOUT = 0x80243008;

// TODO: Map by driver name
static bool usbStarted = false;
// TODO: Check actual status
static bool usbConnected = true;
// TODO: Activation by product id
static bool usbActivated = false;

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
	INFO_LOG(HLE, "sceUsbStart(%s, %i, %08x)", driverName, argsSize, argsPtr);
	usbStarted = true;
	UsbUpdateState();

	if (Usbd::getUsbDriver()->name != NULL &&
			strcmp(driverName, (const char*)Memory::GetPointer(Usbd::getUsbDriver()->name)) == 0) {
		if (Usbd::getUsbDriver()->start_func != NULL) {
			u32 args[2] = { argsSize, argsPtr };
			hleEnqueueCall((u32)Usbd::getUsbDriver()->start_func, 2, args);
		}

		if (Usbd::getUsbDriver()->attach_func != NULL) {
			u32 args[3] = { 1, 0, 0 };
			hleEnqueueCall((u32)Usbd::getUsbDriver()->attach_func, 3, args);
		}

		if (Usbd::getUsbDriver()->intf_chang_func != NULL) {
			u32 args[3] = { 1, 1, 0 };
			hleEnqueueCall((u32)Usbd::getUsbDriver()->intf_chang_func, 3, args);
		}
	}

	return 0;
}

static int sceUsbStop(const char* driverName, u32 argsSize, u32 argsPtr) {
	INFO_LOG(HLE, "sceUsbStop(%s, %i, %08x)", driverName, argsSize, argsPtr);
	usbStarted = false;
	UsbUpdateState();
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
	INFO_LOG(HLE, "sceUsbActivate(%i)", pid);
	usbActivated = true;
	UsbUpdateState();
	return 0;
}

static int sceUsbDeactivate(u32 pid) {
	INFO_LOG(HLE, "sceUsbDeactivate(%i)", pid);
	usbActivated = false;
	UsbUpdateState();
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

static int sceUsbbdRegister(u32 usbDrvAddr) {
	INFO_LOG(HLE, "sceUsbbdRegister");
	if (Memory::IsValidRange(usbDrvAddr, sizeof(PspUsbDriver))) {
		Memory::ReadStruct(usbDrvAddr, Usbd::getUsbDriver());
	}
	INFO_LOG(HLE, "sceUsbbdRegister name : %s", Memory::GetPointer(Usbd::getUsbDriver()->name));
	INFO_LOG(HLE, "sceUsbbdRegister recvctl : %x", Usbd::getUsbDriver()->recvctl_func);
	INFO_LOG(HLE, "sceUsbbdRegister attach : %x", Usbd::getUsbDriver()->attach_func);
	INFO_LOG(HLE, "sceUsbbdRegister detach : %x", Usbd::getUsbDriver()->detach_func);
	INFO_LOG(HLE, "sceUsbbdRegister start_func : %x", Usbd::getUsbDriver()->start_func);
	INFO_LOG(HLE, "sceUsbbdRegister stop_func : %x", Usbd::getUsbDriver()->stop_func);
	return 0;
}

const HLEFunction sceUsbBus_driver[] =
{
	{0x23E51D8F, nullptr,                            "sceUsbbdReqSend",                         '?', ""   },
	{0x913EC15D, nullptr,                            "sceUsbbdReqRecv",                         '?', ""   },
	{0x951A24CC, nullptr,                            "sceUsbbdClearFIFO",                       '?', ""   },
	{0xB1644BE7, &WrapI_U<sceUsbbdRegister>,         "sceUsbbdRegister",                        'i', "x"  },
	{0xC1E2A540, nullptr,                            "sceUsbbdUnregister",                      '?', ""   },
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
