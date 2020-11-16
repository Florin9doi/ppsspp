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

#pragma once

void Register_sceUsb();

void __UsbInit();
void __UsbDoState(PointerWrap &p);

typedef struct {
	u32 name;
	int endpoints;
	u32 endp; // struct UsbEndpoint *endp;
	u32 intp; // struct UsbInterface* intp;
	u32 devp_hi;
	u32 confp_hi;
	u32 devp;
	u32 confp;
	u32 str; // struct StringDescriptor* str;
	u32 recvctl_func; // struct DeviceRequest* req); // used to pull data from a PSP game
	u32 intf_chang_func;
	u32 attach_func;
	u32 detach_func;
	u32 configure_func;
	u32 start_func;
	u32 stop_func;
	u32 link; // struct PspUsbDriver* link;
} PspUsbDriver;


typedef struct {
	u32 endpointPtr;
	u32 data;
	u32 size;
	u32 isControlRequest;
	u32 onComplete_func;
	u32 transmitted;
	u32 returnCode;
	u32 unk1;
	u32 unk2;
	u32 unk3;
} UsbbdDeviceRequest;

namespace Usbd {
	typedef struct {
		PspUsbDriver pspUsbDriver;
		UsbbdDeviceRequest usbDevReq;
	} Config;

	PspUsbDriver *getUsbDriver();
	UsbbdDeviceRequest *getUsbDevReq();
}
