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

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "Common/CommonTypes.h"

enum class PspCmPacketType {
	CONTROL,     // data: the whole packet, control header included.
	BULK,        // data: the payload.
	IN_REQUEST,  // the host wants data from an IN endpoint (0x81, 0x83), no payload.
};

// The link to the host, the PS3 side of usbpspcm.prx: a TCP server the host connects to, and the packets
// of the protocol that go both ways. What a connection or a packet means is up to the functions it is given.
//
// The socket thread only receives and queues. Poll() runs on the thread that owns the PSP (the CPU thread),
// and calls the functions for what was queued.
class PspCmManager {
public:
	using ConnectFn = std::function<void()>;
	using DisconnectFn = std::function<void()>;
	using PacketFn = std::function<void(PspCmPacketType type, int endpoint, std::vector<u8> &data)>;

	// The host connected / the connection was closed (by either side).
	PspCmManager(ConnectFn onConnect, DisconnectFn onDisconnect, PacketFn onPacket);
	~PspCmManager();

	// Listens and starts the socket thread.
	bool Start();
	// Closes everything, forgets what was queued.
	void Stop();
	bool IsRunning() const { return running_; }
	// Closes the connection of the host, if there is one. The host connects again.
	void DropClient();

	// Calls the functions for what the socket thread queued.
	void Poll();
	void ClearInbox();

	// A packet as it is, or the data of an IN endpoint in packets of at most 0x40 bytes.
	// The endpoint is the USB address the host knows (0x81 bulk in, 0x83 interrupt in).
	void Send(const char *buf, int len);
	void SendToHost(int endpoint, const u8 *data, u32 size);

private:
	enum class MsgType { CONNECTED, DISCONNECTED, PACKET };
	struct Msg {
		MsgType type;
		PspCmPacketType packetType = PspCmPacketType::CONTROL;
		int endpoint = 0;
		std::vector<u8> data;
	};

	void Push(Msg &&msg);
	void Thread();

	ConnectFn onConnect_;
	DisconnectFn onDisconnect_;
	PacketFn onPacket_;

	std::atomic<bool> running_{false};
	std::thread thread_;
	std::atomic<int> server_{-1};
	std::atomic<int> client_{-1};

	std::mutex mutex_;
	std::deque<Msg> inbox_;
};
