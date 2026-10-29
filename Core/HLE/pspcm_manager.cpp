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

#include <algorithm>
#include <cstring>
#include <string>

#include "Common/Log.h"
#include "Core/HLE/pspcm_manager.h"
#include <Thread/ThreadUtil.h>

#ifdef _WIN32
#include <WinSock2.h>
#include <Ws2tcpip.h>
#undef min
#undef max
#else
#include <sys/select.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#endif

#ifndef _WIN32
#define closesocket close
#define SD_BOTH SHUT_RDWR
#endif

PspCmManager::PspCmManager(ConnectFn onConnect, DisconnectFn onDisconnect, PacketFn onPacket)
	: onConnect_(std::move(onConnect)), onDisconnect_(std::move(onDisconnect)), onPacket_(std::move(onPacket)) {
}

PspCmManager::~PspCmManager() {
	Stop();
}

static int StartServer() {
	struct addrinfo hints {};
	struct addrinfo* info;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	if (getaddrinfo("127.0.0.1", "27015", &hints, &info) != 0)
	{
		ERROR_LOG(Log::HLE, "pspcm_manager: getaddrinfo error");
		return -1;
	}

	int server_socket = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
	if (server_socket == -1)
	{
		ERROR_LOG(Log::HLE, "pspcm_manager: Error creating socket");
		freeaddrinfo(info);
		return -2;
	}

	if (bind(server_socket, info->ai_addr, info->ai_addrlen) != 0)
	{
		ERROR_LOG(Log::HLE, "pspcm_manager: Error binding");
		freeaddrinfo(info);
		closesocket(server_socket);
		return -3;
	}
	freeaddrinfo(info);

	if (listen(server_socket, 10) != 0)
	{
		ERROR_LOG(Log::HLE, "pspcm_manager: Error listening");
		closesocket(server_socket);
		return -3;
	}

	return server_socket;
}


class DataPk {
	static const int data_max = 0x40; // wMaxPacketSize
	static const int usb_max = 8 + data_max; // the 8 bytes of a setup packet, then the data: 0x48
	static const int net_max = 4 + usb_max; // 0x4c

public:
	union {
		u8 net_data[net_max];
		struct {
			u16 magic;
			u8 totalLen;
			u8 endpoint;
			union {
				u8 usb_data[usb_max];
				struct {
					union {
						u8 req_data[8];
						struct { // DeviceRequest
							u8 bmRequestType;
							u8 bRequest;
							u16 wValue;
							u16 wIndex;
							u16 wLength;
						};
					} req;
					u8 data[data_max];
				};
			};
		};
	};

	int getPkLen() {
		return totalLen;
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
		if (size >= 4) {
			magic = read16(ptr, size, &offset);
			if (magic != 0x0ff0) {
				ERROR_LOG(Log::HLE, "pspcm_manager : magic err %x", magic);
				return 0;
			}
			totalLen = read8(ptr, size, &offset);
			endpoint = read8(ptr, size, &offset);
		}

		if (endpoint == 0 && size >= 12) {
			req.bmRequestType = read8(ptr, size, &offset);
			req.bRequest = read8(ptr, size, &offset);
			req.wValue = read16(ptr, size, &offset);
			req.wIndex = read16(ptr, size, &offset);
			req.wLength = read16(ptr, size, &offset);
			// Only a host to device request carries data, wLength of a device to host request is
			// the size of the buffer it expects back.
			if (req.wLength && (req.bmRequestType & 0x80) == 0) {
				if (req.wLength <= data_max && offset + req.wLength <= size) {
					memcpy(data, ptr + offset, req.wLength);
				}
				else {
					return 0;
				}
			}
		}
		else if (endpoint > 0) {
			if (totalLen < 4 || totalLen - 4 > usb_max || size < totalLen) {
				return 0;
			}
			memcpy(usb_data, ptr + offset, totalLen - 4);
		}
		return getPkLen();
	}
};


// Logs at most the first 64 bytes.
static std::string HexDump(const void *buf, int len) {
	char arr[3 * 64 + 1] = {};
	int pos = 0;
	for (int i = 0; i < len && i < 64; i++) {
		pos += snprintf(arr + pos, sizeof(arr) - pos, "%02x ", ((const unsigned char *)buf)[i]);
	}
	return arr;
}

void PspCmManager::Send(const char *buf, int len) {
	int client = client_;
	if (client < 0) {
		return;
	}
	int ret = send(client, buf, len, 0);
	ERROR_LOG(Log::HLE, "pspcm_manager: send >> : %d/%d [%s]", ret, len, HexDump(buf, len).c_str());

}

// A transfer to the host, framed like the ones received: magic, length (header included), endpoint.
// The endpoint is the USB address the host knows (0x81 bulk in, 0x83 interrupt in).
void PspCmManager::SendToHost(int endpoint, const u8 *data, u32 size) {
	static const u32 maxChunk = 0x40;
	u32 pos = 0;
	do {
		u32 chunk = std::min(size - pos, maxChunk);
		char pkt[4 + maxChunk] = { (char)0xf0, 0x0f, (char)(4 + chunk), (char)endpoint };
		memcpy(pkt + 4, data + pos, chunk);
		Send(pkt, 4 + chunk);
		pos += chunk;
	} while (pos < size);
}

void PspCmManager::Push(Msg &&msg) {
	std::lock_guard<std::mutex> guard(mutex_);
	inbox_.push_back(std::move(msg));
}

// TCP has no packet boundaries. Takes the first whole packet (magic, length, ...) off the front of
// the received bytes. False if the rest isn't complete yet.
static bool TakePacket(std::vector<u8> &stream, std::vector<u8> &packet) {
	while (stream.size() >= 4) {
		if (stream[0] != 0xf0 || stream[1] != 0x0f || stream[2] < 4) {
			WARN_LOG(Log::HLE, "pspcm_manager: lost sync, skipping a byte (%02x)", stream[0]);
			stream.erase(stream.begin());
			continue;
		}
		const size_t len = stream[2];
		if (stream.size() < len) {
			return false;
		}
		packet.assign(stream.begin(), stream.begin() + len);
		stream.erase(stream.begin(), stream.begin() + len);
		return true;
	}
	return false;
}

// Closes the connection to the host, if there is one, and tells the CPU thread.
void PspCmManager::DropClient() {
	// Whoever closes the socket first takes it out of client_.
	int open = client_.exchange(-1);
	if (open >= 0) {
		shutdown(open, SD_BOTH);
		closesocket(open);
		Push({ MsgType::DISCONNECTED });
	}
}

// Socket thread: nothing in here may touch PSP memory or the kernel.
// It waits for the server and the host at once: a host that comes back (rpcs3 restarted) replaces the
// previous connection, even if that one was never seen closing.
void PspCmManager::Thread() {
	SetCurrentThreadName("PSLinkThread");

	std::vector<u8> stream;
	std::vector<u8> packet;

	while (running_) {
		int server = server_;
		int client = client_;
		if (server < 0) {
			break;
		}

		fd_set readSet;
		FD_ZERO(&readSet);
		FD_SET(server, &readSet);
		int maxFd = server;
		if (client >= 0) {
			FD_SET(client, &readSet);
			maxFd = std::max(maxFd, client);
		}
		// A timeout, so that a stop is noticed even if closing the sockets doesn't wake the select.
		struct timeval timeout = { 0, 200 * 1000 };
		int ready = select(maxFd + 1, &readSet, NULL, NULL, &timeout);
		if (!running_) {
			break;
		}
		if (ready < 0) {
			// A socket closed by a stop, or a real error.
			if (server_ < 0) {
				break;
			}
			if (client_ != client) {
				continue;
			}
			ERROR_LOG(Log::HLE, "pspcm_manager: select failed");
			DropClient();
			continue;
		}
		if (ready == 0) {
			continue;
		}

		if (FD_ISSET(server, &readSet)) {
			int newClient = accept(server, NULL, NULL);
			if (newClient < 0) {
				if (running_) {
					ERROR_LOG(Log::HLE, "pspcm_manager: Error accepting");
				}
				break;
			}
			// The host is back: whatever was connected before is gone.
			DropClient();
			stream.clear();
			client_ = newClient;
			if (!running_) {
				// Stopped while accepting, which may have missed this socket.
				break;
			}
			// Replies are small and the host waits for them.
			int noDelay = 1;
			setsockopt(newClient, IPPROTO_TCP, TCP_NODELAY, (const char *)&noDelay, sizeof(noDelay));
			int keepAlive = 1;
			setsockopt(newClient, SOL_SOCKET, SO_KEEPALIVE, (const char *)&keepAlive, sizeof(keepAlive));
			INFO_LOG(Log::HLE, "pspcm_manager: host connected");
			Push({ MsgType::CONNECTED });
			continue;
		}

		if (client >= 0 && FD_ISSET(client, &readSet)) {
			unsigned char recvbuf[255];
			int recvLen = recv(client, (char *)recvbuf, sizeof(recvbuf), 0);
			if (recvLen <= 0) {
				ERROR_LOG(Log::HLE, "pspcm_manager: Connection closed (%d)", recvLen);
				if (client_ == client) {
					DropClient();
				}
				stream.clear();
				continue;
			}

			ERROR_LOG(Log::HLE, "pspcm_manager:   >> received: %d [%s]", recvLen, HexDump(recvbuf, recvLen).c_str());
			stream.insert(stream.end(), recvbuf, recvbuf + recvLen);

			while (TakePacket(stream, packet)) {
				DataPk pk = DataPk();
				int pklen = pk.read(packet.data(), (int)packet.size());
				if (pklen == 0 || pklen > (int)sizeof(pk.net_data)) {
					continue;
				}

				Msg msg;
				msg.type = MsgType::PACKET;
				msg.endpoint = pk.endpoint;
				if (pk.endpoint & 0x80) {
					msg.packetType = PspCmPacketType::IN_REQUEST;
				} else if (pk.endpoint == 0) {
					msg.packetType = PspCmPacketType::CONTROL;
					msg.data.assign(pk.net_data, pk.net_data + pklen);
				} else {
					msg.packetType = PspCmPacketType::BULK;
					msg.data.assign(pk.usb_data, pk.usb_data + (pklen - 4));
				}
				Push(std::move(msg));
			}
		}
	}

	DropClient();
	INFO_LOG(Log::HLE, "pspcm_manager: PSLinkThread ended");
}


bool PspCmManager::Start() {
	if (thread_.joinable()) {
		thread_.join();
	}
	// Listening before the thread starts, so a stop can't miss the server socket.
	server_ = StartServer();
	if (server_ < 0) {
		return false;
	}
	running_ = true;
	thread_ = std::thread(&PspCmManager::Thread, this);
	return true;
}

void PspCmManager::Stop() {
	running_ = false;
	int client = client_.exchange(-1);
	if (client >= 0) {
		shutdown(client, SD_BOTH);
		closesocket(client);
	}
	int server = server_.exchange(-1);
	if (server >= 0) {
		shutdown(server, SD_BOTH);
		closesocket(server);
	}
	if (thread_.joinable()) {
		thread_.join();
	}
	ClearInbox();
}

void PspCmManager::ClearInbox() {
	std::lock_guard<std::mutex> guard(mutex_);
	inbox_.clear();
}

void PspCmManager::Poll() {
	std::deque<Msg> msgs;
	{
		std::lock_guard<std::mutex> guard(mutex_);
		msgs.swap(inbox_);
	}
	for (Msg &msg : msgs) {
		switch (msg.type) {
		case MsgType::CONNECTED:
			onConnect_();
			break;
		case MsgType::DISCONNECTED:
			onDisconnect_();
			break;
		case MsgType::PACKET:
			onPacket_(msg.packetType, msg.endpoint, msg.data);
			break;
		}
	}
}
