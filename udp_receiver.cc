#include "udp_receiver.h"
#include <stdexcept>
#include <iostream>

UDPReceiver::UDPReceiver(uint16_t port) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
        throw std::runtime_error("WSAStartup failed");
    }

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        WSACleanup();
        throw std::runtime_error("socket creation failed");
    }

    DWORD timeout = 5; // ms
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&timeout),
                   sizeof(timeout)) == SOCKET_ERROR) {
        closesocket(sock);
        WSACleanup();
        throw std::runtime_error("setsockopt SO_RCVTIMEO failed");
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        closesocket(sock);
        WSACleanup();
        throw std::runtime_error("bind failed");
    }
}

UDPReceiver::~UDPReceiver() {
    stopReceiveThread();
    closesocket(sock);
    WSACleanup();
}

int UDPReceiver::receive(std::vector<uint8_t>& buf) {
    buf.resize(4096);
    sockaddr_in from;
    int fromlen = sizeof(from);

    int ret = recvfrom(sock, reinterpret_cast<char*>(buf.data()),
                       static_cast<int>(buf.size()), 0,
                       reinterpret_cast<sockaddr*>(&from), &fromlen);

    if (ret == SOCKET_ERROR) {
        int err = WSAGetLastError();
        if (err == WSAETIMEDOUT) return 0;
        std::cerr << "recvfrom failed, err=" << err << std::endl;
        return -1;
    }

    buf.resize(ret);
    return ret;
}

void UDPReceiver::startReceiveThread() {
    if (running) return; // already running
    running = true;
    recv_thread = std::thread(&UDPReceiver::receiveLoop, this);
}

void UDPReceiver::stopReceiveThread() {
    running = false;
    if (recv_thread.joinable()) recv_thread.join();
}

void UDPReceiver::receiveLoop() {
    std::vector<uint8_t> buf(4096);
    sockaddr_in from;
    int fromlen = sizeof(from);

    while (running) {
        int ret = recvfrom(sock, reinterpret_cast<char*>(buf.data()),
                           static_cast<int>(buf.size()), 0,
                           reinterpret_cast<sockaddr*>(&from), &fromlen);
        if (ret > 0) {
            std::lock_guard<std::mutex> lock(mtx);
            latest_packet.assign(buf.begin(), buf.begin() + ret);
        }
        // else timeout, just loop
    }
}

bool UDPReceiver::getLatestPacket(std::vector<uint8_t>& buf) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!latest_packet.empty()) {
        buf = latest_packet;
        latest_packet.clear(); // consume it
        return true;
    }
    return false;
}
