#include "udp_sender.h"
#include <stdexcept>
#include <cstring>
#include <iostream>

UDPSender::UDPSender(const std::string& ip, uint16_t port) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
        throw std::runtime_error("WSAStartup failed");
    }

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        WSACleanup();
        throw std::runtime_error("socket creation failed");
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip.c_str());
}

UDPSender::~UDPSender() {
    closesocket(sock);
    WSACleanup();
}

void UDPSender::send(const std::vector<uint8_t>& data) {
    int ret = ::sendto(sock, reinterpret_cast<const char*>(data.data()),
                       static_cast<int>(data.size()), 0,
                       reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (ret == SOCKET_ERROR) {
        std::cerr << "UDP send failed\n";
    }
}
