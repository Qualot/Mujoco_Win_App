#pragma once
#include <string>
#include <vector>
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")

class UDPSender {
public:
    UDPSender(const std::string& ip, uint16_t port);
    ~UDPSender();

    void send(const std::vector<uint8_t>& data);

private:
    SOCKET sock = INVALID_SOCKET;
    sockaddr_in addr{};
};
