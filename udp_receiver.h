#pragma once
#include <string>
#include <vector>
#include <winsock2.h>
#include <thread>
#include <mutex>
#include <atomic>
#pragma comment(lib, "ws2_32.lib")

class UDPReceiver {
public:
    UDPReceiver(uint16_t port);
    ~UDPReceiver();

    // receive one packet (nonblocking, may return 0 if no data, -1 on error)
    int receive(std::vector<uint8_t>& buf);

    // Start background receive thread
    void startReceiveThread();
    void stopReceiveThread();

    // Get latest received packet (thread-safe)
    // Returns true if a packet was available
    bool getLatestPacket(std::vector<uint8_t>& buf);

private:
    void receiveLoop();  // Thread function

    SOCKET sock = INVALID_SOCKET;
    sockaddr_in addr{};

    std::thread recv_thread;
    std::mutex mtx;
    std::vector<uint8_t> latest_packet;
    std::atomic<bool> running{false};
};
