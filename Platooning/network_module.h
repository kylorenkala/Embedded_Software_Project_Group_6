#ifndef NETWORK_MODULE_H
#define NETWORK_MODULE_H

#include <iostream>
#include <cstring>
#include "common.h"

// --- PLATFORM SPECIFIC HEADERS ---
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    #define close_socket closesocket
    struct WinsockInitializer {
        WinsockInitializer() {
            WSADATA wsaData;
            WSAStartup(MAKEWORD(2, 2), &wsaData);
        }
        ~WinsockInitializer() { WSACleanup(); }
    };
#else
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <sys/socket.h>
    #include <fcntl.h>
    #define close_socket close
    #define SOCKET int
#endif

class NetworkModule {
private:
    int sockfd;
    sockaddr_in myAddr;
    sockaddr_in broadcastAddr;

    #ifdef _WIN32
    static WinsockInitializer init;
    #endif

public:
    NetworkModule(int id) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        // 1. Enable Broadcast
        int broadcast = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (const char*)&broadcast, sizeof(broadcast));

        // 2. Enable Re-use Address (Prevents "Address already in use" errors)
        int reuse = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

        // 3. Bind
        memset(&myAddr, 0, sizeof(myAddr));
        myAddr.sin_family = AF_INET;
        myAddr.sin_port = htons(PORT_BASE + id);
        myAddr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sockfd, (sockaddr*)&myAddr, sizeof(myAddr)) < 0) {
            std::cerr << "Binding failed! Is ID " << id << " already running?\n";
        }

        // 4. Setup Broadcast Target
        memset(&broadcastAddr, 0, sizeof(broadcastAddr));
        broadcastAddr.sin_family = AF_INET;
        broadcastAddr.sin_port = htons(PORT_BASE);
        broadcastAddr.sin_addr.s_addr = inet_addr("255.255.255.255");

        // 5. SET NON-BLOCKING MODE IMMEDIATELY (Crucial Fix)
        #ifdef _WIN32
            u_long mode = 1;
            ioctlsocket(sockfd, FIONBIO, &mode);
        #else
            int flags = fcntl(sockfd, F_GETFL, 0);
            fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
        #endif
    }

    // --- NEW: Safe Flush that won't hang ---
    void flush() {
        char dummy[1024];
        sockaddr_in from;
        socklen_t len = sizeof(from);
        int dropped = 0;

        while (true) {
            // Because we set Non-Blocking in constructor, this will
            // return -1 immediately if buffer is empty.
            int bytes = recvfrom(sockfd, dummy, sizeof(dummy), 0, (sockaddr*)&from, &len);
            if (bytes <= 0) break; // Buffer empty, exit loop
            dropped++;
        }
        if (dropped > 0) std::cout << " [NET] Flushed " << dropped << " old packets.\n";
    }

    bool receive(PlatoonMessage& msg) {
        sockaddr_in from;
        socklen_t len = sizeof(from);

        // No need to set ioctlsocket here anymore, it's done in constructor
        int bytes = recvfrom(sockfd, (char*)&msg, sizeof(msg), 0, (sockaddr*)&from, &len);

        return (bytes > 0);
    }

    void broadcast(const PlatoonMessage& msg) {
        for(int i=0; i<5; i++) {
            broadcastAddr.sin_port = htons(PORT_BASE + i);
            if (i != msg.truckId) {
                sendto(sockfd, (const char*)&msg, sizeof(msg), 0, (sockaddr*)&broadcastAddr, sizeof(broadcastAddr));
            }
        }
        // Visualizer
        broadcastAddr.sin_port = htons(VISUALIZER_PORT);
        sendto(sockfd, (const char*)&msg, sizeof(msg), 0, (sockaddr*)&broadcastAddr, sizeof(broadcastAddr));
    }

    ~NetworkModule() { close_socket(sockfd); }
};

#ifdef _WIN32
WinsockInitializer NetworkModule::init;
#endif

#endif l