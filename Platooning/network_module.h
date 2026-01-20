#ifndef NETWORK_MODULE_H
#define NETWORK_MODULE_H

#include <iostream>
#include <cstring>
#include "common.h"

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
        int broadcast = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (const char*)&broadcast, sizeof(broadcast));
        int reuse = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

        memset(&myAddr, 0, sizeof(myAddr));
        myAddr.sin_family = AF_INET;
        myAddr.sin_port = htons(PORT_BASE + id);
        myAddr.sin_addr.s_addr = INADDR_ANY;
        bind(sockfd, (sockaddr*)&myAddr, sizeof(myAddr));

        memset(&broadcastAddr, 0, sizeof(broadcastAddr));
        broadcastAddr.sin_family = AF_INET;
        broadcastAddr.sin_port = htons(PORT_BASE);
        broadcastAddr.sin_addr.s_addr = inet_addr("255.255.255.255");

        #ifdef _WIN32
            u_long mode = 1;
            ioctlsocket(sockfd, FIONBIO, &mode);
        #else
            int flags = fcntl(sockfd, F_GETFL, 0);
            fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
        #endif
    }

    void flush() {
        char dummy[1024];
        sockaddr_in from;
        socklen_t len = sizeof(from);
        while (recvfrom(sockfd, dummy, sizeof(dummy), 0, (sockaddr*)&from, &len) > 0);
    }

    bool receive(PlatoonMessage& msg) {
        sockaddr_in from;
        socklen_t len = sizeof(from);
        return (recvfrom(sockfd, (char*)&msg, sizeof(msg), 0, (sockaddr*)&from, &len) > 0);
    }

    void broadcast(const PlatoonMessage& msg) {
        for(int i=0; i<5; i++) {
            broadcastAddr.sin_port = htons(PORT_BASE + i);
            if (i != msg.truckId) {
                sendto(sockfd, (const char*)&msg, sizeof(msg), 0, (sockaddr*)&broadcastAddr, sizeof(broadcastAddr));
            }
        }
        broadcastAddr.sin_port = htons(VISUALIZER_PORT);
        sendto(sockfd, (const char*)&msg, sizeof(msg), 0, (sockaddr*)&broadcastAddr, sizeof(broadcastAddr));
    }

    ~NetworkModule() { close_socket(sockfd); }
};

#ifdef _WIN32
inline WinsockInitializer NetworkModule::init;
#endif
#endif