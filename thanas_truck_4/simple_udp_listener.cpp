// listener.cpp
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <cstdint>
#include <cstring>
#include "common.h"   // must contain txPlatoonMessageFrame

int main() {
    // create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    int reuse = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
                &reuse, sizeof(reuse)) < 0) {
        perror("setsockopt(SO_REUSEADDR)");
        return 1;
    }

    // bind to port 5000
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = INADDR_ANY;
    local.sin_port = htons(5000);

    if (bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        perror("bind");
        close(sock);
        return 1;
    }

    std::cout << "Listening on UDP port 5000...\n";

    while (true) {
        txPlatoonMessageFrame msg{};
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);

        ssize_t received = recvfrom(
            sock,
            &msg,
            sizeof(msg),
            0,
            reinterpret_cast<sockaddr*>(&from),
            &from_len
        );

        if (received != sizeof(msg)) {
            std::cerr << "Invalid packet size: " << received << "\n";
            continue;
        }

        // convert from network byte order
        uint16_t distance = ntohs(msg.distance_setpoint);
        uint16_t ebrake   = ntohs(msg.emergency_brake_leader);

        char sender_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &from.sin_addr, sender_ip, sizeof(sender_ip));

        std::cout
            << "Received from "
            << sender_ip << ":" << ntohs(from.sin_port)
            << " | distance_setpoint=" << distance
            << " | emergency_brake=" << ebrake
            << std::endl;
    }

    close(sock);
    return 0;
}
