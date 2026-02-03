#include <fcntl.h>
#include <arpa/inet.h>
#include <cstring>
#include <sys/mman.h>
#include <sys/time.h>
#include <errno.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>
#include "common.h"

void runLeader() {
    if (!data_from_main) {
        std::cerr << "[leader] shared memory not available\n";
        return;
    }

    const int BROADCAST_PORT = 5000;

    distance_travelled = 100; // leader gets a head start
    
    leaderPlatoonMessageFrame tx{}; // to be sent to followers
    followerPlatoonMessageFrame replies[8]; // to be populated by replies from followers
    bool seen[8];
    
    std::memset(seen, 0, sizeof(seen));

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("[leader] socket");
        return;
    }

    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) < 0) {
        perror("[leader] setsockopt SO_BROADCAST");
        close(sock);
        return;
    }

    // Broadcast destination (port 5000) (uses same syntax to define as in follower)
    sockaddr_in bcast{};
    bcast.sin_family = AF_INET;
    bcast.sin_port = htons(BROADCAST_PORT);
    if (inet_pton(AF_INET, "255.255.255.255", &bcast.sin_addr) != 1) {
        std::cerr << "[leader] invalid broadcast address\n";
        close(sock);
        return;
    }

    // Bind so replies arrive on the well-known port
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = INADDR_ANY;
    local.sin_port = htons(BROADCAST_PORT);

    if (bind(sock, (sockaddr*)&local, sizeof(local)) < 0) {
        perror("[leader] bind");
        close(sock);
        return;
    }

    // Short receive window after each broadcast
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 300 * 1000; // 300 ms
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        // non-fatal, but without this recvfrom may block
        perror("[leader] setsockopt SO_RCVTIMEO");
    }

    while (true) {

        tx.type = htons((uint16_t)msg_type::LEADER_TO_FOLLOWER);
        tx.emergency_brake_leader = htons(0);
        tx.distance_setpoint = htons(setpoint);

        // Publish leader position to shared memory
        pthread_mutex_lock(&data_from_main->global_mutex);
        data_from_main->rx_slots[0].position = distance_travelled;
        data_from_main->rx_slots[0].position_ready = true;
        pthread_mutex_unlock(&data_from_main->global_mutex);

        // Broadcast to followers
        ssize_t sent = sendto(sock, &tx, sizeof(tx), 0, (sockaddr*)&bcast, sizeof(bcast));
        if (sent < 0) {
            perror("[leader] sendto");
            // continue to collect replies anyway
        }

        // Clear seen flags for this round
        std::memset(seen, 0, sizeof(seen));

        followerPlatoonMessageFrame rx{};
        // Collect replies until timeout
        while (true) {
            sockaddr_in from{};
            socklen_t from_len = sizeof(from);

            ssize_t n = recvfrom(sock, &rx, sizeof(rx), 0, (sockaddr*)&from, &from_len);
            
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // no more packets in this receive window
                    break;
                }
                if (errno == EINTR) {
                    continue; // retry
                }
                perror("[leader] recvfrom");
                break;
            }

            if (static_cast<size_t>(n) != sizeof(rx)) {
                std::cerr << "[leader] unexpected reply size " << n << ", dropping\n";
                continue;
            }

            if (ntohs(rx.type) != (uint16_t)msg_type::FOLLOWER_TO_LEADER) continue; // block out unnecessary messages

            // Convert fields
            int truck_id = ntohs(rx.truck_id);
            int pos = ntohs(rx.position_in_platoon);
            unsigned short distance_actual = ntohs(rx.distance_actual);

            if (pos >= 8) {
                std::cerr << "[leader] reply with invalid position " << pos << " from truck " << truck_id << "\n";
                continue;
            }

            // Store reply indexed by declared position
            replies[pos] = rx;
            seen[pos] = true;

            // Optionally update in-memory platoon array with parsed, host-order values
            platoon[pos].truck_id = truck_id;
            platoon[pos].distance_to_front = distance_actual;

            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
            
            std::cout << "[leader] follower at pos "
                      << pos
                      << " with id "
                      << truck_id
                      << " reports distance = "
                      << distance_actual << "m"
                      << std::endl;
        }

        // Do any processing on replies[] / seen[] here (not implemented)

        // Advance position and pace broadcasts
        distance_travelled += truck_speed;
        sleep(1);
    }

    close(sock);
}