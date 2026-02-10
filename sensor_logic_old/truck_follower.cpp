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

void runFollower() {

    leaderPlatoonMessageFrame rx{};
    followerPlatoonMessageFrame tx{};

    if (!data_from_main) {
        std::cerr << "[follower] shared memory not available\n";
        return;
    }

    const int LEADER_PORT = 5000;
    const int LOCAL_PORT = 5000 + self.position_in_platoon;

    //-------------RECEIVER SOCKET-----------------------------------------------------------------
    int rx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rx_sock < 0) {
        perror("[follower] receiver socket");
        return;
    }
    int tx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (tx_sock < 0) {
        perror("[follower] sender socket");
        return;
    }

    int reuse = 1;
    setsockopt(rx_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
    setsockopt(rx_sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    // receiver socket - bind to leader's port to listen to broadcast
    sockaddr_in rx_addr{};
    rx_addr.sin_family = AF_INET;
    rx_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    rx_addr.sin_port = htons(LEADER_PORT);

    // sender socket - bind to own port to reply to leader
    sockaddr_in tx_addr{};
    tx_addr.sin_family = AF_INET;
    tx_addr.sin_addr.s_addr = INADDR_ANY;
    tx_addr.sin_port = htons(LOCAL_PORT);

    sockaddr_in leader_dst{};
    leader_dst.sin_family = AF_INET;
    leader_dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    leader_dst.sin_port = htons(LEADER_PORT);
    
    // bind receiver socket to leader's port
    if (bind(rx_sock, (sockaddr*)&rx_addr, sizeof(rx_addr)) < 0) {
        perror("[follower] receiver bind");
        close(rx_sock);
        return;
    }

    // bind sender's socket to local port
    if (bind(tx_sock, (sockaddr*)&tx_addr, sizeof(tx_addr)) < 0) {
        perror("[follower] sender bind");
        close(tx_sock);
        return;
    }

    tx.truck_id = htons(self.truck_id); // static field

    while (true) {
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);

        // receive data from leader
        ssize_t n = recvfrom(rx_sock, &rx, sizeof(rx), 0, (sockaddr*)&from, &from_len);
        
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("[follower] recvfrom");
            continue;
        }
        if (static_cast<size_t>(n) != sizeof(rx)) continue; // ingore unexpected packets

        if (ntohs(rx.type) != (uint16_t)msg_type::LEADER_TO_FOLLOWER) continue;

        int distance_setpoint = ntohs(rx.distance_setpoint);

        // Safely read sensor data from shared memory
        unsigned short sensor = 0;
        bool have_sensor = false;

        pthread_mutex_lock(&data_from_main->global_mutex);
        // validate index range before reading
        size_t tx_slots_count = sizeof(data_from_main->tx_slots) / sizeof(data_from_main->tx_slots[0]);
        if (self.position_in_platoon >= 0 && (size_t)self.position_in_platoon < tx_slots_count) {
            
            if (data_from_main->tx_slots[self.position_in_platoon].sensor_data_ready) {
                
                sensor = data_from_main->tx_slots[self.position_in_platoon].sensor_data;
                have_sensor = true;

                data_from_main->tx_slots[self.position_in_platoon].sensor_data_ready = false; // set for main frame to flip on write
            }
        } else std::cerr << "[follower] invalid position_in_platoon " << self.position_in_platoon << std::endl;
        
        pthread_mutex_unlock(&data_from_main->global_mutex);

        // only populate when there is sensor data available
        if (have_sensor) self.distance_to_front = sensor;

        // Simple control logic
        if (self.distance_to_front > distance_setpoint && truck_speed < 25) truck_speed++;
        else if (self.distance_to_front < distance_setpoint && truck_speed > 15) truck_speed--;

        std::cout << "[follower "
                  << self.position_in_platoon
                  << "] setpoint = "
                  << distance_setpoint
                  << "m | distance to front = "
                  << self.distance_to_front 
                  << "m | speed = "
                  << truck_speed
                  << "m/s"
                  << std::endl;

        // Update own position
        distance_travelled += truck_speed;

        // Publish own position for main process
        pthread_mutex_lock(&data_from_main->global_mutex);
        size_t rx_slots_count = sizeof(data_from_main->rx_slots) / sizeof(data_from_main->rx_slots[0]);
        if (self.position_in_platoon >= 0 && (size_t)self.position_in_platoon < rx_slots_count) {
            data_from_main->rx_slots[self.position_in_platoon].position = distance_travelled;
            data_from_main->rx_slots[self.position_in_platoon].position_ready = true;
        } else std::cerr << "[follower] invalid position when writing position\n";
        pthread_mutex_unlock(&data_from_main->global_mutex);

        // Prepare reply (network order)
        tx.type = htons((uint16_t)msg_type::FOLLOWER_TO_LEADER);
        tx.position_in_platoon = htons(self.position_in_platoon);
        tx.distance_actual = htons(self.distance_to_front);
        tx.emergency_brake_follower = htons(0); // emergency brake logic (todo)

        // send data back to leader
        ssize_t sent = sendto(tx_sock, &tx, sizeof(tx), 0, (sockaddr*)&leader_dst, sizeof(leader_dst));
        if (sent < 0) {
            perror("[follower] sendto");
        } else if (static_cast<size_t>(sent) != sizeof(tx)) {
            std::cerr << "[follower] partial send\n";
        }
        sleep(1);
    }
    close(rx_sock);
    close(tx_sock);
}