#include <fcntl.h>
#include <arpa/inet.h>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>
#include "common.h"

int setpoint = 10;

Platoon self; // give truck identity
Platoon platoon[8]; // make a list of trucks in the platoon

int main() {

    char l_or_f;

    std::cout << "Leader or Follower (temporary): "; std::cin >> l_or_f;
    if (l_or_f == 'l') self.truck_role = truck_roles::LEADER;
    else if (l_or_f == 'f') self.truck_role = truck_roles::FOLLOWER;
    else {
        perror("invalid input\n");
        return 1;
    }

    int slot;
    std::cout << "Truck ID (10 to 17): "; std::cin >> slot;
    if (slot < 10 || slot >= 18){
        perror("invalid ID");
        return 1;
    }
    slot -= 10;

    // allocate shared memory
    const char* name = "/main_frame_memory";

    int file_descriptor = shm_open(name, O_RDWR, 0666);
    if (file_descriptor == -1) {
        perror("shm_open");
        return 1;
    }

    auto* data_from_main = (SharedMemoryLayout*) mmap(
        nullptr,
        sizeof(SharedMemoryLayout),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        file_descriptor,
        0
    );
    if (data_from_main == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    // initialize UDP
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket init error\n");
        return 1;
    }

    int enable_broadcast = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable_broadcast, sizeof(enable_broadcast)) < 0) {
        perror("setsockopt(SO_BROADCAST)");
        return 1;
    } 

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5000);
    if (self.truck_role == truck_roles::LEADER) {
        addr.sin_addr.s_addr = INADDR_BROADCAST; // 255.255.255.255
    } else inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    txPlatoonMessageFrame tx_l_to_f{};
    tx_l_to_f.distance_setpoint = htons(setpoint);
    tx_l_to_f.emergency_brake_leader = htons(0);

    // populate truck identity
    self.truck_id = slot + 10;
    self.udp_port = addr.sin_port;

    while(true) {

        unsigned short truck_position = 10 * slot + 100; // example position
        std::cout << "Travelled " << truck_position << " m" << std::endl;

        // lock mutex and write to memory
        pthread_mutex_lock(&data_from_main->global_mutex);
        data_from_main->rx_slots[slot].position = truck_position;
        data_from_main->rx_slots[slot].request_ready = true;
        data_from_main->tx_slots[slot].response_ready = false;
        pthread_mutex_unlock(&data_from_main->global_mutex);

        while(true) {
            
            pthread_mutex_lock(&data_from_main->global_mutex);
            if (data_from_main->tx_slots[slot].response_ready == true) {
                std::cout << "Distance to front "
                        << data_from_main->tx_slots[slot].sensor_data
                        << " m" << std::endl;

                pthread_mutex_unlock(&data_from_main->global_mutex);
                break;
            }
            pthread_mutex_unlock(&data_from_main->global_mutex);
            sleep(1);
        }

        // send setpoint
        ssize_t sent = sendto(
        sock,
        &tx_l_to_f,
        sizeof(tx_l_to_f),
        0,
        reinterpret_cast<sockaddr*>(&addr),
        sizeof(addr)
    );

    if (sent != sizeof(tx_l_to_f)) perror("sendto");
        sleep(1);
    }

    close(sock);

}
