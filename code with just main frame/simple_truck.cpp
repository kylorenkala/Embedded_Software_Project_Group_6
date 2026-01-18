// reader.cpp
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>
#include "common.h"


int main(int argc, char* argv[]) {

    if (argc != 2) {
        std::cerr << "Usage: worker <slot 0-7>\n";
        return 1;
    }

    int slot = std::atoi(argv[1]); // get truck ID on startup
    if (slot < 0 || slot >= 8) return 1;

    const char* name = "/main_frame_memory";

    int file_descriptor = shm_open(name, O_RDWR, 0666);
    if (file_descriptor == -1) return 1; //error proof

    auto* data_from_main = (SharedMemoryLayout*) mmap(
        nullptr,
        sizeof(SharedMemoryLayout),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        file_descriptor,
        0
    );

    while(true) {

        unsigned short truck_position = 10 * slot + 100; // example position

        pthread_mutex_lock(&data_from_main->global_mutex);
        data_from_main->rx_slots[slot].position = truck_position;
        data_from_main->rx_slots[slot].request_ready = true;
        data_from_main->tx_slots[slot].response_ready = false;
        pthread_mutex_unlock(&data_from_main->global_mutex);


        while(true) {
            
            pthread_mutex_lock(&data_from_main->global_mutex);
            if (data_from_main->tx_slots[slot].response_ready == true) {
                std::cout << "Distance to front " << data_from_main->tx_slots[slot].sensor_data << " m" << std::endl;

                pthread_mutex_unlock(&data_from_main->global_mutex);
                break;
            }
            pthread_mutex_unlock(&data_from_main->global_mutex);
            sleep(1);
        }
        sleep(1);
    }

}
