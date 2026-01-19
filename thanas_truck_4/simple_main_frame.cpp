// writer.cpp
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>
#include "common.h"

int main() {
    const char* name = "/main_frame_memory"; // make shared memory file

    int file_descriptor = shm_open(name, O_CREAT | O_EXCL |O_RDWR, 0666); // open file in read/write mode
    if (file_descriptor == -1) return 1; // error proof

    ftruncate(file_descriptor, sizeof(SharedMemoryLayout));

    auto* data_to_main = (SharedMemoryLayout*) mmap(
        nullptr,
        sizeof(SharedMemoryLayout),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        file_descriptor,
        0
    );

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);

    pthread_mutex_init(&data_to_main->global_mutex, &attr);

    // initialize slots
    for (int i = 0; i < 8; i++) {
        data_to_main->rx_slots[i].request_ready = false;
        data_to_main->tx_slots[i].response_ready = false;
    }

    std::cout << "Main frame running\n";

    // Write loop
    while (true) {
        pthread_mutex_lock(&data_to_main->global_mutex);
        
        for (int i = 0; i < 8; i++) {
            if (data_to_main->rx_slots[i].request_ready == true) {

                unsigned short truck_position = data_to_main->rx_slots[i].position;
                std::cout << "Truck " << i << " reports " << truck_position << std::endl;
                
                // calculate distance to front (previous) truck
                unsigned short distance_result;

                // avoiding i = -1
                if (i > 0) {
                    distance_result = truck_position - data_to_main->rx_slots[i - 1].position; // truck in front
                    data_to_main->tx_slots[i].sensor_data = distance_result;
                    std::cout << "Sent truck " << i << " " << distance_result << " m" << std::endl;
                    std::cout << "" << std::endl;
                } 
                
                data_to_main->tx_slots[i].response_ready = true;
                data_to_main->rx_slots[i].request_ready = false;

            }
        }

        pthread_mutex_unlock(&data_to_main->global_mutex);

        sleep(1);
    }
}
