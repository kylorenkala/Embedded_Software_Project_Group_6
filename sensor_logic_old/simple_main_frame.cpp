#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>
#include "common.h"

int main() {

    const char* name = "/main_frame_memory"; // create shared memory file

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

    std::cout << "Shared Memory initialized.\n";

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);

    pthread_mutex_init(&data_to_main->global_mutex, &attr);

    // initialize slots
    for (int i = 0; i < 8; i++) {
        data_to_main->rx_slots[i].position_ready = false;
        data_to_main->tx_slots[i].sensor_data_ready = false;
    }

    std::cout << "Main frame running...\n";

// cache last known positions to avoid ordering dependency
unsigned short last_position[8] = {};
bool has_position[8] = {};

    while (true) {

        pthread_mutex_lock(&data_to_main->global_mutex);

        for (int i = 0; i < 8; i++) {

            // read only when there is data to read
            if (data_to_main->rx_slots[i].position_ready == true) {
                last_position[i] = data_to_main->rx_slots[i].position;
                has_position[i] = true;
                data_to_main->rx_slots[i].position_ready = false; // reset for truck to raise
            }

            // skip if we still have no position for this truck
            if (has_position[i] == false) continue;

            // no truck in front of leader
            if (i == 0) {
                data_to_main->tx_slots[i].sensor_data = 0;
                data_to_main->tx_slots[i].sensor_data_ready = true; // set for truck to read
                continue;
            }

            // read only when there is data to read from front truck
            if (has_position[i - 1] == false) continue;

            unsigned short front = last_position[i - 1];   // truck in front
            unsigned short current = last_position[i];     // current truck

            // position of truck in front minus position of currently loaded truck
            if (front >= current) {
                data_to_main->tx_slots[i].sensor_data = front - current;
            }
            else {
                data_to_main->tx_slots[i].sensor_data = 0; // avoiding wrap around
            }

            data_to_main->tx_slots[i].sensor_data_ready = true; // ready for truck to read
        }

        pthread_mutex_unlock(&data_to_main->global_mutex);
        //sleep(1);
    }
}
