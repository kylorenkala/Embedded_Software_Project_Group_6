#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>
#include "common.h"

int main() {
    const char* name = "/main_frame_memory";

    // Create shared memory
    int file_descriptor = shm_open(name, O_CREAT | O_EXCL | O_RDWR, 0666);
    if (file_descriptor == -1) {
        std::cerr << "Failed to create shared memory. Maybe already running?\n";
        return 1;
    }

    ftruncate(file_descriptor, sizeof(SharedMemoryLayout));

    auto* data_to_main = (SharedMemoryLayout*) mmap(
        nullptr,
        sizeof(SharedMemoryLayout),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        file_descriptor,
        0
    );

    // Initialize process-shared mutex
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&data_to_main->global_mutex, &attr);

    // Initialize slots
    for (int i = 0; i < MAX_TRUCKS; i++) {
        data_to_main->rx_slots[i].request_ready = false;
        data_to_main->tx_slots[i].response_ready = false;
        data_to_main->tx_slots[i].obstacle_detected = false;
        data_to_main->follower_status[i].is_active = false;
    }

    data_to_main->tick = 0;
    data_to_main->system_running = true;
    data_to_main->leader_cmd.distance_setpoint = 20;
    data_to_main->leader_cmd.emergency_brake_all = false;

    std::cout << "Main frame running\n";
    std::cout << "Commands:\n";
    std::cout << "  o <slot>  - Place obstacle at truck slot\n";
    std::cout << "  c         - Clear all obstacles\n";
    std::cout << "  q         - Quit\n\n";

    while (true) {
        // Handle user commands
        if (std::cin.rdbuf()->in_avail()) {
            char cmd;
            std::cin >> cmd;
            
            if (cmd == 'q') {
                pthread_mutex_lock(&data_to_main->global_mutex);
                data_to_main->system_running = false;
                pthread_mutex_unlock(&data_to_main->global_mutex);
                std::cout << "Shutting down...\n";
                break;
            }
            else if (cmd == 'o') {
                int slot;
                std::cin >> slot;
                if (slot >= 0 && slot < MAX_TRUCKS) {
                    pthread_mutex_lock(&data_to_main->global_mutex);
                    data_to_main->tx_slots[slot].obstacle_detected = true;
                    pthread_mutex_unlock(&data_to_main->global_mutex);
                    std::cout << "Obstacle placed at slot " << slot << "\n";
                }
            }
            else if (cmd == 'c') {
                pthread_mutex_lock(&data_to_main->global_mutex);
                for (int i = 0; i < MAX_TRUCKS; i++) {
                    data_to_main->tx_slots[i].obstacle_detected = false;
                }
                pthread_mutex_unlock(&data_to_main->global_mutex);
                std::cout << "All obstacles cleared\n";
            }
        }

        pthread_mutex_lock(&data_to_main->global_mutex);
        
        for (int i = 0; i < MAX_TRUCKS; i++) {
            if (data_to_main->rx_slots[i].request_ready == true) {
                
                unsigned short truck_position = data_to_main->rx_slots[i].position;
                
                // Calculate distance to front truck (fix the bug from original code)
                unsigned short distance_result;
                if (i == 0) {
                    // First truck (leader) has clear road ahead
                    distance_result = 100;
                } else {
                    // Calculate distance to truck in front
                    unsigned short front_position = data_to_main->rx_slots[i - 1].position;
                    if (front_position > truck_position) {
                        distance_result = front_position - truck_position;
                    } else {
                        distance_result = 0;  // Safety: shouldn't happen
                    }
                }
                
                std::cout << "Truck " << i << " at position " << truck_position 
                          << ", distance to front: " << distance_result << " m\n";
                
                // Send sensor data back to truck
                data_to_main->tx_slots[i].sensor_data = distance_result;
                data_to_main->tx_slots[i].response_ready = true;
                data_to_main->rx_slots[i].request_ready = false;
                
                // Safety check
                if (distance_result < MIN_SAFE_DISTANCE && i > 0) {
                    std::cerr << "WARNING: Truck " << i << " too close! Distance: " 
                              << distance_result << "m\n";
                }
            }
        }
        
        // Increment tick (heartbeat)
        data_to_main->tick++;
        
        pthread_mutex_unlock(&data_to_main->global_mutex);
        
        sleep(1);
    }

    // Cleanup
    pthread_mutex_destroy(&data_to_main->global_mutex);
    munmap(data_to_main, sizeof(SharedMemoryLayout));
    shm_unlink(name);
    close(file_descriptor);
    
    return 0;
}
