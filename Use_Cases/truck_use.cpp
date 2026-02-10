#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>
#include "common.h"

// ========== Follower Truck ==========

void runFollower(int slot, SharedMemoryLayout* data_from_main) {
    unsigned short speed = 0;
    unsigned short position = slot * 100 + 100;  // Starting positions
    unsigned short desired_distance = 20;
    bool emergency_mode = false;
    uint64_t last_tick = 0;
    int missed_ticks = 0;

    std::cout << "[Follower " << slot << "] Starting at position " << position << "\n";

    // Register as active follower
    pthread_mutex_lock(&data_from_main->global_mutex);
    data_from_main->follower_status[slot].is_active = true;
    pthread_mutex_unlock(&data_from_main->global_mutex);

    while (true) {
        pthread_mutex_lock(&data_from_main->global_mutex);
        
        // Check if system is still running
        if (!data_from_main->system_running) {
            pthread_mutex_unlock(&data_from_main->global_mutex);
            std::cout << "[Follower " << slot << "] System shutdown\n";
            break;
        }
        
        // Check heartbeat (tick)
        if (data_from_main->tick > last_tick) {
            last_tick = data_from_main->tick;
            missed_ticks = 0;
        } else {
            missed_ticks++;
            if (missed_ticks >= 5) {
                std::cerr << "[Follower " << slot << "] Lost heartbeat! Emergency stop\n";
                emergency_mode = true;
            }
        }
        
        // Send position to main_frame
        data_from_main->rx_slots[slot].position = position;
        data_from_main->rx_slots[slot].speed = speed;
        data_from_main->rx_slots[slot].emergency_brake = emergency_mode;
        data_from_main->rx_slots[slot].request_ready = true;
        data_from_main->tx_slots[slot].response_ready = false;
        
        pthread_mutex_unlock(&data_from_main->global_mutex);
        
        // Wait for response from main_frame
        while (true) {
            pthread_mutex_lock(&data_from_main->global_mutex);
            
            if (data_from_main->tx_slots[slot].response_ready == true) {
                unsigned short distance = data_from_main->tx_slots[slot].sensor_data;
                bool obstacle = data_from_main->tx_slots[slot].obstacle_detected;
                
                // Read leader commands
                desired_distance = data_from_main->leader_cmd.distance_setpoint;
                
                // Check for leader emergency
                if (data_from_main->leader_cmd.emergency_brake_all && !emergency_mode) {
                    std::cout << "[Follower " << slot << "] Leader emergency signal!\n";
                    emergency_mode = true;
                }
                
                // Check for obstacle
                if (obstacle && !emergency_mode) {
                    std::cout << "[Follower " << slot << "] OBSTACLE detected!\n";
                    emergency_mode = true;
                    data_from_main->follower_status[slot].emergency_active = true;
                }
                
                // Update follower status
                data_from_main->follower_status[slot].actual_distance = distance;
                
                pthread_mutex_unlock(&data_from_main->global_mutex);
                
                // Safety check: collision risk?
                if (!emergency_mode && isCollisionRisk(distance, speed)) {
                    std::cout << "[Follower " << slot << "] Collision risk!\n";
                    emergency_mode = true;
                    
                    pthread_mutex_lock(&data_from_main->global_mutex);
                    data_from_main->follower_status[slot].emergency_active = true;
                    pthread_mutex_unlock(&data_from_main->global_mutex);
                }
                
                // Control logic
                if (emergency_mode) {
                    // Emergency brake
                    if (speed > EMERGENCY_DECEL) {
                        speed -= EMERGENCY_DECEL;
                    } else {
                        speed = 0;
                        emergency_mode = false;  // Stopped safely
                        
                        pthread_mutex_lock(&data_from_main->global_mutex);
                        data_from_main->follower_status[slot].emergency_active = false;
                        pthread_mutex_unlock(&data_from_main->global_mutex);
                    }
                } else {
                    // Normal distance control
                    int error = distance - desired_distance;
                    int accel = error / 5;  // Proportional control
                    
                    if (accel > 2) accel = 2;
                    if (accel < -3) accel = -3;
                    
                    speed += accel;
                    if (speed > 50) speed = 50;  // Max speed
                }
                
                // Update position based on speed
                position += speed;
                
                // Display status
                std::string status = emergency_mode ? "EMERGENCY" : "NORMAL";
                std::string safe = isSafeDistance(distance) ? "SAFE" : "UNSAFE";
                
                std::cout << "[Follower " << slot << "] "
                          << "pos=" << position
                          << " speed=" << speed
                          << " dist=" << distance 
                          << " [" << status << "/" << safe << "]\n";
                
                break;
            }
            
            pthread_mutex_unlock(&data_from_main->global_mutex);
            usleep(100000);  // 100ms
        }
        
        sleep(1);
    }
    
    // Cleanup
    pthread_mutex_lock(&data_from_main->global_mutex);
    data_from_main->follower_status[slot].is_active = false;
    pthread_mutex_unlock(&data_from_main->global_mutex);
}

// ========== Leader Truck ==========

void runLeader(int slot, SharedMemoryLayout* data_from_main) {
    unsigned short position = slot * 100 + 100;
    unsigned short desired_distance = 20;
    bool emergency_brake = false;

    std::cout << "[Leader " << slot << "] Starting\n";
    std::cout << "Commands: + (increase distance), - (decrease), e (emergency), r (reset)\n";

    while (true) {
        pthread_mutex_lock(&data_from_main->global_mutex);
        
        if (!data_from_main->system_running) {
            pthread_mutex_unlock(&data_from_main->global_mutex);
            std::cout << "[Leader " << slot << "] System shutdown\n";
            break;
        }
        
        // Update position
        data_from_main->rx_slots[slot].position = position;
        data_from_main->rx_slots[slot].request_ready = true;
        data_from_main->tx_slots[slot].response_ready = false;
        
        pthread_mutex_unlock(&data_from_main->global_mutex);
        
        // User input
        if (std::cin.rdbuf()->in_avail()) {
            char c;
            std::cin >> c;
            
            if (c == '+') {
                desired_distance += 5;
                std::cout << "[Leader] Distance set to " << desired_distance << "m\n";
            }
            else if (c == '-') {
                if (desired_distance > 10) desired_distance -= 5;
                std::cout << "[Leader] Distance set to " << desired_distance << "m\n";
            }
            else if (c == 'e') {
                emergency_brake = true;
                std::cout << "[Leader] EMERGENCY BRAKE ACTIVATED\n";
            }
            else if (c == 'r') {
                emergency_brake = false;
                std::cout << "[Leader] Emergency reset\n";
            }
        }
        
        // Update leader commands
        pthread_mutex_lock(&data_from_main->global_mutex);
        data_from_main->leader_cmd.distance_setpoint = desired_distance;
        data_from_main->leader_cmd.emergency_brake_all = emergency_brake;
        
        // Check for follower emergencies
        for (int i = 0; i < MAX_TRUCKS; i++) {
            if (data_from_main->follower_status[i].is_active && 
                data_from_main->follower_status[i].emergency_active &&
                !emergency_brake) {
                std::cout << "[Leader] Truck " << i << " triggered emergency!\n";
                emergency_brake = true;
                data_from_main->leader_cmd.emergency_brake_all = true;
            }
        }
        pthread_mutex_unlock(&data_from_main->global_mutex);
        
        // Display platoon status
        std::cout << "\n=== PLATOON STATUS ===\n";
        std::cout << "Leader at slot " << slot 
                  << " | Desired distance: " << desired_distance << "m"
                  << " | Emergency: " << (emergency_brake ? "YES" : "NO") << "\n";
        
        pthread_mutex_lock(&data_from_main->global_mutex);
        for (int i = 0; i < MAX_TRUCKS; i++) {
            if (data_from_main->follower_status[i].is_active) {
                std::cout << "  Follower " << i 
                          << ": distance=" << data_from_main->follower_status[i].actual_distance << "m"
                          << " emergency=" << (data_from_main->follower_status[i].emergency_active ? "YES" : "NO")
                          << "\n";
            }
        }
        pthread_mutex_unlock(&data_from_main->global_mutex);
        
        position += 10;  // Leader moves forward
        sleep(1);
    }
}

// ========== Main ==========

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <slot 0-7> <role l/f>\n";
        std::cerr << "Example: " << argv[0] << " 0 l  (leader at slot 0)\n";
        std::cerr << "Example: " << argv[0] << " 1 f  (follower at slot 1)\n";
        return 1;
    }

    int slot = std::atoi(argv[1]);
    char role = argv[2][0];
    
    if (slot < 0 || slot >= MAX_TRUCKS) {
        std::cerr << "Slot must be 0-" << (MAX_TRUCKS-1) << "\n";
        return 1;
    }

    const char* name = "/main_frame_memory";

    int file_descriptor = shm_open(name, O_RDWR, 0666);
    if (file_descriptor == -1) {
        std::cerr << "Cannot open shared memory. Is main_frame running?\n";
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
        std::cerr << "Failed to map shared memory\n";
        return 1;
    }

    if (role == 'l') {
        runLeader(slot, data_from_main);
    } else {
        runFollower(slot, data_from_main);
    }

    // Cleanup
    munmap(data_from_main, sizeof(SharedMemoryLayout));
    close(file_descriptor);
    
    return 0;
}
