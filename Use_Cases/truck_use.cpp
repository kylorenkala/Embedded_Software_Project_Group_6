#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include "common.h"

// ========== Follower ==========

void runFollower(int id, SharedMemory* shm) {
    double speed = 0.0;
    double desiredDistance = 20.0;
    double actualDistance = 20.0;
    bool emergencyMode = false;
    int localMissedHeartbeats = 0;
    uint64_t lastTick = 0;

    std::cout << "[Follower " << id << "] Starting...\n";
    
    // Register with main_frame via shared memory
    pthread_mutex_lock(&shm->mutex);
    shm->followerStatus[id].isActive = true;
    shm->followerStatus[id].truckId = id;
    pthread_mutex_unlock(&shm->mutex);

    while (true) {
        pthread_mutex_lock(&shm->mutex);
        
        // Check if system is still running
        if (!shm->systemRunning) {
            pthread_mutex_unlock(&shm->mutex);
            std::cout << "[Follower " << id << "] System shutdown\n";
            break;
        }
        
        // Check heartbeat (tick update)
        if (shm->mainData[id].tick > lastTick) {
            lastTick = shm->mainData[id].tick;
            localMissedHeartbeats = 0;
        } else {
            localMissedHeartbeats++;
            if (localMissedHeartbeats >= HEARTBEAT_TIMEOUT) {
                std::cerr << "[Follower " << id << "] Lost communication! Emergency stop\n";
                emergencyMode = true;
            }
        }
        
        // Read sensor data
        actualDistance = shm->mainData[id].distanceToFront;
        
        // Check for obstacle
        if (shm->mainData[id].obstacleDetected && !emergencyMode) {
            std::cout << "[Follower " << id << "] OBSTACLE! Emergency brake\n";
            emergencyMode = true;
            shm->truckData[id].emergencyBrake = true;
            shm->followerStatus[id].emergencyActive = true;
        }
        
        // Read leader commands
        desiredDistance = shm->leaderCmd.desiredDistance;
        
        if (shm->leaderCmd.emergencyBrakeAll && !emergencyMode) {
            std::cout << "[Follower " << id << "] Leader emergency signal\n";
            emergencyMode = true;
        }
        
        // Update follower status
        shm->followerStatus[id].reportedDistance = actualDistance;
        shm->truckData[id].currentSpeed = speed;
        shm->truckData[id].missedHeartbeats = localMissedHeartbeats;
        
        pthread_mutex_unlock(&shm->mutex);
        
        // Safety check: collision risk?
        if (!emergencyMode && isCollisionRisk(actualDistance, speed)) {
            std::cout << "[Follower " << id << "] Collision risk! Braking\n";
            emergencyMode = true;
            
            pthread_mutex_lock(&shm->mutex);
            shm->followerStatus[id].emergencyActive = true;
            pthread_mutex_unlock(&shm->mutex);
        }
        
        // Control logic
        if (emergencyMode) {
            // Emergency brake
            speed -= EMERGENCY_DECEL * 0.5;
            if (speed <= 0.1) {
                speed = 0.0;
                emergencyMode = false; // Can resume
                
                pthread_mutex_lock(&shm->mutex);
                shm->truckData[id].emergencyBrake = false;
                shm->followerStatus[id].emergencyActive = false;
                pthread_mutex_unlock(&shm->mutex);
            }
        } else {
            // Normal distance control
            double error = actualDistance - desiredDistance;
            double accel = 0.15 * error;

            if (accel >  2.0) accel =  2.0;
            if (accel < -3.0) accel = -3.0;

            speed += accel;
            if (speed < 0.0) speed = 0.0;
            if (speed > 25.0) speed = 25.0;
        }
        
        // Display status
        std::string status = emergencyMode ? "EMERGENCY" : "NORMAL";
        std::string safe = isSafeDistance(actualDistance) ? "SAFE" : "UNSAFE";
        
        std::cout << "[Follower " << id << "] "
                  << "speed=" << std::fixed << std::setprecision(1) << speed
                  << " dist=" << actualDistance 
                  << " [" << status << "/" << safe << "]\n";
        
        sleep(1);
    }
    
    // Unregister
    pthread_mutex_lock(&shm->mutex);
    shm->followerStatus[id].isActive = false;
    pthread_mutex_unlock(&shm->mutex);
}

// ========== Leader ==========

void runLeader(int id, SharedMemory* shm) {
    double desiredDistance = 20.0;
    bool emergencyBrake = false;

    std::cout << "[Leader " << id << "] Starting...\n";
    std::cout << "Commands: +/- distance, e=emergency, r=reset, q=quit\n";

    while (true) {
        pthread_mutex_lock(&shm->mutex);
        
        // Check if system is still running
        if (!shm->systemRunning) {
            pthread_mutex_unlock(&shm->mutex);
            std::cout << "[Leader " << id << "] System shutdown\n";
            break;
        }
        
        pthread_mutex_unlock(&shm->mutex);
        
        // User input
        if (std::cin.rdbuf()->in_avail()) {
            char c;
            std::cin >> c;
            
            if (c == '+') {
                desiredDistance += 2.0;
            }
            else if (c == '-') {
                desiredDistance -= 2.0;
                if (desiredDistance < 10.0) desiredDistance = 10.0;
            }
            else if (c == 'e') {
                emergencyBrake = true;
                std::cout << "[Leader] EMERGENCY BRAKE ACTIVATED\n";
            }
            else if (c == 'r') {
                emergencyBrake = false;
                std::cout << "[Leader] Emergency reset\n";
            }
            else if (c == 'q') {
                std::cout << "[Leader] Shutting down\n";
                pthread_mutex_lock(&shm->mutex);
                shm->systemRunning = false;
                pthread_mutex_unlock(&shm->mutex);
                break;
            }
        }
        
        // Write commands to shared memory
        pthread_mutex_lock(&shm->mutex);
        shm->leaderCmd.desiredDistance = desiredDistance;
        shm->leaderCmd.emergencyBrakeAll = emergencyBrake;
        
        // Check for follower emergencies
        for (int i = 0; i < MAX_TRUCKS; i++) {
            if (shm->followerStatus[i].isActive && 
                shm->followerStatus[i].emergencyActive && 
                !emergencyBrake) {
                std::cout << "[Leader] Truck " << i << " triggered emergency!\n";
                emergencyBrake = true;
                shm->leaderCmd.emergencyBrakeAll = true;
            }
        }
        
        pthread_mutex_unlock(&shm->mutex);
        
        // Display platoon status
        std::cout << "\033[2J\033[H"; // Clear screen
        std::cout << "=== PLATOON STATUS (Shared Memory) ===\n";
        std::cout << "Desired Distance: " << desiredDistance << "m | ";
        std::cout << "Emergency: " << (emergencyBrake ? "ACTIVE" : "OFF") << "\n\n";
        
        std::cout << "+--------------+";
        
        pthread_mutex_lock(&shm->mutex);
        
        int followerCount = 0;
        for (int i = 0; i < MAX_TRUCKS; i++) {
            if (shm->followerStatus[i].isActive) {
                std::cout << "           +--------------+";
                followerCount++;
            }
        }
        std::cout << "\n|  Leader " << id << "   |";
        
        for (int i = 0; i < MAX_TRUCKS; i++) {
            if (shm->followerStatus[i].isActive) {
                double dist = shm->followerStatus[i].reportedDistance;
                std::string status = shm->followerStatus[i].emergencyActive ? "!" : " ";
                std::cout << "  " << std::setw(4) << (int)dist << "m " << status
                          << "| Truck " << i << "     |";
            }
        }
        
        pthread_mutex_unlock(&shm->mutex);
        
        std::cout << "\n\nFollowers: " << followerCount << "\n";
        
        sleep(1);
    }
}

// ========== Main ==========

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " <truck_id> <role:l/f>\n";
        std::cout << "Example: " << argv[0] << " 1 l  (truck 1 as leader)\n";
        std::cout << "Example: " << argv[0] << " 2 f  (truck 2 as follower)\n";
        return 1;
    }
    
    int id = std::atoi(argv[1]);
    char role = argv[2][0];
    
    if (id < 0 || id >= MAX_TRUCKS) {
        std::cerr << "Truck ID must be 0-" << (MAX_TRUCKS-1) << "\n";
        return 1;
    }
    
    const char* SHM_NAME = "/platoon_shared_memory";
    
    // Open shared memory
    int shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to open shared memory. Is main_frame running?\n";
        return 1;
    }
    
    SharedMemory* shm = (SharedMemory*)mmap(
        nullptr,
        sizeof(SharedMemory),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        shm_fd,
        0
    );
    
    if (shm == MAP_FAILED) {
        std::cerr << "Failed to map shared memory\n";
        return 1;
    }
    
    // Register truck
    pthread_mutex_lock(&shm->mutex);
    shm->truckData[id].registered = true;
    pthread_mutex_unlock(&shm->mutex);
    
    if (role == 'l') {
        runLeader(id, shm);
    } else {
        runFollower(id, shm);
    }
    
    // Cleanup
    pthread_mutex_lock(&shm->mutex);
    shm->truckData[id].registered = false;
    pthread_mutex_unlock(&shm->mutex);
    
    munmap(shm, sizeof(SharedMemory));
    close(shm_fd);
    
    return 0;
}
