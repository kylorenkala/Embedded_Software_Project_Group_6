#include <iostream>
#include <vector>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fstream>
#include <cstring>
#include "common.h"

// ========== Simple Logger ==========

class Logger {
    std::ofstream file;
public:
    Logger() {
        file.open("platoon_log.txt", std::ios::app);
    }
    
    void log(const std::string& msg) {
        if (file.is_open()) {
            file << msg << "\n";
            file.flush();
        }
    }
    
    ~Logger() {
        if (file.is_open()) file.close();
    }
};

// ========== Main Frame ==========

int main() {
    const char* SHM_NAME = "/platoon_shared_memory";
    
    // Create shared memory
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to create shared memory\n";
        return 1;
    }
    
    ftruncate(shm_fd, sizeof(SharedMemory));
    
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
    
    // Initialize mutex
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shm->mutex, &attr);
    
    // Initialize shared memory
    pthread_mutex_lock(&shm->mutex);
    for (int i = 0; i < MAX_TRUCKS; i++) {
        shm->truckData[i].registered = false;
        shm->truckData[i].currentSpeed = 0.0;
        shm->truckData[i].emergencyBrake = false;
        shm->truckData[i].missedHeartbeats = 0;
        
        shm->mainData[i].tick = 0;
        shm->mainData[i].distanceToFront = 0.0;
        shm->mainData[i].obstacleDetected = false;
        shm->mainData[i].isLeader = false;
        
        shm->followerStatus[i].isActive = false;
    }
    shm->leaderCmd.desiredDistance = 20.0;
    shm->leaderCmd.emergencyBrakeAll = false;
    shm->systemRunning = true;
    pthread_mutex_unlock(&shm->mutex);
    
    Logger logger;
    uint64_t tick = 0;
    double positions[MAX_TRUCKS] = {0};
    
    std::cout << "=== MainFrame Simulation (Shared Memory) ===\n";
    std::cout << "Commands:\n";
    std::cout << "  r <ID>  - Register truck (0-7)\n";
    std::cout << "  u <ID>  - Unregister truck\n";
    std::cout << "  o <ID>  - Place obstacle at truck\n";
    std::cout << "  c       - Clear obstacles\n";
    std::cout << "  l <ID>  - Set truck as leader\n";
    std::cout << "  q       - Quit\n\n";
    
    while (true) {
        // Handle commands
        if (std::cin.rdbuf()->in_avail()) {
            char cmd;
            std::cin >> cmd;
            
            if (cmd == 'q') {
                pthread_mutex_lock(&shm->mutex);
                shm->systemRunning = false;
                pthread_mutex_unlock(&shm->mutex);
                std::cout << "Shutting down...\n";
                break;
            }
            else if (cmd == 'r') {
                int id;
                std::cin >> id;
                if (id >= 0 && id < MAX_TRUCKS) {
                    pthread_mutex_lock(&shm->mutex);
                    shm->truckData[id].registered = true;
                    pthread_mutex_unlock(&shm->mutex);
                    std::cout << "Registered truck " << id << "\n";
                    logger.log("Truck " + std::to_string(id) + " registered");
                }
            }
            else if (cmd == 'u') {
                int id;
                std::cin >> id;
                if (id >= 0 && id < MAX_TRUCKS) {
                    pthread_mutex_lock(&shm->mutex);
                    shm->truckData[id].registered = false;
                    shm->followerStatus[id].isActive = false;
                    pthread_mutex_unlock(&shm->mutex);
                    std::cout << "Unregistered truck " << id << "\n";
                    logger.log("Truck " + std::to_string(id) + " unregistered");
                }
            }
            else if (cmd == 'o') {
                int id;
                std::cin >> id;
                if (id >= 0 && id < MAX_TRUCKS) {
                    pthread_mutex_lock(&shm->mutex);
                    shm->mainData[id].obstacleDetected = true;
                    pthread_mutex_unlock(&shm->mutex);
                    std::cout << "Obstacle placed at truck " << id << "\n";
                    logger.log("OBSTACLE at truck " + std::to_string(id));
                }
            }
            else if (cmd == 'c') {
                pthread_mutex_lock(&shm->mutex);
                for (int i = 0; i < MAX_TRUCKS; i++) {
                    shm->mainData[i].obstacleDetected = false;
                }
                pthread_mutex_unlock(&shm->mutex);
                std::cout << "Obstacles cleared\n";
                logger.log("Obstacles cleared");
            }
            else if (cmd == 'l') {
                int id;
                std::cin >> id;
                if (id >= 0 && id < MAX_TRUCKS) {
                    pthread_mutex_lock(&shm->mutex);
                    for (int i = 0; i < MAX_TRUCKS; i++) {
                        shm->mainData[i].isLeader = (i == id);
                    }
                    pthread_mutex_unlock(&shm->mutex);
                    std::cout << "Truck " << id << " set as leader\n";
                }
            }
        }
        
        pthread_mutex_lock(&shm->mutex);
        
        // Get registered trucks in order
        std::vector<int> activeTrucks;
        for (int i = 0; i < MAX_TRUCKS; i++) {
            if (shm->truckData[i].registered) {
                activeTrucks.push_back(i);
            }
        }
        
        // Update positions (simple spacing)
        for (size_t i = 0; i < activeTrucks.size(); i++) {
            positions[activeTrucks[i]] = i * 25.0;
        }
        
        // Calculate and send sensor data
        for (size_t i = 1; i < activeTrucks.size(); i++) {
            int currId = activeTrucks[i];
            int frontId = activeTrucks[i - 1];
            
            double distance = positions[frontId] - positions[currId];
            
            shm->mainData[currId].distanceToFront = distance;
            shm->mainData[currId].tick = tick;
            
            // Safety warning
            if (distance < MIN_SAFE_DISTANCE) {
                std::cerr << "WARNING: Truck " << currId 
                         << " too close (" << distance << "m)\n";
                logger.log("SAFETY: Truck " + std::to_string(currId) + 
                          " below minimum distance");
            }
        }
        
        // Update leader truck (first truck has no distance reading)
        if (!activeTrucks.empty()) {
            shm->mainData[activeTrucks[0]].tick = tick;
            shm->mainData[activeTrucks[0]].distanceToFront = 100.0; // Leader has clear road
        }
        
        pthread_mutex_unlock(&shm->mutex);
        
        // Display status
        std::cout << "[Tick " << tick << "] Active trucks: " << activeTrucks.size();
        
        pthread_mutex_lock(&shm->mutex);
        bool anyObstacle = false;
        for (int i = 0; i < MAX_TRUCKS; i++) {
            if (shm->mainData[i].obstacleDetected) {
                std::cout << " | OBSTACLE at " << i;
                anyObstacle = true;
            }
        }
        pthread_mutex_unlock(&shm->mutex);
        
        std::cout << "\n";
        
        tick++;
        sleep(1);
    }
    
    // Cleanup
    pthread_mutex_destroy(&shm->mutex);
    munmap(shm, sizeof(SharedMemory));
    shm_unlink(SHM_NAME);
    close(shm_fd);
    
    return 0;
}
