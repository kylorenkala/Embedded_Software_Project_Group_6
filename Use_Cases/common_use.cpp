#ifndef COMMON_H
#define COMMON_H

#include <pthread.h>
#include <cmath>

// ========== Safety Constants ==========

const double MIN_SAFE_DISTANCE = 10.0;
const double EMERGENCY_DECEL = 8.0;
const int HEARTBEAT_TIMEOUT = 5;
const int MAX_TRUCKS = 8;

// ========== Shared Memory Structures ==========

// Data FROM truck TO main_frame
struct TruckToMain {
    double currentSpeed;
    bool registered;
    bool emergencyBrake;
    int missedHeartbeats;
};

// Data FROM main_frame TO truck
struct MainToTruck {
    uint64_t tick;
    double distanceToFront;
    bool obstacleDetected;
    bool isLeader;
};

// Leader communication (UDP replacement via shared memory)
struct LeaderCommands {
    double desiredDistance;
    bool emergencyBrakeAll;
};

struct FollowerStatus {
    int truckId;
    double reportedDistance;
    bool isActive;
    bool emergencyActive;
};

// Main shared memory layout
struct SharedMemory {
    pthread_mutex_t mutex;
    
    // Array for each truck slot
    TruckToMain truckData[MAX_TRUCKS];
    MainToTruck mainData[MAX_TRUCKS];
    
    // Leader-follower communication
    LeaderCommands leaderCmd;
    FollowerStatus followerStatus[MAX_TRUCKS];
    
    // System state
    bool systemRunning;
};

// ========== Safety Functions ==========

inline bool isSafeDistance(double distance) {
    return distance >= MIN_SAFE_DISTANCE;
}

inline double calculateStoppingDistance(double speed) {
    // d = v^2 / (2a) with 20% safety margin
    return (speed * speed) / (2.0 * EMERGENCY_DECEL) * 1.2;
}

inline bool isCollisionRisk(double distance, double speed) {
    return distance < calculateStoppingDistance(speed);
}

#endif
