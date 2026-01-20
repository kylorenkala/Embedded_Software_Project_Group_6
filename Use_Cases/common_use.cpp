#ifndef COMMON_H
#define COMMON_H

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>

// ========== Safety Constants ==========
const double MIN_SAFE_DISTANCE = 10.0;
const double EMERGENCY_DECEL = 8.0;
const int MAX_TRUCKS = 8;

// ========== Message Structures (from main's perspective) ==========

// Truck sends position to main_frame
struct rxMainMessageFrame {
    unsigned short position;
    unsigned short speed;
    bool emergency_brake;
    bool request_ready;
};

// Main_frame sends sensor data to truck
struct txMainMessageFrame {
    unsigned short sensor_data;      // distance to front truck
    bool obstacle_detected;
    bool response_ready;
};

// Leader sends commands to followers
struct LeaderCommandFrame {
    unsigned short distance_setpoint;
    bool emergency_brake_all;
};

// Follower reports status to leader
struct FollowerReportFrame {
    unsigned short actual_distance;
    bool emergency_active;
    bool is_active;
};

// ========== Shared Memory Layout ==========

struct SharedMemoryLayout {
    pthread_mutex_t global_mutex;
    
    // Communication between trucks and main_frame
    rxMainMessageFrame rx_slots[MAX_TRUCKS];
    txMainMessageFrame tx_slots[MAX_TRUCKS];
    
    // Leader-follower communication
    LeaderCommandFrame leader_cmd;
    FollowerReportFrame follower_status[MAX_TRUCKS];
    
    // System state
    uint64_t tick;
    bool system_running;
};

// ========== Simple Safety Functions ==========

inline bool isSafeDistance(unsigned short distance) {
    return distance >= MIN_SAFE_DISTANCE;
}

inline unsigned short calculateStoppingDistance(unsigned short speed) {
    // d = v^2 / (2a) with safety margin
    double speed_d = speed;
    return (unsigned short)((speed_d * speed_d) / (2.0 * EMERGENCY_DECEL) * 1.2);
}

inline bool isCollisionRisk(unsigned short distance, unsigned short speed) {
    return distance < calculateStoppingDistance(speed);
}

#endif
