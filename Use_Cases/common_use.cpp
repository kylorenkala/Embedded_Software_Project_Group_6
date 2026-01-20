#ifndef COMMON_H
#define COMMON_H

#include <cmath>

// ========== Message Structures ==========

struct Heartbeat {
    uint64_t tick;
};

struct SensorMsg {
    double distanceToFront;
    bool obstacleDetected;
};

struct SetpointMsg {
    double desiredDistance;
    bool emergencyBrake;
};

enum class LeaderMsgType {
    Join,
    Leave,
    Distance,
    EmergencyBrake
};

struct LeaderMsg {
    LeaderMsgType type;
    int truckId;
    double distance;
};

// ========== Safety Constants ==========

const double MIN_SAFE_DISTANCE = 10.0;
const double EMERGENCY_DECEL = 8.0;
const int HEARTBEAT_TIMEOUT = 5;

// ========== Simple Safety Functions ==========

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