#ifndef COMMON_H
#define COMMON_H

#include <cstdint>

constexpr int PORT_BASE = 5000;
constexpr int VISUALIZER_PORT = 4999; // <--- NEW: The Camera Port

struct PlatoonMessage {
    int truckId;
    double position;
    double speed;
    bool emergencyBrake;
    long timestamp;
};

#endif