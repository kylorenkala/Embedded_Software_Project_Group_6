#ifndef COMMON_H
#define COMMON_H

constexpr int PORT_BASE = 5000;
constexpr int VISUALIZER_PORT = 4999;
constexpr int MAX_NODES = 10;

struct PlatoonMessage {
    int truckId;
    double position;
    double speed;
    bool emergencyBrake;
    bool isDecoupled;
    long timestamp;
    bool isRadarOnly;

    //Vector [MAX_NODES] --> Matrix [MAX_NODES][MAX_NODES]
    int matrixClock[MAX_NODES][MAX_NODES];
};

#endif