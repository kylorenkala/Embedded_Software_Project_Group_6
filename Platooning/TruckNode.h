#ifndef TRUCK_NODE_H
#define TRUCK_NODE_H

#include "GpuHandler.h"
#include <memory>
#include <pthread.h>
#include <chrono>
#include <fstream>
#include <string>
#include <map>
#include "network_module.h"
#include "VehiclePhysics.h"
#include "PlatoonController.h"
#include "common.h"

class TruckNode {
private:
    int id;
    int targetPlatoonSize = 1;
    double jammingTimer = 0.0;

    // Logging Streams
    std::ofstream eventLog;
    std::ofstream dataLog;
    std::ofstream matrixLog;

    int myMatrix[MAX_NODES][MAX_NODES];

    // Components
    std::unique_ptr<NetworkModule> net;
    VehiclePhysics physics;
    PlatoonController controller;
    std::unique_ptr<GpuHandler> gpu;

    // State Flags
    bool emergencyBrake = false;
    bool isDecoupled = false;
    bool isJamming = false;

    // Threading & Data
    std::map<int, PlatoonMessage> neighbors;

    // --- FIX: HIGH PRECISION TIMING MAP ---
    // Stores the exact system time we received the last packet
    std::map<int, std::chrono::steady_clock::time_point> receptionTimes;
    // --------------------------------------

    pthread_mutex_t stateMutex;

    // Helpers
    void cleanupOldNeighbors();
    void logEvent(std::string type, std::string desc);
    void logData(double targetSpeed, double gapFront);
    void logMatrix();

public:
    TruckNode(int truckId);
    ~TruckNode();

    void setTargetPlatoonSize(int size);

    void runLogic();
    void runCommunication();
    void runInput();

    static void* startComms(void* context);
    static void* startInput(void* context);
};

#endif