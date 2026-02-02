#ifndef TRUCK_NODE_H
#define TRUCK_NODE_H

#include <memory>
#include <pthread.h>
#include <chrono>
#include <fstream>
#include <string>
#include "network_module.h"
#include "VehiclePhysics.h"
#include "PlatoonController.h"
#include "common.h"

class TruckNode {
private:
    int id;
    int targetPlatoonSize = 1;
    double jammingTimer = 0.0;

    // --- UPDATED LOGGING STREAMS ---
    std::ofstream eventLog;   // Text events
    std::ofstream dataLog;    // CSV data
    std::ofstream matrixLog;  // Matrix clock state
    // -------------------------------

    int myMatrix[MAX_NODES][MAX_NODES];

    // Components
    std::unique_ptr<NetworkModule> net;
    VehiclePhysics physics;
    PlatoonController controller;

    // State Flags
    bool emergencyBrake = false;
    bool isDecoupled = false;
    bool isJamming = false;

    // Threading & Data
    std::map<int, PlatoonMessage> neighbors;
    pthread_mutex_t stateMutex;

    // Helpers
    void cleanupOldNeighbors();

    // --- NEW LOGGING HELPERS ---
    void logEvent(std::string type, std::string desc);
    void logData(double targetSpeed, double gapFront);
    void logMatrix();
    // ---------------------------

public:
    TruckNode(int truckId);
    ~TruckNode();

    void setTargetPlatoonSize(int size);

    // Main execution loops
    void runLogic();
    void runCommunication();
    void runInput();

    // Static entry points for pthreads
    static void* startComms(void* context);
    static void* startInput(void* context);
};

#endif