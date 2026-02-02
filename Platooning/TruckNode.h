#ifndef TRUCK_NODE_H
#define TRUCK_NODE_H

#include <memory>
#include <pthread.h>
#include <chrono>
#include <map>
#include <vector>
#include "network_module.h"
#include "VehiclePhysics.h"
#include "PlatoonController.h"
#include "common.h" // Required for MAX_NODES constant

class TruckNode {
private:
    int id;
    int targetPlatoonSize = 1;
    double jammingTimer = 0.0;

    // --- REQUIREMENT: LOGICAL MATRIX CLOCK ---
    // Stores the vector clock state for this node
    int myClock[MAX_NODES];

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
    void logStatus();

public:
    TruckNode(int truckId);
    ~TruckNode();

    void setTargetPlatoonSize(int size);

    // Main execution loops
    void runLogic();         // The "Brain" Loop (Logic + Physics + OpenMP)
    void runCommunication(); // The "Ear" Loop (UDP Sending/Receiving)
    void runInput();         // The "Keyboard" Loop (User Fault Injection)

    // Static entry points for pthreads
    static void* startComms(void* context);
    static void* startInput(void* context);
};

#endif