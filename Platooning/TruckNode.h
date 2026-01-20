#ifndef TRUCK_NODE_H
#define TRUCK_NODE_H

#include <memory>
#include <pthread.h>
#include <chrono>
#include "network_module.h"
#include "VehiclePhysics.h"
#include "PlatoonController.h"

class TruckNode {
private:
    int id;
    int targetPlatoonSize = 1;
    double jammingTimer = 0.0;

    // Components
    std::unique_ptr<NetworkModule> net;
    VehiclePhysics physics;
    PlatoonController controller;

    // State Flags
    bool emergencyBrake = false;
    bool isDecoupled = false;
    bool isJamming = false;

    // Threading
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
    void runLogic();         // The "Brain" Loop
    void runCommunication(); // The "Ear" Loop
    void runInput();         // The "Keyboard" Loop

    // Static entry points for pthreads
    static void* startComms(void* context);
    static void* startInput(void* context);
};

#endif