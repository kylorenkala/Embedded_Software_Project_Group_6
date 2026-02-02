#include "TruckNode.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cmath>

// Constants
const double FAILURE_TIMEOUT = 10.0; // Stop after 10s of no signal
const double SIGNAL_TIMEOUT = 2.0;   // Consider signal lost after 2s

TruckNode::TruckNode(int truckId) : id(truckId), physics(truckId, TARGET_DISTANCE) {
    pthread_mutex_init(&stateMutex, nullptr);
    net = std::make_unique<NetworkModule>(id);

    // Initialize Matrix Clock to 0
    for(int i=0; i<MAX_NODES; i++) {
        for(int j=0; j<MAX_NODES; j++) {
            myMatrix[i][j] = 0;
        }
    }

    // Open Log File
    std::string filename = "truck_log_" + std::to_string(id) + ".txt";
    logFile.open(filename);
    if (logFile.is_open()) {
        logFile << "=== Truck " << id << " Log Started ===\n";
    }
}

TruckNode::~TruckNode() {
    if (logFile.is_open()) logFile.close();
    pthread_mutex_destroy(&stateMutex);
}

void TruckNode::setTargetPlatoonSize(int size) {
    targetPlatoonSize = size;
}

// --- THREAD 1: COMMUNICATION (Unchanged) ---
void TruckNode::runCommunication() {
    while (true) {
        // RECEIVE
        while (true) {
            PlatoonMessage msg{};
            if (!net->receive(msg)) break;

            pthread_mutex_lock(&stateMutex);
            neighbors[msg.truckId] = msg;
            neighbors[msg.truckId].timestamp = time(nullptr);

            // Update Matrix Clock
            for(int i=0; i<MAX_NODES; i++) {
                for(int j=0; j<MAX_NODES; j++) {
                    myMatrix[i][j] = std::max(myMatrix[i][j], msg.matrixClock[i][j]);
                }
            }
            // Update my knowledge of what the sender knows
            if(id < MAX_NODES && msg.truckId < MAX_NODES) {
                 myMatrix[id][msg.truckId] = std::max(myMatrix[id][msg.truckId], msg.matrixClock[msg.truckId][msg.truckId]);
            }
            pthread_mutex_unlock(&stateMutex);
        }

        // BROADCAST
        pthread_mutex_lock(&stateMutex);
        bool jamming = isJamming;

        PlatoonMessage myMsg{};
        myMsg.truckId = id;
        myMsg.position = physics.getPosition();
        myMsg.speed = physics.getSpeed();
        myMsg.emergencyBrake = emergencyBrake;
        myMsg.isDecoupled = isDecoupled;
        myMsg.timestamp = time(nullptr);

        for(int i=0; i<MAX_NODES; i++) {
            for(int j=0; j<MAX_NODES; j++) {
                myMsg.matrixClock[i][j] = myMatrix[i][j];
            }
        }
        pthread_mutex_unlock(&stateMutex);

        if (!jamming) {
            net->broadcast(myMsg);
        }
        usleep(50000);
    }
}

// --- THREAD 2: INPUT (Unchanged) ---
void TruckNode::runInput(){
    std::cout << "--- INPUT READY ('j'=Jam, 'b'=Brake, 'd'=Decouple) ---\n";
    while (true) {
        char c;
        std::cin >> c;
        pthread_mutex_lock(&stateMutex);
        switch(c) {
            case 'b': emergencyBrake = !emergencyBrake;
                      std::cout << (emergencyBrake ? "!!! BRAKING !!!" : ">>> RESUMING") << std::endl; break;
            case 'd': isDecoupled = !isDecoupled;
                      std::cout << (isDecoupled ? ">>> DECOUPLING" : ">>> COUPLING") << std::endl; break;
            case 'j': isJamming = !isJamming;
                      std::cout << (isJamming ? ">>> JAMMING ON" : ">>> JAMMING OFF") << std::endl; break;
        }
        pthread_mutex_unlock(&stateMutex);
    }
}

// --- THREAD 3: LOGIC (SIMPLIFIED) ---
void TruckNode::runLogic() {
    auto lastTime = std::chrono::high_resolution_clock::now();

    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - lastTime;
        double dt = elapsed.count();
        lastTime = now;

        pthread_mutex_lock(&stateMutex);

        // 1. Matrix Clock Tick
        if (id < MAX_NODES) myMatrix[id][id]++;

        // 2. Cleanup Old Neighbors (Maintenance only)
        cleanupOldNeighbors();

        // 3. DETECT COMMUNICATION FAILURE
        // Definition: Either Manual Jamming ('j') OR Leader Signal Lost (> 2s)
        bool commsFailed = isJamming;

        // Check for real signal loss (if we are a follower)
        if (id != 0 && !neighbors.empty()) {
            // Assume Truck 0 is leader for the simple case, or nearest front truck
            // Here we check if *any* truck is visible. If neighbors is empty or all stale, we fail.
            bool leaderAlive = false;
            long currentTime = time(nullptr);

            // Simple check: Is Truck 0 (or leader) alive?
            if (neighbors.count(0)) {
                double age = difftime(currentTime, neighbors[0].timestamp);
                if (age < SIGNAL_TIMEOUT) leaderAlive = true;
            }

            // If I am meant to follow but Leader is dead -> Comms Failed
            if (!leaderAlive && !isJamming) {
                commsFailed = true;
            }
        }

        // 4. HANDLE BEHAVIOR
        if (commsFailed) {
            jammingTimer += dt;

            if (jammingTimer < FAILURE_TIMEOUT) {
                // PHASE 1: Simple "Keep Following" (0 - 10s)
                // "Drive 50kmph constant to maintain flow"
                double blindSpeed = 50.0 / 3.6;
                physics.update(blindSpeed, dt);

                if (id != 0) std::cout << " [COMMS FAIL] Keeping Speed 50km/h (" << (10.0 - jammingTimer) << "s left)\r";
            } else {
                // PHASE 2: Timeout (> 10s)
                // "Whole platoon will stop"
                physics.emergencyStop(dt);
                if (id != 0) std::cout << " [COMMS FAIL] TIMEOUT! Emergency Stop.\r";
            }
        }
        else {
            // NORMAL OPERATION
            jammingTimer = 0.0;

            // Simple Controller Logic (Standard PID)
            // Note: We removed the "Ghost Detection" loop that modified 'neighbors' here.
            // We pass the raw neighbors map directly.
            double targetSpeed = controller.calculateTargetSpeed(
                id,
                physics.getPosition(),
                physics.getSpeed(),
                neighbors,
                isDecoupled,
                emergencyBrake,
                targetPlatoonSize
            );

            physics.update(targetSpeed, dt);
        }

        logStatus();
        pthread_mutex_unlock(&stateMutex);
        usleep(50000); // 20Hz
    }
}

void TruckNode::cleanupOldNeighbors() {
    long now = time(nullptr);
    for (auto it = neighbors.begin(); it != neighbors.end(); ) {
        if (difftime(now, it->second.timestamp) > 10.0) {
            it = neighbors.erase(it);
        } else {
            ++it;
        }
    }
}

void TruckNode::logStatus() {
    if (!logFile.is_open()) return;

    double mySpeed = physics.getSpeed();

    logFile << "------------------------------------------------\n";
    logFile << "TIME: " << time(nullptr) << "\n";
    logFile << "[STATUS] T" << id << " Speed: " << (mySpeed * 3.6) << " km/h";
    if (isJamming) logFile << " [JAMMED]";
    logFile << "\n";

    // Print Matrix
    logFile << "[MATRIX CLOCK]\n      ";
    for(int k=0; k<MAX_NODES; k++) logFile << "T" << k << "  ";
    logFile << "\n";

    for(int i=0; i<MAX_NODES; i++) {
        logFile << "Row " << i << ": ";
        for(int j=0; j<MAX_NODES; j++) {
            logFile << std::setw(3) << myMatrix[i][j] << " ";
        }
        logFile << "\n";
    }
    logFile.flush();
}

// Entry Points
void* TruckNode::startComms(void* ctx) { ((TruckNode*)ctx)->runCommunication(); return nullptr; }
void* TruckNode::startInput(void* ctx) { ((TruckNode*)ctx)->runInput(); return nullptr; }