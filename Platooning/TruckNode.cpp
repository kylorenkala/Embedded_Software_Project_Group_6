#include "TruckNode.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cmath>
#include <vector>    // <--- REQUIRED FOR OPENMP
#include <omp.h>     // <--- REQUIRED FOR OPENMP

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
        logFile << "=== Truck " << id << " Log Started (OpenMP Enabled) ===\n";
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

// --- THREAD 3: LOGIC (OPENMP IMPLEMENTED) ---
// --- THREAD 3: LOGIC (FIXED JAMMING SCENARIO) ---
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

        // 2. Cleanup Old Neighbors
        cleanupOldNeighbors();

        // 3. JAMMING / FAILURE CHECK
        // We handle jamming FIRST because it overrides everything else.
        if (isJamming) {
            jammingTimer += dt;

            if (jammingTimer < 10.0) {
                // PHASE 1: BLIND CRUISE (0 - 10 seconds)
                // Requirement: "Keep driving for 10 seconds at 30m distance and at 50kmh"
                // Since we are jammed, we cannot measure distance.
                // We MUST assume that driving at 50km/h maintains the gap.

                double blindSpeed = 50.0 / 3.6; // 13.8 m/s (50 km/h)
                physics.update(blindSpeed, dt);

                if (id != 0) std::cout << " [JAMMED] Blind Cruise (Time: " << std::fixed << std::setprecision(1) << jammingTimer << "s) - Speed: 50 km/h\r";
            }
            else {
                // PHASE 2: EMERGENCY STOP (> 10 seconds)
                // Requirement: "If comms doesn't restore, whole platoon stops"
                physics.emergencyStop(dt);
                if (id != 0) std::cout << " [JAMMED] TIMEOUT! Emergency Stop.\r";
            }
        }
        else {
            // NORMAL OPERATION (Comms Restored)
            jammingTimer = 0.0;

            // --- OPENMP SENSOR FUSION START ---
            std::vector<PlatoonMessage> sensorData;
            std::vector<int> sensorIds;

            // 1. Flatten Map to Vector for OpenMP
            for (const auto& kv : neighbors) {
                sensorData.push_back(kv.second);
                sensorIds.push_back(kv.first);
            }

            int dataSize = sensorData.size();
            long currentTime = time(nullptr);

            // 2. Parallel Processing (OpenMP)
            #pragma omp parallel for
            for (int i = 0; i < dataSize; i++) {
                double age = difftime(currentTime, sensorData[i].timestamp);

                // Signal Timeout Logic (Ghost Detection)
                if (age > SIGNAL_TIMEOUT) {
                    // FIX: Only panic if the ghost is IN FRONT of us.
                    // If the guy behind me disappears, I don't need to slam the brakes.
                    double relativePos = sensorData[i].position - physics.getPosition();

                    if (relativePos > 0) { // It is AHEAD of me
                        sensorData[i].speed = 0.0;
                        sensorData[i].emergencyBrake = true;
                    }
                    else {
                        // It is BEHIND me. Ignore it for now so I don't stop.
                        // (The 10s timeout in 'cleanupOldNeighbors' will handle it later)
                    }
                }
            }

            // 3. Reconstruct Map for Controller
            std::map<int, PlatoonMessage> processedNeighbors;
            for (int i = 0; i < dataSize; i++) {
                processedNeighbors[sensorIds[i]] = sensorData[i];
            }
            // --- OPENMP SENSOR FUSION END ---

            // 4. PID CONTROL
            double targetSpeed = controller.calculateTargetSpeed(
                id, physics.getPosition(), physics.getSpeed(),
                processedNeighbors,
                isDecoupled, emergencyBrake, targetPlatoonSize
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
        // Change timeout from 10.0 to 1.0 or 1.5 seconds
        if (difftime(now, it->second.timestamp) > 1.5) {
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