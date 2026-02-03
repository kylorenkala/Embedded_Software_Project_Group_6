#include "TruckNode.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <omp.h>
#include <sstream>

// Constants
const double FAILURE_TIMEOUT = 10.0;
const double SIGNAL_TIMEOUT = 10.0;

TruckNode::TruckNode(int truckId) : id(truckId), physics(truckId, TARGET_DISTANCE) {
    pthread_mutex_init(&stateMutex, nullptr);
    net = std::make_unique<NetworkModule>(id);

    for(int i=0; i<MAX_NODES; i++) {
        for(int j=0; j<MAX_NODES; j++) {
            myMatrix[i][j] = 0;
        }
    }

    // LOGGING SETUP
    eventLog.open("truck_events_" + std::to_string(id) + ".txt");
    if (eventLog.is_open()) {
        eventLog << "=== Truck " << id << " Event Log ===\n";
        eventLog << "Timestamp, Event_Type, Description\n";
    }

    dataLog.open("truck_data_" + std::to_string(id) + ".csv");
    if (dataLog.is_open()) {
        dataLog << "sep=,\n";
        dataLog << "Time,Position,Speed_kmh,Target_Speed,Gap_Front,Is_Jammed,Emergency_Brake\n";
    }

    matrixLog.open("truck_matrix_" + std::to_string(id) + ".txt");
}

TruckNode::~TruckNode() {
    if (eventLog.is_open()) eventLog.close();
    if (dataLog.is_open()) dataLog.close();
    if (matrixLog.is_open()) matrixLog.close();
    pthread_mutex_destroy(&stateMutex);
}

void TruckNode::setTargetPlatoonSize(int size) {
    targetPlatoonSize = size;
    logEvent("CONFIG", "Target Platoon Size set to " + std::to_string(size));
}

void TruckNode::logEvent(std::string type, std::string desc) {
    if (!eventLog.is_open()) return;
    long now = time(nullptr);
    eventLog << now << ", " << type << ", " << desc << "\n";
    eventLog.flush();
}

// --- THREAD 1: COMMUNICATION ---
void TruckNode::runCommunication() {
    while (true) {
        while (true) {
            PlatoonMessage msg{};
            if (!net->receive(msg)) break;

            pthread_mutex_lock(&stateMutex);
            neighbors[msg.truckId] = msg;
            neighbors[msg.truckId].timestamp = time(nullptr); // Keep for legacy compatibility

            // --- FIX: RECORD PRECISE RECEPTION TIME ---
            receptionTimes[msg.truckId] = std::chrono::steady_clock::now();
            // ------------------------------------------

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

// --- THREAD 2: INPUT ---
void TruckNode::runInput(){
    std::cout << "--- INPUT READY ('j'=Jam, 'b'=Brake, 'd'=Decouple) ---\n";
    while (true) {
        char c;
        std::cin >> c;
        pthread_mutex_lock(&stateMutex);
        switch(c) {
            case 'b':
                emergencyBrake = !emergencyBrake;
                std::cout << (emergencyBrake ? "!!! BRAKING !!!" : ">>> RESUMING") << std::endl;
                logEvent("INPUT", emergencyBrake ? "User toggled Emergency Brake ON" : "User toggled Emergency Brake OFF");
                break;
            case 'd':
                isDecoupled = !isDecoupled;
                std::cout << (isDecoupled ? ">>> DECOUPLING" : ">>> COUPLING") << std::endl;
                logEvent("INPUT", isDecoupled ? "User toggled Decouple ON" : "User toggled Decouple OFF");
                break;
            case 'j':
                isJamming = !isJamming;
                std::cout << (isJamming ? ">>> JAMMING ON" : ">>> JAMMING OFF") << std::endl;
                logEvent("INPUT", isJamming ? "User toggled Jamming ON" : "User toggled Jamming OFF");
                break;
        }
        pthread_mutex_unlock(&stateMutex);
    }
}

// ... (previous code remains the same)

// --- THREAD 3: LOGIC (BENCHMARK READY) ---
void TruckNode::runLogic() {
    auto lastTime = std::chrono::high_resolution_clock::now();

    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - lastTime;
        double dt = elapsed.count();
        lastTime = now;

        pthread_mutex_lock(&stateMutex);

        if (id < MAX_NODES) myMatrix[id][id]++;
        cleanupOldNeighbors();

        double currentTargetSpeed = 0.0;
        double gapFront = -1.0;

        if (isJamming) {
            jammingTimer += dt;
            if (jammingTimer < 10.0) {
                // PHASE 1: BLIND CRUISE
                double blindSpeed = 50.0 / 3.6;
                physics.update(blindSpeed, dt);
                currentTargetSpeed = blindSpeed;
                if (id != 0) std::cout << " [JAMMED] Blind Cruise (" << std::fixed << std::setprecision(1) << jammingTimer << "s)\r";
            } else {
                // PHASE 2: EMERGENCY STOP
                physics.emergencyStop(dt);
                currentTargetSpeed = 0.0;
                if (id != 0) std::cout << " [JAMMED] TIMEOUT! Emergency Stop.\r";
                if (jammingTimer - dt < 10.0) {
                    logEvent("CRITICAL", "Jamming Timeout Exceeded (10s). Initiating Emergency Stop.");
                }
            }
        } else {
            // NORMAL OPERATION
            jammingTimer = 0.0;

            // =========================================================
            //               STEP 1: PREPARE DATA
            // =========================================================
            // Flatten the map into vectors (Common for both CPU and GPU)
            std::vector<double> h_pos, h_spd, h_age;
            std::vector<int> h_ids;
            auto steadyNow = std::chrono::steady_clock::now();

            for (const auto& kv : neighbors) {
                h_ids.push_back(kv.first);
                h_pos.push_back(kv.second.position);
                h_spd.push_back(kv.second.speed);

                if (receptionTimes.count(kv.first)) {
                    h_age.push_back(std::chrono::duration<double>(steadyNow - receptionTimes[kv.first]).count());
                } else {
                    h_age.push_back(0.0);
                }
            }

            // Output containers
            std::vector<double> out_pos(h_pos.size()), out_spd(h_pos.size());
            std::vector<int> out_brake(h_pos.size());

            // =========================================================
            //               STEP 2: STRESS TEST (THE EXPERIMENT)
            // =========================================================
            // We repeat the calculation 10,000 times to simulate a heavy load.
            const int STRESS_ITERATIONS = 5000;

            // Start Benchmark Timer
            auto startBench = std::chrono::high_resolution_clock::now();

            if (!h_pos.empty()) {

                // -----------------------------------------------------
                // OPTION A: GPU (OpenCL) - CURRENTLY ACTIVE
                // -----------------------------------------------------
                /*for (int k = 0; k < STRESS_ITERATIONS; k++) {
                    gpu->runSensorFusion(h_pos, h_spd, h_age, out_pos, out_spd, out_brake,
                                         physics.getPosition(), SIGNAL_TIMEOUT, FAILURE_TIMEOUT);
                }*/

                // -----------------------------------------------------
                // OPTION B: CPU (OpenMP) - CURRENTLY COMMENTED OUT
                // To test CPU: Comment out Option A, and Uncomment this block
                // -----------------------------------------------------

                for (int k = 0; k < STRESS_ITERATIONS; k++) {
                    // Reset outputs for fairness
                    out_pos = h_pos;
                    out_spd = h_spd;
                    std::fill(out_brake.begin(), out_brake.end(), 0);

                    #pragma omp parallel for
                    for (size_t i = 0; i < h_pos.size(); i++) {
                        double age = h_age[i];

                        // 1. Position Extrapolation
                        if (age > 0.0 && age < FAILURE_TIMEOUT) {
                            out_pos[i] += (h_spd[i] * age);
                        }

                        // 2. Ghost Detection
                        if (age > SIGNAL_TIMEOUT) {
                            if ((out_pos[i] - physics.getPosition()) > 0) {
                                out_spd[i] = 0.0;
                                out_brake[i] = 1;
                            }
                        }
                    }
                }

            }

            // Stop Benchmark Timer
            auto endBench = std::chrono::high_resolution_clock::now();
            double duration_us = std::chrono::duration<double, std::micro>(endBench - startBench).count();

            // Log Performance (Every 50 ticks)
            static int logCounter = 0;
            if (logCounter++ % 50 == 0 && !h_pos.empty()) {
                 std::cout << "[BENCHMARK] Time for " << STRESS_ITERATIONS
                           << " iterations: " << (int)duration_us << " microseconds" << std::endl;
            }

            // =========================================================
            //               STEP 3: APPLY RESULTS
            // =========================================================
            std::map<int, PlatoonMessage> processedNeighbors;
            for (size_t i = 0; i < h_pos.size(); i++) {
                int tId = h_ids[i];
                PlatoonMessage msg = neighbors[tId];

                // Apply the calculated physics (from the last iteration)
                msg.position = out_pos[i];
                msg.speed = out_spd[i];
                if (out_brake[i] == 1) {
                    msg.emergencyBrake = true;
                }

                processedNeighbors[tId] = msg;

                // Gap logging calculation
                if (tId != id) {
                    double dist = msg.position - physics.getPosition();
                    if (dist > 0 && (gapFront < 0 || dist < gapFront)) gapFront = dist;
                }
            }

            // 4. Run Controller
            double targetSpeed = controller.calculateTargetSpeed(
                id, physics.getPosition(), physics.getSpeed(),
                processedNeighbors, isDecoupled, emergencyBrake, targetPlatoonSize
            );

            currentTargetSpeed = targetSpeed;
            physics.update(targetSpeed, dt);
        }

        logData(currentTargetSpeed, gapFront);
        logMatrix();

        pthread_mutex_unlock(&stateMutex);
        usleep(50000);
    }
}

void TruckNode::cleanupOldNeighbors() {
    // We still use the coarse time for cleanup as 1s precision is fine for deletion
    long now = time(nullptr);
    for (auto it = neighbors.begin(); it != neighbors.end(); ) {
        if (difftime(now, it->second.timestamp) > FAILURE_TIMEOUT) {
            logEvent("NETWORK", "Lost connection to Truck " + std::to_string(it->first) + " (Timeout)");
            // Clean up the reception time map as well to prevent memory leaks
            receptionTimes.erase(it->first);
            it = neighbors.erase(it);
        } else {
            ++it;
        }
    }
}

void TruckNode::logData(double targetSpeed, double gapFront) {
    if (!dataLog.is_open()) return;

    dataLog << std::fixed << std::setprecision(2)
            << time(nullptr) << ","
            << physics.getPosition() << ","
            << (physics.getSpeed() * 3.6) << ","
            << targetSpeed << ","
            << gapFront << ","
            << (isJamming ? "1" : "0") << ","
            << (emergencyBrake ? "1" : "0") << "\n";
}

void TruckNode::logMatrix() {
    if (!matrixLog.is_open()) return;
    matrixLog << "TIME: " << time(nullptr) << "\n";
    for(int i=0; i<MAX_NODES; i++) {
        for(int j=0; j<MAX_NODES; j++) {
            matrixLog << myMatrix[i][j] << " ";
        }
        matrixLog << "\n";
    }
    matrixLog << "-----------------\n";
}

// Entry Points
void* TruckNode::startComms(void* ctx) { ((TruckNode*)ctx)->runCommunication(); return nullptr; }
void* TruckNode::startInput(void* ctx) { ((TruckNode*)ctx)->runInput(); return nullptr; }