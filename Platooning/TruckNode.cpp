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

    // Initialize Matrix Clock
    for(int i=0; i<MAX_NODES; i++) {
        for(int j=0; j<MAX_NODES; j++) {
            myMatrix[i][j] = 0;
        }
    }

    // Initialize GPU
    gpu = std::make_unique<GpuHandler>();

    // Logging
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
            neighbors[msg.truckId].timestamp = time(nullptr);

            // Record precise reception time
            receptionTimes[msg.truckId] = std::chrono::steady_clock::now();

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

// --- THREAD 3: LOGIC (GPU ENABLED) ---
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
                double blindSpeed = 50.0 / 3.6;
                physics.update(blindSpeed, dt);
                currentTargetSpeed = blindSpeed;
                if (id != 0) std::cout << " [JAMMED] Blind Cruise (" << std::fixed << std::setprecision(1) << jammingTimer << "s)\r";
            } else {
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

            // 1. Prepare Data Vectors (Flatten the Map for GPU)
            std::vector<double> h_pos, h_spd, h_age;
            std::vector<int> h_ids;
            auto steadyNow = std::chrono::steady_clock::now();

            for (const auto& kv : neighbors) {
                h_ids.push_back(kv.first);
                h_pos.push_back(kv.second.position);
                h_spd.push_back(kv.second.speed);

                // Calculate precise age on CPU first
                if (receptionTimes.count(kv.first)) {
                    h_age.push_back(std::chrono::duration<double>(steadyNow - receptionTimes[kv.first]).count());
                } else {
                    h_age.push_back(0.0);
                }
            }

            // 2. Run on GPU (OpenCL)
            std::vector<double> out_pos, out_spd;
            std::vector<int> out_brake;

            if (!h_pos.empty()) {
                gpu->runSensorFusion(h_pos, h_spd, h_age, out_pos, out_spd, out_brake,
                                     physics.getPosition(), SIGNAL_TIMEOUT, FAILURE_TIMEOUT);
            }

            // 3. Map Results Back to Objects
            std::map<int, PlatoonMessage> processedNeighbors;
            for (size_t i = 0; i < h_pos.size(); i++) {
                int tId = h_ids[i];
                PlatoonMessage msg = neighbors[tId];

                // Apply GPU results
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
    long now = time(nullptr);
    for (auto it = neighbors.begin(); it != neighbors.end(); ) {
        if (difftime(now, it->second.timestamp) > FAILURE_TIMEOUT) {
            logEvent("NETWORK", "Lost connection to Truck " + std::to_string(it->first) + " (Timeout)");
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