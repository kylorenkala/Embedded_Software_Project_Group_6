#include "TruckNode.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <omp.h>
#include <sstream>
#include <profiler.h>


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
        //pycpa
        double t_comms_start;
        Profiler::recordStart("Comms", t_comms_start);

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
        Profiler::recordEnd("Comms", t_comms_start);
        usleep(50000);
    }
}

// --- THREAD 2: INPUT ---
void TruckNode::runInput(){
    std::cout << "--- INPUT READY ('j'=Jam, 'b'=Brake, 'd'=Decouple) ---\n";
    while (true) {
        char c;
        std::cin >> c;
        double t_input_start;
        Profiler::recordStart("Input", t_input_start);
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
        Profiler::recordEnd("Input", t_input_start);
    }
}

// --- THREAD 3: LOGIC ---
void TruckNode::runLogic() {
    auto lastTime = std::chrono::high_resolution_clock::now();

    while (true) {
        double t_logic_start;
        Profiler::recordStart("Logic", t_logic_start);

        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - lastTime;
        double dt = elapsed.count();
        lastTime = now;

        pthread_mutex_lock(&stateMutex);

        if (id < MAX_NODES) myMatrix[id][id]++;
        cleanupOldNeighbors();

        double currentTargetSpeed = 0.0;
        double gapFront = -1.0;

        if (isJamming) { // jamming logic runs first
            jammingTimer += dt; // start timer
            if (jammingTimer < 10.0) {
                double blindSpeed = 50.0 / 3.6;
                physics.update(blindSpeed, dt); // set constant speed
                currentTargetSpeed = blindSpeed;
                if (id != 0) std::cout << " [JAMMED] Blind Cruise (" << std::fixed << std::setprecision(1) << jammingTimer << "s)\r";
            } else { // timer runs out
                physics.emergencyStop(dt);
                currentTargetSpeed = 0.0;
                if (id != 0) std::cout << " [JAMMED] TIMEOUT! Emergency Stop.\r";
                if (jammingTimer - dt < 10.0) {
                    logEvent("CRITICAL", "Jamming Timeout Exceeded (10s). Initiating Emergency Stop.");
                }
            }
        } else { // connection restored
            jammingTimer = 0.0;

            // --- PREPARE DATA FOR OPENMP ---
            std::vector<PlatoonMessage> sensorData;
            std::vector<int> sensorIds;
            std::vector<double> preciseAges; // New Vector for smooth ages

            // Capture the current precise time for age calculation
            auto steadyNow = std::chrono::steady_clock::now();

            for (const auto& kv : neighbors) {
                sensorData.push_back(kv.second);
                sensorIds.push_back(kv.first);

                // --- CALCULATE PRECISE AGE ---
                // Calculate age in floating-point seconds (e.g., 0.123s)
                if (receptionTimes.count(kv.first)) {
                    double age = std::chrono::duration<double>(steadyNow - receptionTimes[kv.first]).count();
                    preciseAges.push_back(age);
                } else {
                    preciseAges.push_back(0.0);
                }
            }

            int dataSize = sensorData.size();

            #pragma omp parallel for
            for (int i = 0; i < dataSize; i++) {
                // Use the precise age we calculated earlier
                double age = preciseAges[i];

                // --- FIX: SMOOTH POSITION EXTRAPOLATION ---
                // Now using high-precision age, so the ghost moves smoothly
                if (age > 0.0 && age < FAILURE_TIMEOUT) {
                    sensorData[i].position += (sensorData[i].speed * age);
                }

                if (age > SIGNAL_TIMEOUT) {
                    double relativePos = sensorData[i].position - physics.getPosition();
                    if (relativePos > 0) {
                        sensorData[i].speed = 0.0;
                        sensorData[i].emergencyBrake = true;
                    }
                }
            }

            std::map<int, PlatoonMessage> processedNeighbors;
            for (int i = 0; i < dataSize; i++) {
                processedNeighbors[sensorIds[i]] = sensorData[i];
                if (sensorIds[i] != id) {
                    double dist = sensorData[i].position - physics.getPosition();
                    if (dist > 0 && (gapFront < 0 || dist < gapFront)) {
                        gapFront = dist;
                    }
                }
            }

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

        Profiler::recordEnd("Logic", t_logic_start);
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
