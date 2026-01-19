#include "TruckNode.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>

TruckNode::TruckNode(int truckId) 
    : id(truckId), physics(truckId, TARGET_DISTANCE) 
{
    pthread_mutex_init(&stateMutex, nullptr);
    net = std::make_unique<NetworkModule>(id);
    net->flush();
}

TruckNode::~TruckNode() {
    pthread_mutex_destroy(&stateMutex);
}

void TruckNode::setTargetPlatoonSize(int size) {
    targetPlatoonSize = size;
}

// --- THREAD 1: COMMUNICATION ---
void TruckNode::runCommunication() {
    while (true) {
        // RECEIVE LOOP
        while (true) {
            PlatoonMessage msg{};

            // 1. Check if packet exists
            if (!net->receive(msg)) break;

            // 2. Debug Print (You added this)
            //std::cout << "DEBUG: Received msg from Truck " << msg.truckId << "\n";

            // 3. SAVE THE MESSAGE (CRITICAL STEP)
            pthread_mutex_lock(&stateMutex);
            neighbors[msg.truckId] = msg;                     // <--- MUST BE HERE
            neighbors[msg.truckId].timestamp = time(nullptr); // <--- MUST BE HERE
            pthread_mutex_unlock(&stateMutex);
        }

        // BROADCAST
        pthread_mutex_lock(&stateMutex);
        bool jamming = isJamming;
        // Construct message from Physics component
        PlatoonMessage myMsg{id, physics.getPosition(), physics.getSpeed(), emergencyBrake, isDecoupled, (long)time(nullptr)};
        pthread_mutex_unlock(&stateMutex);

        if (!jamming) {
            net->broadcast(myMsg);
        }
        usleep(50000); 
    }
}

// --- THREAD 2: INPUT ---
void TruckNode::runInput() {
    std::cout << "--- INPUT READY (Enter after key) ---\n";
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
const double GHOST_TIMEOUT = 10.0;
const double SIGNAL_TIMEOUT = 2.0;
// --- THREAD 3: LOGIC (MAIN) ---
void TruckNode::runLogic() {
    auto lastTime = std::chrono::high_resolution_clock::now();

    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - lastTime;
        double dt = elapsed.count();
        lastTime = now;

        pthread_mutex_lock(&stateMutex);

        cleanupOldNeighbors(); // Deletes only after 10 seconds

        if (isJamming) {
            physics.emergencyStop(dt);
        } else {
            // --- GHOST LOGIC START ---
            // 1. Create a temporary copy of neighbors so we don't mess up the real data
            std::map<int, PlatoonMessage> percepts = neighbors;
            long currentTime = time(nullptr);

            for (auto& kv : percepts) {
                double age = difftime(currentTime, kv.second.timestamp);

                // 2. If signal is "Stale" (> 2.0s) but not "Dead" (< 10.0s)
                if (age > SIGNAL_TIMEOUT) {
                    // Force the logic to treat it as a STOPPED obstacle
                    kv.second.speed = 0.0;
                    kv.second.emergencyBrake = true;

                    // Visual warning so you know it's happening
                    if (id != 0) std::cout << " [WARNING] Ghost Detected: T" << kv.first << " (Age: " << age << "s)\r";
                }
            }
            // --- GHOST LOGIC END ---

            // 3. Pass the 'percepts' (the modified copy) to the controller
            double targetSpeed = controller.calculateTargetSpeed(
                id, physics.getPosition(), percepts, isDecoupled, emergencyBrake, targetPlatoonSize
            );

            physics.update(targetSpeed, dt);
        }

        logStatus();
        pthread_mutex_unlock(&stateMutex);

        usleep(50000);
    }
}


void TruckNode::cleanupOldNeighbors() {
    long now = time(nullptr);

    for (auto it = neighbors.begin(); it != neighbors.end(); ) {
        double age = difftime(now, it->second.timestamp);

        // Only DELETE if it has been gone for a LONG time (10s)
        if (age > GHOST_TIMEOUT) {
            std::cout << "[T" << id << "] Removing Ghost Truck " << it->first << "\n";
            it = neighbors.erase(it);
        } else {
            ++it;
        }
    }
}

void TruckNode::logStatus() {
    if (id == 0) return; // Leader usually doesn't need to spam logs

    double myPos = physics.getPosition();
    double mySpeed = physics.getSpeed();

    // 1. Find the truck directly ahead
    double minGap = 99999.0;
    int frontId = -1;

    for (const auto& kv : neighbors) {
        double diff = kv.second.position - myPos;

        // We only care about trucks IN FRONT (diff > 0)
        // and we want the CLOSEST one (diff < minGap)
        if (diff > 0 && diff < minGap) {
            minGap = diff;
            frontId = kv.first;
        }
    }

    // 2. Print Status
    std::cout << "[T" << id << "] ";
    if (isJamming) std::cout << "(NO SIGNAL) ";

    std::cout << "Speed: " << std::fixed << std::setprecision(1) << (mySpeed * 3.6) << " km/h";

    if (frontId != -1) {
        std::cout << " | Gap to T" << frontId << ": " << minGap << "m";
    } else {
        std::cout << " | (No truck ahead)";
    }
    std::cout << "\n";
}
// --- THREAD ENTRY POINTS ---
void* TruckNode::startComms(void* ctx) { ((TruckNode*)ctx)->runCommunication(); return nullptr; }
void* TruckNode::startInput(void* ctx) { ((TruckNode*)ctx)->runInput(); return nullptr; }