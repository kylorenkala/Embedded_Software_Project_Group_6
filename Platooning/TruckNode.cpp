#include "TruckNode.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>

const double GHOST_TIMEOUT = 10.0;
const double SIGNAL_TIMEOUT = 2.0;

TruckNode::TruckNode(int truckId)
    : id(truckId), physics(truckId, TARGET_DISTANCE)
{
    pthread_mutex_init(&stateMutex, nullptr);
    net = std::make_unique<NetworkModule>(id);

    for (int i = 0; i < MAX_NODES; i++) {
        myClock[i] = 0;
    }

    net->flush();
}

TruckNode::~TruckNode() {
    pthread_mutex_destroy(&stateMutex);
}

void TruckNode::setTargetPlatoonSize(int size) {
    targetPlatoonSize = size;
}

// --- THREAD 1: COMMUNICATION ---
// --- THREAD 1: COMMUNICATION ---
void TruckNode::runCommunication() {
    while (true) {
        // --- PART A: RECEIVE LOOP ---
        // We process ALL waiting packets to ensure our state is up-to-date
        while (true) {
            PlatoonMessage msg{};

            // 1. Try to get a packet (Non-blocking)
            if (!net->receive(msg)) break;

            // 2. CRITICAL SECTION: Update State & Clocks
            pthread_mutex_lock(&stateMutex);

            // A. Update Neighbor Data
            neighbors[msg.truckId] = msg;
            neighbors[msg.truckId].timestamp = time(nullptr); // Update "Wall Clock" for Ghost Detection

            // B. Update Logical Matrix Clock (Synchronization Requirement)
            // Algorithm: LocalClock[i] = max(LocalClock[i], ReceivedClock[i])
            for (int i = 0; i < MAX_NODES; i++) {
                // We update our known history of the system based on the sender's history
                if (i < MAX_NODES) {
                    myClock[i] = std::max(myClock[i], msg.matrixClock[i]);
                }
            }

            pthread_mutex_unlock(&stateMutex);
        }

        // --- PART B: BROADCAST LOOP ---
        // Prepare the message to send to others

        pthread_mutex_lock(&stateMutex);
        bool jamming = isJamming; // Local copy to avoid holding mutex during I/O

        // 1. Construct the Message
        PlatoonMessage myMsg{};
        myMsg.truckId = id;
        myMsg.position = physics.getPosition();
        myMsg.speed = physics.getSpeed();
        myMsg.emergencyBrake = emergencyBrake;
        myMsg.isDecoupled = isDecoupled;
        myMsg.timestamp = time(nullptr);

        // 2. Attach Logical Matrix Clock
        // Send our current knowledge of the system time to everyone else
        for (int i = 0; i < MAX_NODES; i++) {
            myMsg.matrixClock[i] = myClock[i];
        }

        pthread_mutex_unlock(&stateMutex);

        // 3. Send (If not jamming)
        if (!jamming) {
            net->broadcast(myMsg);
        }

        // Broadcast rate: ~20Hz (Every 50ms)
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

// --- THREAD 3: LOGIC (MAIN) ---
// In TruckNode.cpp

void TruckNode::runLogic() {
    auto lastTime = std::chrono::high_resolution_clock::now();

    while (true) {
        // 1. Calculate Delta Time (dt)
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - lastTime;
        double dt = elapsed.count();
        lastTime = now;

        pthread_mutex_lock(&stateMutex);

        // --- FIX 2: Increment My Logical Clock ---
        // This marks a new "event" or "state change" in the distributed system
        if (id < MAX_NODES) {
            myClock[id]++;
        }

        // 2. Cleanup "Dead" Trucks (> 10s silence)
        cleanupOldNeighbors();

        // 3. JAMMING LOGIC
        if (isJamming) {
            // Increment timer
            jammingTimer += dt;

            if (jammingTimer < 10.0) {
                // PHASE 1: BLIND CRUISE (0 - 10 seconds)
                // "Drive on constant speed (50kmh)" ignoring sensors
                double blindSpeed = 50.0 / 3.6; // 13.8 m/s
                physics.update(blindSpeed, dt);

                if (id == 0) std::cout << " [JAMMED] Blind Cruising (" << (10.0 - jammingTimer) << "s left)\r";
            } else {
                // PHASE 2: EMERGENCY STOP (> 10 seconds)
                physics.emergencyStop(dt);
                if (id == 0) std::cout << " [JAMMED] Timeout! Stopping.\r";
            }
        }
        else {
            // NORMAL OPERATION
            jammingTimer = 0.0; // Reset timer

            // --- GHOST LOGIC START ---
            // Create a temporary copy of neighbors to modify for the Controller.
            // We use a copy so we don't corrupt the actual network data.
            std::map<int, PlatoonMessage> percepts = neighbors;
            long currentTime = time(nullptr);

            for (auto& kv : percepts) {
                double age = difftime(currentTime, kv.second.timestamp);

                // If signal is "Stale" (> 2.0s) but not yet deleted (< 10.0s)
                if (age > SIGNAL_TIMEOUT) {
                    // Force the logic to treat it as a STOPPED obstacle
                    kv.second.speed = 0.0;
                    kv.second.emergencyBrake = true;

                    // Visual warning so you know it's happening
                    if (id != 0) std::cout << " [WARNING] Ghost Detected: T" << kv.first << " (Age: " << age << "s)\r";
                }
            }
            // --- GHOST LOGIC END ---

            // 4. CALCULATE TARGET SPEED
            // CRITICAL FIX: We pass 'physics.getSpeed()' as the 3rd argument.
            // This allows the controller to calculate Stopping Distance.
            double targetSpeed = controller.calculateTargetSpeed(
                id,
                physics.getPosition(),
                physics.getSpeed(),    // <--- NEW ARGUMENT (Fixes Crashing)
                percepts,              // Use the 'Ghost-Aware' map copy
                isDecoupled,
                emergencyBrake,
                targetPlatoonSize
            );

            // 5. UPDATE PHYSICS
            physics.update(targetSpeed, dt);
        }

        logStatus();
        pthread_mutex_unlock(&stateMutex);

        // Run at ~20 Hz
        usleep(50000);
    }
}

;
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
    if (id == 0) return;

    double myPos = physics.getPosition();
    double mySpeed = physics.getSpeed();

    // 1. Find the truck directly ahead
    double minGap = 99999.0;
    int frontId = -1;

    for (const auto& kv : neighbors) {
        double diff = kv.second.position - myPos;
        if (diff > 0 && diff < minGap) {
            minGap = diff;
            frontId = kv.first;
        }
        // REMOVED THE CLOCK PRINT FROM HERE
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

    // --- FIX 3: Print Clock ONCE at the end ---
    std::cout << " [CLOCK]: [";
    for(int i=0; i<4; i++) {
        std::cout << myClock[i] << " ";
    }
    std::cout << "]\n";
    // ------------------------------------------
}// --- THREAD ENTRY POINTS ---
void* TruckNode::startComms(void* ctx) { ((TruckNode*)ctx)->runCommunication(); return nullptr; }
void* TruckNode::startInput(void* ctx) { ((TruckNode*)ctx)->runInput(); return nullptr; }