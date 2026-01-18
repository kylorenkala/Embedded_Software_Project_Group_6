#include <iostream>
#include <pthread.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <map>
#include <chrono>
#include <iomanip>
#include <unistd.h>
#include "common.h"
#include "network_module.h"

// --- Configuration ---
const double KMH_TO_MS = 1.0 / 3.6;
const double LEADER_FIXED_SPEED = 50.0 * KMH_TO_MS;
const double MAX_SPEED = 100.0 * KMH_TO_MS;
const double MAX_ACCEL = 3.0;
const double MAX_BRAKE = 5.0;
const double TARGET_DISTANCE = 30.0;
const double EXTRA_GAP_DISTANCE = 30.0; // Extra space when Decoupled (Total 60m)
const double TIMEOUT_SEC = 2.0;
const double K_P = 1.0;
const double GAP_TOLERANCE = 1.0;

// --- Global State ---
struct TruckState {
    int id;
    int targetPlatoonSize = 1;
    double speed = 0.0;
    double position = 0.0;
    bool emergencyBrake = false;
    bool isDecoupled = false; // NEW: Are we leaving a gap?
    std::map<int, PlatoonMessage> neighbors;
};

TruckState state;
pthread_mutex_t stateMutex;

struct TruckRank {
    int id;
    double position;
    bool isDecoupled; // Store this for ranking logic
};

// ==========================================
// THREAD 1: COMMUNICATION
// ==========================================
void* commsLoop(void* arg) {
    NetworkModule* net = (NetworkModule*)arg;
    while (true) {
        while (true) {
            PlatoonMessage msg;
            if (!net->receive(msg)) break;
            pthread_mutex_lock(&stateMutex);
            state.neighbors[msg.truckId] = msg;
            state.neighbors[msg.truckId].timestamp = time(NULL);
            pthread_mutex_unlock(&stateMutex);
        }

        PlatoonMessage myMsg;
        pthread_mutex_lock(&stateMutex);
        myMsg.truckId = state.id;
        myMsg.position = state.position;
        myMsg.speed = state.speed;
        myMsg.emergencyBrake = state.emergencyBrake;

        // --- NEW: Send Decoupled Status ---
        // Note: We are hijacking the boolean packing or adding a field.
        // For simplicity, we assume PlatoonMessage has been updated or we reuse a field.
        // *IMPORTANT*: To make this work without changing common.h and breaking Python,
        // we will pack this into the struct. See common.h note below.
        myMsg.isDecoupled = state.isDecoupled;

        myMsg.timestamp = time(NULL);
        pthread_mutex_unlock(&stateMutex);

        net->broadcast(myMsg);
        usleep(50000);
    }
    return NULL;
}

// ==========================================
// THREAD 2: INPUT (Added 'd' command)
// ==========================================
void* inputLoop(void* arg) {
    while(true) {
        char c;
        std::cin >> c;

        switch(c) {
            case 'b': // Brake
                pthread_mutex_lock(&stateMutex);
                state.emergencyBrake = !state.emergencyBrake;
                std::cout << (state.emergencyBrake ? "!!! BRAKING !!!" : ">>> RESUMING") << "\n";
                pthread_mutex_unlock(&stateMutex);
                break;

            case 'd': // NEW: Decouple (Create Gap)
                pthread_mutex_lock(&stateMutex);
                state.isDecoupled = !state.isDecoupled;
                if (state.isDecoupled)
                    std::cout << ">>> DECOUPLING (Opening Gap) <<<\n";
                else
                    std::cout << ">>> COUPLING (Closing Gap) <<<\n";
                pthread_mutex_unlock(&stateMutex);
                break;

            default: break;
        }
    }
    return NULL;
}

// ==========================================
// MAIN LOGIC LOOP (Gap Propagation)
// ==========================================
void logicLoop() {
    double dt = 0.1;

    while (true) {
        pthread_mutex_lock(&stateMutex);
        long now = time(NULL);

        // 1. CLEANUP
        for (auto it = state.neighbors.begin(); it != state.neighbors.end(); ) {
            if (now - it->second.timestamp > TIMEOUT_SEC) {
                it = state.neighbors.erase(it);
            } else {
                ++it;
            }
        }

        switch (state.id) {
            case 0: { // LEADER
                int currentSize = 1;
                for(auto const& [key, val] : state.neighbors) currentSize++;

                if (currentSize < state.targetPlatoonSize) {
                    state.speed = 0.0;
                    std::cout << "[WAITING] Found " << currentSize << "/" << state.targetPlatoonSize << "\r" << std::flush;
                }
                else if (state.emergencyBrake) {
                    state.speed = 0.0;
                }
                else {
                    if (state.speed < LEADER_FIXED_SPEED) state.speed += MAX_ACCEL * dt;
                    else state.speed = LEADER_FIXED_SPEED;
                }
                break;
            }

            default: { // FOLLOWER
                int leaderId = 0;
                if (state.neighbors.count(leaderId)) {
                    state.emergencyBrake = state.neighbors[leaderId].emergencyBrake;

                    if (state.emergencyBrake) {
                        state.speed = 0.0;
                    } else {
                        // 1. Sorting Logic
                        std::vector<TruckRank> platoon;
                        platoon.push_back({state.id, state.position, state.isDecoupled});
                        for (auto const& [nid, msg] : state.neighbors) {
                            platoon.push_back({nid, msg.position, msg.isDecoupled});
                        }

                        // Sort Descending (Furthest first)
                        std::sort(platoon.begin(), platoon.end(), [](const TruckRank& a, const TruckRank& b) {
                            return a.position > b.position;
                        });

                        int myRank = 0;
                        int decoupledCountAhead = 0;
                        int truckDirectlyAheadId = -1;

                        // 2. Calculate Rank AND Extra Gaps
                        // We must sum up the gaps of everyone ahead of us
                        for (int i = 0; i < platoon.size(); i++) {
                            if (platoon[i].id == state.id) {
                                myRank = i;
                                if (i > 0) truckDirectlyAheadId = platoon[i-1].id;
                                // Include MYOWN decoupling request in the gap calculation
                                if (state.isDecoupled) decoupledCountAhead++;
                                break;
                            }
                            // If a truck ahead is decoupled, it pushes ME back too
                            if (platoon[i].isDecoupled) {
                                decoupledCountAhead++;
                            }
                        }

                        // 3. Calculate Targets (Base Distance + Extra Decouple Distance)
                        double baseDistance = myRank * TARGET_DISTANCE;
                        double extraDistance = decoupledCountAhead * EXTRA_GAP_DISTANCE;
                        double totalTargetDistance = baseDistance + extraDistance;

                        // Leader Tracking
                        double leaderPos = state.neighbors[leaderId].position;
                        double leaderSpeed = state.neighbors[leaderId].speed;
                        double timeSinceUpdate = difftime(now, state.neighbors[leaderId].timestamp);
                        if (timeSinceUpdate >= 0.0 && timeSinceUpdate < 1.0) {
                            leaderPos += leaderSpeed * timeSinceUpdate;
                        }

                        double myTargetPos = leaderPos - totalTargetDistance;
                        double distError = myTargetPos - state.position;

                        // Control
                        double desiredSpeed;
                        if (std::abs(distError) < GAP_TOLERANCE) {
                            desiredSpeed = leaderSpeed;
                        } else {
                            desiredSpeed = leaderSpeed + (K_P * distError);
                        }

                        // Safety
                        if (truckDirectlyAheadId != -1 && state.neighbors.count(truckDirectlyAheadId)) {
                            double frontPos = state.neighbors[truckDirectlyAheadId].position;
                            if ((frontPos - state.position) < 20.0) {
                                desiredSpeed = std::min(desiredSpeed, state.neighbors[truckDirectlyAheadId].speed);
                            }
                        }

                        // Physics
                        if (desiredSpeed > MAX_SPEED) desiredSpeed = MAX_SPEED;
                        if (desiredSpeed < 0) desiredSpeed = 0;

                        if (state.speed < desiredSpeed) state.speed += std::min(desiredSpeed - state.speed, MAX_ACCEL * dt);
                        else if (state.speed > desiredSpeed) state.speed -= std::min(state.speed - desiredSpeed, MAX_BRAKE * dt);
                    }
                } else {
                    if (state.speed > 0) state.speed -= MAX_BRAKE * dt;
                    if (state.speed < 0) state.speed = 0;
                }
                break;
            }
        }

        state.position += state.speed * dt;

        // Logging
        if (state.id != 0) {
             std::cout << "[T" << state.id << "] ";
             if (state.isDecoupled) std::cout << "(DECOUPLED) ";
             std::cout << "Spd: " << (state.speed * 3.6) << " km/h";

             // Check gap
             int aheadId = -1;
             double minDiff = 99999.0;
             for (auto const& [nid, msg] : state.neighbors) {
                 double diff = msg.position - state.position;
                 if (diff > 0 && diff < minDiff) { minDiff = diff; aheadId = nid; }
             }
             if(aheadId != -1) {
                 std::cout << " | Behind T" << aheadId << ": " << std::fixed << std::setprecision(1) << minDiff << "m";
             }
             std::cout << "\n";
        }

        pthread_mutex_unlock(&stateMutex);
        usleep(100000);
    }
}

int main() {
    std::cout << "--- CACC PLATOON WITH DECOUPLING ---\n";
    std::cout << "Press 'd' to Open/Close gap for merging traffic.\n";
    std::cout << "Enter Truck ID: ";
    std::cin >> state.id;

    if (state.id == 0) {
        std::cout << "Enter Target Platoon Size: ";
        std::cin >> state.targetPlatoonSize;
    }

    // Initialize Position
    state.position = -(state.id * TARGET_DISTANCE);

    // Initialize Mutex
    if (pthread_mutex_init(&stateMutex, NULL) != 0) return 1;

    // Setup Network
    NetworkModule net(state.id);
    net.flush();

    // 1. Start Communication Thread
    pthread_t commsThreadId;
    pthread_create(&commsThreadId, NULL, commsLoop, (void*)&net);

    // 2. Start Input Thread (For EVERYONE now, not just Leader)
    // --- FIX: Removed "if (state.id == 0)" check here ---
    pthread_t inputThreadId;
    pthread_create(&inputThreadId, NULL, inputLoop, NULL);

    // 3. Start Logic Loop
    logicLoop();

    return 0;
}