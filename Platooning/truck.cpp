#include <iostream>
#include <pthread.h>
#include <cmath>
#include <vector>
#include <algorithm> // For std::sort
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
    std::map<int, PlatoonMessage> neighbors;
};

TruckState state;
pthread_mutex_t stateMutex;

// Helper struct for sorting
struct TruckRank {
    int id;
    double position;
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
        myMsg.timestamp = time(NULL);
        pthread_mutex_unlock(&stateMutex);

        net->broadcast(myMsg);
        usleep(50000);
    }
    return NULL;
}

// ==========================================
// THREAD 2: INPUT
// ==========================================
void* inputLoop(void* arg) {
    while(true) {
        char c; std::cin >> c;
        if (c == 'b') {
            pthread_mutex_lock(&stateMutex);
            state.emergencyBrake = !state.emergencyBrake;
            std::cout << (state.emergencyBrake ? "!!! BRAKING !!!" : ">>> RESUMING") << "\n";
            pthread_mutex_unlock(&stateMutex);
        }
    }
    return NULL;
}

// ==========================================
// MAIN LOGIC LOOP (Position-Based Sorting)
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

        // 2. LEADER LOGIC
        if (state.id == 0) {
            int currentSize = 1;
            for(auto const& [key, val] : state.neighbors) currentSize++;

            if (currentSize < state.targetPlatoonSize) {
                state.speed = 0.0;
                std::cout << "[WAITING] Found " << currentSize << "/" << state.targetPlatoonSize << " trucks...\r" << std::flush;
            }
            else if (state.emergencyBrake) {
                state.speed = 0.0;
            }
            else {
                if (state.speed < LEADER_FIXED_SPEED) state.speed += MAX_ACCEL * dt;
                else state.speed = LEADER_FIXED_SPEED;
            }
        }

        // 3. FOLLOWER LOGIC
        else {
            int leaderId = 0;
            if (state.neighbors.count(leaderId)) {
                state.emergencyBrake = state.neighbors[leaderId].emergencyBrake;

                if (state.emergencyBrake) {
                    state.speed = 0.0;
                } else {
                    // --- NEW SORTING LOGIC ---
                    // 1. Create a list of EVERYONE (Me + Neighbors)
                    std::vector<TruckRank> platoon;
                    platoon.push_back({state.id, state.position}); // Add Myself
                    for (auto const& [nid, msg] : state.neighbors) {
                        platoon.push_back({nid, msg.position});
                    }

                    // 2. Sort by POSITION (Descending: Largest/Furthest Ahead -> Smallest/Last)
                    std::sort(platoon.begin(), platoon.end(), [](const TruckRank& a, const TruckRank& b) {
                        return a.position > b.position;
                    });

                    // 3. Find My Rank (Index in the list)
                    int myRank = 0;
                    int truckDirectlyAheadId = -1;

                    for (int i = 0; i < platoon.size(); i++) {
                        if (platoon[i].id == state.id) {
                            myRank = i;
                            if (i > 0) truckDirectlyAheadId = platoon[i-1].id;
                            break;
                        }
                    }

                    // Rank 0 = Leader (0m target)
                    // Rank 1 = 1st Follower (30m target)
                    // Rank 2 = 2nd Follower (60m target)
                    double myTargetDistance = myRank * TARGET_DISTANCE;

                    // Leader Tracking
                    double leaderPos = state.neighbors[leaderId].position;
                    double leaderSpeed = state.neighbors[leaderId].speed;
                    double timeSinceUpdate = difftime(now, state.neighbors[leaderId].timestamp);
                    if (timeSinceUpdate >= 0.0 && timeSinceUpdate < 1.0) {
                        leaderPos += leaderSpeed * timeSinceUpdate;
                    }

                    double myTargetPos = leaderPos - myTargetDistance;
                    double distError = myTargetPos - state.position;

                    // Control (Deadband)
                    double desiredSpeed;
                    if (std::abs(distError) < GAP_TOLERANCE) {
                        desiredSpeed = leaderSpeed;
                    } else {
                        desiredSpeed = leaderSpeed + (K_P * distError);
                    }

                    // Collision Avoidance (Use truckDirectlyAheadId determined by sort)
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
        }

        state.position += state.speed * dt;

        // Logging
        if (state.id != 0) {
             std::cout << "[T" << state.id << "] Spd: " << (state.speed * 3.6) << " km/h";
             // Find who is ahead physically
             int aheadId = -1;
             // (We could reuse the sort result, but for logging simplicity we iterate)
             double minDiff = 99999.0;
             for (auto const& [nid, msg] : state.neighbors) {
                 double diff = msg.position - state.position;
                 if (diff > 0 && diff < minDiff) { minDiff = diff; aheadId = nid; }
             }

             if(aheadId != -1) {
                 std::cout << " | Behind T" << aheadId << " (" << std::fixed << std::setprecision(1) << minDiff << "m)";
             }
             std::cout << "\n";
        }

        pthread_mutex_unlock(&stateMutex);
        usleep(100000);
    }
}

int main() {
    std::cout << "--- POSITION-BASED RANKING PLATOON ---\n";
    std::cout << "Enter Truck ID: ";
    std::cin >> state.id;

    if (state.id == 0) {
        std::cout << "Enter Target Platoon Size: ";
        std::cin >> state.targetPlatoonSize;
    }

    // Always start somewhat separated so they sort correctly immediately
    state.position = -(state.id * TARGET_DISTANCE);

    if (pthread_mutex_init(&stateMutex, NULL) != 0) return 1;

    NetworkModule net(state.id);
    net.flush();

    pthread_t commsThreadId;
    pthread_create(&commsThreadId, NULL, commsLoop, (void*)&net);

    if (state.id == 0) {
        pthread_t inputThreadId;
        pthread_create(&inputThreadId, NULL, inputLoop, NULL);
    }

    logicLoop();
    return 0;
}