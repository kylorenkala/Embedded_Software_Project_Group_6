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
const double EXTRA_GAP_DISTANCE = 30.0;
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
    bool isDecoupled = false;
    bool isJamming = false; // <--- NEW: Simulate Comm Failure
    std::map<int, PlatoonMessage> neighbors;
};

TruckState state;
pthread_mutex_t stateMutex;

struct TruckRank {
    int id;
    double position;
    bool isDecoupled;
};

// ==========================================
// THREAD 1: COMMUNICATION (With Failure Sim)
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

        // --- NEW: ONLY BROADCAST IF RADIO IS WORKING ---
        bool currentlyJamming = false;

        PlatoonMessage myMsg;
        pthread_mutex_lock(&stateMutex);
        currentlyJamming = state.isJamming; // Check status

        myMsg.truckId = state.id;
        myMsg.position = state.position;
        myMsg.speed = state.speed;
        myMsg.emergencyBrake = state.emergencyBrake;
        myMsg.isDecoupled = state.isDecoupled;
        myMsg.timestamp = time(NULL);
        pthread_mutex_unlock(&stateMutex);

        if (!currentlyJamming) {
            net->broadcast(myMsg);
        }
        // If jamming, we simply SKIP the broadcast.
        // Other trucks will think we disappeared.

        usleep(50000);
    }
    return NULL;
}

// ==========================================
// THREAD 2: INPUT (Added 'j')
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

            case 'd': // Decouple
                pthread_mutex_lock(&stateMutex);
                state.isDecoupled = !state.isDecoupled;
                std::cout << (state.isDecoupled ? ">>> DECOUPLING" : ">>> COUPLING") << "\n";
                pthread_mutex_unlock(&stateMutex);
                break;

            case 'j': // NEW: Simulate Comm Failure
                pthread_mutex_lock(&stateMutex);
                state.isJamming = !state.isJamming;
                if (state.isJamming)
                    std::cout << ">>> RADIO FAILURE SIMULATED (Jamming) <<<\n";
                else
                    std::cout << ">>> RADIO RESTORED <<<\n";
                pthread_mutex_unlock(&stateMutex);
                break;

            default: break;
        }
    }
    return NULL;
}

// ==========================================
// MAIN LOGIC LOOP
// ==========================================
void logicLoop() {
    double dt = 0.1;

    while (true) {
        pthread_mutex_lock(&stateMutex);
        long now = time(NULL);

        // 1. CLEANUP (The robust part: Remove trucks that stop talking)
        for (auto it = state.neighbors.begin(); it != state.neighbors.end(); ) {
            if (now - it->second.timestamp > TIMEOUT_SEC) {
                // Determine if this is a "Communication Failure" handling
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
                        // Sorting & Rank Logic
                        std::vector<TruckRank> platoon;
                        platoon.push_back({state.id, state.position, state.isDecoupled});
                        for (auto const& [nid, msg] : state.neighbors) {
                            platoon.push_back({nid, msg.position, msg.isDecoupled});
                        }

                        std::sort(platoon.begin(), platoon.end(), [](const TruckRank& a, const TruckRank& b) {
                            return a.position > b.position;
                        });

                        int myRank = 0;
                        int decoupledCountAhead = 0;
                        int truckDirectlyAheadId = -1;

                        for (int i = 0; i < platoon.size(); i++) {
                            if (platoon[i].id == state.id) {
                                myRank = i;
                                if (i > 0) truckDirectlyAheadId = platoon[i-1].id;
                                if (state.isDecoupled) decoupledCountAhead++;
                                break;
                            }
                            if (platoon[i].isDecoupled) decoupledCountAhead++;
                        }

                        double baseDistance = myRank * TARGET_DISTANCE;
                        double extraDistance = decoupledCountAhead * EXTRA_GAP_DISTANCE;
                        double totalTargetDistance = baseDistance + extraDistance;

                        // Dead Reckoning (Stability during minor packet loss)
                        double leaderPos = state.neighbors[leaderId].position;
                        double leaderSpeed = state.neighbors[leaderId].speed;
                        double timeSinceUpdate = difftime(now, state.neighbors[leaderId].timestamp);
                        if (timeSinceUpdate >= 0.0 && timeSinceUpdate < 1.0) {
                            leaderPos += leaderSpeed * timeSinceUpdate;
                        }

                        double myTargetPos = leaderPos - totalTargetDistance;
                        double distError = myTargetPos - state.position;

                        double desiredSpeed;
                        if (std::abs(distError) < GAP_TOLERANCE) desiredSpeed = leaderSpeed;
                        else desiredSpeed = leaderSpeed + (K_P * distError);

                        if (truckDirectlyAheadId != -1 && state.neighbors.count(truckDirectlyAheadId)) {
                            double frontPos = state.neighbors[truckDirectlyAheadId].position;
                            if ((frontPos - state.position) < 20.0) {
                                desiredSpeed = std::min(desiredSpeed, state.neighbors[truckDirectlyAheadId].speed);
                            }
                        }

                        if (desiredSpeed > MAX_SPEED) desiredSpeed = MAX_SPEED;
                        if (desiredSpeed < 0) desiredSpeed = 0;

                        if (state.speed < desiredSpeed) state.speed += std::min(desiredSpeed - state.speed, MAX_ACCEL * dt);
                        else if (state.speed > desiredSpeed) state.speed -= std::min(state.speed - desiredSpeed, MAX_BRAKE * dt);
                    }
                } else {
                    // --- FAIL SAFE ---
                    // "I have lost the leader. It is not safe to drive."
                    // Decelerate to 0.
                    if (state.speed > 0) state.speed -= MAX_BRAKE * dt;
                    if (state.speed < 0) state.speed = 0;
                }
                break;
            }
        }

        state.position += state.speed * dt;

        if (state.id != 0) {
             std::cout << "[T" << state.id << "] ";
             if (state.isJamming) std::cout << "(NO SIGNAL) ";
             std::cout << "Spd: " << (state.speed * 3.6) << " km/h\n";
        }

        pthread_mutex_unlock(&stateMutex);
        usleep(100000);
    }
}

int main() {
    std::cout << "--- ROBUST PLATOON SYSTEM ---\n";
    std::cout << "Keys: 'b'=Brake, 'd'=Decouple, 'j'=Simulate Failure\n";
    std::cout << "Enter Truck ID: ";
    std::cin >> state.id;

    if (state.id == 0) {
        std::cout << "Enter Target Platoon Size: ";
        std::cin >> state.targetPlatoonSize;
    }

    state.position = -(state.id * TARGET_DISTANCE);

    if (pthread_mutex_init(&stateMutex, NULL) != 0) return 1;

    NetworkModule net(state.id);
    net.flush();

    pthread_t commsThreadId;
    pthread_create(&commsThreadId, NULL, commsLoop, (void*)&net);

    pthread_t inputThreadId;
    pthread_create(&inputThreadId, NULL, inputLoop, NULL);

    logicLoop();
    return 0;
}