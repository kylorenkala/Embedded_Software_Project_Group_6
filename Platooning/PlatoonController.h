#ifndef PLATOON_CONTROLLER_H
#define PLATOON_CONTROLLER_H

#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include "common.h"

// Controller Constants
const double LEADER_FIXED_SPEED = 50.0 * (1.0/3.6);
const double TARGET_DISTANCE = 30.0;
const double EXTRA_GAP_DISTANCE = 30.0;
const double K_P = 1.0;
const double GAP_TOLERANCE = 1.0;

class PlatoonController {
private:
    struct RankInfo {
        int id;
        double position;
        bool isDecoupled;
    };

public:
    double calculateTargetSpeed(int myId, 
                                double myPos, 
                                const std::map<int, PlatoonMessage>& neighbors, 
                                bool isDecoupled, 
                                bool emergencyBrake, 
                                int targetPlatoonSize) 
    {
        // 1. SAFETY OVERRIDE
        if (emergencyBrake) return 0.0;

        // 2. LEADER LOGIC
        if (myId == 0) {
            int currentSize = 1 + neighbors.size();
            if (currentSize < targetPlatoonSize) {
                std::cout << "[WAITING] Found " << currentSize << "/" << targetPlatoonSize << "\r" << std::flush;
                return 0.0;
            }
            return LEADER_FIXED_SPEED;
        }

        // 3. FOLLOWER LOGIC (Sorting)
        std::vector<RankInfo> platoon;
        platoon.push_back({myId, myPos, isDecoupled});
        for (const auto& kv : neighbors) {
            platoon.push_back({kv.first, kv.second.position, kv.second.isDecoupled});
        }
        
        // Sort: Furthest ahead first
        std::sort(platoon.begin(), platoon.end(), [](const RankInfo& a, const RankInfo& b) {
            return a.position > b.position;
        });

        // 4. Calculate Rank & Gaps
        int myRank = 0;
        int extraGaps = 0;
        
        for (int i = 0; i < platoon.size(); i++) {
            if (platoon[i].id == myId) {
                myRank = i;
                if (isDecoupled) extraGaps++; // I want a gap
                break;
            }
            if (platoon[i].isDecoupled) extraGaps++; // Someone ahead wants a gap
        }

        // 5. Get Leader Data (Rank 0)
        int leaderId = platoon[0].id;
        if (leaderId == myId) return 0.0; // I am the accidental leader? Stop.
        if (neighbors.find(leaderId) == neighbors.end()) return 0.0; // Leader lost

        // 6. Global Brake Check
        const auto& leaderMsg = neighbors.at(leaderId);
        if (leaderMsg.emergencyBrake) return 0.0;

        // 7. PID Control
        double desiredDist = (myRank * TARGET_DISTANCE) + (extraGaps * EXTRA_GAP_DISTANCE);
        
        // Dead Reckoning (Physics Prediction)
        double leaderPos = leaderMsg.position;
        double timeSinceUpdate = difftime(time(nullptr), leaderMsg.timestamp);
        if (timeSinceUpdate > 0 && timeSinceUpdate < 1.0) {
            leaderPos += leaderMsg.speed * timeSinceUpdate;
        }

        double targetPos = leaderPos - desiredDist;
        double error = targetPos - myPos;

        double desiredSpeed = leaderMsg.speed;
        if (std::abs(error) > GAP_TOLERANCE) {
            desiredSpeed += (K_P * error);
        }

        // 8. Anti-Collision (Front Truck Check)
        int truckAheadId = (myRank > 0) ? platoon[myRank - 1].id : -1;
        if (truckAheadId != -1 && neighbors.count(truckAheadId)) {
            if ((neighbors.at(truckAheadId).position - myPos) < 20.0) {
                desiredSpeed = std::min(desiredSpeed, neighbors.at(truckAheadId).speed);
            }
        }

        return std::max(0.0, std::min(desiredSpeed, MAX_SPEED));
    }
};

#endif