#ifndef PLATOON_CONTROLLER_H
#define PLATOON_CONTROLLER_H

#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include "common.h"

// --- CONTROLLER CONSTANTS ---
// Leader cruises at 50 km/h (approx 13.8 m/s)
const double LEADER_FIXED_SPEED = 50.0 * (1.0/3.6);
const double TARGET_DISTANCE = 30.0;     // Standard gap
const double EXTRA_GAP_DISTANCE = 30.0;  // Extra gap when Decoupled
const double K_P = 1.0;                  // Proportional Gain for PID
const double GAP_TOLERANCE = 1.0;        // 1m deadband to prevent jitter
const double MAX_SPEED_LIMIT = 100.0 * (1.0/3.6); // Hard limit 100 km/h

// Physics limits for safe braking calculation (Must match VehiclePhysics)
const double PHYS_MAX_BRAKE = 5.0;

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
                                double myCurrentSpeed, // Input needed for stopping dist calc
                                const std::map<int, PlatoonMessage>& neighbors,
                                bool isDecoupled,
                                bool emergencyBrake,
                                int targetPlatoonSize)
    {
        // 1. SELF-SAFETY OVERRIDE
        // If I am braking manually, return 0 immediately.
        if (emergencyBrake) return 0.0;

        // 2. LEADER STARTUP LOGIC
        if (myId == 0) {
            int currentSize = 1 + neighbors.size();
            if (currentSize < targetPlatoonSize) {
                // Wait for the full team before moving
                 // (Optional: Un-comment to see waiting status)
                 // std::cout << "[WAITING] Found " << currentSize << "/" << targetPlatoonSize << "\r" << std::flush;
                return 0.0;
            }
        }

        // 3. GLOBAL EMERGENCY BRAKE (The "Digital Tow Bar")
        // If ANY truck in the platoon is braking, everyone must stop.
        for (const auto& kv : neighbors) {
            if (kv.second.emergencyBrake) {
                if (myId == 0) {
                    std::cout << " [LEADER] Global Emergency Stop triggered by T" << kv.first << "\r" << std::flush;
                }
                return 0.0;
            }
        }

        // 4. SORT PLATOON (Who is where?)
        std::vector<RankInfo> platoon;
        platoon.push_back({myId, myPos, isDecoupled});
        for (const auto& kv : neighbors) {
            platoon.push_back({kv.first, kv.second.position, kv.second.isDecoupled});
        }

        // Sort descending: Largest Position (Front) to Smallest Position (Back)
        std::sort(platoon.begin(), platoon.end(), [](const RankInfo& a, const RankInfo& b) {
            return a.position > b.position;
        });

        // 5. IDENTIFY RANK & GAPS
        int myRank = -1;
        int myIndex = -1;
        int extraGaps = 0;

        for (int i = 0; i < platoon.size(); i++) {
            if (platoon[i].id == myId) {
                myRank = i; // Rank 0 = Leader, Rank 1 = 2nd truck...
                myIndex = i;
                if (isDecoupled) extraGaps++;
                break;
            }
            if (platoon[i].isDecoupled) extraGaps++; // Add space for decoupled trucks ahead
        }

        // 6. REAR REGROUPING CHECK (Lagging Protection)
        // Check the truck physically BEHIND me.
        if (myIndex != -1 && (myIndex + 1) < platoon.size()) {
            double rearPos = platoon[myIndex + 1].position;
            double rearGap = myPos - rearPos;

            // If the follower is lost (> 300m), wait for them to catch up.
            // This is for non-emergency lagging only.
            if (rearGap > 300.0) {
                 if (myId == 0) std::cout << " [LEADER] Waiting for platoon (Rear Gap: " << (int)rearGap << "m)\r" << std::flush;
                 return 0.0;
            }
        }

        // If I am Leader, and no one is braking or lagging badly, I cruise.
        if (myId == 0) return LEADER_FIXED_SPEED;

        // 7. FOLLOWER LOGIC: FIND TARGET
        int leaderId = platoon[0].id;
        // Safety: If I think I'm following myself or leader is missing, stop.
        if (leaderId == myId) return 0.0;
        if (neighbors.find(leaderId) == neighbors.end()) return 0.0;

        const auto& leaderMsg = neighbors.at(leaderId);

        // 8. PID CONTROL (Distance Keeping)
        double desiredDist = (myRank * TARGET_DISTANCE) + (extraGaps * EXTRA_GAP_DISTANCE);

        // Dead Reckoning: Predict where the leader is NOW, not 50ms ago.
        double leaderPos = leaderMsg.position;
        double timeSinceUpdate = difftime(time(nullptr), leaderMsg.timestamp);
        if (timeSinceUpdate > 0 && timeSinceUpdate < 1.0) {
            leaderPos += leaderMsg.speed * timeSinceUpdate;
        }

        double targetPos = leaderPos - desiredDist;
        double error = targetPos - myPos;
        double desiredSpeed = leaderMsg.speed;

        // Apply Proportional Control
        if (std::abs(error) > GAP_TOLERANCE) {
            desiredSpeed += (K_P * error);
        }

        // 9. PHYSICS-AWARE ANTI-COLLISION (The Crash Fix)
        // Look at the truck immediately in front of me.
        int truckAheadId = (myRank > 0) ? platoon[myRank - 1].id : -1;

        if (truckAheadId != -1 && neighbors.count(truckAheadId)) {
            double distToFront = neighbors.at(truckAheadId).position - myPos;

            // Calculate my Minimum Stopping Distance: d = v^2 / 2a
            double myStoppingDist = (myCurrentSpeed * myCurrentSpeed) / (2.0 * PHYS_MAX_BRAKE);

            // Safety Buffer: Always keep 10m extra
            double safeLimit = myStoppingDist + 10.0;

            // CRITICAL CHECK:
            // If the gap is smaller than what I need to stop, I must BRAKE NOW.
            if (distToFront < safeLimit) {
                return 0.0;
            }

            // Proximity Speed Cap:
            // If within 30m, do not exceed the speed of the truck ahead.
            if (distToFront < 30.0) {
                 desiredSpeed = std::min(desiredSpeed, neighbors.at(truckAheadId).speed);
            }
        }

        // 10. FINAL CLAMP
        return std::max(0.0, std::min(desiredSpeed, MAX_SPEED_LIMIT));
    }
};

#endif