#ifndef PLATOON_CONTROLLER_H
#define PLATOON_CONTROLLER_H

#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include "common.h"

// --- CONTROLLER CONSTANTS ---
const double TARGET_DISTANCE = 30.0;     // Standard gap in meters
const double EXTRA_GAP_DISTANCE = 30.0;  // Additional gap when decoupling (Total 60m)
const double K_P = 1.0;                  // Proportional Gain for PID
const double GAP_TOLERANCE = 1.0;        // Deadband to prevent jitter
const double PHYS_MAX_BRAKE = 5.0;       // Must match VehiclePhysics.h

class PlatoonController {
public:
    /**
     * Calculates the desired speed for the truck based on the platoon state.
     * Uses Dynamic Predecessor Discovery to handle node failures/deletions.
     */
    double calculateTargetSpeed(int myId,
                                double myPos,
                                double myCurrentSpeed,
                                const std::map<int, PlatoonMessage>& neighbors,
                                bool isDecoupled,
                                bool emergencyBrake,
                                int targetPlatoonSize)
    {
        // 1. SAFETY OVERRIDE: Emergency Brake
        if (emergencyBrake) {
            return 0.0;
        }

        // 2. LEADER LOGIC (ID 0)
        // The leader simply maintains a cruise speed of 50 km/h.
        if (myId == 0) {
            return 50.0 / 3.6; // ~13.8 m/s
        }

        // 3. DYNAMIC PREDECESSOR DISCOVERY
        // Instead of hardcoding (myId - 1), we scan for the closest truck
        // that is physically AHEAD of us.
        int predecessorId = -1;
        double minDistance = std::numeric_limits<double>::max();

        for (const auto& [id, msg] : neighbors) {
            // Skip myself
            if (id == myId) continue;

            double diff = msg.position - myPos;

            // Check if this truck is AHEAD (diff > 0) and is the CLOSEST one found so far
            if (diff > 0 && diff < minDistance) {
                minDistance = diff;
                predecessorId = id;
            }
        }

        // 4. FALLBACK: NO PREDECESSOR FOUND
        // If I am not the leader, but I see no one in front of me,
        // it usually means I am lost or the leader hasn't started broadcasting yet.
        // Safety behavior: Stop.
        if (predecessorId == -1) {
            return 0.0;
        }

        // 5. PID CONTROL LOGIC
        // We track the predecessor found above.
        const PlatoonMessage& leaderMsg = neighbors.at(predecessorId);

        // Determine Target Gap
        double desiredDist = TARGET_DISTANCE;
        if (isDecoupled) {
            desiredDist += EXTRA_GAP_DISTANCE; // Target 60m instead of 30m
        }

        // Calculate Error
        // minDistance is the actual gap. desiredDist is what we want.
        double error = minDistance - desiredDist;
        double targetSpeed = leaderMsg.speed;

        // Apply Proportional Control (P-Controller)
        // If error > 0 (too far), we speed up.
        // If error < 0 (too close), we slow down relative to leader.
        if (std::abs(error) > GAP_TOLERANCE) {
            targetSpeed += (K_P * error);
        }

        // 6. PHYSICS-AWARE ANTI-COLLISION (Safety Layer)
        // Calculate stopping distances to ensure we don't crash even if PID wants to speed up.
        // Formula: d = v^2 / 2a
        double myStoppingDist = (myCurrentSpeed * myCurrentSpeed) / (2.0 * PHYS_MAX_BRAKE);

        // We add a 5-meter safety buffer
        double safeLimit = myStoppingDist + 5.0;

        // If the actual gap is smaller than my safe stopping distance, FORCE BRAKE.
        if (minDistance < safeLimit) {
            return 0.0;
        }

        // Ensure target speed is never negative
        if (targetSpeed < 0) targetSpeed = 0;

        return targetSpeed;
    }
};

#endif