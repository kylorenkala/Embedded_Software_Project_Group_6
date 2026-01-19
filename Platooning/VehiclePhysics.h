#ifndef VEHICLE_PHYSICS_H
#define VEHICLE_PHYSICS_H

#include <algorithm>
#include "common.h"

// Physics Constants
const double KMH_TO_MS = 1.0 / 3.6;
const double MAX_SPEED = 100.0 * KMH_TO_MS;
const double MAX_ACCEL = 3.0;
const double MAX_BRAKE = 5.0;

class VehiclePhysics {
private:
    double speed = 0.0;
    double position = 0.0;

public:
    VehiclePhysics(int id, double startDist) {
        position = -(id * startDist); // Initialize position based on ID
    }

    void update(double targetSpeed, double dt) {
        if (speed < targetSpeed) {
            speed += std::min(targetSpeed - speed, MAX_ACCEL * dt);
        } else if (speed > targetSpeed) {
            speed -= std::min(speed - targetSpeed, MAX_BRAKE * dt);
        }
        position += speed * dt;
    }

    // Force stop (for jamming/emergency)
    void emergencyStop(double dt) {
        if (speed > 0) speed -= MAX_BRAKE * dt;
        if (speed < 0) speed = 0;
        position += speed * dt;
    }

    double getSpeed() const { return speed; }
    double getPosition() const { return position; }
};

#endif