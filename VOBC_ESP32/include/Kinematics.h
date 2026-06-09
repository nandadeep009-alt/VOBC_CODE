#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "OnboardState.h"

class KinematicsEngine {
public:
    // Constructor initializes baseline tracking states
    KinematicsEngine();
    
    /**
     * @brief Computes a single-tick kinematics update for the vehicle.
     * * @param state Reference to the unified Onboard State container (Single Source of Truth)
     * @param dt_sec Delta time passed since the last loop execution, in seconds
     */
    void computeTrajectoryTick(VehicleState& state, float dt_sec);

private:
    /**
     * @brief Calculates the safe distance required to come to a complete stop.
     * Uses standard linear deceleration tracking profiles.
     */
    float calculateStoppingDistance(float currentVel, float decelerationRate);
};

#endif