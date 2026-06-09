#include "Kinematics.h"

KinematicsEngine::KinematicsEngine() {}

float KinematicsEngine::calculateStoppingDistance(float currentVel, float decelerationRate) {
    if (decelerationRate <= 0.0f) return 999.0f;
    // Classic physics braking formula: d = v^2 / (2 * a)
    return (currentVel * currentVel) / (2.0f * decelerationRate);
}

void KinematicsEngine::computeTrajectoryTick(VehicleState& state, float dt_sec) {
    // Fail-Safe Override: If an interlocking fault or emergency brake trips, halt immediately
    if (state.status == STATE_EMERGENCY_BRAKE || state.interlockingFault) {
        state.currentVelocity = 0.0f;
        return;
    }

    // 1. Calculate remaining distance vector to destination platform
    float distanceToTarget = state.limitOfAuthority - state.currentPosition;
    
    // Handle loop track wrap-around arithmetic safely (0-100 scale footprint)
    if (distanceToTarget < 0.0f) {
        distanceToTarget += 100.0f; 
    }

    // 2. Compute safety braking profile threshold distance
    float requiredStoppingDist = calculateStoppingDistance(state.currentVelocity, state.targetDeceleration);

    // 3. Trajectory Profiler State Machine Logic
    if (distanceToTarget <= 0.1f || state.limitOfAuthority == 0.0f) {
        // Target reached precisely -> Return to stationary idle hold
        state.currentVelocity = 0.0f;
        state.status = STATE_IDLE;
    } 
    else if (distanceToTarget <= requiredStoppingDist) {
        // Safe Braking Curve Intercepted -> Decelerate down to station platform target
        state.status = STATE_RUNNING;
        state.currentVelocity -= state.targetDeceleration * dt_sec;
        if (state.currentVelocity < 0.0f) state.currentVelocity = 0.0f;
    } 
    else {
        // Line Clear -> Accelerate up toward lowest ceiling speed constraint
        state.status = STATE_RUNNING;
        float targetCeiling = min(state.civilSpeedLimit, state.engineerMaxSpeed);
        
        if (state.currentVelocity < targetCeiling) {
            state.currentVelocity += state.targetAcceleration * dt_sec;
            if (state.currentVelocity > targetCeiling) state.currentVelocity = targetCeiling;
        } else if (state.currentVelocity > targetCeiling) {
            state.currentVelocity -= state.targetDeceleration * dt_sec;
            if (state.currentVelocity < targetCeiling) state.currentVelocity = targetCeiling;
        }
    }

    // 4. Dead reckoning odometer integration update
    // Simple scaling factor: mapping frequency output updates smoothly onto 0-100 track position footprint
    float positionDelta = (state.currentVelocity * 0.01f) * dt_sec; 
    state.currentPosition += positionDelta;
    
    // Maintain strict boundary normalization on circular loop track layout
    if (state.currentPosition >= 100.0f) {
        state.currentPosition -= 100.0f;
    }
}