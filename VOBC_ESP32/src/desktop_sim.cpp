#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

enum MotionStatus
{
    STATE_IDLE,
    STATE_RUNNING,
    STATE_EMERGENCY_BRAKE
};

struct VehicleState
{
    float currentPosition = 0.0f;
    float currentVelocity = 0.0f;
    float limitOfAuthority = 0.0f;
    float civilSpeedLimit = 0.0f;
    float engineerMaxSpeed = 0.0f;
    float targetAcceleration = 25.0f;
    float targetDeceleration = 20.0f;
    float stationDwellTime = 30.0f;
    MotionStatus status = STATE_IDLE;
    unsigned long lastHeartbeatTime = 0;
    bool interlockingFault = false;
};

class KinematicsEngine
{
public:
    KinematicsEngine() {}
    void computeTrajectoryTick(VehicleState &state, float dt_sec)
    {
        if (state.status == STATE_EMERGENCY_BRAKE || state.interlockingFault)
        {
            state.currentVelocity = 0.0f;
            return;
        }

        float distanceToTarget = state.limitOfAuthority - state.currentPosition;
        if (distanceToTarget < 0.0f)
            distanceToTarget += 100.0f;

        float requiredStoppingDist = calculateStoppingDistance(state.currentVelocity, state.targetDeceleration);

        if (distanceToTarget <= 0.1f || state.limitOfAuthority == 0.0f)
        {
            state.currentVelocity = 0.0f;
            state.status = STATE_IDLE;
        }
        else if (distanceToTarget <= requiredStoppingDist)
        {
            state.status = STATE_RUNNING;
            state.currentVelocity -= state.targetDeceleration * dt_sec;
            if (state.currentVelocity < 0.0f)
                state.currentVelocity = 0.0f;
        }
        else
        {
            state.status = STATE_RUNNING;
            float targetCeiling = std::min(state.civilSpeedLimit, state.engineerMaxSpeed);
            if (state.currentVelocity < targetCeiling)
            {
                state.currentVelocity += state.targetAcceleration * dt_sec;
                if (state.currentVelocity > targetCeiling)
                    state.currentVelocity = targetCeiling;
            }
            else if (state.currentVelocity > targetCeiling)
            {
                state.currentVelocity -= state.targetDeceleration * dt_sec;
                if (state.currentVelocity < targetCeiling)
                    state.currentVelocity = targetCeiling;
            }
        }

        float positionDelta = (state.currentVelocity * 0.01f) * dt_sec;
        state.currentPosition += positionDelta;
        if (state.currentPosition >= 100.0f)
            state.currentPosition -= 100.0f;
    }

private:
    float calculateStoppingDistance(float currentVel, float decelerationRate)
    {
        if (decelerationRate <= 0.0f)
            return 999.0f;
        return (currentVel * currentVel) / (2.0f * decelerationRate);
    }
};

int main()
{
    VehicleState state;
    KinematicsEngine engine;

    // Initialize test scenario
    state.currentPosition = 0.0f;   // start at zero
    state.currentVelocity = 0.0f;   // standstill
    state.limitOfAuthority = 50.0f; // target at 50 units (0-100 loop)
    state.civilSpeedLimit = 120.0f; // Hz-like units
    state.engineerMaxSpeed = 100.0f;
    state.targetAcceleration = 10.0f; // Hz/s
    state.targetDeceleration = 20.0f; // Hz/s

    const float simDurationSec = 20.0f;
    const float dt = 0.1f; // 100 ms steps
    int steps = static_cast<int>(simDurationSec / dt);

    std::cout << "t(s),position,velocity,status\n";
    for (int i = 0; i <= steps; ++i)
    {
        float t = i * dt;
        engine.computeTrajectoryTick(state, dt);
        const char *statusStr = (state.status == STATE_IDLE) ? "IDLE" : (state.status == STATE_RUNNING) ? "RUNNING"
                                                                                                        : "EMERGENCY";
        std::cout << t << "," << state.currentPosition << "," << state.currentVelocity << "," << statusStr << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
