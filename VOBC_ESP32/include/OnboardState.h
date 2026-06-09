#ifndef ONBOARD_STATE_H
#define ONBOARD_STATE_H

#include <Arduino.h>

enum MotionStatus {
    STATE_IDLE,
    STATE_RUNNING,
    STATE_EMERGENCY_BRAKE
};

struct VehicleState {
    // Live Telemetry Registers
    float currentPosition = 0.0;    // Train odometer position (0-100)
    float currentVelocity = 0.0;    // Real-time output frequency (Hz)
    
    // Wayside Constraints (Received via MQTT)
    float limitOfAuthority = 0.0;   
    float civilSpeedLimit = 0.0;    
    float engineerMaxSpeed = 0.0;   
    float targetAcceleration = 25.0; // Dynamic ramp rate (Hz/s)
    float targetDeceleration = 20.0; // Dynamic braking rate (Hz/s)
    float stationDwellTime = 30.0;  
    
    // System Performance Trackers
    MotionStatus status = STATE_IDLE;
    uint8_t consecutiveModbusErrors = 0;
    unsigned long lastHeartbeatTime = 0;
    bool interlockingFault = false;
};

#endif