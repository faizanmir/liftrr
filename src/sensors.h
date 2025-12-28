#pragma once

#include <Arduino.h>
#include <Adafruit_BNO055.h>  // for sensors_event_t

// Raw sample from IMU + laser.
struct SensorSample {
    sensors_event_t event;  // orientation etc. from BNO055

    uint8_t s = 0;          // system calibration status
    uint8_t g = 0;          // gyro calibration status
    uint8_t a = 0;          // accel calibration status
    uint8_t m = 0;          // mag calibration status

    int16_t rawDist = 0;    // latest raw distance reading (mm)
};

// Pose relative to current calibration offsets.
struct RelativePose {
    int16_t relDist;   // mm
    float   relRoll;   // deg
    float   relPitch;  // deg
    float   relYaw;    // deg (normalized to [-180, 180])
};

// Initialize all sensors (IMU + laser).
// Assumes Wire.begin(...) was already called in setup().
void sensorsInit();

// Read IMU + laser into a SensorSample.
// Updates global 'distance' and 'laserValid' (from globals.h) as side effects.
void sensorsRead(SensorSample& out);

// Compute pose relative to current calibration offsets
// (laserOffset, rollOffset, pitchOffset, yawOffset from globals.h).
void sensorsComputePose(const SensorSample& sample, RelativePose& out);