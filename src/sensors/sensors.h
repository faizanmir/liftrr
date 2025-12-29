#pragma once

#include <Arduino.h>
#include <Adafruit_BNO055.h>

namespace liftrr {
namespace sensors {

// Raw IMU + laser sample.
struct SensorSample {
    sensors_event_t event;  // orientation etc. from BNO055

    uint8_t s = 0;          // system calibration status
    uint8_t g = 0;          // gyro calibration status
    uint8_t a = 0;          // accel calibration status
    uint8_t m = 0;          // mag calibration status

    int16_t rawDist = 0;    // latest raw distance reading (mm)
};

// Pose relative to calibration offsets.
struct RelativePose {
    int16_t relDist;   // mm
    float   relRoll;   // deg
    float   relPitch;  // deg
    float   relYaw;    // deg (normalized to [-180, 180])
};

enum DeviceFacing {
    FACING_UP,
    FACING_DOWN,
    FACING_LEFT,
    FACING_RIGHT,
};

// Initialize sensors (assumes Wire.begin).
void sensorsInit();

// Read IMU + laser into a SensorSample.
void sensorsRead(SensorSample& out);

// Compute pose relative to offsets.
void sensorsComputePose(const SensorSample& sample, RelativePose& out);

// Coarse facing direction from roll/pitch.
DeviceFacing sensorsFacingDirection(const RelativePose& pose);

} // namespace sensors
} // namespace liftrr
