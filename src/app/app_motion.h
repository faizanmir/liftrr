#pragma once

#include <Arduino.h>
#include "config.h"
#include "sensors.h"

namespace liftrr {
namespace app {

struct MotionState {
    unsigned long lastMotionTime = 0;
    int16_t lastRelDist = 0;
    float lastRoll = 0.0f;
    float lastPitch = 0.0f;
    float lastYaw = 0.0f;
};

void initMotionState(MotionState &state, unsigned long nowMs);
void updateCalibrationFlags(const SensorSample &sample, bool &isCalibrated);
void enforceCalibrationModeGuard(bool isCalibrated, bool laserValid, DeviceMode &deviceMode);
void updateMotionAndMode(const RelativePose &pose,
                         unsigned long nowMs,
                         MotionState &state,
                         DeviceMode &deviceMode,
                         bool isCalibrated,
                         bool laserValid);

} // namespace app
} // namespace liftrr
