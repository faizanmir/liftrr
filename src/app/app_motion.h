#pragma once

#include <Arduino.h>
#include "core/config.h"
#include "sensors/sensors.h"

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
void updateCalibrationFlags(const liftrr::sensors::SensorSample &sample, bool &isCalibrated);
void enforceCalibrationModeGuard(bool isCalibrated,
                                 bool laserValid,
                                 liftrr::core::DeviceMode &deviceMode);
void updateMotionAndMode(const liftrr::sensors::RelativePose &pose,
                         unsigned long nowMs,
                         MotionState &state,
                         liftrr::core::DeviceMode &deviceMode,
                         bool isCalibrated,
                         bool laserValid);

} // namespace app
} // namespace liftrr
