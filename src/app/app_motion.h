#pragma once

#include <Arduino.h>
#include "core/config.h"
#include "sensors/sensors.h"
#include "core/globals.h"

namespace liftrr {
namespace app {

struct MotionState {
    unsigned long lastMotionTime = 0;
    int16_t lastRelDist = 0;
    float lastRoll = 0.0f;
    float lastPitch = 0.0f;
    float lastYaw = 0.0f;
};

class MotionController {
public:
    void initMotionState(MotionState &state, unsigned long nowMs);
    void updateCalibrationStatus(const liftrr::sensors::SensorSample &sample,
                                 liftrr::sensors::SensorManager &sensors);
    void enforceCalibrationModeGuard(const liftrr::sensors::SensorManager &sensors,
                                     liftrr::core::RuntimeState &runtime);
    void updateMotionAndMode(const liftrr::sensors::RelativePose &pose,
                             unsigned long nowMs,
                             MotionState &state,
                             liftrr::core::RuntimeState &runtime);
};

} // namespace app
} // namespace liftrr
