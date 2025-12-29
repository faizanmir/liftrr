#include "app/app_motion.h"

namespace liftrr {
namespace app {

void MotionController::initMotionState(MotionState &state, unsigned long nowMs) {
    state.lastMotionTime = nowMs;
    state.lastRelDist = 0;
    state.lastRoll = 0.0f;
    state.lastPitch = 0.0f;
    state.lastYaw = 0.0f;
}

void MotionController::updateCalibrationStatus(const liftrr::sensors::SensorSample &sample,
                                               liftrr::sensors::SensorManager &sensors) {
    sensors.updateCalibrationStatus(sample);
}

void MotionController::enforceCalibrationModeGuard(const liftrr::sensors::SensorManager &sensors,
                                                   liftrr::core::RuntimeState &runtime) {
    liftrr::core::DeviceMode mode = runtime.deviceMode();
    if (mode == liftrr::core::MODE_RUN) {
        if (!sensors.isCalibrated()) {
            runtime.setDeviceMode(liftrr::core::MODE_CALIBRATE);
        }
    } else if (mode == liftrr::core::MODE_CALIBRATE) {
        if (sensors.isCalibrated() && sensors.laserValid()) {
            runtime.setDeviceMode(liftrr::core::MODE_RUN);
        }
    }
}

void MotionController::updateMotionAndMode(const liftrr::sensors::RelativePose &pose,
                                           unsigned long nowMs,
                                           MotionState &state,
                                           liftrr::core::RuntimeState &runtime) {

  // 1. Detect significant motion
  bool moved = false;
  
  // Thresholds: 15mm distance, 5 degrees rotation
  if (abs(pose.relDist - state.lastRelDist) > 15) moved = true;
  if (abs(pose.relRoll - state.lastRoll) > 5.0f)  moved = true;
  if (abs(pose.relPitch - state.lastPitch) > 5.0f) moved = true;
  if (abs(pose.relYaw - state.lastYaw) > 5.0f)    moved = true;

  if (moved) {
    state.lastMotionTime = nowMs;
    
    // --- ADDED LOGIC: Wake up from IDLE ---
    if (runtime.deviceMode() == liftrr::core::MODE_IDLE) {
      runtime.setDeviceMode(liftrr::core::MODE_RUN);
    }
    // --------------------------------------

    state.lastRelDist = pose.relDist;
    state.lastRoll = pose.relRoll;
    state.lastPitch = pose.relPitch;
    state.lastYaw = pose.relYaw;
  }

  // 2. Auto-IDLE Timeout
  const unsigned long IDLE_TIMEOUT_MS = 30000;

  if (runtime.deviceMode() == liftrr::core::MODE_RUN) {
    if (nowMs - state.lastMotionTime > IDLE_TIMEOUT_MS) {
      runtime.setDeviceMode(liftrr::core::MODE_IDLE);
    }
  }
}

} // namespace app
} // namespace liftrr
