#include "app/app_motion.h"
#include "globals.h"

namespace liftrr {
namespace app {

void initMotionState(MotionState &state, unsigned long nowMs) {
  state.lastMotionTime = nowMs;
  state.lastRelDist = 0;
  state.lastRoll = 0.0f;
  state.lastPitch = 0.0f;
  state.lastYaw = 0.0f;
}

void updateCalibrationFlags(const SensorSample &sample, bool &isCalibrated) {
  // Require system calibration >= 2 for "calibrated" status (0-3 scale)
  // 3 is fully calibrated, but 2 is often usable.
  isCalibrated = (sample.s >= 2);
}

void enforceCalibrationModeGuard(bool isCalibrated, bool laserValid,
                                 DeviceMode &deviceMode) {
  if (deviceMode == MODE_RUN) {
    if (!isCalibrated) {
      // lost calibration in run mode -> switch to IDLE or handle gracefully?
      // For now, let's auto-switch to IDLE to force user to recalibrate/wait
      deviceMode = MODE_IDLE;
    }
  }
}

void updateMotionAndMode(const RelativePose &pose, unsigned long nowMs,
                         MotionState &state, DeviceMode &deviceMode,
                         bool isCalibrated, bool laserValid) {

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
    if (deviceMode == MODE_IDLE) {
      deviceMode = MODE_RUN;
    }
    // --------------------------------------

    state.lastRelDist = pose.relDist;
    state.lastRoll = pose.relRoll;
    state.lastPitch = pose.relPitch;
    state.lastYaw = pose.relYaw;
  }

  // 2. Auto-IDLE Timeout
  const unsigned long IDLE_TIMEOUT_MS = 30000;

  if (deviceMode == MODE_RUN) {
    if (nowMs - state.lastMotionTime > IDLE_TIMEOUT_MS) {
      deviceMode = MODE_IDLE;
    }
  }
}

} // namespace app
} // namespace liftrr
