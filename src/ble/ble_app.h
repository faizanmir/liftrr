#pragma once

#include "sensors/sensors.h"

namespace liftrr {
namespace ble {

// Call once from setup.
void bleAppInit();

// Call each loop iteration.
void bleAppLoop();

// Interface for mode changes.
struct IModeApplier {
  virtual ~IModeApplier() = default;
  // mode is expected to be one of: "RUN", "IDLE", "DUMP".
  virtual void applyMode(const char* mode) = 0;
};

// Register an optional mode applier.
void bleAppSetModeApplier(IModeApplier* applier);

// Notify orientation status.
void bleAppNotifyFacing(liftrr::sensors::DeviceFacing facing);

// Notify calibration success.
void bleAppNotifyCalibration(bool imuCalibrated, bool laserValid);
} // namespace ble
} // namespace liftrr
