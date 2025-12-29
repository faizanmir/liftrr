#pragma once

#include "ble/ble.h"
#include "core/globals.h"
#include "sensors/sensors.h"
#include "storage/storage.h"

namespace liftrr {
namespace ble {

// Interface for mode changes.
struct IModeApplier {
  virtual ~IModeApplier() = default;
  // mode is expected to be one of: "RUN", "IDLE", "DUMP".
  virtual void applyMode(const char* mode) = 0;
};

class BleApp {
public:
  BleApp(liftrr::ble::BleManager &ble,
         liftrr::core::RuntimeState &runtime,
         liftrr::sensors::SensorManager &sensors,
         liftrr::storage::StorageManager &storage);
  ~BleApp();

  void init();
  void loop();

  void setModeApplier(IModeApplier *applier);
  void notifyFacing(liftrr::sensors::DeviceFacing facing);
  void notifyCalibration(bool imuCalibrated, bool laserValid);

private:
  liftrr::ble::BleManager &ble_;
  liftrr::core::RuntimeState &runtime_;
  liftrr::sensors::SensorManager &sensors_;
  liftrr::storage::StorageManager &storage_;
  BleCallbacks *callbacks_;
};
} // namespace ble
} // namespace liftrr
