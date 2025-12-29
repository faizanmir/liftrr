#pragma once

#include "ble/ble.h"
#include "comm/bt_classic.h"
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
         liftrr::storage::StorageManager &storage,
         liftrr::comm::BtClassicManager &btClassic);
  ~BleApp();

  void init();
  void loop();

  void setModeApplier(IModeApplier *applier);
  void notifyFacing(liftrr::sensors::DeviceFacing facing);
  void notifyCalibration(bool imuCalibrated, bool laserValid);

private:
  void handleRawCommand(const std::string &raw);
  void onConnected();
  void onDisconnected();
  void clearPendingSession();

  friend class AppBleCallbacks;

  liftrr::ble::BleManager &ble_;
  liftrr::core::RuntimeState &runtime_;
  liftrr::sensors::SensorManager &sensors_;
  liftrr::storage::StorageManager &storage_;
  liftrr::comm::BtClassicManager &bt_classic_;
  BleCallbacks *callbacks_;
  IModeApplier *mode_applier_;
  bool pending_session_start_;
  String pending_session_id_;
  String pending_lift_;
  bool pending_time_sync_;
  uint32_t time_sync_requested_ms_;

  static const uint32_t kTimeSyncTimeoutMs = 10000;
};
} // namespace ble
} // namespace liftrr
