#pragma once

#include "app_motion.h"
#include "comm/bt_classic.h"
#include "core/globals.h"
#include "sensors/sensors.h"
#include "storage/storage.h"

namespace liftrr {
namespace app {

class SerialCommandHandler {
public:
    SerialCommandHandler(liftrr::core::RuntimeState &runtime,
                         liftrr::storage::StorageManager &storage,
                         liftrr::sensors::SensorManager &sensors,
                         liftrr::comm::BtClassicManager &btClassic);

    void handleSerialCommands(MotionState &motionState);

private:
    void handleJsonCommand(const String &line);
    void processPendingSession();
    void clearPendingSession();

    liftrr::core::RuntimeState &runtime_;
    liftrr::storage::StorageManager &storage_;
    liftrr::sensors::SensorManager &sensors_;
    liftrr::comm::BtClassicManager &bt_classic_;
    bool pending_session_start_;
    String pending_session_id_;
    String pending_lift_;
};

} // namespace app
} // namespace liftrr
