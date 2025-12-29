#pragma once

#include "app_motion.h"
#include "core/globals.h"
#include "sensors/sensors.h"
#include "storage/storage.h"

namespace liftrr {
namespace app {

class SerialCommandHandler {
public:
    SerialCommandHandler(liftrr::core::RuntimeState &runtime,
                         liftrr::storage::StorageManager &storage,
                         liftrr::sensors::SensorManager &sensors);

    void handleSerialCommands(MotionState &motionState);

private:
    liftrr::core::RuntimeState &runtime_;
    liftrr::storage::StorageManager &storage_;
    liftrr::sensors::SensorManager &sensors_;
};

} // namespace app
} // namespace liftrr
