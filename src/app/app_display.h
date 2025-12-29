#pragma once

#include <Adafruit_SSD1306.h>

#include "ble/ble.h"
#include "core/globals.h"
#include "sensors/sensors.h"
#include "storage/storage.h"
#include "ui/ui.h"

namespace liftrr {
namespace app {

class DisplayManager {
public:
    DisplayManager(Adafruit_SSD1306 &display,
                   liftrr::ui::UiRenderer &ui,
                   liftrr::storage::StorageManager &storage,
                   liftrr::ble::BleManager &ble,
                   liftrr::sensors::SensorManager &sensors,
                   liftrr::core::RuntimeState &runtime);

    void renderDumpScreen();
    void renderIdleScreen();
    void renderCalibrationOrWarmupScreen(const liftrr::sensors::SensorSample &sample);
    void renderTrackingScreen(const liftrr::sensors::RelativePose &pose);
    void renderOrientationWarningScreen(liftrr::sensors::DeviceFacing facing);

private:
    const char *modeLabel(liftrr::core::DeviceMode mode) const;
    void drawHeader(const char *title);
    void drawInfoLine(int y);

    Adafruit_SSD1306 &display_;
    liftrr::ui::UiRenderer &ui_;
    liftrr::storage::StorageManager &storage_;
    liftrr::ble::BleManager &ble_;
    liftrr::sensors::SensorManager &sensors_;
    liftrr::core::RuntimeState &runtime_;
};

} // namespace app
} // namespace liftrr
