#include "app_display.h"

#include <Arduino.h>

namespace liftrr {
namespace app {

DisplayManager::DisplayManager(Adafruit_SSD1306 &display,
                               liftrr::ui::UiRenderer &ui,
                               liftrr::storage::StorageManager &storage,
                               liftrr::ble::BleManager &ble,
                               liftrr::sensors::SensorManager &sensors,
                               liftrr::core::RuntimeState &runtime)
    : display_(display),
      ui_(ui),
      storage_(storage),
      ble_(ble),
      sensors_(sensors),
      runtime_(runtime) {}

const char *DisplayManager::modeLabel(liftrr::core::DeviceMode mode) const {
    switch (mode) {
        case liftrr::core::MODE_RUN:  return "RUN";
        case liftrr::core::MODE_DUMP: return "DUMP";
        case liftrr::core::MODE_IDLE: return "IDLE";
        default:        return "UNK";
    }
}

void DisplayManager::drawHeader(const char *title) {
    display_.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
    display_.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display_.setTextSize(1);
    display_.setCursor(2, 1);
    display_.print(title);
    display_.setTextColor(SSD1306_WHITE);
}

void DisplayManager::drawInfoLine(int y) {
    display_.setTextSize(1);
    display_.setTextColor(SSD1306_WHITE);

    display_.setCursor(0, y);
    display_.print(modeLabel(runtime_.deviceMode()));

    display_.setCursor(30, y);
    display_.print(storage_.isSessionActive() ? "S:ON" : "S:--");

    display_.setCursor(60, y);
    display_.print(ble_.isConnected() ? "B:ON" : "B:--");

    display_.setCursor(90, y);
    display_.print((sensors_.isCalibrated() && sensors_.laserValid()) ? "C:OK" : "C:--");
}

void DisplayManager::renderDumpScreen() {
    display_.clearDisplay();
    drawHeader("DUMP");
    drawInfoLine(12);

    display_.setTextSize(2);
    display_.setCursor(10, 24);
    display_.println("DUMP MODE");
    display_.setTextSize(1);
    display_.setCursor(8, 48);
    display_.println("Send 'p' over serial");
    display_.display();
}

void DisplayManager::renderIdleScreen() {
    display_.clearDisplay();
    drawHeader("IDLE");
    drawInfoLine(12);

    display_.setTextSize(2);
    display_.setCursor(15, 26);
    display_.println("IDLE MODE");
    display_.display();
}

void DisplayManager::renderCalibrationOrWarmupScreen(const liftrr::sensors::SensorSample &sample) {
    drawHeader("SETUP");

    display_.setTextSize(1);
    display_.setCursor(0, 12);
    display_.print("G:"); display_.print(sample.g);
    display_.print(" A:"); display_.print(sample.a);
    display_.print(" M:"); display_.print(sample.m);

    if (!sensors_.isCalibrated()) {
        display_.setCursor(4, 24);
        display_.println("IMU CALIBRATION");
        display_.setCursor(4, 34);
        display_.println("- Rotate device slowly");
        display_.setCursor(4, 44);
        display_.println("  in all directions");
        display_.setCursor(4, 54);
        display_.println("- Keep bar steady after");
    } else if (!sensors_.laserValid()) {
        display_.setCursor(4, 24);
        display_.println("LASER CALIBRATION");
        display_.setCursor(4, 34);
        display_.println("- Point sensor at floor");
        display_.setCursor(4, 44);
        display_.println("- Hold still 1-2 sec");
    }
}

void DisplayManager::renderTrackingScreen(const liftrr::sensors::RelativePose &pose) {
    bool tracking = (sensors_.laserOffset() != 0);
    ui_.drawStatusBar(tracking);
    drawInfoLine(12);

    ui_.drawVerticalBar(pose.relDist);
    ui_.drawHorizon(pose.relRoll);

    int xPos = 10;
    int absDist = abs(pose.relDist);
    if (absDist < 10)       xPos = 45;
    else if (absDist < 100) xPos = 35;
    else if (absDist < 1000) xPos = 15;

    display_.setTextSize(3);
    display_.setCursor(xPos, 22);
    display_.print(pose.relDist);

    display_.setTextSize(1);
    int unitX = xPos + (absDist >= 1000 ? 75 : (absDist >= 100 ? 55 : 38));
    display_.setCursor(unitX, 34);
    display_.print("mm");

    display_.setCursor(0, 50);
    display_.print("P:"); display_.print((int)pose.relPitch);
    display_.setCursor(60, 50);
    display_.print("Y:"); display_.print((int)pose.relYaw);
}

void DisplayManager::renderOrientationWarningScreen(liftrr::sensors::DeviceFacing facing) {
    display_.clearDisplay();
    drawHeader("ORIENTATION");
    drawInfoLine(12);

    display_.setTextSize(2);
    display_.setCursor(8, 26);
    display_.println("FACE");
    display_.setCursor(8, 44);
    display_.println("UP/DOWN");

    display_.setTextSize(1);
    display_.setCursor(90, 50);
    display_.print(facing == liftrr::sensors::FACING_LEFT ? "LEFT" : "RIGHT");
}

} // namespace app
} // namespace liftrr
