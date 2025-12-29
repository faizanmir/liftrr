#include "app_display.h"

#include <Arduino.h>

#include "core/globals.h"
#include "storage/storage.h"
#include "ui/ui.h"

namespace liftrr {
namespace app {

static const char* modeLabel(liftrr::core::DeviceMode mode) {
    switch (mode) {
        case liftrr::core::MODE_RUN:  return "RUN";
        case liftrr::core::MODE_DUMP: return "DUMP";
        case liftrr::core::MODE_IDLE: return "IDLE";
        default:        return "UNK";
    }
}

static void drawHeader(const char* title) {
    liftrr::core::display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
    liftrr::core::display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    liftrr::core::display.setTextSize(1);
    liftrr::core::display.setCursor(2, 1);
    liftrr::core::display.print(title);
    liftrr::core::display.setTextColor(SSD1306_WHITE);
}

static void drawInfoLine(int y) {
    liftrr::core::display.setTextSize(1);
    liftrr::core::display.setTextColor(SSD1306_WHITE);

    liftrr::core::display.setCursor(0, y);
    liftrr::core::display.print(modeLabel(liftrr::core::deviceMode));

    liftrr::core::display.setCursor(30, y);
    liftrr::core::display.print(liftrr::storage::storageIsSessionActive() ? "S:ON" : "S:--");

    liftrr::core::display.setCursor(60, y);
    liftrr::core::display.print(liftrr::gBleManager.isConnected() ? "B:ON" : "B:--");

    liftrr::core::display.setCursor(90, y);
    liftrr::core::display.print((liftrr::core::isCalibrated && liftrr::core::laserValid) ? "C:OK" : "C:--");
}

void renderDumpScreen() {
    liftrr::core::display.clearDisplay();
    drawHeader("DUMP");
    drawInfoLine(12);

    liftrr::core::display.setTextSize(2);
    liftrr::core::display.setCursor(10, 24);
    liftrr::core::display.println("DUMP MODE");
    liftrr::core::display.setTextSize(1);
    liftrr::core::display.setCursor(8, 48);
    liftrr::core::display.println("Send 'p' over serial");
    liftrr::core::display.display();
}

void renderIdleScreen() {
    liftrr::core::display.clearDisplay();
    drawHeader("IDLE");
    drawInfoLine(12);

    liftrr::core::display.setTextSize(2);
    liftrr::core::display.setCursor(15, 26);
    liftrr::core::display.println("IDLE MODE");
    liftrr::core::display.display();
}

void renderCalibrationOrWarmupScreen(const liftrr::sensors::SensorSample& sample) {
    drawHeader("SETUP");

    liftrr::core::display.setTextSize(1);
    liftrr::core::display.setCursor(0, 12);
    liftrr::core::display.print("G:"); liftrr::core::display.print(sample.g);
    liftrr::core::display.print(" A:"); liftrr::core::display.print(sample.a);
    liftrr::core::display.print(" M:"); liftrr::core::display.print(sample.m);

    if (!liftrr::core::isCalibrated) {
        // IMU calibration hints
        liftrr::core::display.setCursor(4, 24);
        liftrr::core::display.println("IMU CALIBRATION");
        liftrr::core::display.setCursor(4, 34);
        liftrr::core::display.println("- Rotate device slowly");
        liftrr::core::display.setCursor(4, 44);
        liftrr::core::display.println("  in all directions");
        liftrr::core::display.setCursor(4, 54);
        liftrr::core::display.println("- Keep bar steady after");
    } else if (!liftrr::core::laserValid) {
        // Laser calibration hints
        liftrr::core::display.setCursor(4, 24);
        liftrr::core::display.println("LASER CALIBRATION");
        liftrr::core::display.setCursor(4, 34);
        liftrr::core::display.println("- Point sensor at floor");
        liftrr::core::display.setCursor(4, 44);
        liftrr::core::display.println("- Hold still 1-2 sec");
    }
}

void renderTrackingScreen(const liftrr::sensors::RelativePose& pose) {
    bool tracking = (liftrr::core::laserOffset != 0);
    liftrr::ui::drawStatusBar(tracking);
    drawInfoLine(12);

    liftrr::ui::drawVerticalBar(pose.relDist);
    liftrr::ui::drawHorizon(pose.relRoll);

    // Dynamic Text Centering
    int xPos = 10;
    int absDist = abs(pose.relDist);
    if (absDist < 10)       xPos = 45;
    else if (absDist < 100) xPos = 35;
    else if (absDist < 1000) xPos = 15;

    liftrr::core::display.setTextSize(3);
    liftrr::core::display.setCursor(xPos, 22);
    liftrr::core::display.print(pose.relDist);

    liftrr::core::display.setTextSize(1);
    int unitX = xPos + (absDist >= 1000 ? 75 : (absDist >= 100 ? 55 : 38));
    liftrr::core::display.setCursor(unitX, 34);
    liftrr::core::display.print("mm");

    liftrr::core::display.setCursor(0, 50);
    liftrr::core::display.print("P:"); liftrr::core::display.print((int)pose.relPitch);
    liftrr::core::display.setCursor(60, 50);
    liftrr::core::display.print("Y:"); liftrr::core::display.print((int)pose.relYaw);
}

void renderOrientationWarningScreen(liftrr::sensors::DeviceFacing facing) {
    liftrr::core::display.clearDisplay();
    drawHeader("ORIENTATION");
    drawInfoLine(12);

    liftrr::core::display.setTextSize(2);
    liftrr::core::display.setCursor(8, 26);
    liftrr::core::display.println("FACE");
    liftrr::core::display.setCursor(8, 44);
    liftrr::core::display.println("UP/DOWN");

    liftrr::core::display.setTextSize(1);
    liftrr::core::display.setCursor(90, 50);
    liftrr::core::display.print(facing == liftrr::sensors::FACING_LEFT ? "LEFT" : "RIGHT");
}

} // namespace app
} // namespace liftrr
