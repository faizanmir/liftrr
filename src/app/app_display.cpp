#include "app_display.h"

#include <Arduino.h>

#include "globals.h"
#include "storage.h"
#include "ui.h"

namespace liftrr {
namespace app {

static const char* modeLabel(DeviceMode mode) {
    switch (mode) {
        case MODE_RUN:  return "RUN";
        case MODE_DUMP: return "DUMP";
        case MODE_IDLE: return "IDLE";
        default:        return "UNK";
    }
}

static void drawHeader(const char* title) {
    display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(2, 1);
    display.print(title);
    display.setTextColor(SSD1306_WHITE);
}

static void drawInfoLine(int y) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, y);
    display.print(modeLabel(deviceMode));

    display.setCursor(30, y);
    display.print(storageIsSessionActive() ? "S:ON" : "S:--");

    display.setCursor(60, y);
    display.print(liftrr::gBleManager.isConnected() ? "B:ON" : "B:--");

    display.setCursor(90, y);
    display.print((isCalibrated && laserValid) ? "C:OK" : "C:--");
}

void renderDumpScreen() {
    display.clearDisplay();
    drawHeader("DUMP");
    drawInfoLine(12);

    display.setTextSize(2);
    display.setCursor(10, 24);
    display.println("DUMP MODE");
    display.setTextSize(1);
    display.setCursor(8, 48);
    display.println("Send 'p' over serial");
    display.display();
}

void renderIdleScreen() {
    display.clearDisplay();
    drawHeader("IDLE");
    drawInfoLine(12);

    display.setTextSize(2);
    display.setCursor(15, 26);
    display.println("IDLE MODE");
    display.display();
}

void renderCalibrationOrWarmupScreen(const SensorSample& sample) {
    drawHeader("SETUP");

    display.setTextSize(1);
    display.setCursor(0, 12);
    display.print("G:"); display.print(sample.g);
    display.print(" A:"); display.print(sample.a);
    display.print(" M:"); display.print(sample.m);

    if (!isCalibrated) {
        // IMU calibration hints
        display.setCursor(4, 24);
        display.println("IMU CALIBRATION");
        display.setCursor(4, 34);
        display.println("- Rotate device slowly");
        display.setCursor(4, 44);
        display.println("  in all directions");
        display.setCursor(4, 54);
        display.println("- Keep bar steady after");
    } else if (!laserValid) {
        // Laser calibration hints
        display.setCursor(4, 24);
        display.println("LASER CALIBRATION");
        display.setCursor(4, 34);
        display.println("- Point sensor at floor");
        display.setCursor(4, 44);
        display.println("- Hold still 1-2 sec");
    }
}

void renderTrackingScreen(const RelativePose& pose) {
    bool tracking = (laserOffset != 0);
    drawStatusBar(tracking);
    drawInfoLine(12);

    drawVerticalBar(pose.relDist);
    drawHorizon(pose.relRoll);

    // Dynamic Text Centering
    int xPos = 10;
    int absDist = abs(pose.relDist);
    if (absDist < 10)       xPos = 45;
    else if (absDist < 100) xPos = 35;
    else if (absDist < 1000) xPos = 15;

    display.setTextSize(3);
    display.setCursor(xPos, 22);
    display.print(pose.relDist);

    display.setTextSize(1);
    int unitX = xPos + (absDist >= 1000 ? 75 : (absDist >= 100 ? 55 : 38));
    display.setCursor(unitX, 34);
    display.print("mm");

    display.setCursor(0, 50);
    display.print("P:"); display.print((int)pose.relPitch);
    display.setCursor(60, 50);
    display.print("Y:"); display.print((int)pose.relYaw);
}

} // namespace app
} // namespace liftrr
