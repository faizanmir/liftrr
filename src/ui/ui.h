#pragma once
#include <Adafruit_SSD1306.h>

#include "sensors/sensors.h"

namespace liftrr {
namespace ui {

extern const unsigned char checkmark[];

class UiRenderer {
public:
    UiRenderer(Adafruit_SSD1306 &display, liftrr::sensors::SensorManager &sensors);

    void drawRuggedFrame();
    void drawTiltGauge(float roll);
    void drawStatusBar(bool recording);
    void drawVerticalBar(int16_t currentVal);
    void drawHorizon(float roll);

private:
    Adafruit_SSD1306 &display_;
    liftrr::sensors::SensorManager &sensors_;
};

} // namespace ui
} // namespace liftrr
