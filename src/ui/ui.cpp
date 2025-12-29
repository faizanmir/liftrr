#include "ui/ui.h"

#include <Arduino.h>

#include "core/config.h"

namespace liftrr {
namespace ui {

const unsigned char checkmark [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x18, 0x40, 0x30,
	0x60, 0x60, 0x30, 0xC0, 0x19, 0x80, 0x0F, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

UiRenderer::UiRenderer(Adafruit_SSD1306 &display, liftrr::sensors::SensorManager &sensors)
    : display_(display), sensors_(sensors) {}

void UiRenderer::drawRuggedFrame() {
    int len = 8;
    display_.drawLine(0, 0, len, 0, SSD1306_WHITE);
    display_.drawLine(0, 0, 0, len, SSD1306_WHITE);
    display_.drawLine(127, 0, 127 - len, 0, SSD1306_WHITE);
    display_.drawLine(127, 0, 127, len, SSD1306_WHITE);
    display_.drawLine(0, 63, len, 63, SSD1306_WHITE);
    display_.drawLine(0, 63, 0, 63 - len, SSD1306_WHITE);
    display_.drawLine(127, 63, 127 - len, 63, SSD1306_WHITE);
    display_.drawLine(127, 63, 127, 63 - len, SSD1306_WHITE);
}

void UiRenderer::drawTiltGauge(float roll) {
    int y = 58; 
    int w = 80;
    int x = (SCREEN_WIDTH - w) / 2;
    
    display_.drawLine(x, y + 2, x + w, y + 2, SSD1306_WHITE);
    display_.drawLine(SCREEN_WIDTH / 2, y, SCREEN_WIDTH / 2, y + 4, SSD1306_WHITE);
    display_.drawLine(x, y, x, y + 4, SSD1306_WHITE);
    display_.drawLine(x + w, y, x + w, y + 4, SSD1306_WHITE);

    int indicatorX = map(constrain((int)roll, -20, 20), -20, 20, x, x + w);
    display_.fillRect(indicatorX - 2, y, 5, 5, SSD1306_WHITE);
}

void UiRenderer::drawStatusBar(bool recording) {
    display_.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
    display_.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display_.setTextSize(1);
    
    display_.setCursor(2, 1);
    if (sensors_.laserOffset() == 0) display_.print("SETUP");
    else display_.print(recording ? "REC" : "LIVE");

    display_.setCursor(50, 1);
    display_.print("SD:--");

    if ((millis() / 500) % 2 == 0) {
        display_.fillCircle(122, 4, 2, SSD1306_BLACK);
    }
    
    display_.setTextColor(SSD1306_WHITE);
}


void UiRenderer::drawVerticalBar(int16_t currentVal) {
    int x = 118;
    int y = 14;
    int w = 8;
    int h = 48;
    int maxRange = 1000; // +/- 1000mm range

    // Draw Frame
    display_.drawRect(x, y, w, h, SSD1306_WHITE);
    
    // Calculate Center Y of the bar
    int centerY = y + (h / 2);

    // Draw Center Line (Zero Point)
    display_.drawFastHLine(x - 2, centerY, w + 4, SSD1306_WHITE);

    // Constrain input to range
    int16_t val = constrain(currentVal, -maxRange, maxRange);

    // Map value to pixel height (half height of bar is max range)
    // Height available in one direction = (h/2) - 2 padding
    int pixelHeight = map(abs(val), 0, maxRange, 0, (h / 2) - 2);

    if (pixelHeight > 0) {
        if (val > 0) {
            // Positive: Fill UP from Center
            // Top Y = CenterY - pixelHeight
            display_.fillRect(x + 2, centerY - pixelHeight, w - 4, pixelHeight, SSD1306_WHITE);
        } else {
            // Negative: Fill DOWN from Center
            // Top Y = CenterY + 1 (start below line)
            display_.fillRect(x + 2, centerY + 1, w - 4, pixelHeight, SSD1306_WHITE);
        }
    }
}

void UiRenderer::drawHorizon(float roll) {
    int y = 58; int w = 60; int x = (118 - w) / 2; 
    
    display_.drawFastHLine(x, y, w, SSD1306_WHITE);
    display_.drawFastVLine(x + w / 2, y - 2, 5, SSD1306_WHITE); 
    
    int indicatorX = map(constrain((int)roll, -20, 20), -20, 20, x, x + w);
    display_.drawRect(indicatorX - 3, y - 3, 7, 7, SSD1306_WHITE);
    if (abs(roll) < 2) {
        display_.fillRect(indicatorX - 1, y - 1, 3, 3, SSD1306_WHITE);
    }
}

} // namespace ui
} // namespace liftrr
