#pragma once
#include "core/globals.h"

namespace liftrr {
namespace ui {

extern const unsigned char checkmark[];

void drawRuggedFrame();
void drawTiltGauge(float roll);
void drawStatusBar(bool recording);
void drawVerticalBar(int16_t currentVal);
void drawHorizon(float roll);

} // namespace ui
} // namespace liftrr
