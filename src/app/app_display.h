#pragma once

#include "sensors.h"

namespace liftrr {
namespace app {

void renderDumpScreen();
void renderIdleScreen();
void renderCalibrationOrWarmupScreen(const SensorSample &sample);
void renderTrackingScreen(const RelativePose &pose);

} // namespace app
} // namespace liftrr
