#pragma once

#include "sensors/sensors.h"

namespace liftrr {
namespace app {

void renderDumpScreen();
void renderIdleScreen();
void renderCalibrationOrWarmupScreen(const liftrr::sensors::SensorSample &sample);
void renderTrackingScreen(const liftrr::sensors::RelativePose &pose);
void renderOrientationWarningScreen(liftrr::sensors::DeviceFacing facing);

} // namespace app
} // namespace liftrr
