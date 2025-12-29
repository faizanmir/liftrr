#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "core/config.h"
#include "ble/ble.h"

namespace liftrr {
namespace core {

extern Adafruit_SSD1306 display;
extern Adafruit_BNO055 bno;
extern Adafruit_VL53L1X laser;

extern unsigned long lastScreenUpdate;
extern unsigned long lastLogTime;
extern unsigned long lastAutoDumpTime;
extern int64_t gEpochAtSyncMs;
extern uint32_t gMillisAtSyncMs;

extern bool isCalibrated;

extern int16_t distance;
extern bool laserValid;

extern int16_t laserOffset;
extern float rollOffset;
extern float pitchOffset;
extern float yawOffset;

extern int lastBtnState;
extern unsigned long btnPressTime;

extern DeviceMode deviceMode;

} // namespace core
} // namespace liftrr

namespace liftrr {
extern ble::BleManager gBleManager; // Global BLE manager instance (defined exactly once in globals.cpp)
} // namespace liftrr
