#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "ble/ble.h"

// Forward declarations to avoid circular includes.
// globals.h is included by many .cpp files, so it should stay lightweight.
// --- EXTERNAL OBJECTS ---
extern Adafruit_SSD1306 display;
extern Adafruit_BNO055 bno;
extern Adafruit_VL53L1X laser;

// --- EXTERNAL VARIABLES ---
extern unsigned long lastScreenUpdate;
extern unsigned long lastLogTime;
extern unsigned long lastAutoDumpTime;

// State
extern bool isCalibrated;

// Data
extern int16_t distance;
extern bool laserValid;

// Offsets
extern int16_t laserOffset;
extern float rollOffset;
extern float pitchOffset;
extern float yawOffset;

// Button
extern int lastBtnState;
extern unsigned long btnPressTime;

extern DeviceMode deviceMode;

namespace liftrr {
extern ble::BleManager gBleManager; // Global BLE manager instance (defined exactly once in globals.cpp)
} // namespace liftrr
