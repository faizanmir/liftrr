#include "globals.h"
#include "ble/ble.h"

// --- OBJECT INSTANTIATION ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_VL53L1X laser = Adafruit_VL53L1X();


// --- VARIABLE INITIALIZATION ---
unsigned long lastScreenUpdate = 0;
unsigned long lastLogTime = 0;
unsigned long lastAutoDumpTime = 0; 

bool isCalibrated = false;

int16_t distance = 0;
bool laserValid = false; 

int16_t laserOffset = 0;
float rollOffset = 0;
float pitchOffset = 0;
float yawOffset = 0; 

int lastBtnState = HIGH;
unsigned long btnPressTime = 0;

DeviceMode deviceMode = MODE_IDLE;

namespace liftrr {
ble::BleManager gBleManager;  // Global BLE manager instance (defined exactly once in globals.cpp)
} // namespace liftrr
