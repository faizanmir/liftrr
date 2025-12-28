#include "app/app_display.h"
#include "ble/ble_app.h"
#include "globals.h"
#include "rtc.h"
#include "sensors.h"
#include "storage.h"
#include "sensors.h"
#include "storage_indicators.h"
#include "rtc.h"
#include "ble/ble_app.h"
#include "app/app_display.h"
#include "app/app_motion.h"
#include "app/serial_commands.h"

// Motion → IDLE/RUN auto-switch state
static liftrr::app::MotionState gMotionState;

struct ModeApplier : liftrr::ble::IModeApplier {
  explicit ModeApplier(liftrr::app::MotionState *state) : state_(state) {}

  void applyMode(const char *mode) override {
    if (state_)
      state_->lastMotionTime = millis();

    if (strcmp(mode, "RUN") == 0) deviceMode = MODE_RUN;
    else if (strcmp(mode, "IDLE") == 0) deviceMode = MODE_IDLE;
    else if (strcmp(mode, "DUMP") == 0) deviceMode = MODE_DUMP;
    else if (strcmp(mode, "CALIBRATE") == 0) deviceMode = MODE_CALIBRATE;
  }

private:
  liftrr::app::MotionState *state_;
};

static ModeApplier gModeApplier(&gMotionState);



// ======================================================
//  Hardware init helpers
// ======================================================

static void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Display Init Failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("BOOT SEQUENCE...");
    display.display();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("--- SYSTEM START ---");

  // 1. Init I2C Bus
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeout(50);
  Serial.println("I2C Bus Initialized");
  delay(100);

  //2 Sensors
  sensorsInit();
 
  // 3. Display
  initDisplay();

  // 4. SD card
  storageInitSd();

  // 5. Motion state
  liftrr::app::initMotionState(gMotionState, millis());

  // 6. Storage Indicators
  defineStorageIndicators();

  // 8. BLE
  liftrr::ble::bleAppInit();
  liftrr::ble::bleAppSetModeApplier(&gModeApplier);

  Serial.println("--- SETUP COMPLETE ---");
}

void loop() {
  liftrr::app::handleSerialCommands(gMotionState);
  liftrr::ble::bleAppLoop(); // BLE periodic work

  unsigned long currentMillis = millis();
  int64_t sessionTimestampMs = liftrr::currentEpochMs();
  if (sessionTimestampMs <= 0) {
    sessionTimestampMs = (int64_t)currentMillis;
  }

  // --- MODE_DUMP: dedicated screen, no sensing/logging ---
  if (deviceMode == MODE_DUMP) {
    if (currentMillis - lastScreenUpdate >= SCREEN_INTERVAL) {
      lastScreenUpdate = currentMillis;
      liftrr::app::renderDumpScreen();
    }
    return;
  }

  //--- 1. Read sensors ---
  SensorSample sample;
  sensorsRead(sample);

  // --- 2. Update calibration readiness flags ---
  liftrr::app::updateCalibrationFlags(sample, isCalibrated);
  liftrr::app::enforceCalibrationModeGuard(isCalibrated, laserValid,
                                           deviceMode);

  // --- 3. Compute relative pose ---
  RelativePose pose;
  sensorsComputePose(sample, pose);

  //-- 4. SD logging: only in RUN mode with an active session ---
    if (deviceMode == MODE_RUN && storageIsSessionActive() && isCalibrated && laserValid) {
        storageLogSample(sessionTimestampMs,
                         sample.rawDist,
                         pose.relDist,
                         pose.relRoll,
                         pose.relPitch,
                         pose.relYaw);
  }

  // --- 5. Motion detection and auto mode transitions ---
    liftrr::app::updateMotionAndMode(pose, currentMillis, gMotionState, deviceMode, isCalibrated, laserValid);

  // --- 6. Display update (10 Hz) ---
  if (currentMillis - lastScreenUpdate >= SCREEN_INTERVAL) {
    lastScreenUpdate = currentMillis;
    display.clearDisplay();

    if (deviceMode == MODE_IDLE) {
      liftrr::app::renderIdleScreen();
    } else {
      // MODE_RUN or MODE_CALIBRATE
      if (deviceMode == MODE_CALIBRATE || !laserValid) {
        liftrr::app::renderCalibrationOrWarmupScreen(sample);
      } else {
        liftrr::app::renderTrackingScreen(pose);
      }
      display.display();
    }
  }
}
