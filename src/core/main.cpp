#include "app/app_display.h"
#include "app/app_motion.h"
#include "app/serial_commands.h"
#include "ble/ble_app.h"
#include "comm/bt_classic.h"
#include "core/globals.h"
#include "core/rtc.h"
#include "sensors/sensors.h"
#include "storage/storage.h"
#include "storage/storage_indicators.h"

// Motion auto-switch state.
static liftrr::app::MotionState gMotionState;

struct ModeApplier : liftrr::ble::IModeApplier {
  explicit ModeApplier(liftrr::app::MotionState *state) : state_(state) {}

  void applyMode(const char *mode) override {
    if (state_)
      state_->lastMotionTime = millis();

    if (strcmp(mode, "RUN") == 0) liftrr::core::deviceMode = liftrr::core::MODE_RUN;
    else if (strcmp(mode, "IDLE") == 0) liftrr::core::deviceMode = liftrr::core::MODE_IDLE;
    else if (strcmp(mode, "DUMP") == 0) liftrr::core::deviceMode = liftrr::core::MODE_DUMP;
    else if (strcmp(mode, "CALIBRATE") == 0) liftrr::core::deviceMode = liftrr::core::MODE_CALIBRATE;
  }

private:
  liftrr::app::MotionState *state_;
};

static ModeApplier gModeApplier(&gMotionState);



// Hardware init helpers.

static void initDisplay() {
  if (!liftrr::core::display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Display Init Failed");
  } else {
    liftrr::core::display.clearDisplay();
    liftrr::core::display.setTextSize(1);
    liftrr::core::display.setTextColor(SSD1306_WHITE);
    liftrr::core::display.setCursor(0, 0);
    liftrr::core::display.println("BOOT SEQUENCE...");
    liftrr::core::display.display();
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
  liftrr::sensors::sensorsInit();
 
  // 3. Display
  initDisplay();

  // 4. SD card
  liftrr::storage::storageInitSd();

  // 5. Motion state
  liftrr::app::initMotionState(gMotionState, millis());

  // 6. Storage Indicators
  liftrr::storage::defineStorageIndicators();

  // 8. BLE
  liftrr::ble::bleAppInit();
  liftrr::ble::bleAppSetModeApplier(&gModeApplier);

  // 9. Classic Bluetooth
  liftrr::comm::btClassicInit("LIFTRR");
}

void loop() {
  liftrr::app::handleSerialCommands(gMotionState);
  liftrr::ble::bleAppLoop(); // BLE periodic work
  liftrr::comm::btClassicLoop();

  unsigned long currentMillis = millis();
  int64_t sessionTimestampMs = liftrr::core::currentEpochMs();
  if (sessionTimestampMs <= 0) {
    sessionTimestampMs = (int64_t)currentMillis;
  }

  // --- liftrr::core::MODE_DUMP: dedicated screen, no sensing/logging ---
  if (liftrr::core::deviceMode == liftrr::core::MODE_DUMP) {
    if (currentMillis - liftrr::core::lastScreenUpdate >= SCREEN_INTERVAL) {
      liftrr::core::lastScreenUpdate = currentMillis;
      liftrr::app::renderDumpScreen();
    }
    return;
  }

  //--- 1. Read sensors ---
  liftrr::sensors::SensorSample sample;
  liftrr::sensors::sensorsRead(sample);

  // --- 2. Update calibration readiness flags ---
  liftrr::app::updateCalibrationFlags(sample, liftrr::core::isCalibrated);
  liftrr::ble::bleAppNotifyCalibration(liftrr::core::isCalibrated, liftrr::core::laserValid);
  liftrr::app::enforceCalibrationModeGuard(liftrr::core::isCalibrated, liftrr::core::laserValid,
                                           liftrr::core::deviceMode);

  // --- 3. Compute relative pose ---
  liftrr::sensors::RelativePose pose;
  liftrr::sensors::sensorsComputePose(sample, pose);
  liftrr::sensors::DeviceFacing facing = liftrr::sensors::sensorsFacingDirection(pose);
  liftrr::ble::bleAppNotifyFacing(facing);

  //-- 4. SD logging: only in RUN mode with an active session ---
    if (liftrr::core::deviceMode == liftrr::core::MODE_RUN && liftrr::storage::storageIsSessionActive() && liftrr::core::isCalibrated && liftrr::core::laserValid) {
        liftrr::storage::storageLogSample(sessionTimestampMs,
                         sample.rawDist,
                         pose.relDist,
                         pose.relRoll,
                         pose.relPitch,
                         pose.relYaw);
  }

  // --- 5. Motion detection and auto mode transitions ---
    liftrr::app::updateMotionAndMode(pose, currentMillis, gMotionState, liftrr::core::deviceMode, liftrr::core::isCalibrated, liftrr::core::laserValid);

  // --- 6. Display update (10 Hz) ---
  if (currentMillis - liftrr::core::lastScreenUpdate >= SCREEN_INTERVAL) {
    liftrr::core::lastScreenUpdate = currentMillis;
    liftrr::core::display.clearDisplay();

    if (liftrr::core::deviceMode == liftrr::core::MODE_IDLE) {
      liftrr::app::renderIdleScreen();
    } else {
      // liftrr::core::MODE_RUN or liftrr::core::MODE_CALIBRATE
      if (liftrr::core::deviceMode == liftrr::core::MODE_CALIBRATE || !liftrr::core::laserValid) {
        liftrr::app::renderCalibrationOrWarmupScreen(sample);
      } else {
        if (facing == liftrr::sensors::FACING_LEFT || facing == liftrr::sensors::FACING_RIGHT) {
          liftrr::app::renderOrientationWarningScreen(facing);
        } else {
          liftrr::app::renderTrackingScreen(pose);
        }
      }
      liftrr::core::display.display();
    }
  }
}
