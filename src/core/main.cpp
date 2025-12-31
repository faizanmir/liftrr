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
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L1X.h>
#include <Esp.h>

// Motion auto-switch state.
static liftrr::app::MotionState gMotionState;
static liftrr::app::MotionController gMotionController;

// Hardware peripherals.
static Adafruit_SSD1306 gDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static Adafruit_BNO055 gBno(55, 0x28);
static Adafruit_VL53L1X gLaser;
static bool gDisplayOk = false;

// Adapters + managers.
static liftrr::sensors::Bno055Sensor gImuAdapter(gBno);
static liftrr::sensors::Vl53l1xSensor gLaserAdapter(gLaser, 0x29, &Wire, true);
static liftrr::sensors::SensorManager gSensorManager(gImuAdapter, gLaserAdapter);
static liftrr::storage::StorageManager gStorageManager(SD, liftrr::storage::pulseSDCardLED);
static liftrr::core::RuntimeState gRuntimeState;
static liftrr::comm::BtClassicManager gBtClassic(SD);
static liftrr::ble::BleManager gBleManager;
static liftrr::ble::BleApp gBleApp(gBleManager, gRuntimeState, gSensorManager, gStorageManager, gBtClassic);
static liftrr::ui::UiRenderer gUi(gDisplay, gSensorManager);
static liftrr::app::DisplayManager gDisplayManager(
    gDisplay, gUi, gStorageManager, gBleManager, gSensorManager, gRuntimeState);
static liftrr::app::SerialCommandHandler gSerialHandler(
    gRuntimeState, gStorageManager, gSensorManager, gBtClassic);

struct ModeApplier : liftrr::ble::IModeApplier {
  ModeApplier(liftrr::core::RuntimeState &runtime, liftrr::app::MotionState *state)
      : runtime_(runtime), state_(state) {}

  void applyMode(const char *mode) override {
    if (state_)
      state_->lastMotionTime = millis();

    if (strcmp(mode, "RUN") == 0) runtime_.setDeviceMode(liftrr::core::MODE_RUN);
    else if (strcmp(mode, "IDLE") == 0) runtime_.setDeviceMode(liftrr::core::MODE_IDLE);
    else if (strcmp(mode, "DUMP") == 0) runtime_.setDeviceMode(liftrr::core::MODE_DUMP);
    else if (strcmp(mode, "CALIBRATE") == 0) runtime_.setDeviceMode(liftrr::core::MODE_CALIBRATE);
  }

private:
  liftrr::core::RuntimeState &runtime_;
  liftrr::app::MotionState *state_;
};

static ModeApplier gModeApplier(gRuntimeState, &gMotionState);



// Hardware init helpers.

static void initDisplay() {
  gDisplayOk = gDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  if (!gDisplayOk) {
    Serial.println("Display Init Failed");
  } else {
    gDisplay.clearDisplay();
    gDisplay.setTextSize(1);
    gDisplay.setTextColor(SSD1306_WHITE);
    gDisplay.setCursor(0, 0);
    gDisplay.println("BOOT SEQUENCE...");
    gDisplay.display();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("--- SYSTEM START ---");
  Serial.print("Flash size bytes: ");
  Serial.println(ESP.getFlashChipSize());
  Serial.print("Flash free bytes: ");
  Serial.println(ESP.getFreeSketchSpace());

  // 1. Init I2C Bus
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeout(50);
  Serial.println("I2C Bus Initialized");
  delay(100);

  //2 Sensors
  gSensorManager.init();
 
  // 3. Display
  initDisplay();
  Serial.println("Sensor health: IMU=OK");
  Serial.println("Sensor health: LASER=OK");
  Serial.print("Sensor health: DISPLAY=");
  Serial.println(gDisplayOk ? "OK" : "FAIL");

  // 4. SD card
  gStorageManager.initSd();

  // 5. Motion state
  gMotionController.initMotionState(gMotionState, millis());

  // 6. Storage Indicators
  liftrr::storage::defineStorageIndicators();

  // 8. BLE
  gBleApp.init();
  gBleApp.setModeApplier(&gModeApplier);

  // 9. Classic Bluetooth
  gBtClassic.init("LIFTRR");
}

void loop() {
  gSerialHandler.handleSerialCommands(gMotionState);
  gBleApp.loop(); // BLE periodic work
  gBtClassic.loop();

  unsigned long currentMillis = millis();
  int64_t sessionTimestampMs = liftrr::core::currentEpochMs();
  if (sessionTimestampMs <= 0) {
    sessionTimestampMs = (int64_t)currentMillis;
  }

  // --- liftrr::core::MODE_DUMP: dedicated screen, no sensing/logging ---
  if (gRuntimeState.deviceMode() == liftrr::core::MODE_DUMP) {
    if (currentMillis - gRuntimeState.lastScreenUpdate() >= SCREEN_INTERVAL) {
      gRuntimeState.setLastScreenUpdate(currentMillis);
      gDisplayManager.renderDumpScreen();
    }
    return;
  }

  //--- 1. Read sensors ---
  liftrr::sensors::SensorSample sample;
  gSensorManager.read(sample);

  // --- 2. Update calibration readiness flags ---
  gMotionController.updateCalibrationStatus(sample, gSensorManager);
  gBleApp.notifyCalibration(gSensorManager.isCalibrated(), gSensorManager.laserValid());
  gMotionController.enforceCalibrationModeGuard(gSensorManager, gRuntimeState);

  // --- 3. Compute relative pose ---
  liftrr::sensors::RelativePose pose;
  gSensorManager.computePose(sample, pose);
  liftrr::sensors::DeviceFacing facing = gSensorManager.facingDirection(pose);
  gBleApp.notifyFacing(facing);

  //-- 4. SD logging: only in RUN mode with an active session ---
    if (gRuntimeState.deviceMode() == liftrr::core::MODE_RUN &&
        gStorageManager.isSessionActive() &&
        gSensorManager.isCalibrated() &&
        gSensorManager.laserValid()) {
        gStorageManager.logSample(sessionTimestampMs,
                                  sample.rawDist,
                                  pose.relDist,
                                  pose.relRoll,
                                  pose.relPitch,
                                  pose.relYaw);
  }

  // --- 5. Motion detection and auto mode transitions ---
    gMotionController.updateMotionAndMode(pose, currentMillis, gMotionState, gRuntimeState);

  // --- 6. Display update (10 Hz) ---
  if (currentMillis - gRuntimeState.lastScreenUpdate() >= SCREEN_INTERVAL) {
    gRuntimeState.setLastScreenUpdate(currentMillis);
    gDisplay.clearDisplay();

    if (gRuntimeState.deviceMode() == liftrr::core::MODE_IDLE) {
      gDisplayManager.renderIdleScreen();
    } else {
      // liftrr::core::MODE_RUN or liftrr::core::MODE_CALIBRATE
      if (gRuntimeState.deviceMode() == liftrr::core::MODE_CALIBRATE || !gSensorManager.laserValid()) {
        gDisplayManager.renderCalibrationOrWarmupScreen(sample);
      } else {
        if (facing == liftrr::sensors::FACING_LEFT || facing == liftrr::sensors::FACING_RIGHT) {
          gDisplayManager.renderOrientationWarningScreen(facing);
        } else {
          gDisplayManager.renderTrackingScreen(pose);
        }
      }
      gDisplay.display();
    }
  }
}
