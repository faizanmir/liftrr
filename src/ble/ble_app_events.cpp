#include "ble_app_internal.h"

#include <Arduino.h>

#include "ble.h"
#include "comm/bt_classic.h"

namespace liftrr {
namespace ble {

class AppBleCallbacks : public BleCallbacks {
public:
    AppBleCallbacks(BleManager &ble,
                    liftrr::core::RuntimeState &runtime,
                    liftrr::sensors::SensorManager &sensors,
                    liftrr::storage::StorageManager &storage)
        : ble_(ble), runtime_(runtime), sensors_(sensors), storage_(storage) {}

    void onRawCommand(const std::string &raw) override {
        Serial.println("[BLE] Command received:");
        Serial.println(raw.c_str());
        handleBleCommand(ble_, runtime_, sensors_, storage_, raw);
    }

    void onConnected() override {
        Serial.println("[BLE] Client connected");
        ble_.sendStatus(F("{\"event\":\"CONNECTED\"}"));
        gPendingTimeSync = true;
        gTimeSyncRequestedMs = millis();
        sendBleEvt(ble_, "time.sync.request", [&](JsonObject out) {
            out["timeoutMs"] = kTimeSyncTimeoutMs;
        });

        if (!liftrr::comm::btClassicIsConnected()) {
            liftrr::comm::btClassicInit("LIFTRR");
            sendBleEvt(ble_, "bt_classic.required", [&](JsonObject out) {
                out["status"] = "not_connected";
            });
        }
    }

    void onDisconnected() override {
        Serial.println("[BLE] Client disconnected");
        gPendingTimeSync = false;
    }

private:
    BleManager &ble_;
    liftrr::core::RuntimeState &runtime_;
    liftrr::sensors::SensorManager &sensors_;
    liftrr::storage::StorageManager &storage_;
};

BleApp::BleApp(liftrr::ble::BleManager &ble,
               liftrr::core::RuntimeState &runtime,
               liftrr::sensors::SensorManager &sensors,
               liftrr::storage::StorageManager &storage)
    : ble_(ble),
      runtime_(runtime),
      sensors_(sensors),
      storage_(storage),
      callbacks_(new AppBleCallbacks(ble_, runtime_, sensors_, storage_)) {}

BleApp::~BleApp() {
    delete callbacks_;
    callbacks_ = nullptr;
}

void BleApp::init() {
    BleInitConfig bleCfg;
    bleCfg.deviceName = "LIFTRR";
    bleCfg.mtu = 185;

    ble_.begin(bleCfg, callbacks_);
}

void BleApp::loop() {
    ble_.loop();

    if (gPendingTimeSync &&
        (millis() - gTimeSyncRequestedMs) >= kTimeSyncTimeoutMs) {
        gPendingTimeSync = false;
        sendBleEvt(ble_, "time.sync.timeout", [&](JsonObject out) {
            out["timeoutMs"] = kTimeSyncTimeoutMs;
        });
    }

    if (gPendingSessionStart &&
        runtime_.deviceMode() == liftrr::core::MODE_RUN &&
        !storage_.isSessionActive() &&
        sensors_.isCalibrated() &&
        sensors_.laserValid()) {

        storage_.startSession(gPendingSessionId,
                              gPendingLift.length() ? gPendingLift.c_str() : "unknown",
                              sensors_.laserOffset(),
                              sensors_.rollOffset(),
                              sensors_.pitchOffset(),
                              sensors_.yawOffset());

        sendBleEvt(ble_, "session.started", [&](JsonObject out) {
            out["sessionId"] = gPendingSessionId;
            out["lift"]      = gPendingLift;
            out["auto"]      = true;
        });

        clearPendingSession();
    }
}

void BleApp::setModeApplier(IModeApplier* applier) {
    gModeApplier = applier;
}

void BleApp::notifyFacing(liftrr::sensors::DeviceFacing facing) {
    static bool hasLast = false;
    static liftrr::sensors::DeviceFacing lastFacing = liftrr::sensors::FACING_DOWN;
    static bool lastOk = true;

    bool ok = (facing == liftrr::sensors::FACING_UP || facing == liftrr::sensors::FACING_DOWN);
    if (hasLast && facing == lastFacing && ok == lastOk) {
        return;
    }

    hasLast = true;
    lastFacing = facing;
    lastOk = ok;

    if (!ble_.isConnected()) return;

    const char *facingStr = "UNKNOWN";
    switch (facing) {
        case liftrr::sensors::FACING_UP:    facingStr = "UP"; break;
        case liftrr::sensors::FACING_DOWN:  facingStr = "DOWN"; break;
        case liftrr::sensors::FACING_LEFT:  facingStr = "LEFT"; break;
        case liftrr::sensors::FACING_RIGHT: facingStr = "RIGHT"; break;
        default: break;
    }

    sendBleEvt(ble_, "orientation.status", [&](JsonObject out) {
        out["facing"] = facingStr;
        out["ok"] = ok;
    });
}

void BleApp::notifyCalibration(bool imuCalibrated, bool laserValid) {
    static bool lastImuCalibrated = false;

    if (!imuCalibrated) {
        lastImuCalibrated = false;
        return;
    }
    if (lastImuCalibrated) return;
    lastImuCalibrated = true;

    if (!ble_.isConnected()) return;

    sendBleEvt(ble_, "calibration.succeeded", [&](JsonObject out) {
        out["imu"] = imuCalibrated;
        out["laser"] = laserValid;
        out["ready"] = imuCalibrated && laserValid;
    });
}

} // namespace ble
} // namespace liftrr
