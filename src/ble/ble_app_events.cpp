#include "ble_app_internal.h"

#include <Arduino.h>

#include "ble.h"
namespace liftrr {
namespace ble {

class AppBleCallbacks : public BleCallbacks {
public:
    explicit AppBleCallbacks(BleApp &app) : app_(app) {}

    void onRawCommand(const std::string &raw) override {
        Serial.println("[BLE] Command received:");
        Serial.println(raw.c_str());
        app_.handleRawCommand(raw);
    }

    void onConnected() override {
        app_.onConnected();
    }

    void onDisconnected() override {
        app_.onDisconnected();
    }

private:
    BleApp &app_;
};

BleApp::BleApp(liftrr::ble::BleManager &ble,
               liftrr::core::RuntimeState &runtime,
               liftrr::sensors::SensorManager &sensors,
               liftrr::storage::StorageManager &storage,
               liftrr::comm::BtClassicManager &btClassic)
    : ble_(ble),
      runtime_(runtime),
      sensors_(sensors),
      storage_(storage),
      bt_classic_(btClassic),
      callbacks_(new AppBleCallbacks(*this)),
      mode_applier_(nullptr),
      pending_session_start_(false),
      pending_session_id_(""),
      pending_lift_(""),
      pending_time_sync_(false),
      time_sync_requested_ms_(0) {}

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

void BleApp::onConnected() {
    Serial.println("[BLE] Client connected");
    ble_.sendStatus(F("{\"event\":\"CONNECTED\"}"));
    pending_time_sync_ = true;
    time_sync_requested_ms_ = millis();
    sendBleEvt(ble_, "time.sync.request", [&](JsonObject out) {
        out["timeoutMs"] = kTimeSyncTimeoutMs;
    });

    if (!bt_classic_.isConnected()) {
        bt_classic_.init("LIFTRR");
        sendBleEvt(ble_, "bt_classic.required", [&](JsonObject out) {
            out["status"] = "not_connected";
        });
    }
}

void BleApp::onDisconnected() {
    Serial.println("[BLE] Client disconnected");
    pending_time_sync_ = false;
}

void BleApp::clearPendingSession() {
    pending_session_start_ = false;
    pending_session_id_ = "";
    pending_lift_ = "";
}

void BleApp::loop() {
    ble_.loop();

    if (pending_time_sync_ &&
        (millis() - time_sync_requested_ms_) >= kTimeSyncTimeoutMs) {
        pending_time_sync_ = false;
        sendBleEvt(ble_, "time.sync.timeout", [&](JsonObject out) {
            out["timeoutMs"] = kTimeSyncTimeoutMs;
        });
    }

    if (pending_session_start_ &&
        runtime_.deviceMode() == liftrr::core::MODE_RUN &&
        !storage_.isSessionActive() &&
        sensors_.isCalibrated() &&
        sensors_.laserValid()) {

        storage_.startSession(pending_session_id_,
                              pending_lift_.length() ? pending_lift_.c_str() : "unknown",
                              sensors_.laserOffset(),
                              sensors_.rollOffset(),
                              sensors_.pitchOffset(),
                              sensors_.yawOffset());

        sendBleEvt(ble_, "session.started", [&](JsonObject out) {
            out["sessionId"] = pending_session_id_;
            out["lift"]      = pending_lift_;
            out["auto"]      = true;
        });

        clearPendingSession();
    }
}

void BleApp::setModeApplier(IModeApplier* applier) {
    mode_applier_ = applier;
}

void BleApp::notifyFacing(liftrr::sensors::DeviceFacing facing) {
    static bool hasLast = false;
    static liftrr::sensors::DeviceFacing lastFacing = liftrr::sensors::FACING_DOWN;
    static bool lastOk = true;

    if (!ble_.isConnected()) return;
    if (!sensors_.isCalibrated() && !storage_.isSessionActive()) {
        return;
    }

    bool ok = (facing == liftrr::sensors::FACING_UP || facing == liftrr::sensors::FACING_DOWN);
    if (hasLast && facing == lastFacing && ok == lastOk) {
        return;
    }

    hasLast = true;
    lastFacing = facing;
    lastOk = ok;

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
