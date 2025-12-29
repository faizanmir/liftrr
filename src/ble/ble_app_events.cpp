#include "ble_app_internal.h"

#include <Arduino.h>

#include "ble.h"
#include "comm/bt_classic.h"
#include "core/globals.h"
#include "storage/storage.h"

namespace liftrr {
namespace ble {

class AppBleCallbacks : public BleCallbacks {
public:
    explicit AppBleCallbacks(BleManager &ble) : ble_(ble) {}

    void onRawCommand(const std::string &raw) override {
        Serial.println("[BLE] Command received:");
        Serial.println(raw.c_str());
        handleBleCommand(raw);
    }

    void onConnected() override {
        Serial.println("[BLE] Client connected");
        ble_.sendStatus(F("{\"event\":\"CONNECTED\"}"));
        gPendingTimeSync = true;
        gTimeSyncRequestedMs = millis();
        sendBleEvt("time.sync.request", [&](JsonObject out) {
            out["timeoutMs"] = kTimeSyncTimeoutMs;
        });

        if (!liftrr::comm::btClassicIsConnected()) {
            liftrr::comm::btClassicInit("LIFTRR");
            sendBleEvt("bt_classic.required", [&](JsonObject out) {
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
};

static AppBleCallbacks gBleCallbacks(liftrr::gBleManager);

void bleAppInit() {
    BleInitConfig bleCfg;
    bleCfg.deviceName = "LIFTRR";
    bleCfg.mtu = 185;

    liftrr::gBleManager.begin(bleCfg, &gBleCallbacks);
}

void bleAppLoop() {
    liftrr::gBleManager.loop();

    if (gPendingTimeSync &&
        (millis() - gTimeSyncRequestedMs) >= kTimeSyncTimeoutMs) {
        gPendingTimeSync = false;
        sendBleEvt("time.sync.timeout", [&](JsonObject out) {
            out["timeoutMs"] = kTimeSyncTimeoutMs;
        });
    }

    if (gPendingSessionStart &&
        liftrr::core::deviceMode == liftrr::core::MODE_RUN &&
        !liftrr::storage::storageIsSessionActive() &&
        liftrr::core::isCalibrated &&
        liftrr::core::laserValid) {

        liftrr::storage::storageStartSession(gPendingSessionId,
                            gPendingLift.length() ? gPendingLift.c_str() : "unknown",
                            liftrr::core::laserOffset,
                            liftrr::core::rollOffset,
                            liftrr::core::pitchOffset,
                            liftrr::core::yawOffset);

        sendBleEvt("session.started", [&](JsonObject out) {
            out["sessionId"] = gPendingSessionId;
            out["lift"]      = gPendingLift;
            out["auto"]      = true;
        });

        clearPendingSession();
    }
}

void bleAppSetModeApplier(IModeApplier* applier) {
    gModeApplier = applier;
}

void bleAppNotifyFacing(liftrr::sensors::DeviceFacing facing) {
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

    if (!liftrr::gBleManager.isConnected()) return;

    const char *facingStr = "UNKNOWN";
    switch (facing) {
        case liftrr::sensors::FACING_UP:    facingStr = "UP"; break;
        case liftrr::sensors::FACING_DOWN:  facingStr = "DOWN"; break;
        case liftrr::sensors::FACING_LEFT:  facingStr = "LEFT"; break;
        case liftrr::sensors::FACING_RIGHT: facingStr = "RIGHT"; break;
        default: break;
    }

    sendBleEvt("orientation.status", [&](JsonObject out) {
        out["facing"] = facingStr;
        out["ok"] = ok;
    });
}

void bleAppNotifyCalibration(bool imuCalibrated, bool laserValid) {
    static bool lastImuCalibrated = false;

    if (!imuCalibrated) {
        lastImuCalibrated = false;
        return;
    }
    if (lastImuCalibrated) return;
    lastImuCalibrated = true;

    if (!liftrr::gBleManager.isConnected()) return;

    sendBleEvt("calibration.succeeded", [&](JsonObject out) {
        out["imu"] = imuCalibrated;
        out["laser"] = laserValid;
        out["ready"] = imuCalibrated && laserValid;
    });
}

} // namespace ble
} // namespace liftrr
