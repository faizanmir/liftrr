#include "ble_app_internal.h"

#include <ArduinoJson.h>

#include "comm/bt_classic.h"
#include "core/rtc.h"
#include <SD.h>

namespace liftrr {
namespace ble {

void handleBleCommand(liftrr::ble::BleManager &ble,
                      liftrr::core::RuntimeState &runtime,
                      liftrr::sensors::SensorManager &sensors,
                      liftrr::storage::StorageManager &storage,
                      const std::string &raw) {
    if (!ble.isConnected()) return;

    if (raw.empty()) return;
    if (raw.size() > 2048) {
        sendBleResp(ble, "unknown", "", false, "PAYLOAD_TOO_LARGE", "Payload exceeds 2048 bytes", nullptr);
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, raw);
    if (err) {
        sendBleResp(ble, "unknown", "", false, "BAD_JSON", err.c_str(), nullptr);
        return;
    }

    const char *nameC = doc["name"] | doc["cmd"] | "";
    String name = String(nameC);

    const char *ref = doc["id"] | "";
    JsonObject body = doc["body"].is<JsonObject>() ? doc["body"].as<JsonObject>() : JsonObject();

    if (name.length() == 0) {
        sendBleResp(ble, "unknown", ref, false, "MISSING_NAME", "Missing 'name' (or legacy 'cmd')", nullptr);
        return;
    }

    if (equalsIgnoreCase(name, "ping")) {
        sendBleResp(ble, "ping", ref, true, "OK", "", [&](JsonObject out) {
            out["uptimeMs"] = (uint32_t)millis();
            out["epochMs"]  = liftrr::core::currentEpochMs();
            out["fw"]       = "dev";
        });
        return;
    }

    if (equalsIgnoreCase(name, "capabilities.get")) {
        sendBleResp(ble, "capabilities.get", ref, true, "OK", "", [&](JsonObject out) {
            JsonObject device = out["device"].to<JsonObject>();
            device["model"] = "LIFTRR";
            device["fw"]    = "dev";

            out["maxMtu"] = 185;

            JsonObject features = out["features"].to<JsonObject>();
            features["time.sync"]     = true;
            features["mode.set"]      = true;
            features["session.start"] = true;
            features["session.end"]   = true;
            features["sessions.list"] = true;
            features["session.stream"] = true;
            features["session.stream.bt_classic"] = true;
        });
        return;
    }

    if (equalsIgnoreCase(name, "time.sync")) {
        int64_t phoneEpoch = readI64(body, doc, "phoneEpochMs", (int64_t)0);

        if (phoneEpoch <= 0) {
            sendBleResp(ble, "time.sync", ref, false, "BAD_ARGS", "Missing/invalid phoneEpochMs", nullptr);
            return;
        }

        liftrr::core::timeSyncSetEpochMs(phoneEpoch);
        gPendingTimeSync = false;

        sendBleResp(ble, "time.sync", ref, true, "OK", "", [&](JsonObject out) {
            out["epochAtSyncMs"]  = liftrr::core::timeSyncEpochMs();
            out["millisAtSyncMs"] = liftrr::core::timeSyncMillisMs();
        });
        return;
    }

    if (equalsIgnoreCase(name, "mode.set")) {
        String modeStr = String(readStr(body, doc, "mode", ""));

        if (modeStr.length() == 0) {
            sendBleResp(ble, "mode.set", ref, false, "BAD_ARGS", "Missing mode", nullptr);
            return;
        }

        if (!modeStr.equalsIgnoreCase("RUN") &&
            !modeStr.equalsIgnoreCase("IDLE") &&
            !modeStr.equalsIgnoreCase("DUMP")) {
            sendBleResp(ble, "mode.set", ref, false, "BAD_ARGS", "mode must be RUN/IDLE/DUMP", nullptr);
            return;
        }

        if (gModeApplier) {
            gModeApplier->applyMode(modeStr.c_str());
        } else if (modeStr.equalsIgnoreCase("RUN")) {
            runtime.setDeviceMode(liftrr::core::MODE_RUN);
        } else if (modeStr.equalsIgnoreCase("IDLE")) {
            runtime.setDeviceMode(liftrr::core::MODE_IDLE);
        } else if (modeStr.equalsIgnoreCase("DUMP")) {
            runtime.setDeviceMode(liftrr::core::MODE_DUMP);
        } else if (modeStr.equalsIgnoreCase("CALIBRATE")) {
            runtime.setDeviceMode(liftrr::core::MODE_CALIBRATE);
        }

        sendBleResp(ble, "mode.set", ref, true, "OK", "", [&](JsonObject out) {
            out["mode"] = modeStr;
        });
        return;
    }

    if (equalsIgnoreCase(name, "session.start")) {
        if (storage.isSessionActive()) {
            sendBleResp(ble, "session.start", ref, false, "ALREADY_ACTIVE", "Session already active", nullptr);
            return;
        }

        if (gModeApplier) gModeApplier->applyMode("RUN");
        else runtime.setDeviceMode(liftrr::core::MODE_RUN);

        const char *liftC = readStr(body, doc, "lift", "unknown");
        const char *sidC  = readStr(body, doc, "sessionId", (const char*)nullptr);

        String sid;
        if (sidC && sidC[0] != '\0') sid = String(sidC);
        else {
            int64_t e = liftrr::core::currentEpochMs();
            sid = (e > 0) ? String((long long)e) : String(millis());
        }

        if (!sensors.isCalibrated() || !sensors.laserValid()) {
            gPendingSessionStart = true;
            gPendingSessionId    = sid;
            gPendingLift         = String(liftC);

            sendBleResp(ble, "session.start", ref, false, "CALIBRATION_REQUIRED",
                        "Calibration required; session will auto-start when ready.",
                        [&](JsonObject out) {
                            out["pending"]   = true;
                            out["sessionId"] = sid;
                            out["lift"]      = liftC;
                            out["mode"]      = "RUN";
                        });
            return;
        }

        storage.startSession(sid,
                             liftC,
                             sensors.laserOffset(),
                             sensors.rollOffset(),
                             sensors.pitchOffset(),
                             sensors.yawOffset());
        clearPendingSession();

        sendBleResp(ble, "session.start", ref, true, "OK", "", [&](JsonObject out) {
            out["sessionId"] = sid;
            out["lift"]      = liftC;
            out["mode"]      = "RUN";
        });
        return;
    }

    if (equalsIgnoreCase(name, "session.end")) {
        if (gPendingSessionStart && !storage.isSessionActive()) {
            clearPendingSession();
            sendBleResp(ble, "session.end", ref, true, "OK", "Canceled pending session.start", nullptr);
            return;
        }

        if (!storage.isSessionActive()) {
            sendBleResp(ble, "session.end", ref, false, "NOT_ACTIVE", "No active session", nullptr);
            return;
        }

        storage.endSession();
        sendBleResp(ble, "session.end", ref, true, "OK", "", nullptr);
        return;
    }

    if (equalsIgnoreCase(name, "sessions.list")) {
        int64_t cursorIn = readI64(body, doc, "cursor", (int64_t)0);
        int64_t limitIn = readI64(body, doc, "limit", (int64_t)15);
        if (cursorIn < 0) cursorIn = 0;
        if (limitIn <= 0) limitIn = 15;
        if (limitIn > 15) limitIn = 15;

        size_t nextCursor = (size_t)cursorIn;
        bool hasMore = false;
        bool ok = storage.readSessionIndex(
            (size_t)cursorIn,
            (size_t)limitIn,
            &nextCursor,
            &hasMore,
            discardSessionIndexItem,
            nullptr);
        if (!ok) {
            sendBleResp(ble, "sessions.list", ref, false, "SD_ERROR", "Failed to read session index", nullptr);
            return;
        }

        sendBleResp(ble, "sessions.list", ref, true, "OK", "", [&](JsonObject out) {
            SessionIndexListCtx ctx{out.createNestedArray("items")};
            storage.readSessionIndex(
                (size_t)cursorIn,
                (size_t)limitIn,
                &nextCursor,
                &hasMore,
                appendSessionIndexItem,
                &ctx);
            out["nextCursor"] = (uint32_t)nextCursor;
            out["hasMore"] = hasMore;
        });
        return;
    }

    if (equalsIgnoreCase(name, "session.stream")) {
        const char *sidC = readStr(body, doc, "sessionId", "");
        if (!sidC || sidC[0] == '\0') {
            sendBleResp(ble, "session.stream", ref, false, "BAD_ARGS", "Missing sessionId", nullptr);
            return;
        }

        if (!liftrr::comm::btClassicIsConnected()) {
            sendBleResp(ble, "session.stream", ref, false, "NO_BT_CLASSIC", "Classic Bluetooth not connected", nullptr);
            return;
        }

        String sessionId = String(sidC);
        String indexedName;
        if (!storage.findSessionInIndex(sessionId, indexedName)) {
            sendBleResp(ble, "session.stream", ref, false, "NOT_FOUND", "Session file not found", nullptr);
            return;
        }

        String path = String("/sessions/") + indexedName;
        if (!SD.exists(path)) {
            sendBleResp(ble, "session.stream", ref, false, "NOT_FOUND", "Indexed file missing on SD", nullptr);
            return;
        }

        File f = SD.open(path, FILE_READ);
        if (!f) {
            sendBleResp(ble, "session.stream", ref, false, "SD_ERROR", "Failed to open session file", nullptr);
            return;
        }
        size_t size = f.size();
        f.close();

        sendBleResp(ble, "session.stream", ref, true, "OK", "", [&](JsonObject out) {
            out["sessionId"] = sessionId;
            out["size"] = (uint32_t)size;
        });

        if (!liftrr::comm::btClassicStartFileStream(path, size, sessionId)) {
            sendBleEvt(ble, "session.file.error", [&](JsonObject out) {
                out["sessionId"] = sessionId;
                out["code"] = "BT_CLASSIC_STREAM_FAILED";
            });
        }
        return;
    }

    sendBleResp(ble, name.c_str(), ref, false, "UNSUPPORTED", "Command not supported on this firmware", nullptr);
}

} // namespace ble
} // namespace liftrr
