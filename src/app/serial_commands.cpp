#include "serial_commands.h"

#include <functional>
#include <ArduinoJson.h>

#include "ble/ble_app_internal.h"
#include "core/rtc.h"
#include <SD.h>

namespace liftrr {
namespace app {

SerialCommandHandler::SerialCommandHandler(liftrr::core::RuntimeState &runtime,
                                           liftrr::storage::StorageManager &storage,
                                           liftrr::sensors::SensorManager &sensors,
                                           liftrr::comm::BtClassicManager &btClassic)
    : runtime_(runtime),
      storage_(storage),
      sensors_(sensors),
      bt_classic_(btClassic),
      pending_session_start_(false) {}

static void sendSerialResp(
        const char *name,
        const char *ref,
        bool ok,
        const char *code,
        const char *msg,
        const std::function<void(JsonObject)> &fillBody) {
    JsonDocument resp;
    resp["v"]    = 1;
    resp["id"]   = String(millis());
    int64_t epoch = liftrr::core::currentEpochMs();
    resp["ts"]   = (epoch > 0) ? epoch : (int64_t)millis();
    resp["src"]  = "device";
    resp["dst"]  = "host";
    resp["kind"] = "resp";
    resp["name"] = name;
    if (ref && ref[0] != '\0') resp["ref"] = ref;
    resp["ok"]   = ok;
    resp["code"] = code ? code : (ok ? "OK" : "ERR");
    if (msg && msg[0] != '\0') resp["msg"] = msg;

    JsonObject body = resp["body"].to<JsonObject>();
    if (fillBody) fillBody(body);

    serializeJson(resp, Serial);
    Serial.println();
}

static void sendSerialEvt(
        const char *name,
        const std::function<void(JsonObject)> &fillBody) {
    JsonDocument evt;
    evt["v"]    = 1;
    evt["id"]   = String(millis());
    int64_t epoch = liftrr::core::currentEpochMs();
    evt["ts"]   = (epoch > 0) ? epoch : (int64_t)millis();
    evt["src"]  = "device";
    evt["dst"]  = "host";
    evt["kind"] = "evt";
    evt["name"] = name;

    JsonObject body = evt["body"].to<JsonObject>();
    if (fillBody) fillBody(body);

    serializeJson(evt, Serial);
    Serial.println();
}

static const char* readStr(JsonObject body, JsonDocument &doc, const char *key, const char *defVal) {
    if (body) return body[key] | defVal;
    return doc[key] | defVal;
}

static int64_t readI64(JsonObject body, JsonDocument &doc, const char *key, int64_t defVal) {
    if (body) return body[key] | defVal;
    return doc[key] | defVal;
}

void SerialCommandHandler::clearPendingSession() {
    pending_session_start_ = false;
    pending_session_id_ = "";
    pending_lift_ = "";
}

void SerialCommandHandler::processPendingSession() {
    if (!pending_session_start_) return;
    if (runtime_.deviceMode() != liftrr::core::MODE_RUN) return;
    if (storage_.isSessionActive()) return;
    if (!sensors_.isCalibrated() || !sensors_.laserValid()) return;

    storage_.startSession(pending_session_id_,
                          pending_lift_.length() ? pending_lift_.c_str() : "unknown",
                          sensors_.laserOffset(),
                          sensors_.rollOffset(),
                          sensors_.pitchOffset(),
                          sensors_.yawOffset());

    sendSerialEvt("session.started", [&](JsonObject out) {
        out["sessionId"] = pending_session_id_;
        out["lift"]      = pending_lift_;
        out["auto"]      = true;
    });

    clearPendingSession();
}

void SerialCommandHandler::handleSerialCommands(MotionState &motionState) {
    processPendingSession();

    if (!Serial.available()) return;

    // JSON commands (newline-terminated). Examples for PlatformIO/Arduino Serial Monitor:
    // {"id":"1","name":"ping","body":{}}
    // {"id":"2","name":"capabilities.get","body":{}}
    // {"id":"3","name":"time.sync","body":{"phoneEpochMs":1710000000000}}
    // {"id":"4","name":"mode.set","body":{"mode":"RUN"}}   // RUN|IDLE|DUMP
    // {"id":"5","name":"session.start","body":{"lift":"deadlift","sessionId":"optional"}}
    // {"id":"6","name":"session.end","body":{}}
    // {"id":"7","name":"sessions.list","body":{"cursor":0,"limit":15}}
    // {"id":"8","name":"session.stream","body":{"sessionId":"1710000000000"}}
    // Notes: use "Newline" line ending; send one JSON per line.
    if (Serial.peek() == '{') {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length()) {
            handleJsonCommand(line);
        }
        return;
    }

    char cmd = Serial.read();

    switch (cmd) {
        case 'm':
            // Cycle device mode manually
            runtime_.setDeviceMode(static_cast<liftrr::core::DeviceMode>(
                    (static_cast<int>(runtime_.deviceMode()) + 1) % 3));
            Serial.print("Device Mode changed to: ");
            if (runtime_.deviceMode() == liftrr::core::MODE_RUN)  Serial.println("MODE_RUN");
            if (runtime_.deviceMode() == liftrr::core::MODE_DUMP) Serial.println("MODE_DUMP");
            if (runtime_.deviceMode() == liftrr::core::MODE_IDLE) Serial.println("MODE_IDLE");
            motionState.lastMotionTime = millis(); // reset idle timer
            break;

        case 'r':
            runtime_.setDeviceMode(liftrr::core::MODE_RUN);
            motionState.lastMotionTime = millis();
            Serial.println("Forced MODE_RUN");
            break;

        case 's': {
            // Start a session with a simple auto-generated ID
            int64_t epoch = liftrr::core::currentEpochMs();
            String sid = (epoch > 0) ? String((long long)epoch) : String(millis());
            if (storage_.isSessionActive()) {
                Serial.println("Session already active, cannot start new one.");
                break;
            }
            if (runtime_.deviceMode() != liftrr::core::MODE_RUN) {
                Serial.println("Device not in RUN mode, cannot start session.");
                break;
            }
            if (runtime_.deviceMode() == liftrr::core::MODE_RUN) {
                storage_.startSession(sid,
                                      "lift",
                                      sensors_.laserOffset(),
                                      sensors_.rollOffset(),
                                      sensors_.pitchOffset(),
                                      sensors_.yawOffset());
                Serial.println("Session started via serial 's'");
            }
            break;
        }

        case 'e':
            if (!storage_.isSessionActive()) {
                Serial.println("No active session to end.");
                break;
            }

            storage_.endSession();
            Serial.println("Session ended via serial 'e'");
            break;

        case 'i': {
            if (!storage_.initSd()) {
                Serial.println("SD init failed.");
                break;
            }

            Serial.println("--- /sessions/index.ndjson ---");
            File idx = SD.open("/sessions/index.ndjson", FILE_READ);
            if (idx) {
                while (idx.available()) {
                    String line = idx.readStringUntil('\n');
                    line.trim();
                    if (line.length()) Serial.println(line);
                }
                idx.close();
            } else {
                Serial.println("(missing)");
            }

            Serial.println("--- /sessions entries ---");
            File dir = SD.open("/sessions");
            if (!dir || !dir.isDirectory()) {
                Serial.println("(missing or not a dir)");
                if (dir) dir.close();
                break;
            }

            File entry = dir.openNextFile();
            while (entry) {
                Serial.print(entry.isDirectory() ? "DIR  " : "FILE ");
                Serial.print(entry.name());
                Serial.print("  ");
                Serial.println(entry.size());
                entry.close();
                entry = dir.openNextFile();
            }
            dir.close();
            break;
        }

        case 'd': {
            liftrr::sensors::SensorSample sample;
            sensors_.read(sample);
            Serial.print("cal: s="); Serial.print(sample.s);
            Serial.print(" g="); Serial.print(sample.g);
            Serial.print(" a="); Serial.print(sample.a);
            Serial.print(" m="); Serial.println(sample.m);
            Serial.print("laserValid="); Serial.print(sensors_.laserValid() ? "1" : "0");
            Serial.print(" dist="); Serial.println(sample.rawDist);
            Serial.print("isCalibrated="); Serial.println(sensors_.isCalibrated() ? "1" : "0");
            break;
        }

        default:
            // ignore unknown commands
            break;
    }
}

void SerialCommandHandler::handleJsonCommand(const String &line) {
    if (line.length() > 2048) {
        sendSerialResp("unknown", "", false, "PAYLOAD_TOO_LARGE", "Payload exceeds 2048 bytes", nullptr);
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) {
        sendSerialResp("unknown", "", false, "BAD_JSON", err.c_str(), nullptr);
        return;
    }

    const char *nameC = doc["name"] | doc["cmd"] | "";
    String name = String(nameC);

    const char *ref = doc["id"] | "";
    JsonObject body = doc["body"].is<JsonObject>() ? doc["body"].as<JsonObject>() : JsonObject();

    if (name.length() == 0) {
        sendSerialResp("unknown", ref, false, "MISSING_NAME", "Missing 'name' (or legacy 'cmd')", nullptr);
        return;
    }

    if (name.equalsIgnoreCase("ping")) {
        sendSerialResp("ping", ref, true, "OK", "", [&](JsonObject out) {
            out["uptimeMs"] = (uint32_t)millis();
            out["epochMs"]  = liftrr::core::currentEpochMs();
            out["fw"]       = "dev";
        });
        return;
    }

    if (name.equalsIgnoreCase("capabilities.get")) {
        sendSerialResp("capabilities.get", ref, true, "OK", "", [&](JsonObject out) {
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

    if (name.equalsIgnoreCase("time.sync")) {
        int64_t phoneEpoch = readI64(body, doc, "phoneEpochMs", (int64_t)0);
        if (phoneEpoch <= 0) {
            sendSerialResp("time.sync", ref, false, "BAD_ARGS", "Missing/invalid phoneEpochMs", nullptr);
            return;
        }

        liftrr::core::timeSyncSetEpochMs(phoneEpoch);

        sendSerialResp("time.sync", ref, true, "OK", "", [&](JsonObject out) {
            out["epochAtSyncMs"]  = liftrr::core::timeSyncEpochMs();
            out["millisAtSyncMs"] = liftrr::core::timeSyncMillisMs();
        });
        return;
    }

    if (name.equalsIgnoreCase("mode.set")) {
        String modeStr = String(readStr(body, doc, "mode", ""));

        if (modeStr.length() == 0) {
            sendSerialResp("mode.set", ref, false, "BAD_ARGS", "Missing mode", nullptr);
            return;
        }

        if (!modeStr.equalsIgnoreCase("RUN") &&
            !modeStr.equalsIgnoreCase("IDLE") &&
            !modeStr.equalsIgnoreCase("DUMP")) {
            sendSerialResp("mode.set", ref, false, "BAD_ARGS", "mode must be RUN/IDLE/DUMP", nullptr);
            return;
        }

        if (modeStr.equalsIgnoreCase("RUN")) {
            runtime_.setDeviceMode(liftrr::core::MODE_RUN);
        } else if (modeStr.equalsIgnoreCase("IDLE")) {
            runtime_.setDeviceMode(liftrr::core::MODE_IDLE);
        } else if (modeStr.equalsIgnoreCase("DUMP")) {
            runtime_.setDeviceMode(liftrr::core::MODE_DUMP);
        } else if (modeStr.equalsIgnoreCase("CALIBRATE")) {
            runtime_.setDeviceMode(liftrr::core::MODE_CALIBRATE);
        }

        sendSerialResp("mode.set", ref, true, "OK", "", [&](JsonObject out) {
            out["mode"] = modeStr;
        });
        return;
    }

    if (name.equalsIgnoreCase("session.start")) {
        if (storage_.isSessionActive()) {
            sendSerialResp("session.start", ref, false, "ALREADY_ACTIVE", "Session already active", nullptr);
            return;
        }

        runtime_.setDeviceMode(liftrr::core::MODE_RUN);

        const char *liftC = readStr(body, doc, "lift", "unknown");
        const char *sidC  = readStr(body, doc, "sessionId", (const char*)nullptr);

        String sid;
        if (sidC && sidC[0] != '\0') sid = String(sidC);
        else {
            int64_t e = liftrr::core::currentEpochMs();
            sid = (e > 0) ? String((long long)e) : String(millis());
        }

        if (!sensors_.isCalibrated() || !sensors_.laserValid()) {
            pending_session_start_ = true;
            pending_session_id_ = sid;
            pending_lift_ = String(liftC);

            sendSerialResp("session.start", ref, false, "CALIBRATION_REQUIRED",
                           "Calibration required; session will auto-start when ready.",
                           [&](JsonObject out) {
                               out["pending"]   = true;
                               out["sessionId"] = sid;
                               out["lift"]      = liftC;
                               out["mode"]      = "RUN";
                           });
            return;
        }

        storage_.startSession(sid,
                              liftC,
                              sensors_.laserOffset(),
                              sensors_.rollOffset(),
                              sensors_.pitchOffset(),
                              sensors_.yawOffset());
        clearPendingSession();

        sendSerialResp("session.start", ref, true, "OK", "", [&](JsonObject out) {
            out["sessionId"] = sid;
            out["lift"]      = liftC;
            out["mode"]      = "RUN";
        });
        return;
    }

    if (name.equalsIgnoreCase("session.end")) {
        if (pending_session_start_ && !storage_.isSessionActive()) {
            clearPendingSession();
            sendSerialResp("session.end", ref, true, "OK", "Canceled pending session.start", nullptr);
            return;
        }

        if (!storage_.isSessionActive()) {
            sendSerialResp("session.end", ref, false, "NOT_ACTIVE", "No active session", nullptr);
            return;
        }

        storage_.endSession();
        sendSerialResp("session.end", ref, true, "OK", "", nullptr);
        return;
    }

    if (name.equalsIgnoreCase("sessions.list")) {
        int64_t cursorIn = readI64(body, doc, "cursor", (int64_t)0);
        int64_t limitIn = readI64(body, doc, "limit", (int64_t)15);
        if (cursorIn < 0) cursorIn = 0;
        if (limitIn <= 0) limitIn = 15;
        if (limitIn > 15) limitIn = 15;

        size_t nextCursor = (size_t)cursorIn;
        bool hasMore = false;
        bool ok = storage_.readSessionIndex(
            (size_t)cursorIn,
            (size_t)limitIn,
            &nextCursor,
            &hasMore,
            liftrr::ble::discardSessionIndexItem,
            nullptr);
        if (!ok) {
            sendSerialResp("sessions.list", ref, false, "SD_ERROR", "Failed to read session index", nullptr);
            return;
        }

        sendSerialResp("sessions.list", ref, true, "OK", "", [&](JsonObject out) {
            JsonArray items = out["items"].to<JsonArray>();
            liftrr::ble::SessionIndexListCtx ctx{items};
            storage_.readSessionIndex(
                (size_t)cursorIn,
                (size_t)limitIn,
                &nextCursor,
                &hasMore,
                liftrr::ble::appendSessionIndexItem,
                &ctx);
            out["nextCursor"] = (uint32_t)nextCursor;
            out["hasMore"] = hasMore;
        });
        return;
    }

    if (name.equalsIgnoreCase("session.stream")) {
        const char *sidC = readStr(body, doc, "sessionId", "");
        if (!sidC || sidC[0] == '\0') {
            sendSerialResp("session.stream", ref, false, "BAD_ARGS", "Missing sessionId", nullptr);
            return;
        }

        if (!bt_classic_.isConnected()) {
            sendSerialResp("session.stream", ref, false, "NO_BT_CLASSIC", "Classic Bluetooth not connected", nullptr);
            return;
        }

        if (!storage_.initSd()) {
            sendSerialResp("session.stream", ref, false, "SD_ERROR", "SD init failed", nullptr);
            return;
        }

        String sessionId = String(sidC);
        String indexedName;
        if (!storage_.findSessionInIndex(sessionId, indexedName)) {
            sendSerialResp("session.stream", ref, false, "NOT_FOUND", "Session file not found", nullptr);
            return;
        }

        String path = String("/sessions/") + indexedName;
        if (!SD.exists(path)) {
            sendSerialResp("session.stream", ref, false, "NOT_FOUND", "Indexed file missing on SD", nullptr);
            return;
        }

        File f = SD.open(path, FILE_READ);
        if (!f) {
            sendSerialResp("session.stream", ref, false, "SD_ERROR", "Failed to open session file", nullptr);
            return;
        }
        size_t size = f.size();
        f.close();

        sendSerialResp("session.stream", ref, true, "OK", "", [&](JsonObject out) {
            out["sessionId"] = sessionId;
            out["size"] = (uint32_t)size;
        });

        if (!bt_classic_.startFileStream(path, size, sessionId)) {
            sendSerialEvt("session.file.error", [&](JsonObject out) {
                out["sessionId"] = sessionId;
                out["code"] = "BT_CLASSIC_STREAM_FAILED";
            });
        }
        return;
    }

    sendSerialResp(name.c_str(), ref, false, "UNSUPPORTED", "Command not supported on this firmware", nullptr);
}

} // namespace app
} // namespace liftrr
