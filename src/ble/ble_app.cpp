#include "ble_app.h"

#include <Arduino.h>
#include <ArduinoJson.h>

#include "ble.h"
#include "globals.h"
#include "storage.h"
#include "rtc.h"

namespace liftrr {
namespace ble {

static IModeApplier *gModeApplier = nullptr;

static void handleBleCommand(const std::string &raw);

// Callbacks object: BLE stack will call these when stuff happens
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
    }

    void onDisconnected() override {
        Serial.println("[BLE] Client disconnected");
    }

private:
    BleManager &ble_;
};

// These must live long enough (global/static lifetime)
static AppBleCallbacks gBleCallbacks(liftrr::gBleManager);

// -----------------------------------------------------------------------------
// BLE APP PROTOCOL (phone -> device)
// -----------------------------------------------------------------------------
// The phone sends JSON strings over BLE. The handler expects this shape:
// {
//   "id":   "<request-id>",          // required for correlation (echoed back as "ref")
//   "name": "<command-name>",        // preferred (legacy key: "cmd")
//   "body": { ... }                  // optional payload per command
// }
//
// The device replies with a JSON "resp" envelope:
// {
//   "kind":"resp", "name":"...", "ref":"<id>", "ok":true/false, "code":"...",
//   "ts":<epochMs-or-millis>, "body":{...}
// }
//
// And may also emit async events ("evt"), e.g. session.started.
// -----------------------------------------------------------------------------

// ======================================================
//  Time helpers
// ======================================================
// We prefer a real wall-clock epoch time (ms) from an RTC.
// If RTC is not available, we fall back to mapping phone epoch -> device millis.
// NOTE: millis() wraps around ~49.7 days (uint32_t), so epoch mapping is preferred.

static int64_t  gEpochAtSyncMs  = 0;
static uint32_t gMillisAtSyncMs = 0;

static int64_t currentEpochMs() {
    if (gEpochAtSyncMs <= 0) return 0;
    uint32_t now = millis();
    return gEpochAtSyncMs + (int64_t)(now - gMillisAtSyncMs);
}

// ======================================================
//  Pending session start (auto-start after calibration)
// ======================================================
// session.start may arrive before the device is calibrated / laser is valid.
// In that case we remember the request and auto-start once the device becomes ready.
// This keeps the phone UX simple: user can press "Start" and the device will start
// as soon as calibration completes.

static bool   gPendingSessionStart = false;
static String gPendingSessionId;
static String gPendingLift;

static void clearPendingSession() {
    gPendingSessionStart = false;
    gPendingSessionId    = "";
    gPendingLift         = "";
}

// ======================================================
//  JSON response helpers
// ======================================================
// sendBleResp(): reply to a request (correlated via ref=request.id)
// sendBleEvt():  async event (not tied to a request)
//
// We intentionally keep the envelopes consistent so the phone can parse them with
// a single decoder.

static void sendBleResp(
        const char *name,
        const char *ref,
        bool ok,
        const char *code,
        const char *msg,
        const std::function<void(JsonObject)> &fillBody) {

    // Response envelope. Keep this stable for the mobile app.
    // NOTE: StaticJsonDocument size must be big enough for worst-case body.
    StaticJsonDocument<1024> resp;
    resp["v"]    = 1;
    resp["id"]   = String(millis());
    int64_t epoch = currentEpochMs();
    resp["ts"]   = (epoch > 0) ? epoch : (int64_t)millis();
    resp["src"]  = "device";
    resp["dst"]  = "phone";
    resp["kind"] = "resp";
    resp["name"] = name;
    if (ref && ref[0] != '\0') resp["ref"] = ref;
    resp["ok"]   = ok;
    resp["code"] = code ? code : (ok ? "OK" : "ERR");
    if (msg && msg[0] != '\0') resp["msg"] = msg;

    // Command-specific payload goes here.
    JsonObject body = resp.createNestedObject("body");
    if (fillBody) fillBody(body);

    String out;
    serializeJson(resp, out);
    liftrr::gBleManager.sendStatus(out);
}

static void sendBleEvt(
        const char *name,
        const std::function<void(JsonObject)> &fillBody) {

    // Event envelope. Similar to responses but without ref/ok/code.
    StaticJsonDocument<384> evt;
    evt["v"]    = 1;
    evt["id"]   = String(millis());
    int64_t epoch = currentEpochMs();
    evt["ts"]   = (epoch > 0) ? epoch : (int64_t)millis();
    evt["src"]  = "device";
    evt["dst"]  = "phone";
    evt["kind"] = "evt";
    evt["name"] = name;

    JsonObject body = evt.createNestedObject("body");
    if (fillBody) fillBody(body);

    String out;
    serializeJson(evt, out);
    liftrr::gBleManager.sendStatus(out);
}

static bool equalsIgnoreCase(const String &a, const char *b) {
    return a.equalsIgnoreCase(String(b));
}

struct SessionIndexListCtx {
    JsonArray items;
};

static bool discardSessionIndexItem(const char *,
                                    uint32_t,
                                    uint64_t,
                                    size_t,
                                    void *) {
    return true;
}

static bool appendSessionIndexItem(const char *name,
                                   uint32_t size,
                                   uint64_t mtimeMs,
                                   size_t lineIndex,
                                   void *ctx) {
    if (!ctx) return false;
    auto *listCtx = static_cast<SessionIndexListCtx*>(ctx);
    JsonObject item = listCtx->items.createNestedObject();
    item["name"] = name;
    item["size"] = size;
    item["mtime"] = (unsigned long long)mtimeMs;
    item["line"] = (uint32_t)lineIndex;
    return true;
}

// Small helpers to read values either from `body` (preferred) or from the root document
// (legacy / backward-compatible path).
static const char* readStr(JsonObject body, JsonDocument &doc, const char *key, const char *defVal) {
    if (body) return body[key] | defVal;
    return doc[key] | defVal;
}

static int64_t readI64(JsonObject body, JsonDocument &doc, const char *key, int64_t defVal) {
    if (body) return body[key] | defVal;
    return doc[key] | defVal;
}

// ======================================================
//  Public API
// ======================================================

void bleAppInit() {
    BleInitConfig bleCfg;
    bleCfg.deviceName = "LIFTRR";
    bleCfg.mtu = 185;

    liftrr::gBleManager.begin(bleCfg, &gBleCallbacks);
}

void bleAppLoop() {
    liftrr::gBleManager.loop();

    // Auto-start pending session once calibration becomes ready
    if (gPendingSessionStart &&
        deviceMode == MODE_RUN &&
        !storageIsSessionActive() &&
        isCalibrated &&
        laserValid) {

        storageStartSession(gPendingSessionId,
                            gPendingLift.length() ? gPendingLift.c_str() : "unknown",
                            laserOffset,
                            rollOffset,
                            pitchOffset,
                            yawOffset);

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

// ======================================================
//  Command handler
// ======================================================

static void handleBleCommand(const std::string &raw) {
    if (!liftrr::gBleManager.isConnected()) return;

    if (raw.empty()) return;
    if (raw.size() > 2048) {
        sendBleResp("unknown", "", false, "PAYLOAD_TOO_LARGE", "Payload exceeds 2048 bytes", nullptr);
        return;
    }

    // Parse the incoming JSON command.
    // We cap payload size earlier to avoid memory blow-ups.
    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, raw);
    if (err) {
        sendBleResp("unknown", "", false, "BAD_JSON", err.c_str(), nullptr);
        return;
    }

    // Command name: prefer "name"; support legacy "cmd".
    const char *nameC = doc["name"] | doc["cmd"] | "";
    String name = String(nameC);

    // Request correlation id: the device will echo this back as "ref" in responses.
    const char *ref = doc["id"] | "";
    JsonObject body = doc["body"].is<JsonObject>() ? doc["body"].as<JsonObject>() : JsonObject();

    if (name.length() == 0) {
        sendBleResp("unknown", ref, false, "MISSING_NAME", "Missing 'name' (or legacy 'cmd')", nullptr);
        return;
    }

    // ---------------------------------------------------------------------
    // Commands implemented in this firmware:
    // - ping
    // - capabilities.get
    // - time.sync
    // - mode.set
    // - session.start
    // - session.end
    // - sessions.list
    // ---------------------------------------------------------------------

    // ping
    // Request:
    //   {"id":"...","name":"ping","body":{}}
    // Response body includes uptimeMs/epochMs/fw.
    if (equalsIgnoreCase(name, "ping")) {
        sendBleResp("ping", ref, true, "OK", "", [&](JsonObject out) {
            out["uptimeMs"] = (uint32_t)millis();
            out["epochMs"]  = currentEpochMs();
            out["fw"]       = "dev";
        });
        return;
    }

    // capabilities.get
    // Request:
    //   {"id":"...","name":"capabilities.get","body":{}}
    // Response body describes device model, fw, mtu, and feature flags.
    if (equalsIgnoreCase(name, "capabilities.get")) {
        sendBleResp("capabilities.get", ref, true, "OK", "", [&](JsonObject out) {
            JsonObject device = out.createNestedObject("device");
            device["model"] = "LIFTRR";
            device["fw"]    = "dev";

            out["maxMtu"] = 185;

            JsonObject features = out.createNestedObject("features");
            features["time.sync"]     = true;
            features["mode.set"]      = true;
            features["session.start"] = true;
            features["session.end"]   = true;
            features["sessions.list"] = true;
        });
        return;
    }

    // time.sync  body: { phoneEpochMs: <int64> }
    if (equalsIgnoreCase(name, "time.sync")) {
        // Phone provides wall-clock time in epoch milliseconds.
        // We store (epochAtSync, millisAtSync) so we can reconstruct epoch later.
        int64_t phoneEpoch = readI64(body, doc, "phoneEpochMs", (int64_t)0);

        if (phoneEpoch <= 0) {
            sendBleResp("time.sync", ref, false, "BAD_ARGS", "Missing/invalid phoneEpochMs", nullptr);
            return;
        }

        gEpochAtSyncMs  = phoneEpoch;
        gMillisAtSyncMs = millis();
    

        sendBleResp("time.sync", ref, true, "OK", "", [&](JsonObject out) {
            out["epochAtSyncMs"]  = gEpochAtSyncMs;
            out["millisAtSyncMs"] = gMillisAtSyncMs;
        });
        return;
    }

    // mode.set body: { mode: "RUN"|"IDLE"|"DUMP" }
    // This updates the global deviceMode used elsewhere in the firmware.
    if (equalsIgnoreCase(name, "mode.set")) {
        String modeStr = String(readStr(body, doc, "mode", ""));

        if (modeStr.length() == 0) {
            sendBleResp("mode.set", ref, false, "BAD_ARGS", "Missing mode", nullptr);
            return;
        }

        if (!modeStr.equalsIgnoreCase("RUN") &&
            !modeStr.equalsIgnoreCase("IDLE") &&
            !modeStr.equalsIgnoreCase("DUMP")) {
            sendBleResp("mode.set", ref, false, "BAD_ARGS", "mode must be RUN/IDLE/DUMP", nullptr);
            return;
        }

        if (gModeApplier) {
            gModeApplier->applyMode(modeStr.c_str());
        } else if (modeStr.equalsIgnoreCase("RUN")) {
            deviceMode = MODE_RUN;
        } else if (modeStr.equalsIgnoreCase("IDLE")) {
            deviceMode = MODE_IDLE;
        } else if (modeStr.equalsIgnoreCase("DUMP")) {
            deviceMode = MODE_DUMP;
        }

        sendBleResp("mode.set", ref, true, "OK", "", [&](JsonObject out) {
            out["mode"] = modeStr;
        });
        return;
    }


    // session.start
    // - Forces deviceMode=RUN.
    // - If not calibrated / laser invalid: returns CALIBRATION_REQUIRED and marks it pending.
    //   The main loop will auto-start when ready and emit evt: session.started.
    // Body:
    //   {"lift":"deadlift|squat|bench|...", "sessionId":"optional"}
    if (equalsIgnoreCase(name, "session.start")) {
        if (storageIsSessionActive()) {
            sendBleResp("session.start", ref, false, "ALREADY_ACTIVE", "Session already active", nullptr);
            return;
        }

        if (gModeApplier) gModeApplier->applyMode("RUN");
        else deviceMode = MODE_RUN;

        const char *liftC = readStr(body, doc, "lift", "unknown");
        const char *sidC  = readStr(body, doc, "sessionId", (const char*)nullptr);

        String sid;
        if (sidC && sidC[0] != '\0') sid = String(sidC);
        else {
            int64_t e = currentEpochMs();
            sid = (e > 0) ? String((long long)e) : String(millis());
        }

        if (!isCalibrated || !laserValid) {
            gPendingSessionStart = true;
            gPendingSessionId    = sid;
            gPendingLift         = String(liftC);

            sendBleResp("session.start", ref, false, "CALIBRATION_REQUIRED",
                        "Calibration required; session will auto-start when ready.",
                        [&](JsonObject out) {
                            out["pending"]   = true;
                            out["sessionId"] = sid;
                            out["lift"]      = liftC;
                            out["mode"]      = "RUN";
                        });
            return;
        }

        storageStartSession(sid, liftC, laserOffset, rollOffset, pitchOffset, yawOffset);
        clearPendingSession();

        sendBleResp("session.start", ref, true, "OK", "", [&](JsonObject out) {
            out["sessionId"] = sid;
            out["lift"]      = liftC;
            out["mode"]      = "RUN";
        });
        return;
    }

    // session.end
    // - Ends an active session.
    // - If a session.start is pending (waiting for calibration), cancels the pending start.
    if (equalsIgnoreCase(name, "session.end")) {
        if (gPendingSessionStart && !storageIsSessionActive()) {
            clearPendingSession();
            sendBleResp("session.end", ref, true, "OK", "Canceled pending session.start", nullptr);
            return;
        }

        if (!storageIsSessionActive()) {
            sendBleResp("session.end", ref, false, "NOT_ACTIVE", "No active session", nullptr);
            return;
        }

        storageEndSession();
        sendBleResp("session.end", ref, true, "OK", "", nullptr);
        return;
    }

    // sessions.list
    // Reads entries from /sessions/index.ndjson with cursor-based paging.
    // Body:
    //   {"cursor":<lineIndex>, "limit":<maxItems>}
    if (equalsIgnoreCase(name, "sessions.list")) {
        int64_t cursorIn = readI64(body, doc, "cursor", (int64_t)0);
        int64_t limitIn = readI64(body, doc, "limit", (int64_t)15);
        if (cursorIn < 0) cursorIn = 0;
        if (limitIn <= 0) limitIn = 15;
        if (limitIn > 15) limitIn = 15;

        size_t nextCursor = (size_t)cursorIn;
        bool hasMore = false;
        bool ok = storageReadSessionIndex(
            (size_t)cursorIn,
            (size_t)limitIn,
            &nextCursor,
            &hasMore,
            discardSessionIndexItem,
            nullptr);
        if (!ok) {
            sendBleResp("sessions.list", ref, false, "SD_ERROR", "Failed to read session index", nullptr);
            return;
        }

        sendBleResp("sessions.list", ref, true, "OK", "", [&](JsonObject out) {
            SessionIndexListCtx ctx{out.createNestedArray("items")};
            storageReadSessionIndex(
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

    // No other BLE file listing commands in this firmware.


    // unknown
    sendBleResp(name.c_str(), ref, false, "UNSUPPORTED", "Command not supported on this firmware", nullptr);
}

} // namespace ble
} // namespace liftrr
