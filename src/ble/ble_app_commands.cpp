#include "ble_app_internal.h"

#include <ArduinoJson.h>

#include "comm/bt_classic.h"
#include "core/rtc.h"
#include <SD.h>

namespace liftrr {
namespace ble {

struct BleCommandContext {
    BleApp &app;
    liftrr::ble::BleManager &ble;
    liftrr::core::RuntimeState &runtime;
    liftrr::sensors::SensorManager &sensors;
    liftrr::storage::StorageManager &storage;
    liftrr::comm::BtClassicManager &btClassic;
    IModeApplier *modeApplier;
};

class BleCommandBase {
public:
    virtual ~BleCommandBase() = default;
    virtual const char *name() const = 0;

    void run(BleCommandContext &ctx, const char *ref, JsonDocument &doc, JsonObject body) {
        applyCommon(ctx, doc, body);
        handle(ctx, ref, doc, body);
    }

    bool matches(const String &cmd) const {
        return equalsIgnoreCase(cmd, name());
    }

protected:
    virtual void handle(BleCommandContext &ctx, const char *ref, JsonDocument &doc, JsonObject body) = 0;

    void applyCommon(BleCommandContext &, JsonDocument &doc, JsonObject body) const {
        int64_t phoneEpoch = readI64(body, doc, "phoneEpochMs", (int64_t)0);
        if (phoneEpoch > 0) {
            liftrr::core::timeSyncSetEpochMs(phoneEpoch);
        }
    }

    void setPendingSession(BleCommandContext &ctx, const String &sid, const String &lift) const {
        ctx.app.pending_session_start_ = true;
        ctx.app.pending_session_id_ = sid;
        ctx.app.pending_lift_ = lift;
    }

    bool hasPendingSession(BleCommandContext &ctx) const {
        return ctx.app.pending_session_start_;
    }

    void clearPendingSession(BleCommandContext &ctx) const {
        ctx.app.clearPendingSession();
    }

    void setPendingTimeSync(BleCommandContext &ctx, bool value) const {
        ctx.app.pending_time_sync_ = value;
    }
};

namespace {

class PingCommand : public BleCommandBase {
public:
    const char *name() const override { return "ping"; }

protected:
    void handle(BleCommandContext &ctx, const char *ref, JsonDocument &, JsonObject) override {
        sendBleResp(ctx.ble, "ping", ref, true, "OK", "", [&](JsonObject out) {
            out["uptimeMs"] = (uint32_t)millis();
            out["epochMs"]  = liftrr::core::currentEpochMs();
            out["fw"]       = "dev";
        });
    }
};

class CapabilitiesCommand : public BleCommandBase {
public:
    const char *name() const override { return "capabilities.get"; }

protected:
    void handle(BleCommandContext &ctx, const char *ref, JsonDocument &, JsonObject) override {
        sendBleResp(ctx.ble, "capabilities.get", ref, true, "OK", "", [&](JsonObject out) {
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
            features["sessions.clear"] = true;
        });
    }
};

class TimeSyncCommand : public BleCommandBase {
public:
    const char *name() const override { return "time.sync"; }

protected:
    void handle(BleCommandContext &ctx, const char *ref, JsonDocument &doc, JsonObject body) override {
        int64_t phoneEpoch = readI64(body, doc, "phoneEpochMs", (int64_t)0);
        if (phoneEpoch <= 0) {
            sendBleResp(ctx.ble, "time.sync", ref, false, "BAD_ARGS",
                        "Missing/invalid phoneEpochMs", nullptr);
            return;
        }

        liftrr::core::timeSyncSetEpochMs(phoneEpoch);
        setPendingTimeSync(ctx, false);

        sendBleResp(ctx.ble, "time.sync", ref, true, "OK", "", [&](JsonObject out) {
            out["epochAtSyncMs"]  = liftrr::core::timeSyncEpochMs();
            out["millisAtSyncMs"] = liftrr::core::timeSyncMillisMs();
        });
    }
};

class ModeSetCommand : public BleCommandBase {
public:
    const char *name() const override { return "mode.set"; }

protected:
    void handle(BleCommandContext &ctx, const char *ref, JsonDocument &doc, JsonObject body) override {
        String modeStr = String(readStr(body, doc, "mode", ""));

        if (modeStr.length() == 0) {
            sendBleResp(ctx.ble, "mode.set", ref, false, "BAD_ARGS", "Missing mode", nullptr);
            return;
        }

        if (!modeStr.equalsIgnoreCase("RUN") &&
            !modeStr.equalsIgnoreCase("IDLE") &&
            !modeStr.equalsIgnoreCase("DUMP")) {
            sendBleResp(ctx.ble, "mode.set", ref, false, "BAD_ARGS", "mode must be RUN/IDLE/DUMP", nullptr);
            return;
        }

        if (ctx.modeApplier) {
            ctx.modeApplier->applyMode(modeStr.c_str());
        } else if (modeStr.equalsIgnoreCase("RUN")) {
            ctx.runtime.setDeviceMode(liftrr::core::MODE_RUN);
        } else if (modeStr.equalsIgnoreCase("IDLE")) {
            ctx.runtime.setDeviceMode(liftrr::core::MODE_IDLE);
        } else if (modeStr.equalsIgnoreCase("DUMP")) {
            ctx.runtime.setDeviceMode(liftrr::core::MODE_DUMP);
        } else if (modeStr.equalsIgnoreCase("CALIBRATE")) {
            ctx.runtime.setDeviceMode(liftrr::core::MODE_CALIBRATE);
        }

        sendBleResp(ctx.ble, "mode.set", ref, true, "OK", "", [&](JsonObject out) {
            out["mode"] = modeStr;
        });
    }
};

class SessionStartCommand : public BleCommandBase {
public:
    const char *name() const override { return "session.start"; }

protected:
    void handle(BleCommandContext &ctx, const char *ref, JsonDocument &doc, JsonObject body) override {
        if (ctx.storage.isSessionActive()) {
            sendBleResp(ctx.ble, "session.start", ref, false, "ALREADY_ACTIVE",
                        "Session already active", nullptr);
            return;
        }

        if (ctx.modeApplier) ctx.modeApplier->applyMode("RUN");
        else ctx.runtime.setDeviceMode(liftrr::core::MODE_RUN);

        const char *liftC = readStr(body, doc, "lift", "unknown");

        int64_t e = liftrr::core::currentEpochMs();
        String sid = ctx.storage.buildSessionId(liftC, e);

        if (!ctx.sensors.isCalibrated() || !ctx.sensors.laserValid()) {
            setPendingSession(ctx, sid, String(liftC));

            sendBleResp(ctx.ble, "session.start", ref, false, "CALIBRATION_REQUIRED",
                        "Calibration required; session will auto-start when ready.",
                        [&](JsonObject out) {
                            out["pending"]   = true;
                            out["sessionId"] = sid;
                            out["lift"]      = liftC;
                            out["mode"]      = "RUN";
                        });
            return;
        }

        ctx.storage.startSession(sid,
                                 liftC,
                                 ctx.sensors.laserOffset(),
                                 ctx.sensors.rollOffset(),
                                 ctx.sensors.pitchOffset(),
                                 ctx.sensors.yawOffset());
        clearPendingSession(ctx);

        sendBleResp(ctx.ble, "session.start", ref, true, "OK", "", [&](JsonObject out) {
            out["sessionId"] = sid;
            out["lift"]      = liftC;
            out["mode"]      = "RUN";
        });
    }
};

class SessionEndCommand : public BleCommandBase {
public:
    const char *name() const override { return "session.end"; }

protected:
    void handle(BleCommandContext &ctx, const char *ref, JsonDocument &, JsonObject) override {
        if (hasPendingSession(ctx) && !ctx.storage.isSessionActive()) {
            clearPendingSession(ctx);
            sendBleResp(ctx.ble, "session.end", ref, true, "OK", "Canceled pending session.start", nullptr);
            return;
        }

        if (!ctx.storage.isSessionActive()) {
            sendBleResp(ctx.ble, "session.end", ref, false, "NOT_ACTIVE", "No active session", nullptr);
            return;
        }

        ctx.storage.endSession();
        sendBleResp(ctx.ble, "session.end", ref, true, "OK", "", nullptr);
    }
};

class SessionsListCommand : public BleCommandBase {
public:
    const char *name() const override { return "sessions.list"; }

protected:
    void handle(BleCommandContext &ctx, const char *ref, JsonDocument &doc, JsonObject body) override {
        int64_t cursorIn = readI64(body, doc, "cursor", (int64_t)0);
        int64_t limitIn = readI64(body, doc, "limit", (int64_t)15);
        if (cursorIn < 0) cursorIn = 0;
        if (limitIn <= 0) limitIn = 15;
        if (limitIn > 15) limitIn = 15;

        if (!ctx.btClassic.isConnected()) {
            sendBleResp(ctx.ble, "sessions.list", ref, false, "NO_BT_CLASSIC",
                        "Classic Bluetooth not connected", nullptr);
            return;
        }

        size_t nextCursor = (size_t)cursorIn;
        bool hasMore = false;
        bool ok = ctx.storage.readSessionIndex(
            (size_t)cursorIn,
            (size_t)limitIn,
            &nextCursor,
            &hasMore,
            discardSessionIndexItem,
            nullptr);
        if (!ok) {
            sendBleResp(ctx.ble, "sessions.list", ref, false, "SD_ERROR",
                        "Failed to read session index", nullptr);
            return;
        }

        JsonDocument resp;
        resp["v"]    = 1;
        resp["id"]   = String(millis());
        int64_t epoch = liftrr::core::currentEpochMs();
        resp["ts"]   = (epoch > 0) ? epoch : (int64_t)millis();
        resp["src"]  = "device";
        resp["dst"]  = "phone";
        resp["kind"] = "resp";
        resp["name"] = "sessions.list";
        if (ref && ref[0] != '\0') resp["ref"] = ref;
        resp["ok"]   = true;
        resp["code"] = "OK";

        JsonObject out = resp["body"].to<JsonObject>();
        JsonArray items = out["items"].to<JsonArray>();
        SessionIndexListCtx ctxList{items};
        ctx.storage.readSessionIndex(
            (size_t)cursorIn,
            (size_t)limitIn,
            &nextCursor,
            &hasMore,
            appendSessionIndexItem,
            &ctxList);
        out["nextCursor"] = (uint32_t)nextCursor;
        out["hasMore"] = hasMore;

        String payload;
        serializeJson(resp, payload);
        ctx.btClassic.sendJsonLine(payload);

        sendBleResp(ctx.ble, "sessions.list", ref, true, "SENT_VIA_BT_CLASSIC", "", nullptr);
    }
};

class SessionStreamCommand : public BleCommandBase {
public:
    const char *name() const override { return "session.stream"; }

protected:
    void handle(BleCommandContext &ctx, const char *ref, JsonDocument &doc, JsonObject body) override {
        const char *sidC = readStr(body, doc, "sessionId", "");
        if (!sidC || sidC[0] == '\0') {
            sendBleResp(ctx.ble, "session.stream", ref, false, "BAD_ARGS", "Missing sessionId", nullptr);
            return;
        }

        if (!ctx.btClassic.isConnected()) {
            sendBleResp(ctx.ble, "session.stream", ref, false, "NO_BT_CLASSIC",
                        "Classic Bluetooth not connected", nullptr);
            return;
        }

        if (!ctx.storage.initSd()) {
            sendBleResp(ctx.ble, "session.stream", ref, false, "SD_ERROR", "SD init failed", nullptr);
            return;
        }

        String sessionId = String(sidC);
        if (sessionId.endsWith(".csv") || sessionId.endsWith(".tmp")) {
            int dot = sessionId.lastIndexOf('.');
            if (dot > 0) sessionId = sessionId.substring(0, dot);
        }

        String indexedName;
        if (!ctx.storage.findSessionInIndex(sessionId, indexedName)) {
            sendBleResp(ctx.ble, "session.stream", ref, false, "NOT_FOUND", "Session file not found", nullptr);
            return;
        }

        String path = String("/sessions/") + indexedName;
        if (!SD.exists(path)) {
            sendBleResp(ctx.ble, "session.stream", ref, false, "NOT_FOUND", "Indexed file missing on SD", nullptr);
            return;
        }

        File f = SD.open(path, FILE_READ);
        if (!f) {
            sendBleResp(ctx.ble, "session.stream", ref, false, "SD_ERROR", "Failed to open session file", nullptr);
            return;
        }
        size_t size = f.size();
        f.close();

        sendBleResp(ctx.ble, "session.stream", ref, true, "OK", "", [&](JsonObject out) {
            out["sessionId"] = sessionId;
            out["size"] = (uint32_t)size;
        });

        if (!ctx.btClassic.startFileStream(path, size, sessionId)) {
            sendBleEvt(ctx.ble, "session.file.error", [&](JsonObject out) {
                out["sessionId"] = sessionId;
                out["code"] = "BT_CLASSIC_STREAM_FAILED";
            });
        }
    }
};

class SessionsClearCommand : public BleCommandBase {
public:
    const char *name() const override { return "sessions.clear"; }

protected:
    void handle(BleCommandContext &ctx, const char *ref, JsonDocument &, JsonObject) override {
        if (ctx.storage.isSessionActive()) {
            sendBleResp(ctx.ble, "sessions.clear", ref, false, "SESSION_ACTIVE",
                        "End the active session before clearing.", nullptr);
            return;
        }

        if (!ctx.storage.clearSessions()) {
            sendBleResp(ctx.ble, "sessions.clear", ref, false, "SD_ERROR",
                        "Failed to clear sessions.", nullptr);
            return;
        }

        sendBleResp(ctx.ble, "sessions.clear", ref, true, "OK", "", nullptr);
    }
};

static PingCommand kPingCommand;
static CapabilitiesCommand kCapabilitiesCommand;
static TimeSyncCommand kTimeSyncCommand;
static ModeSetCommand kModeSetCommand;
static SessionStartCommand kSessionStartCommand;
static SessionEndCommand kSessionEndCommand;
static SessionsListCommand kSessionsListCommand;
static SessionStreamCommand kSessionStreamCommand;
static SessionsClearCommand kSessionsClearCommand;

static BleCommandBase *const kCommands[] = {
    &kPingCommand,
    &kCapabilitiesCommand,
    &kTimeSyncCommand,
    &kModeSetCommand,
    &kSessionStartCommand,
    &kSessionEndCommand,
    &kSessionsListCommand,
    &kSessionStreamCommand,
    &kSessionsClearCommand,
};

} // namespace

void BleApp::handleRawCommand(const std::string &raw) {
    if (!ble_.isConnected()) return;

    if (raw.empty()) return;
    if (raw.size() > 2048) {
        sendBleResp(ble_, "unknown", "", false, "PAYLOAD_TOO_LARGE", "Payload exceeds 2048 bytes", nullptr);
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, raw);
    if (err) {
        sendBleResp(ble_, "unknown", "", false, "BAD_JSON", err.c_str(), nullptr);
        return;
    }

    const char *nameC = doc["name"] | doc["cmd"] | "";
    String name = String(nameC);

    const char *ref = doc["id"] | "";
    JsonObject body = doc["body"].is<JsonObject>() ? doc["body"].as<JsonObject>() : JsonObject();

    if (name.length() == 0) {
        sendBleResp(ble_, "unknown", ref, false, "MISSING_NAME", "Missing 'name' (or legacy 'cmd')", nullptr);
        return;
    }

    BleCommandContext ctx{*this, ble_, runtime_, sensors_, storage_, bt_classic_, mode_applier_};
    for (BleCommandBase *cmd : kCommands) {
        if (cmd->matches(name)) {
            cmd->run(ctx, ref, doc, body);
            return;
        }
    }

    sendBleResp(ble_, name.c_str(), ref, false, "UNSUPPORTED", "Command not supported on this firmware", nullptr);
}

} // namespace ble
} // namespace liftrr
