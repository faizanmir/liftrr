#include "ble_app_internal.h"

#include "core/rtc.h"

namespace liftrr {
namespace ble {

void sendBleResp(
        liftrr::ble::BleManager &ble,
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
    resp["dst"]  = "phone";
    resp["kind"] = "resp";
    resp["name"] = name;
    if (ref && ref[0] != '\0') resp["ref"] = ref;
    resp["ok"]   = ok;
    resp["code"] = code ? code : (ok ? "OK" : "ERR");
    if (msg && msg[0] != '\0') resp["msg"] = msg;

    JsonObject body = resp["body"].to<JsonObject>();
    if (fillBody) fillBody(body);

    String out;
    serializeJson(resp, out);
    ble.sendStatus(out);
}

void sendBleEvt(
        liftrr::ble::BleManager &ble,
        const char *name,
        const std::function<void(JsonObject)> &fillBody) {

    JsonDocument evt;
    evt["v"]    = 1;
    evt["id"]   = String(millis());
    int64_t epoch = liftrr::core::currentEpochMs();
    evt["ts"]   = (epoch > 0) ? epoch : (int64_t)millis();
    evt["src"]  = "device";
    evt["dst"]  = "phone";
    evt["kind"] = "evt";
    evt["name"] = name;

    JsonObject body = evt["body"].to<JsonObject>();
    if (fillBody) fillBody(body);

    String out;
    serializeJson(evt, out);
    ble.sendStatus(out);
}

bool equalsIgnoreCase(const String &a, const char *b) {
    return a.equalsIgnoreCase(String(b));
}

bool discardSessionIndexItem(const char *,
                             uint32_t,
                             uint64_t,
                             size_t,
                             void *) {
    return true;
}

bool appendSessionIndexItem(const char *name,
                            uint32_t size,
                            uint64_t mtimeMs,
                            size_t lineIndex,
                            void *ctx) {
    if (!ctx) return false;
    auto *listCtx = static_cast<SessionIndexListCtx*>(ctx);
    JsonObject item = listCtx->items.add<JsonObject>();
    item["name"] = name;
    item["size"] = size;
    item["mtime"] = (unsigned long long)mtimeMs;
    item["line"] = (uint32_t)lineIndex;
    return true;
}

const char* readStr(JsonObject body, JsonDocument &doc, const char *key, const char *defVal) {
    if (body) return body[key] | defVal;
    return doc[key] | defVal;
}

int64_t readI64(JsonObject body, JsonDocument &doc, const char *key, int64_t defVal) {
    if (body) return body[key] | defVal;
    return doc[key] | defVal;
}

} // namespace ble
} // namespace liftrr
