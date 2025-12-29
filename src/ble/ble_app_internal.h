#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <functional>
#include <string>

#include "ble_app.h"

namespace liftrr {
namespace ble {

extern IModeApplier* gModeApplier;

extern bool gPendingSessionStart;
extern String gPendingSessionId;
extern String gPendingLift;

void clearPendingSession();

extern bool gPendingTimeSync;
extern uint32_t gTimeSyncRequestedMs;
extern const uint32_t kTimeSyncTimeoutMs;

void sendBleResp(
        liftrr::ble::BleManager &ble,
        const char *name,
        const char *ref,
        bool ok,
        const char *code,
        const char *msg,
        const std::function<void(JsonObject)> &fillBody);

void sendBleEvt(
        liftrr::ble::BleManager &ble,
        const char *name,
        const std::function<void(JsonObject)> &fillBody);

bool equalsIgnoreCase(const String &a, const char *b);

struct SessionIndexListCtx {
    JsonArray items;
};

bool discardSessionIndexItem(const char *,
                             uint32_t,
                             uint64_t,
                             size_t,
                             void *);

bool appendSessionIndexItem(const char *name,
                            uint32_t size,
                            uint64_t mtimeMs,
                            size_t lineIndex,
                            void *ctx);

const char* readStr(JsonObject body, JsonDocument &doc, const char *key, const char *defVal);
int64_t readI64(JsonObject body, JsonDocument &doc, const char *key, int64_t defVal);

void handleBleCommand(liftrr::ble::BleManager &ble,
                      liftrr::core::RuntimeState &runtime,
                      liftrr::sensors::SensorManager &sensors,
                      liftrr::storage::StorageManager &storage,
                      const std::string &raw);

} // namespace ble
} // namespace liftrr
