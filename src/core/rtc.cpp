#include "core/rtc.h"

namespace liftrr {
namespace core {

int64_t gEpochAtSyncMs = 0;
uint32_t gMillisAtSyncMs = 0;

int64_t currentEpochMs() {
    if (gEpochAtSyncMs <= 0) return 0;
    uint32_t now = millis();
    return gEpochAtSyncMs + (int64_t)(now - gMillisAtSyncMs);
}

void timeSyncSetEpochMs(int64_t epochMs) {
    gEpochAtSyncMs = epochMs;
    gMillisAtSyncMs = millis();
}

int64_t timeSyncEpochMs() {
    return gEpochAtSyncMs;
}

uint32_t timeSyncMillisMs() {
    return gMillisAtSyncMs;
}

bool timeSyncIsValid() {
    return gEpochAtSyncMs > 0;
}

} // namespace core
} // namespace liftrr
