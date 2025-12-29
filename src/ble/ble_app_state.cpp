#include "ble_app_internal.h"

namespace liftrr {
namespace ble {

IModeApplier* gModeApplier = nullptr;

bool   gPendingSessionStart = false;
String gPendingSessionId;
String gPendingLift;

bool gPendingTimeSync = false;
uint32_t gTimeSyncRequestedMs = 0;
const uint32_t kTimeSyncTimeoutMs = 10000;

void clearPendingSession() {
    gPendingSessionStart = false;
    gPendingSessionId    = "";
    gPendingLift         = "";
}

} // namespace ble
} // namespace liftrr
