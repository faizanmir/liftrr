#pragma once

#include <Arduino.h>

namespace liftrr {

int64_t currentEpochMs();
void timeSyncSetEpochMs(int64_t epochMs);
int64_t timeSyncEpochMs();
uint32_t timeSyncMillisMs();
bool timeSyncIsValid();

} // namespace liftrr
