#pragma once

#include <Arduino.h>

namespace liftrr {
namespace comm {

bool btClassicInit(const char *deviceName);
bool btClassicIsConnected();
bool btClassicStartFileStream(const String &path,
                              size_t size,
                              const String &sessionId);
void btClassicLoop();

} // namespace comm
} // namespace liftrr
