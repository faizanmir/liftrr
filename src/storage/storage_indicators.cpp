#include <Arduino.h>
#include "core/config.h"
#include "storage/storage_indicators.h"

namespace liftrr {
namespace storage {

void defineStorageIndicators() {
    pinMode(LED_SD, OUTPUT);
    digitalWrite(LED_SD, LOW);
}

void pulseSDCardLED() {
    digitalWrite(LED_SD, HIGH);
    delayMicroseconds(300);   // 0.3 ms – visible but doesn’t wreck timing
    digitalWrite(LED_SD, LOW);
}

} // namespace storage
} // namespace liftrr
