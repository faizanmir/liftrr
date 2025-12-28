#include <Arduino.h>
#include "config.h"

void defineStorageIndicators() {
pinMode(LED_SD, OUTPUT);
digitalWrite(LED_SD, LOW);
}

void pulseSDCardLED() {
    digitalWrite(LED_SD, HIGH);
    delayMicroseconds(300);   // 0.3 ms – visible but doesn’t wreck timing
    digitalWrite(LED_SD, LOW);
}