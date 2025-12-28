

#include <Arduino.h>
#include <Wire.h>

#include "globals.h"
#include "sensors.h"

// Initialize all sensors (IMU + laser distance).
//
// Assumes I2C (Wire.begin) has already been called in setup() before this.
void sensorsInit()
{
    // Make I2C a little more tolerant on breadboards / longer wires.
#if defined(ESP32)
    Wire.setClock(100000);   // 100 kHz is the most reliable default
    Wire.setTimeout(50);     // avoid hanging forever on a bad transaction
#endif

    // --- IMU: BNO055 ---
    // Force BNO055 to 0x28 so it doesn't collide with VL53/VL618-type lasers
    // which commonly default to 0x29.
    if (!bno.begin(OPERATION_MODE_NDOF)) {
        Serial.println("BNO Fail (addr 0x28)");
        while (1) { delay(1000); }
    }
    bno.setExtCrystalUse(true);
    delay(100);
    Serial.println("BNO055 initialized");

    // --- Laser distance sensor ---
    // Keep laser at 0x29.
    if (!laser.begin(0x29, &Wire, true))
    {
        Serial.println("Laser init failed. Halting.");
        while (1) { delay(1000); }
    }
    laser.startRanging();
    laser.setTimingBudget(50); // 50 ms budget
    Serial.println("Laser initialized");
}

// Read IMU + laser state into a SensorSample.
// Updates the global 'distance' and 'laserValid' as used elsewhere.
void sensorsRead(SensorSample &sample)
{
    // IMU orientation + calibration status
    sensors_event_t event;
    bno.getEvent(&event);

    uint8_t s = 0, g = 0, a = 0, m = 0;
    bno.getCalibration(&s, &g, &a, &m);

    // Print occasionally (non-blocking) so we don't starve other work.
    static unsigned long lastPrintMs = 0;
    unsigned long now = millis();
    if (now - lastPrintMs >= 250) {
        Serial.print("IMU Ori X:"); Serial.print(event.orientation.x, 2);
        Serial.print(" Y:");        Serial.print(event.orientation.y, 2);
        Serial.print(" Z:");        Serial.print(event.orientation.z, 2);
        Serial.print(" | Calib S:"); Serial.print(s);
        Serial.print(" G:");        Serial.print(g);
        Serial.print(" A:");        Serial.print(a);
        Serial.print(" M:");        Serial.println(m);
        lastPrintMs = now;
    }

    sample.event = event;
    sample.s = s;
    sample.g = g;
    sample.a = a;
    sample.m = m;

    // Laser distance
    if (laser.dataReady())
    {
        int16_t newDist = laser.distance();
        if (newDist != -1)
        {
            distance = newDist;
            laserValid = true;
        }
        laser.clearInterrupt();
    }

    sample.rawDist = distance;
}

// Compute pose relative to the current calibration offsets.
//
// Uses global offsets:
//   laserOffset, rollOffset, pitchOffset, yawOffset
void sensorsComputePose(const SensorSample &sample, RelativePose &out)
{
    out.relDist = sample.rawDist - laserOffset;
    out.relRoll = sample.event.orientation.y - rollOffset;
    out.relPitch = sample.event.orientation.z - pitchOffset;
    out.relYaw = sample.event.orientation.x - yawOffset;

    // Normalize yaw to [-180, 180] range
    if (out.relYaw > 180.0f)
        out.relYaw -= 360.0f;
    if (out.relYaw < -180.0f)
        out.relYaw += 360.0f;
}
