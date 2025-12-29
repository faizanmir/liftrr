

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "core/globals.h"
#include "sensors/sensors.h"

// Initialize sensors (assumes Wire.begin already ran).
namespace liftrr {
namespace sensors {

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
    if (!liftrr::core::bno.begin(OPERATION_MODE_NDOF)) {
        Serial.println("BNO Fail (addr 0x28)");
        while (1) { delay(1000); }
    }
    liftrr::core::bno.setExtCrystalUse(true);
    delay(100);
    Serial.println("BNO055 initialized");

    // --- Laser distance sensor ---
    // Keep laser at 0x29.
    if (!liftrr::core::laser.begin(0x29, &Wire, true))
    {
        Serial.println("Laser init failed. Halting.");
        while (1) { delay(1000); }
    }
    liftrr::core::laser.startRanging();
    liftrr::core::laser.setTimingBudget(50); // 50 ms budget
    Serial.println("Laser initialized");
}

// Read IMU + laser into a sample.
void sensorsRead(SensorSample &sample)
{
    // IMU orientation + calibration status
    sensors_event_t event;
    liftrr::core::bno.getEvent(&event);

    uint8_t s = 0, g = 0, a = 0, m = 0;
    liftrr::core::bno.getCalibration(&s, &g, &a, &m);
    sample.event = event;
    sample.s = s;
    sample.g = g;
    sample.a = a;
    sample.m = m;

    // Laser distance
    if (liftrr::core::laser.dataReady())
    {
        int16_t newDist = liftrr::core::laser.distance();
        if (newDist != -1)
        {
            liftrr::core::distance = newDist;
            liftrr::core::laserValid = true;
        }
        liftrr::core::laser.clearInterrupt();
    }

    sample.rawDist = liftrr::core::distance;
}

// Compute pose relative to calibration offsets.
void sensorsComputePose(const SensorSample &sample, RelativePose &out)
{
    out.relDist = sample.rawDist - liftrr::core::laserOffset;
    out.relRoll = sample.event.orientation.y - liftrr::core::rollOffset;
    out.relPitch = sample.event.orientation.z - liftrr::core::pitchOffset;
    out.relYaw = sample.event.orientation.x - liftrr::core::yawOffset;

    // Normalize yaw to [-180, 180] range
    if (out.relYaw > 180.0f)
        out.relYaw -= 360.0f;
    if (out.relYaw < -180.0f)
        out.relYaw += 360.0f;
}

DeviceFacing sensorsFacingDirection(const RelativePose& pose)
{
    const float kFacingThresholdDeg = 60.0f;

    float absPitch = fabsf(pose.relPitch);
    float absRoll = fabsf(pose.relRoll);

    if (absPitch >= absRoll) {
        return (pose.relPitch >= kFacingThresholdDeg) ? FACING_UP : FACING_DOWN;
    }

    return (pose.relRoll >= kFacingThresholdDeg) ? FACING_RIGHT : FACING_LEFT;
}

} // namespace sensors
} // namespace liftrr
