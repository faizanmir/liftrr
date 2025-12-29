#include <Arduino.h>
#include <math.h>

#include "sensors/sensors.h"

namespace liftrr {
namespace sensors {

Bno055Sensor::Bno055Sensor(Adafruit_BNO055 &imu,
                           adafruit_bno055_opmode_t mode)
    : imu_(imu), mode_(mode) {}

bool Bno055Sensor::begin() {
    return imu_.begin(mode_);
}

void Bno055Sensor::setExtCrystalUse(bool use) {
    imu_.setExtCrystalUse(use);
}

void Bno055Sensor::getEvent(sensors_event_t *event) {
    imu_.getEvent(event);
}

void Bno055Sensor::getCalibration(uint8_t *s, uint8_t *g, uint8_t *a, uint8_t *m) {
    imu_.getCalibration(s, g, a, m);
}

Vl53l1xSensor::Vl53l1xSensor(Adafruit_VL53L1X &laser,
                             uint8_t address,
                             TwoWire *wire,
                             bool debug)
    : laser_(laser), address_(address), wire_(wire), debug_(debug) {}

bool Vl53l1xSensor::begin() {
    return laser_.begin(address_, wire_, debug_);
}

void Vl53l1xSensor::startRanging() {
    laser_.startRanging();
}

void Vl53l1xSensor::setTimingBudget(uint16_t budgetMs) {
    laser_.setTimingBudget(budgetMs);
}

bool Vl53l1xSensor::dataReady() {
    return laser_.dataReady();
}

int16_t Vl53l1xSensor::distance() {
    return laser_.distance();
}

void Vl53l1xSensor::clearInterrupt() {
    laser_.clearInterrupt();
}

SensorManager::SensorManager(IIMUSensor &imu, IDistanceSensor &laser)
    : imu_(imu),
      laser_(laser),
      last_distance_(0),
      laser_valid_(false),
      is_calibrated_(false),
      laser_offset_(0),
      roll_offset_(0.0f),
      pitch_offset_(0.0f),
      yaw_offset_(0.0f) {}

void SensorManager::init() {
    if (!imu_.begin()) {
        Serial.println("BNO Fail (addr 0x28)");
        while (1) { delay(1000); }
    }
    imu_.setExtCrystalUse(true);
    delay(100);
    Serial.println("BNO055 initialized");

    if (!laser_.begin()) {
        Serial.println("Laser init failed. Halting.");
        while (1) { delay(1000); }
    }
    laser_.startRanging();
    laser_.setTimingBudget(50);
    Serial.println("Laser initialized");
}

void SensorManager::read(SensorSample &sample) {
    sensors_event_t event;
    imu_.getEvent(&event);

    uint8_t s = 0, g = 0, a = 0, m = 0;
    imu_.getCalibration(&s, &g, &a, &m);
    sample.event = event;
    sample.s = s;
    sample.g = g;
    sample.a = a;
    sample.m = m;

    if (laser_.dataReady()) {
        int16_t newDist = laser_.distance();
        if (newDist != -1) {
            last_distance_ = newDist;
            laser_valid_ = true;
        }
        laser_.clearInterrupt();
    }

    sample.rawDist = last_distance_;
}

void SensorManager::computePose(const SensorSample &sample, RelativePose &out) const {
    out.relDist = sample.rawDist - laser_offset_;
    out.relRoll = sample.event.orientation.y - roll_offset_;
    out.relPitch = sample.event.orientation.z - pitch_offset_;
    out.relYaw = sample.event.orientation.x - yaw_offset_;

    if (out.relYaw > 180.0f) out.relYaw -= 360.0f;
    if (out.relYaw < -180.0f) out.relYaw += 360.0f;
}

DeviceFacing SensorManager::facingDirection(const RelativePose &pose) const {
    const float kFacingThresholdDeg = 60.0f;

    float absPitch = fabsf(pose.relPitch);
    float absRoll = fabsf(pose.relRoll);

    if (absPitch >= absRoll) {
        return (pose.relPitch >= kFacingThresholdDeg) ? FACING_UP : FACING_DOWN;
    }

    return (pose.relRoll >= kFacingThresholdDeg) ? FACING_RIGHT : FACING_LEFT;
}

void SensorManager::updateCalibrationStatus(const SensorSample &sample) {
    is_calibrated_ = (sample.s >= 2);
}

bool SensorManager::isCalibrated() const {
    return is_calibrated_;
}

bool SensorManager::laserValid() const {
    return laser_valid_;
}

int16_t SensorManager::lastDistanceMm() const {
    return last_distance_;
}

int16_t SensorManager::laserOffset() const {
    return laser_offset_;
}

float SensorManager::rollOffset() const {
    return roll_offset_;
}

float SensorManager::pitchOffset() const {
    return pitch_offset_;
}

float SensorManager::yawOffset() const {
    return yaw_offset_;
}

void SensorManager::setLaserOffset(int16_t value) {
    laser_offset_ = value;
}

void SensorManager::setRollOffset(float value) {
    roll_offset_ = value;
}

void SensorManager::setPitchOffset(float value) {
    pitch_offset_ = value;
}

void SensorManager::setYawOffset(float value) {
    yaw_offset_ = value;
}

} // namespace sensors
} // namespace liftrr
