#pragma once

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L1X.h>
#include <Wire.h>

namespace liftrr {
namespace sensors {

// Raw IMU + laser sample.
struct SensorSample {
    sensors_event_t event;  // orientation etc. from BNO055

    uint8_t s = 0;          // system calibration status
    uint8_t g = 0;          // gyro calibration status
    uint8_t a = 0;          // accel calibration status
    uint8_t m = 0;          // mag calibration status

    int16_t rawDist = 0;    // latest raw distance reading (mm)
};

// Pose relative to calibration offsets.
struct RelativePose {
    int16_t relDist;   // mm
    float   relRoll;   // deg
    float   relPitch;  // deg
    float   relYaw;    // deg (normalized to [-180, 180])
};

enum DeviceFacing {
    FACING_UP,
    FACING_DOWN,
    FACING_LEFT,
    FACING_RIGHT,
};

class IIMUSensor {
public:
    virtual ~IIMUSensor() = default;
    virtual bool begin() = 0;
    virtual void setExtCrystalUse(bool use) = 0;
    virtual void getEvent(sensors_event_t *event) = 0;
    virtual void getCalibration(uint8_t *s, uint8_t *g, uint8_t *a, uint8_t *m) = 0;
};

class IDistanceSensor {
public:
    virtual ~IDistanceSensor() = default;
    virtual bool begin() = 0;
    virtual void startRanging() = 0;
    virtual void setTimingBudget(uint16_t budgetMs) = 0;
    virtual bool dataReady() = 0;
    virtual int16_t distance() = 0;
    virtual void clearInterrupt() = 0;
};

class Bno055Sensor : public IIMUSensor {
public:
    explicit Bno055Sensor(Adafruit_BNO055 &imu,
                          adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
    bool begin() override;
    void setExtCrystalUse(bool use) override;
    void getEvent(sensors_event_t *event) override;
    void getCalibration(uint8_t *s, uint8_t *g, uint8_t *a, uint8_t *m) override;

private:
    Adafruit_BNO055 &imu_;
    adafruit_bno055_opmode_t mode_;
};

class Vl53l1xSensor : public IDistanceSensor {
public:
    Vl53l1xSensor(Adafruit_VL53L1X &laser, uint8_t address, TwoWire *wire, bool debug);
    bool begin() override;
    void startRanging() override;
    void setTimingBudget(uint16_t budgetMs) override;
    bool dataReady() override;
    int16_t distance() override;
    void clearInterrupt() override;

private:
    Adafruit_VL53L1X &laser_;
    uint8_t address_;
    TwoWire *wire_;
    bool debug_;
};

class SensorManager {
public:
    SensorManager(IIMUSensor &imu, IDistanceSensor &laser);

    void init();
    void read(SensorSample &out);
    void computePose(const SensorSample &sample, RelativePose &out) const;
    DeviceFacing facingDirection(const RelativePose &pose) const;

    void updateCalibrationStatus(const SensorSample &sample);

    bool isCalibrated() const;
    bool laserValid() const;
    int16_t lastDistanceMm() const;

    int16_t laserOffset() const;
    float rollOffset() const;
    float pitchOffset() const;
    float yawOffset() const;

    void setLaserOffset(int16_t value);
    void setRollOffset(float value);
    void setPitchOffset(float value);
    void setYawOffset(float value);

private:
    IIMUSensor &imu_;
    IDistanceSensor &laser_;
    int16_t last_distance_;
    bool laser_valid_;
    bool is_calibrated_;
    int16_t laser_offset_;
    float roll_offset_;
    float pitch_offset_;
    float yaw_offset_;
};

} // namespace sensors
} // namespace liftrr
