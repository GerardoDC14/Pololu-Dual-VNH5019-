#pragma once
#include "Wire.h"
#include "Adafruit_Sensor.h"

#define MPU6050_I2CADDR_DEFAULT 0x68
enum mpu6050_gyro_range_t { MPU6050_RANGE_500_DEG };
enum mpu6050_accel_range_t { MPU6050_RANGE_4_G };
enum mpu6050_bandwidth_t { MPU6050_BAND_21_HZ };

class Adafruit_MPU6050 {
public:
  bool begin(uint8_t = MPU6050_I2CADDR_DEFAULT, TwoWire * = &Wire,
             int32_t = 0) { return true; }
  void setGyroRange(mpu6050_gyro_range_t) {}
  void setAccelerometerRange(mpu6050_accel_range_t) {}
  void setFilterBandwidth(mpu6050_bandwidth_t) {}
  bool getEvent(sensors_event_t *, sensors_event_t *, sensors_event_t *) {
    return true;
  }
};
