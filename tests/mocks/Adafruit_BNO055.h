#pragma once
#include "Wire.h"
#include "Adafruit_Sensor.h"

#define BNO055_ADDRESS_A 0x28
enum adafruit_bno055_opmode_t { OPERATION_MODE_IMUPLUS = 0x08 };

class Adafruit_BNO055 {
public:
  enum adafruit_vector_type_t { VECTOR_GYROSCOPE = 0x14, VECTOR_EULER = 0x1A };
  Adafruit_BNO055(int32_t = -1, uint8_t = BNO055_ADDRESS_A, TwoWire * = &Wire) {}
  bool begin(adafruit_bno055_opmode_t = OPERATION_MODE_IMUPLUS) { return true; }
  void setExtCrystalUse(bool) {}
  bool getEvent(sensors_event_t *, adafruit_vector_type_t) { return true; }
};
