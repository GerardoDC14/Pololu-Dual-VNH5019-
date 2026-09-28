#pragma once
#include <cstdint>

struct sensors_vec_t { float x = 0, y = 0, z = 0; };
struct sensors_event_t {
  sensors_vec_t orientation;
  sensors_vec_t gyro;
  sensors_vec_t acceleration;
  float temperature = 0;
};
