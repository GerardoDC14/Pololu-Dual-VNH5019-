#pragma once
#include <cstdint>

struct TwoWire {
  bool begin(int = -1, int = -1, uint32_t = 0) { return true; }
  bool setClock(uint32_t) { return true; }
};

inline TwoWire Wire;
