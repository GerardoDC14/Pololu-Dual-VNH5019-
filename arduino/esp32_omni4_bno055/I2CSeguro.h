#pragma once
#include <Arduino.h>
#include <Wire.h>
namespace bus {
inline bool escribir(uint8_t direccion, uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(direccion);
  if (Wire.write(registro) != 1 || Wire.write(valor) != 1) return false;
  return Wire.endTransmission(true) == 0;
}
inline bool leer(uint8_t direccion, uint8_t registro, uint8_t *datos, size_t n) {
  Wire.beginTransmission(direccion);
  if (Wire.write(registro) != 1 || Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(direccion, n, true) != n) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < n; ++i) {
    if (!Wire.available()) return false;
    datos[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}
inline bool esperarId(uint8_t direccion, uint8_t registro, uint8_t esperado, uint32_t limiteMs) {
  const uint32_t inicio = millis();
  do {
    uint8_t id = 0;
    if (leer(direccion, registro, &id, 1) && id == esperado) return true;
    delay(10);
  } while (uint32_t(millis() - inicio) < limiteMs);
  return false;
}
inline int16_t little(const uint8_t *p) {
  return static_cast<int16_t>(uint16_t(p[0]) | (uint16_t(p[1]) << 8));
}
inline int16_t big(const uint8_t *p) {
  return static_cast<int16_t>((uint16_t(p[0]) << 8) | uint16_t(p[1]));
}
} // namespace bus
