#pragma once
#include <Arduino.h>

namespace receptor {
constexpr uint8_t PINES[4] = {34, 35, 36, 39}; // CH1, CH2, CH4, CH5
constexpr uint32_t TIMEOUT_US = 100000;
struct Canal {
  uint8_t pin;
  volatile uint32_t subida = 0, anterior = 0, periodo = 0, ultima = 0;
  volatile uint16_t ancho = 0;
  volatile uint8_t consecutivos = 0;
  volatile bool alto = false, visto = false;
};
static Canal canales[4];
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void ARDUINO_ISR_ATTR flanco(void *arg) {
  Canal &c = *static_cast<Canal *>(arg);
  uint32_t ahora = micros();
  bool alto = digitalRead(c.pin);
  portENTER_CRITICAL_ISR(&mux);
  if (alto) {
    c.periodo = c.visto ? uint32_t(ahora - c.anterior) : 0;
    c.anterior = c.subida = ahora;
    c.alto = c.visto = true;
  } else if (c.alto) {
    uint32_t ancho = ahora - c.subida;
    c.alto = false;
    c.ancho = ancho <= 65535 ? ancho : 0;
    if (ancho >= 900 && ancho <= 2100 && c.periodo >= 5000 && c.periodo <= 40000) {
      c.ultima = ahora;
      if (c.consecutivos < 3) ++c.consecutivos;
    } else {
      c.consecutivos = 0;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void iniciar() {
  for (int i = 0; i < 4; ++i) {
    canales[i].pin = PINES[i];
    pinMode(PINES[i], INPUT);
    attachInterruptArg(PINES[i], flanco, &canales[i], CHANGE);
  }
}

bool leer(uint16_t pulsos[4]) {
  bool valido = true;
  portENTER_CRITICAL(&mux);
  uint32_t ahora = micros();
  for (int i = 0; i < 4; ++i) {
    pulsos[i] = canales[i].ancho;
    if (canales[i].consecutivos < 3 || uint32_t(ahora - canales[i].ultima) > TIMEOUT_US)
      valido = false;
  }
  portEXIT_CRITICAL(&mux);
  return valido;
}
}
