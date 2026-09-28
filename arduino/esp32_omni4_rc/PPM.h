#pragma once

#include <Arduino.h>

namespace ppm {

constexpr uint8_t MAX_CHANNELS = 6;

volatile uint16_t ppmChannels[MAX_CHANNELS];
volatile uint8_t currentChannel = 0;
volatile uint8_t receivedChannels = 0;
volatile uint32_t lastEdgeTime = 0;
volatile bool newPPMFrame = false;
portMUX_TYPE ppmMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR ppmISR() {
  uint32_t now = micros();
  uint32_t interval = now - lastEdgeTime;

  portENTER_CRITICAL_ISR(&ppmMux);
  lastEdgeTime = now;

  // Un intervalo largo separa dos tramas PPM.
  if (interval > 3000) {
    if (currentChannel > 0) {
      receivedChannels = currentChannel;
      newPPMFrame = true;
    }
    currentChannel = 0;
    portEXIT_CRITICAL_ISR(&ppmMux);
    return;
  }

  if (interval >= 800 && interval <= 2200 && currentChannel < MAX_CHANNELS) {
    ppmChannels[currentChannel++] = interval;
  }
  portEXIT_CRITICAL_ISR(&ppmMux);
}

inline void comenzar(uint8_t pin) {
  for (uint8_t i = 0; i < MAX_CHANNELS; ++i) ppmChannels[i] = 1500;
  currentChannel = 0;
  receivedChannels = 0;
  newPPMFrame = false;
  lastEdgeTime = micros();
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin), ppmISR, RISING);
}

inline bool leer(uint16_t salida[MAX_CHANNELS], uint8_t &cantidad) {
  portENTER_CRITICAL(&ppmMux);
  bool hayFrame = newPPMFrame;
  cantidad = receivedChannels > MAX_CHANNELS ? MAX_CHANNELS : receivedChannels;
  if (hayFrame) {
    for (uint8_t i = 0; i < cantidad; ++i) salida[i] = ppmChannels[i];
    newPPMFrame = false;
  }
  portEXIT_CRITICAL(&ppmMux);
  return hayFrame;
}

} // namespace ppm
