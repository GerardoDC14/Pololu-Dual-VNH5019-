#pragma once

#include <Arduino.h>

namespace pwm {

constexpr uint8_t MAX_CHANNELS = 6;
constexpr uint8_t UNASSIGNED_PIN = 255;

// Pins for CH1, CH2 (unused), CH3, CH4, CH5, CH6
const uint8_t DEFAULT_PINS[MAX_CHANNELS] = {27, UNASSIGNED_PIN, 15, 34, 35, 12};

volatile uint16_t pwmChannels[MAX_CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
volatile uint32_t riseTimes[MAX_CHANNELS] = {0};
volatile uint32_t lastUpdateUs[MAX_CHANNELS] = {0};
volatile uint32_t publicadaUs = 0;
volatile bool pendiente = false;
volatile uint32_t tramasValidas = 0;
volatile uint32_t tramasInvalidas = 0;

uint8_t channelPins[MAX_CHANNELS];
portMUX_TYPE pwmMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR __attribute__((noinline)) handleInterrupt(uint8_t ch) {
  if (ch >= MAX_CHANNELS || channelPins[ch] == UNASSIGNED_PIN) return;
  
  uint32_t now = micros();
  bool state = digitalRead(channelPins[ch]);

  portENTER_CRITICAL_ISR(&pwmMux);
  if (state) {
    riseTimes[ch] = now;
  } else {
    uint32_t pulseWidth = now - riseTimes[ch];
    if (pulseWidth >= 800 && pulseWidth <= 2200) {
      pwmChannels[ch] = pulseWidth;
      lastUpdateUs[ch] = now;
      publicadaUs = now;
      pendiente = true;
      tramasValidas++;
    } else {
      tramasInvalidas++;
    }
  }
  portEXIT_CRITICAL_ISR(&pwmMux);
}

void IRAM_ATTR isr0() { handleInterrupt(0); }
void IRAM_ATTR isr1() { handleInterrupt(1); }
void IRAM_ATTR isr2() { handleInterrupt(2); }
void IRAM_ATTR isr3() { handleInterrupt(3); }
void IRAM_ATTR isr4() { handleInterrupt(4); }
void IRAM_ATTR isr5() { handleInterrupt(5); }

typedef void (*ISRFunc)();
const ISRFunc isrTable[MAX_CHANNELS] = {isr0, isr1, isr2, isr3, isr4, isr5};

inline void comenzar(const uint8_t pins[] = DEFAULT_PINS, uint8_t numPins = MAX_CHANNELS) {
  portENTER_CRITICAL(&pwmMux);
  publicadaUs = micros();
  pendiente = false;
  tramasValidas = 0;
  tramasInvalidas = 0;

  for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
    pwmChannels[i] = 1500;
    riseTimes[i] = 0;
    lastUpdateUs[i] = publicadaUs;
    channelPins[i] = (i < numPins) ? pins[i] : UNASSIGNED_PIN;
  }
  portEXIT_CRITICAL(&pwmMux);

  for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
    if (channelPins[i] != UNASSIGNED_PIN) {
      pinMode(channelPins[i], INPUT);
      attachInterrupt(digitalPinToInterrupt(channelPins[i]), isrTable[i], CHANGE);
    }
  }
}

inline void comenzar(uint8_t dummyPin) {
  (void)dummyPin;
  comenzar(DEFAULT_PINS, MAX_CHANNELS);
}

// Fixed function signature: Removed default parameter to prevent overload ambiguity
inline bool leer(uint16_t salida[MAX_CHANNELS], uint8_t &cantidad, uint32_t &marcaUs, uint8_t *canalesTrama) {
  portENTER_CRITICAL(&pwmMux);
  bool nueva = pendiente;
  cantidad = MAX_CHANNELS;
  marcaUs = publicadaUs;
  if (canalesTrama) *canalesTrama = MAX_CHANNELS;

  for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
    salida[i] = pwmChannels[i];
  }
  pendiente = false;
  portEXIT_CRITICAL(&pwmMux);
  return nueva;
}

inline bool leer(uint16_t salida[MAX_CHANNELS], uint8_t &cantidad, uint32_t &marcaUs) {
  return leer(salida, cantidad, marcaUs, nullptr);
}

inline bool leer(uint16_t salida[MAX_CHANNELS], uint8_t &cantidad) {
  uint32_t marcaUs;
  return leer(salida, cantidad, marcaUs);
}

inline void diagnostico(uint32_t &validas, uint32_t &invalidas, uint16_t &intervalo, uint8_t *canales = nullptr) {
  portENTER_CRITICAL(&pwmMux);
  validas = tramasValidas;
  invalidas = tramasInvalidas;
  intervalo = 0;
  if (canales) *canales = MAX_CHANNELS;
  portEXIT_CRITICAL(&pwmMux);
}

} // namespace pwm

namespace ppm = pwm;