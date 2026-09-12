#pragma once
#include <cstdint>
#define ARDUINO_ISR_ATTR
#define INPUT 0
#define CHANGE 3
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL_ISR(x) ((void)(x))
#define portEXIT_CRITICAL_ISR(x) ((void)(x))
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
inline uint32_t tiempoUs = 0;
inline bool nivel = false;
inline uint32_t micros() { return tiempoUs; }
inline int digitalRead(int) { return nivel; }
inline void pinMode(int, int) {}
inline void attachInterruptArg(int, void (*)(void *), void *, int) {}
