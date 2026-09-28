#pragma once
#include <cstdint>
#include <cstdlib>
#define ARDUINO_ISR_ATTR
#define IRAM_ATTR
#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1
#define CHANGE 3
#define RISING 1
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL_ISR(x) ((void)(x))
#define portEXIT_CRITICAL_ISR(x) ((void)(x))
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
inline uint32_t tiempoUs = 0;
inline uint32_t tiempoMs = 0;
inline bool nivel = false;
inline uint32_t micros() { return tiempoUs; }
inline uint32_t millis() { return tiempoMs; }
inline int digitalRead(int) { return nivel; }
inline void digitalWrite(int, int) {}
inline void pinMode(int, int) {}
inline void attachInterruptArg(int, void (*)(void *), void *, int) {}
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void delay(uint32_t) {}
inline bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
inline bool ledcWrite(uint8_t, uint32_t) { return true; }
template <typename T> T constrain(T valor, T minimo, T maximo) {
  return valor < minimo ? minimo : (valor > maximo ? maximo : valor);
}
struct SerialMock {
  void begin(uint32_t) {}
  void println(const char *) {}
  template <typename... Args> void printf(const char *, Args...) {}
};
inline SerialMock Serial;
