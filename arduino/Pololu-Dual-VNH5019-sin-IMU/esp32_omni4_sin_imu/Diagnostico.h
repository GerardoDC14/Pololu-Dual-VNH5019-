#pragma once
#include <Arduino.h>
#include <esp_system.h>
#include <driver/gpio.h>
namespace diagnostico {
inline void salida(uint8_t pin, int nivel) {
  // Arduino-ESP32 3.x ignora digitalWrite antes de pinMode. Precargar el
  // registro GPIO evita pasar por LOW al habilitar un relé activo en LOW.
  gpio_set_level(static_cast<gpio_num_t>(pin), nivel);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, nivel);
}
inline const char *causa(esp_reset_reason_t razon) {
  switch (razon) {
    case ESP_RST_POWERON: return "ENCENDIDO (o perdida completa de alimentacion)";
    case ESP_RST_EXT: return "RESET EXTERNO";
    case ESP_RST_SW: return "RESET POR SOFTWARE";
    case ESP_RST_PANIC: return "PANIC: fallo del programa; conservar backtrace";
    case ESP_RST_INT_WDT: return "WATCHDOG DE INTERRUPCIONES";
    case ESP_RST_TASK_WDT: return "WATCHDOG DE TAREA";
    case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BAJA TENSION (BROWNOUT): revisar alimentacion";
    case ESP_RST_DEEPSLEEP: return "SALIDA DE SUSPENSION";
    default: return "OTRA CAUSA";
  }
}
inline void inicio() {
  const auto razon = esp_reset_reason();
  Serial.printf("\nARRANQUE | RESET=%d | %s\n", int(razon), causa(razon));
  Serial.println("Salidas detenidas. No se reinicia automaticamente ante fallas.");
}
} // namespace diagnostico
