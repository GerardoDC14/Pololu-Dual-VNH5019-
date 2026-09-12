# Pruebas de lógica

Desde la raíz del repositorio:

```bash
g++ -std=c++17 -Wall -Wextra -Werror tests/control_omni_test.cpp -o /tmp/control_omni_test
/tmp/control_omni_test

g++ -std=c++17 -Wall -Wextra -Werror -Itests/mocks tests/receptor_pwm_test.cpp -o /tmp/receptor_pwm_test
/tmp/receptor_pwm_test
```

`control_omni_test.cpp`: signos de la mezcla, normalización, zona muerta,
secuencia OFF/ON, rampa, inversión y desbordamiento del temporizador.

`receptor_pwm_test.cpp`: flancos PWM simulados, validación de periodo y ancho,
timeout, recuperación y desbordamiento de `micros()`.
`mocks/Arduino.h` sólo se usa para esta prueba en la computadora.

Compilar el sketch con el core ESP32 instalado:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 arduino/esp32_omni4_rc
```

`SOLO_RECEPTOR` selecciona lectura o control de motores. El cableado, polaridades
y failsafe se ajustan con el receptor y la base según [omni4.md](../docs/omni4.md).
