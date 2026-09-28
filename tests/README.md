# Pruebas de lógica

Desde la raíz del repositorio:

```bash
g++ -std=c++17 -Wall -Wextra -Werror tests/control_omni_test.cpp -o /tmp/control_omni_test
/tmp/control_omni_test

g++ -std=c++17 -Wall -Wextra -Werror -Itests/mocks tests/ppm_test.cpp -o /tmp/ppm_test
/tmp/ppm_test
```

`control_omni_test.cpp`: signos de la mezcla, normalización, zona muerta,
pulso de solenoide por cambio, secuencia OFF/ON heredada, rampa, inversión y
desbordamiento del temporizador.

`ppm_test.cpp`: trama PPM simulada, separación de canales, conteo y rechazo de
intervalos fuera de rango.
`mocks/Arduino.h` sólo se usa para esta prueba en la computadora.

Compilar el sketch con el core ESP32 instalado:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 arduino/esp32_omni4_rc
```

El pinout, los canales y el failsafe están descritos en el [README](../README.md#configuración-actual-devkit-v1-ppm-y-solenoide).
