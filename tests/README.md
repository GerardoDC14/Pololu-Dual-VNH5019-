# Pruebas de lógica

Desde la raíz del repositorio:

```bash
g++ -std=c++17 -Wall -Wextra -Werror tests/control_omni_test.cpp -o /tmp/control_omni_test
/tmp/control_omni_test

g++ -std=c++17 -Wall -Wextra -Werror tests/control_campo_test.cpp -o /tmp/control_campo_test
/tmp/control_campo_test

g++ -std=c++17 -Wall -Wextra -Werror -Itests/mocks tests/ppm_test.cpp -o /tmp/ppm_test
/tmp/ppm_test
```

`control_omni_test.cpp`: signos de la mezcla, normalización, zona muerta,
pulso de solenoide por cambio, secuencia OFF/ON heredada, rampa, inversión y
desbordamiento del temporizador.

`control_campo_test.cpp`: envoltura angular, transformación campo→robot,
selector CH6 y referencia angular integrada por la palanca de giro.

`ppm_test.cpp`: trama PPM simulada, separación de canales, conteo y rechazo de
intervalos fuera de rango.
Los headers de `mocks/` sólo se usan para validar la sintaxis en la computadora;
no sustituyen las librerías reales ni prueban comunicación I²C.

Compilar el sketch con el core ESP32 instalado:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 arduino/esp32_omni4_rc
arduino-cli compile --fqbn esp32:esp32:esp32 arduino/esp32_omni4_bno055
arduino-cli compile --fqbn esp32:esp32:esp32 arduino/esp32_omni4_mpu6050
```

El pinout, los canales y el failsafe están descritos en el [README](../README.md#versiones-disponibles).
