# Control RC sin IMU

Basado en el ZIP Pololu-Dual-VNH5019-normal proporcionado. Esta entrega contiene un único sketch, sin variantes anteriores con sensores.

Abra `esp32_omni4_sin_imu/esp32_omni4_sin_imu.ino` en Arduino IDE, conservando los cuatro encabezados en la misma carpeta. Use la misma placa y entorno ESP32 del código original (API LEDC de Arduino-ESP32 3.x).

## Canales

| Canal | Función | GPIO |
| --- | --- | --- |
| CH1 | Giro | 27 |
| CH2 | Sin uso | — |
| CH3 | Avance/retroceso | 15 |
| CH4 | Lateral | 34 |
| CH5 | Solenoide, lógica original | 35 |
| CH6 | Solo lectura para diagnóstico; sin acción | 12 |

Se eliminó toda inicialización y lectura de la IMU, I2C, referencia angular, transformación de coordenadas y corrección automática. CH6 ya no ordena 0°/180°. La IMU puede permanecer desconectada.

Se conserva la mezcla de motores del código base, los pines, sentidos, filtros RC, rampas, límite de PWM del 60 %, solenoide y diagnóstico de drivers. Centre CH1/CH3/CH4 durante 500 ms para habilitar. Centrar los tres mandos detiene los motores. La recepción y su timeout global de 100 ms se mantienen como en el original.

El movimiento depende directamente del RC en los ejes del robot. Sin IMU no hay corrección automática de orientación ni garantía de trayectoria recta ante diferencias mecánicas entre ruedas.

## Verificación

Prueba de integración del sketch con hardware simulado, sin bibliotecas de IMU ni Wire: arranque, tres ejes, CH6 sin acción, parada al centrar, pérdida global de recepción y fallo DIAG. Ejecutar desde esta carpeta:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I tests/mocks tests/control.cpp -o /tmp/control_sin_imu
/tmp/control_sin_imu
```

La prueba pasó. No se realizó compilación con el core real de ESP32 ni prueba física en el robot.
