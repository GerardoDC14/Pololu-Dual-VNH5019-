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

## Geometría: el frente es la llanta F

Base en cruz (+), vista superior. El frente del robot apunta hacia la llanta **F**; I queda a la izquierda, T atrás y D a la derecha. La tracción positiva de todas las ruedas es antihoraria.

| Llanta | Posición | Driver | Avance | Lateral | Giro |
| --- | --- | --- | --- | --- | --- |
| F | frente | A / M1 | 0 | +1 | +1 |
| I | izquierda | A / M2 | -1 | 0 | +1 |
| T | atrás | B / M1 | 0 | -1 | +1 |
| D | derecha | B / M2 | +1 | 0 | +1 |

Convención de ejes sin cambios: avance + = hacia el frente, lateral + = hacia la izquierda (hacia I), giro + = antihorario. Con el frente sobre una llanta, avance lo producen I y D, y lateral lo producen F y T; las otras dos ruedas giran libres por sus rodillos. Antes, con el frente sobre la diagonal F–I, las cuatro ruedas contribuían a cada eje.

Consecuencias: la velocidad máxima de traslación es ~30 % menor (1/√2) a igual PWM por rueda, y el solenoide debe quedar físicamente montado apuntando hacia F. Los pines, `invertido` y el orden F, I, T, D no cambian.

## Verificación

Prueba de integración del sketch con hardware simulado, sin bibliotecas de IMU ni Wire: arranque, tres ejes, CH6 sin acción, parada al centrar, pérdida global de recepción y fallo DIAG. Ejecutar desde esta carpeta:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I tests/mocks tests/control.cpp -o /tmp/control_sin_imu
/tmp/control_sin_imu
```

La prueba pasó. No se realizó compilación con el core real de ESP32 ni prueba física en el robot.
