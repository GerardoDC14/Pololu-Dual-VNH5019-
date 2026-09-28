# Cuatro motores, receptor PPM y solenoide

Configuración sin IMU para una ESP32 DevKit V1 con módulo WROOM-32, dos Pololu
Dual VNH5019, cuatro ruedas omni y un receptor configurado con salida PPM
compuesta. El [sketch](../arduino/esp32_omni4_rc/esp32_omni4_rc.ino) usa un solo
GPIO para recibir hasta seis canales consecutivos.

## Distribución de GPIO

| Señal | GPIO |
| --- | ---: |
| PPM del receptor | 27 |
| Motor 1: INA / INB / PWM | 5 / 18 / 22 |
| Motor 2: INA / INB / PWM | 17 / 19 / 25 |
| Motor 3: INA / INB / PWM | 26 / 21 / 14 |
| Motor 4: INA / INB / PWM | 32 / 33 / 23 |
| EN/DIAG compartido | 16 |
| Mando de solenoide | 13 |

La entrada PPM debe estar limitada a 3.3 V. VDD de los drivers va a 3V3 y el
receptor, ESP32, drivers y etapa del solenoide deben compartir referencia GND.
El código interpreta EN/DIAG bajo como falla y corta los cuatro motores.

## Asignación PPM

| Canal | Función |
| --- | --- |
| CH1 | Giro |
| CH2 | Sin uso; sólo diagnóstico |
| CH3 | Avance / reversa |
| CH4 | Movimiento lateral |
| CH5 | Pulso del solenoide al cambiar la palanca |

Los canales de movimiento se convierten de 1000–2000 µs a −1…1, con centro en
1500 µs y zona muerta de ±20 µs. La mezcla produce referencias para las ruedas
frontal, izquierda, trasera y derecha. M2 está invertido en software.

Un intervalo entre flancos mayor de 3000 µs marca la separación entre tramas.
Los intervalos de canal aceptados son 800–2200 µs. Se exigen cinco canales y una
trama nueva dentro de los últimos 100 ms; de lo contrario, los motores y el
solenoide se apagan.

## Salida del solenoide

GPIO 13 produce un pulso activo en HIGH de 150 ms cada vez que CH5 cambia de
estado, tanto OFF→ON como ON→OFF. Mantener la palanca en una posición no prolonga
el pulso. El primer valor recibido sólo establece la referencia, por lo que el
arranque o la recuperación de señal no disparan la salida.

GPIO 13 debe conectarse a una etapa de potencia, no directamente a la bobina.
Para una bobina DC usar un MOSFET lógico o transistor dimensionado, resistencia
de compuerta/base, resistencia que mantenga la etapa apagada durante reset y un
diodo de rueda libre en paralelo con la bobina. La fuente del solenoide debe
soportar su corriente. Para un módulo activo en LOW, cambiar
`SOLENOIDE_ACTIVO_EN_HIGH` a `false`.

## Puesta en marcha

1. Cargar con Arduino-ESP32 3.x y la placa **ESP32 Dev Module**.
2. Dejar desconectada la potencia de motores y solenoide.
3. Abrir el monitor serial a 115200 baud y confirmar los valores CH1–CH5.
4. Apagar el transmisor y comprobar que `PPM=FAIL` aparece en menos de 100 ms.
5. Cambiar CH5 en ambos sentidos y medir en GPIO 13 un pulso de 150 ms.
6. Probar la etapa del solenoide con una carga segura antes de conectar la bobina.
7. Probar los motores con las ruedas elevadas y confirmar signos y polaridades.

La salida actual cambia el duty y el sentido sin rampa. Antes de pruebas con alta
inercia o corriente conviene reincorporar limitación de aceleración y una pausa
de inversión.
