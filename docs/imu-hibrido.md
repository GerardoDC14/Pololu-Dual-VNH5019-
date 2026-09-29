# Control híbrido orientado al campo

Las variantes BNO055 y MPU6050 conservan la mezcla de cuatro motores, pero
separan la dirección de traslación de la orientación del chasis. Al encender,
la orientación física del robot se define como 0°. El robot debe estar inmóvil
y con el solenoide apuntando hacia la portería contraria. El frente del chasis
es la diagonal entre las ruedas F e I; ése es el eje que la IMU conserva como
referencia inicial.

## Canales

| Canal | Función |
| --- | --- |
| CH1 | Modifica continuamente la referencia angular |
| CH2 | Sin uso |
| CH3 | Avance/reversa respecto al campo |
| CH4 | Izquierda/derecha respecto al campo |
| CH5 | Pulso de 150 ms del solenoide por cambio de estado |
| CH6 | Selector de rumbo: extremo alto 0°, centro sin orden, extremo bajo 180° |

CH6 actúa por transición. Entrar al extremo alto fija la referencia en 0° y
entrar al extremo bajo la fija en 180°. Mantener la palanca en un extremo no
bloquea CH1. Para repetir una orden se regresa primero al centro. La posición de
CH6 durante el arranque sólo se registra y no genera un giro inesperado.

## Transformación de la palanca

CH3 y CH4 forman el vector de traslación expresado respecto al campo. Si `yaw`
es el ángulo antihorario del chasis respecto al frente inicial:

```text
avanceRobot =  cos(yaw)·avanceCampo + sin(yaw)·lateralCampo
lateralRobot = -sin(yaw)·avanceCampo + cos(yaw)·lateralCampo
```

Los valores relativos al robot entran después a la mezcla diagonal F–I. Por ello,
empujar CH3 hacia adelante sigue moviendo el robot hacia la portería aunque el
chasis esté girado.

CH1 no manda directamente PWM de giro. Con una velocidad máxima inicial de
120°/s, desplaza la referencia angular mientras permanece fuera de la zona
muerta. Al soltarlo, la referencia deja de cambiar y un controlador PID mantiene
ese ángulo.

## Cableado de las variantes IMU

GPIO 21 y 22 quedan reservados para I²C. Frente al sketch sin IMU cambian dos
señales de motor:

| Función | Sin IMU | Con BNO055 o MPU6050 |
| --- | ---: | ---: |
| M1 PWM | GPIO 22 | GPIO 4 |
| M3 INB | GPIO 21 | GPIO 2 |
| I²C SDA | — | GPIO 21 |
| I²C SCL | — | GPIO 22 |

El resto se conserva: PPM 27, solenoide 13, EN/DIAG 16, M1 INA/INB 5/18,
M2 17/19/25, M3 INA/PWM 26/14 y M4 32/33/23.

GPIO 2 y GPIO 4 son pines de arranque del ESP32. Sólo deben conectarse a entradas
de alta impedancia del driver, sin pull-ups externos. Comprobar que ambos queden
en LOW durante el encendido antes de conectar potencia a los motores.

La IMU y el ESP32 se alimentan a 3.3 V, salvo que el módulo específico incluya
regulador y adaptación de nivel claramente documentados. SDA y SCL requieren
pull-ups a 3.3 V; muchos módulos ya los incluyen.

Montar la IMU rígidamente, aproximadamente horizontal y con su eje Z vertical.
Si se cambia su orientación física será necesario ajustar el eje y
`SIGNO_YAW`; no basta con cambiar el signo al azar.

## BNO055

El sketch usa la librería **Adafruit BNO055** y el modo `IMUPLUS`, que fusiona
giroscopio y acelerómetro sin usar el magnetómetro. Esto evita que motores y
cables de potencia alteren el rumbo, aunque el yaw relativo todavía puede
derivar. La dirección inicial se resta como cero después de la inicialización.
La dirección I²C inicial es 0x28. Si el módulo no tiene cristal externo de
32.768 kHz, cambiar `BNO_USAR_CRISTAL_EXTERNO` a `false`.

Dependencias del Library Manager:

- Adafruit BNO055
- Adafruit Unified Sensor
- Adafruit BusIO

## MPU6050

El MPU6050 no entrega un yaw absoluto. El sketch mide durante aproximadamente
1.8 s el sesgo de `gyro.z` y después integra la velocidad angular. Mover el robot
durante esa calibración crea un error permanente. Esta versión tendrá más deriva
que la BNO055 y puede necesitar volver a encender o reestablecer el cero entre
pruebas. La dirección I²C inicial es 0x68; para módulos configurados como 0x69,
cambiar `DIRECCION_MPU6050`.

Dependencias del Library Manager:

- Adafruit MPU6050
- Adafruit Unified Sensor
- Adafruit BusIO

## Ajuste y prueba

1. Probar con las ruedas elevadas y sin conectar el solenoide real.
2. Encender con el frente hacia la portería y no mover el robot durante la
   inicialización.
3. Girarlo manualmente en sentido antihorario. El monitor debe mostrar un `yaw`
   positivo. Si ocurre lo contrario, invertir `SIGNO_YAW`.
4. Con CH1 centrado, girar suavemente el robot a mano. Las ruedas deben intentar
   recuperar la referencia, no aumentar el error. Si lo aumentan, detener y
   corregir el signo antes de seguir.
5. Verificar CH3/CH4 con el robot a 0°, 90°, 180° y −90°.
6. Mover CH6 desde el centro hacia cada extremo y comprobar referencias 0° y
   −180° en el monitor.
7. Ajustar primero `KP_RUMBO`, después `KD_RUMBO` y sólo al final
   `KI_RUMBO`. Los valores incluidos son puntos de partida, no una calibración.

Una falla de IMU, una trama PPM de menos de seis canales, 100 ms sin trama o
EN/DIAG bajo detienen los cuatro motores. El solenoide también se apaga al perder
PPM. Mientras el movimiento está bloqueado, la referencia se iguala al yaw
medido para evitar un giro brusco al recuperar la señal.
