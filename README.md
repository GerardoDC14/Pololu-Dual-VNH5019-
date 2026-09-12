# Pololu Dual VNH5019 + ESP32

Control del motor 1 por serial con un **ESP32-WROOM-32** y el driver
**Pololu Dual VNH5019**. Permite seleccionar el sentido de giro, ajustar el duty
y detener el motor. El motor 2 queda deshabilitado.

- [Código Arduino](arduino/esp32_motor1/esp32_motor1.ino)
- [Manual del driver (PDF)](dual_vnh5019_motor_driver_shield.PDF)

## Hardware

- ESP32 DevKit con módulo ESP32-WROOM-32.
- Pololu Dual VNH5019.
- Motor DC con escobillas.
- Fuente compatible con el voltaje y la corriente del motor.
- Cable USB y conexiones para señales y potencia.

Este pinout corresponde al ESP32 original; no se debe trasladar directamente
a un ESP32-C3 o ESP32-S3.

## Conexiones

![Conexiones del VNH5019 con un microcontrolador](docs/images/conexiones-driver.jpg)

*Diagrama general de Pololu, manual p. 20. Muestra ambos canales; este programa
utiliza únicamente M1. Para el ESP32, VDD se conecta a 3.3 V. Las conexiones
específicas se indican en la tabla siguiente.*

### ESP32 → driver

Usar los nombres impresos en el header lateral del driver. Los números de la
tabla son **GPIO**, no posiciones físicas del conector del ESP32.

| ESP32 | VNH5019 | Función |
| --- | --- | --- |
| 3V3 | VDD | Alimentación de los pull-ups de EN/DIAG |
| GND | GND | Tierra común |
| GPIO 25 | M1INA | Sentido de giro, entrada A |
| GPIO 26 | M1INB | Sentido de giro, entrada B |
| GPIO 27 | M1PWM | PWM del motor 1 |
| GPIO 32 | M1EN/DIAG | Habilitación y lectura de falla del motor 1 |
| GPIO 18 | M2INA | Entrada A del motor 2, mantenida en LOW |
| GPIO 19 | M2INB | Entrada B del motor 2, mantenida en LOW |
| GPIO 23 | M2PWM | PWM del motor 2, mantenido en LOW |
| GPIO 33 | M2EN/DIAG | Motor 2 deshabilitado con LOW |

GPIO 32 debe estar conectado: el programa lo usa como salida LOW para
deshabilitar M1 y como entrada para liberar la habilitación y leer fallas.
El nivel HIGH lo establece el pull-up del driver a VDD; no se fuerza desde el ESP32.
Las entradas de control del VNH5019 aceptan lógica de 3.3 V.

### Alimentación y motor

| Conexión | Terminal del driver |
| --- | --- |
| Positivo de la fuente del motor | VIN grande |
| Negativo de la fuente del motor | GND grande |
| Terminales del motor 1 | M1A y M1B |
| Motor 2 | Sin conectar |
| VOUT, M1CS y M2CS | Sin conectar |

![Distribución de alimentación del driver](docs/images/alimentacion-driver.jpg)

*Distribución de alimentación, manual de Pololu p. 22. VOUT corresponde a la
alimentación del motor después de la protección de polaridad; no es una salida
regulada de 3.3 V.*

- Quitar el jumper **ARDVIN=VOUT**.
- Alimentar el ESP32 por USB y el driver por las terminales grandes VIN/GND.
- Conectar las tierras de ESP32, driver y fuente del motor.
- Llevar la corriente del motor por cable y terminales adecuados, no por el ESP32
  ni por un protoboard.
- El driver admite 5.5–24 V. Pololu no recomienda baterías de 24 V nominales:
  cargadas pueden superar ese voltaje y activar la protección.
- Dejar las salidas de corriente M1CS/M2CS desconectadas. Pueden superar 3.3 V;
  no se usan en este programa.

El manual especifica hasta 12 A por canal, condicionado por la disipación térmica,
y 30 A pico. Las terminales incluidas tienen una capacidad nominal de 16 A.
La selección de fuente, motor y cableado debe considerar la corriente de arranque
y de bloqueo, no solamente la corriente sin carga. Ver pp. 9 y 14–15 del manual.

## Cargar el programa

1. Instalar **esp32 by Espressif Systems**, versión **3.x**, desde el administrador
   de placas de Arduino IDE.
2. Abrir [`esp32_motor1.ino`](arduino/esp32_motor1/esp32_motor1.ino).
3. Seleccionar la placa correspondiente al ESP32-WROOM-32; normalmente
   **ESP32 Dev Module**, y el puerto USB.
4. Cargar el programa. No requiere la librería de Pololu.
5. Abrir el monitor serial a **115200 baud**, con **Nueva línea** o **Ambos NL y CR**.

Al iniciar, M1 permanece deshabilitado y el duty configurado es 30%.
La salida serial muestra:

```text
M1 detenido. Duty: 30%.
adelante | atras | parar | duty 0..100 | estado | ayuda
```

## Comandos seriales

Enviar un comando por línea. Se aceptan mayúsculas y minúsculas.

| Comando | Resultado |
| --- | --- |
| `adelante` | Selecciona avance: INA=HIGH, INB=LOW |
| `atras` | Selecciona reversa: INA=LOW, INB=HIGH |
| `duty 60` | Configura el duty al 60%; admite enteros de 0 a 100 |
| `parar` o `s` | Deshabilita M1 y cancela el movimiento pendiente |
| `estado` | Muestra dirección solicitada, duty, PWM aplicado, espera y falla |
| `ayuda` | Muestra los comandos |

“Avance” y “reversa” dependen de cómo estén conectados los cables del motor;
no indican un sentido horario absoluto.

Ejemplo de operación, enviando cada línea cuando corresponda:

```text
duty 40
adelante
estado
duty 60
atras
parar
```

Cambiar el duty mientras está detenido no lo arranca por sí solo. Si ya se envió
un comando de dirección, un duty mayor que cero habilita el movimiento en ese
sentido. Durante el giro, el nuevo duty se aplica con una rampa.

`duty 0` ejecuta un paro y borra la dirección solicitada. Para volver a moverlo,
configurar un duty mayor que cero y enviar `adelante` o `atras`.

En `estado`, la dirección vale `0` para detenido, `1` para avance y `-1` para
reversa. Es el estado solicitado al programa, no una medición del movimiento.
El PWM aplicado se muestra en la escala 0–255.

## PWM y rampas

La configuración está al inicio del sketch:

```cpp
const uint32_t FRECUENCIA_PWM = 20000;
const uint32_t PASO_RAMPA_MS = 10;
const uint32_t PAUSA_CAMBIO_MS = 1000;
```

La frecuencia es **20 kHz**, con resolución de **8 bits**. El porcentaje enviado
por serial se convierte a una cuenta PWM de 0 a 255 con redondeo:

```cpp
objetivo = (dutyPorcentaje * 255 + 50) / 100;
```

### Arranque y ajuste de duty

El PWM aplicado se acerca al objetivo **una cuenta cada 10 ms**, tanto al subir
como al bajar. Esto equivale aproximadamente a 39.2 puntos porcentuales de duty
por segundo. El tiempo nominal de la rampa es:

```text
tiempo ≈ |PWM objetivo − PWM inicial| × 10 ms
```

| Cambio solicitado | Cuentas PWM | Tiempo aproximado |
| --- | --- | --- |
| 0 → 30% | 0 → 77 | 0.77 s |
| 0 → 60% | 0 → 153 | 1.53 s |
| 30 → 60% | 77 → 153 | 0.76 s |
| 60 → 20% | 153 → 51 | 1.02 s |
| 0 → 100% | 0 → 255 | 2.55 s |

Son tiempos nominales: el programa usa `millis()` y ejecuta como máximo un paso
por actualización. La carga del loop puede alargarlos; el primer paso también
depende del tiempo transcurrido desde la actualización anterior.
Reducir `PASO_RAMPA_MS` acelera la rampa; aumentarlo la hace más lenta.

### Cambio de sentido

Al recibir el sentido contrario, el programa:

1. Deshabilita el puente y lleva el PWM a cero **sin rampa de bajada**.
2. Espera a completar **1000 ms desde el último corte de una salida con PWM activo**.
3. Cambia INA/INB y libera EN/DIAG.
4. Sube desde cero hasta el duty configurado con la misma rampa de 10 ms por cuenta.

Por ejemplo, invertir con duty de 60% toma aproximadamente **1 s de espera +
1.53 s de rampa** para alcanzar el nuevo objetivo. Si antes se envió `parar`,
el tiempo ya transcurrido desde el corte cuenta dentro del segundo de espera.
Durante la espera se siguen procesando comandos; `parar` cancela la inversión.

La espera se ajusta con `PAUSA_CAMBIO_MS`. Es una pausa fija: no hay encoder
para confirmar que el eje ya se detuvo. El tiempo necesario depende del motor,
la inercia y la carga.

### Paro y fallas

`parar`, `s` y `duty 0` cortan la salida inmediatamente, sin rampa ni frenado
activo. El motor queda libre y puede seguir girando por inercia.

Una falla detectada en EN/DIAG durante la operación o un error al configurar o
actualizar el PWM deshabilita M1 y bloquea nuevos comandos de movimiento hasta
reset. Un paro normal sí permite volver a arrancar por serial.

La rampa controla el **duty**, no la velocidad ni la corriente. El programa no
implementa regulación de RPM, límite de corriente por software ni lectura de M1CS.

## Archivos y referencias

```text
README.md
arduino/
  README.md
  esp32_motor1/
    esp32_motor1.ino
docs/images/
  conexiones-driver.jpg
  alimentacion-driver.jpg
dual_vnh5019_motor_driver_shield.PDF
```

El programa fue probado en hardware por el autor. También se verificaron mediante
simulación de las funciones Arduino el procesamiento de comandos, las rampas,
el cambio de sentido, el paro y el bloqueo por falla. Esto no caracteriza la
respuesta mecánica ni térmica del conjunto motor-driver.

- [Manual incluido](dual_vnh5019_motor_driver_shield.PDF), secciones 3.c, 4.b y 8.
- [Conexiones del VNH5019 — Pololu](https://www.pololu.com/docs/0J49/4.b).
- [API LEDC — Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html).

El PDF y los dos diagramas son material de **Pololu Corporation**. Las imágenes
se extrajeron de las páginas 20 y 22 del manual incluido, sin modificar su contenido.
