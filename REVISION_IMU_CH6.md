# Actualización V4: signo de giroscopio según registros del robot

Registro aportado: PPM=OK, BAD=0, CAUSA=SIGNO_GYRO, dY=-43.6°, iW=+46.2°.
El cambio de heading y la integral del gyro tienen sentidos opuestos y magnitud
similar. Se cambia SOLO SIGNO_GYRO_Z de -1 a +1, manteniendo SIGNO_YAW=-1.
No se deduce el signo del gyro únicamente de si el módulo está boca abajo.
La lógica CH6 puntual de V3 y las protecciones permanecen activas.

La falla SIGNO_GYRO queda enclavada hasta reset. Después del bloqueo,
IMU=FAIL es resultado de que el código deja de leerla al existir fallaPermanente;
yaw y w impresos son los últimos valores guardados, no lecturas nuevas.
DIAG=FAIL también puede ser consecuencia del software: bloquearPWM() fuerza
PIN_EN_DIAG a LOW. No prueba por sí solo una segunda avería eléctrica.
PPM=OK, contadores RX en aumento y EDAD de 9–19 ms confirman recepción activa
en ese tramo. Los sticks de ese tramo están centrados.

Instalar V4, reiniciar con el robot inmóvil y sticks en neutro 500 ms.
Antes de moverlo con motores, probar giro manual del chasis con potencia de
motores desconectada y lógica/IMU alimentadas. dY e iW deben tener el mismo signo
en ambos sentidos de giro (usar ventanas con movimiento suficiente). Después
comprobar en el piso, a baja potencia, que el error angular disminuya. El signo
de yaw respecto al giro de motores aún requiere esa verificación física.
No desactivar VIGILAR_SIGNO_GYRO ni cambiar ganancias para evitar el bloqueo.
Si persiste temblor con signos coherentes, continuar con ajuste de fricción/PID.

Se verificó sintaxis del sketch completo con stubs y pasaron los tests de lógica
existentes. No se ha compilado contra el core ESP32 real ni probado físicamente.

---

# Control V3: CH6 puntual y estabilización continua de rumbo

Abrir `arduino/esp32_omni4_bno055/esp32_omni4_bno055.ino` con todos los
headers de su carpeta. Firmware serie: `PWM6-BNO055-CH6-PUNTUAL-V4-GYRO`.
Se conserva el pinout, el receptor PWM individual, la transformación campo→robot,
la polaridad de motores, el PID y el cero de encendido. Arduino-ESP32 3.x.

## Funcionamiento

CH6 ya no es un bloqueo permanente en 0°/180°. Es una orden puntual:

| Entrada/estado | Comportamiento |
|---|---|
| Cambiar CH6 de centro a arriba físico | Alinear hacia 0° respecto al arranque |
| Cambiar CH6 de centro a abajo físico | Alinear hacia 180° respecto al arranque |
| Cambiar directamente entre extremos | Iniciar una nueva alineación al objetivo elegido |
| Mantener CH6 en un extremo tras llegar | No reactivar alineación; giro libre con CH1 |
| CH3/CH4 durante la alineación | Trasladarse y continuar alineando simultáneamente |
| CH1 durante la alineación | Cancelar alineación y aceptar giro manual inmediatamente |
| Volver CH6 al centro | Cancelar alineación y adoptar el rumbo actual |
| Soltar CH1 mientras se traslada | Guardar el rumbo medido y estabilizarlo con la IMU |
| Trasladarse solo con CH3 o CH4 | Corregir desviación de yaw respecto al rumbo guardado |
| Todos los sticks neutros, sin alineación pendiente | PWM cero; sin perseguir una referencia vieja |
| Todos los sticks neutros, con alineación pendiente | Seguir alineando hasta terminar o agotar tiempo |
| Encender con CH6 en un extremo | No ordenar giro espontáneo: requiere cambiar el switch |

Para repetir el objetivo, regresar CH6 al centro y elegir nuevamente un extremo.
La selección inicial del switch no inicia un giro, igual que en la versión previa.

El error se envuelve a [-180°,180°): por ejemplo, 170°→180° exige +10°,
no -350°. No se acumulan vueltas completas. A exactamente 180° ambas rutas
son equivalentes; el código escoge la negativa. Esto evita vueltas adicionales
por cálculo angular; no garantiza cero sobrepaso físico sin ajustar el PID.

Para dar por terminada la alineación se exigen 150 ms continuos dentro de
±1.5° con velocidad angular de hasta 15°/s. Al llegar se limpia la integral y
se guarda el rumbo medido, sin objetivo absoluto pendiente. Si no alcanza el
objetivo en 4 s, termina la maniobra y aparece `AUTO_TO=SI`. Ese indicador
permanece hasta la siguiente selección de CH6 o el bloqueo/rearme del control.
Ajustar TIEMPO_ALINEACION_S solo después de comprobar signos y velocidad real.

## Lectura y corrección continuas

No hay while ni espera de llegada a 0/180. En cada ciclo de control (10 ms)
se lee la IMU y se calcula el rumbo. Las entradas RC se capturan por interrupción
y los ejes se filtran al disponer de un nuevo conjunto de pulsos asignados,
independientemente de CH6 y del estado de alineación. CH1/CH3/CH4 se procesan
en todos los modos. El lector no vuelve a usar datos de un canal como si
hubieran sido actualizados por los pulsos de otro.

La transformación campo→robot permanece activa en todos los estados: CH3 y
CH4 siguen referidos al campo definido al encender, no al frente instantáneo
del chasis. Por ejemplo, girar el chasis 180° no invierte el sentido de CH3 en
el campo. La estabilización mantiene el yaw durante la traslación; la IMU por
sí sola no mide ni corrige deriva lateral, deslizamiento o posición XY. Para
cerrar el lazo de trayectoria se requiere odometría/localización adicional.

Se mantiene el paro en neutro fuera de una alineación. En ese reposo no se
aplica torque para resistir un empujón; se toma el rumbo actual como referencia
para la próxima traslación. Durante la traslación sí se conserva la referencia
hasta que el usuario gire con CH1 o seleccione otro rumbo con CH6.

## Polaridad, recepción y fallas

Se conserva INVERTIR_CH6=true: según el comentario original, arriba físico
transmite pulso bajo y abajo físico transmite pulso alto. Verificar por Serial:
arriba físico debe mostrar MODO=SEL_0 y abajo MODO=SEL_180. Si se invierten,
cambiar INVERTIR_CH6. SEL indica la posición del switch; AUTO indica si sigue
pendiente la maniobra. Con AUTO=NO, SEL_0/SEL_180 no bloquean el rumbo.

Se mantienen las correcciones anteriores de recepción: cada canal asignado
(CH1, CH3, CH4, CH5, CH6) debe tener pulsos válidos recientes. Pérdida individual:
paro al superar 100 ms más hasta un ciclo de control. Pulso inválido detectado:
invalidación de ese canal. Recuperación: 500 ms con sticks centrados.
El neutro crudo prevalece sobre la memoria de los filtros y se borran las
rampas al detener. No se modificó el control del solenoide.

Si el receptor sigue transmitiendo la última orden con la emisora apagada
(failsafe HOLD), el timeout de pulsos no detectará pérdida de radio. Configurar
failsafe con CH1/CH3/CH4 neutros y verificar CH5 para evitar disparos.

## Prueba en el robot

1. Levantar las ruedas y encender el robot inmóvil. Serial 115200:
   verificar firmware V3, IMU=OK, DIAG=OK y LISTO=SI.
2. Soltar sticks: CH1/CH3/CH4 deben quedar en su centro ±30 us; si no,
   ajustar los centros configurados o los trims. Confirmar PWM=0,0,0,0.
3. Con potencia de motores desconectada, girar el chasis manualmente: yaw
   y w deben cambiar con el mismo signo. Luego comprobar a baja potencia
   que el giro corrector disminuya |err|. Si lo aumenta, revisar signos.
4. Con espacio libre en el piso, cambiar centro→arriba sin tocar sticks:
   AUTO=SI, ref=0; al llegar AUTO=NO y paro. Repetir centro→abajo:
   ref=-180 (equivale a +180). No hacer la prueba de llegada con ruedas al
   aire: el chasis no gira y no se puede verificar el seguimiento físico.
5. Sin mover CH6 del extremo, girar con CH1; soltarlo y avanzar con CH3.
   La ref debe ser el rumbo nuevo, no el antiguo 0/180. Repetir con CH4.
6. Seleccionar CH6 y accionar CH3/CH4: AUTO debe seguir SI mientras alinea.
   Accionar CH1: debe cancelar AUTO y aceptar el giro manual.
7. Soltar todo fuera de una alineación: HOLD=REPOSO, GP=0 y PWM cero.
8. Desconectar un canal dejando los otros activos: PPM=FAIL y PWM cero.
   Verificar también failsafe con emisora apagada y ruedas levantadas.

El BNO055 sigue en IMUPLUS: yaw puede derivar y el cero es relativo al arranque,
no una referencia física absoluta permanente. Fuente oficial Bosch:
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf

## Validación realizada

Tests de computadora control_omni, control_campo, ppm y bno_regression:
aprobados. La regresión cubre CH6 puntual, asentamiento, camino angular corto,
traslación durante alineación, prioridad de CH1, liberación tras 0/180, rumbo
nuevo con CH6 sostenido, timeout, paro en neutro y pérdidas de pulsos por canal.
Sintaxis del sketch completo con stubs Arduino/I2C: aprobada.
No hay arduino-cli/core ESP32 instalado: no se produjo binario ni se compiló
contra el core real; falta validar en la placa y ajustar el PID en el piso.

Ejecutar regresión desde la raíz:
```
g++ -std=c++17 -Wall -Wextra -Werror -Itests/mocks tests/bno_regression_test.cpp -o /tmp/bno_regression_test
/tmp/bno_regression_test
```
