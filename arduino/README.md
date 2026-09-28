# Sketches

| Carpeta | Uso |
| --- | --- |
| [esp32_motor1](esp32_motor1/esp32_motor1.ino) | Prueba de M1 por serial |
| [esp32_omni4_rc_original](esp32_omni4_rc_original/esp32_omni4_rc_original.ino) | Copia sin cambios funcionales del ZIP recibido |
| [esp32_omni4_rc](esp32_omni4_rc/esp32_omni4_rc.ino) | Cuatro motores por PPM y pulso de solenoide en GPIO 13, sin IMU |
| [esp32_omni4_bno055](esp32_omni4_bno055/esp32_omni4_bno055.ino) | Control orientado al campo con BNO055 |
| [esp32_omni4_mpu6050](esp32_omni4_mpu6050/esp32_omni4_mpu6050.ino) | Control orientado al campo con MPU6050 |

Arduino-ESP32 **3.x**, placa **ESP32 Dev Module** para DevKit V1/WROOM-32.
Todas las versiones RC usan una trama PPM compuesta en GPIO 27. Las variantes
IMU exigen seis canales y reservan GPIO 21/22 para I²C.

[Versiones y pinout](../README.md#versiones-disponibles) · [Control híbrido](../docs/imu-hibrido.md)
