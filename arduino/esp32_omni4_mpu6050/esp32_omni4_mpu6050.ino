#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#include "PPM.h"
#include "ControlOmni.h"
#include "ControlCampo.h"

// I2C usa los pines normales 21/22. Por ello, en esta variante M1_PWM y
// M3_INB se reubican a GPIO 4 y GPIO 2, respectivamente.
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
constexpr uint8_t DIRECCION_MPU6050 = MPU6050_I2CADDR_DEFAULT;
constexpr uint8_t PIN_PPM = 27;
constexpr uint8_t PIN_SOLENOIDE = 13;
constexpr uint8_t PIN_EN_DIAG = 16;

constexpr uint8_t M1_INA = 5;
constexpr uint8_t M1_INB = 18;
constexpr uint8_t M1_PWM = 4;
constexpr uint8_t M2_INA = 17;
constexpr uint8_t M2_INB = 19;
constexpr uint8_t M2_PWM = 25;
constexpr uint8_t M3_INA = 26;
constexpr uint8_t M3_INB = 2;
constexpr uint8_t M3_PWM = 14;
constexpr uint8_t M4_INA = 32;
constexpr uint8_t M4_INB = 33;
constexpr uint8_t M4_PWM = 23;

constexpr bool SOLENOIDE_ACTIVO_EN_HIGH = true;
constexpr uint32_t PULSO_SOLENOIDE_MS = 150;
constexpr uint32_t TIMEOUT_PPM_MS = 100;
constexpr uint32_t CONTROL_MS = 5;
constexpr uint32_t PWM_FRECUENCIA = 20000;
constexpr uint8_t PWM_RESOLUCION = 8;
constexpr float DUTY_MAX = 1.00f;
constexpr int CENTRO = 1500;
constexpr int ZONA_MUERTA = 20;

// Ajustar estos valores con el robot elevado. SIGNO_YAW debe hacer que un giro
// antihorario produzca yaw positivo.
constexpr float SIGNO_YAW = 1.0f;
constexpr float VELOCIDAD_REFERENCIA_DEG_S = 120.0f;
constexpr float KP_RUMBO = 0.018f;
constexpr float KI_RUMBO = 0.0005f;
constexpr float KD_RUMBO = 0.0025f;

struct Motor {
  uint8_t ina, inb, pwm;
  bool invertido;
};

Motor motores[4] = {
  /* F */ {M1_INA, M1_INB, M1_PWM, false},
  /* I */ {M2_INA, M2_INB, M2_PWM, true},
  /* T */ {M3_INA, M3_INB, M3_PWM, false},
  /* D */ {M4_INA, M4_INB, M4_PWM, false}
};

Adafruit_MPU6050 mpu;
omni::DisparoPorCambio disparoSolenoide;
campo::SelectorRumbo selectorRumbo;
campo::ControlRumbo controlRumbo;

bool imuDisponible = false;
float sesgoGyroZ = 0.0f;
float yawIntegrado = 0.0f;
uint32_t ultimaLecturaImuUs = 0;

void escribirSolenoide(bool activo) {
  digitalWrite(PIN_SOLENOIDE,
               activo == SOLENOIDE_ACTIVO_EN_HIGH ? HIGH : LOW);
}

void detenerMotores() {
  for (auto &motor : motores) {
    digitalWrite(motor.ina, LOW);
    digitalWrite(motor.inb, LOW);
    ledcWrite(motor.pwm, 0);
  }
}

void aplicarMotor(Motor &motor, int velocidad) {
  velocidad = constrain(velocidad, -255, 255);
  if (motor.invertido) velocidad = -velocidad;
  digitalWrite(motor.ina, velocidad > 0 ? HIGH : LOW);
  digitalWrite(motor.inb, velocidad < 0 ? HIGH : LOW);
  ledcWrite(motor.pwm, abs(velocidad));
}

bool configurarMotores() {
  bool correcto = true;
  for (auto &motor : motores) {
    digitalWrite(motor.ina, LOW);
    digitalWrite(motor.inb, LOW);
    pinMode(motor.ina, OUTPUT);
    pinMode(motor.inb, OUTPUT);
    if (!ledcAttach(motor.pwm, PWM_FRECUENCIA, PWM_RESOLUCION)) correcto = false;
  }
  detenerMotores();
  return correcto;
}

bool inicializarMPU() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!mpu.begin(DIRECCION_MPU6050, &Wire)) return false;
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  // Calibracion del sesgo: el robot debe permanecer completamente inmovil.
  constexpr int MUESTRAS_SESGO = 600;
  double suma = 0.0;
  for (int i = 0; i < MUESTRAS_SESGO; ++i) {
    sensors_event_t aceleracion, giro, temperatura;
    if (!mpu.getEvent(&aceleracion, &giro, &temperatura)) return false;
    suma += giro.gyro.z;
    delay(3);
  }
  sesgoGyroZ = float(suma / MUESTRAS_SESGO);
  yawIntegrado = 0.0f;
  ultimaLecturaImuUs = micros();
  return true;
}

bool leerMPU(float &yaw, float &velocidadYaw) {
  sensors_event_t aceleracion, giro, temperatura;
  if (!mpu.getEvent(&aceleracion, &giro, &temperatura) ||
      isnan(giro.gyro.z)) return false;

  uint32_t ahoraUs = micros();
  float dt = uint32_t(ahoraUs - ultimaLecturaImuUs) * 1.0e-6f;
  ultimaLecturaImuUs = ahoraUs;
  velocidadYaw = SIGNO_YAW * (giro.gyro.z - sesgoGyroZ) * 57.2957795131f;
  if (dt > 0.0001f && dt < 0.1f) {
    yawIntegrado = campo::envolverGrados(yawIntegrado + velocidadYaw * dt);
  }
  yaw = yawIntegrado;
  return true;
}

void setup() {
  Serial.begin(115200);
  escribirSolenoide(false);
  pinMode(PIN_SOLENOIDE, OUTPUT);
  pinMode(PIN_EN_DIAG, INPUT);
  bool pwmOK = configurarMotores();
  ppm::comenzar(PIN_PPM);
  imuDisponible = inicializarMPU();
  controlRumbo.fijar(0.0f);

  Serial.printf("MPU6050=%s PWM=%s. Mantener inmovil durante la calibracion.\n",
                imuDisponible ? "OK" : "FAIL", pwmOK ? "OK" : "FAIL");
  if (!pwmOK) imuDisponible = false;
}

void loop() {
  static uint16_t canales[ppm::MAX_CHANNELS] = {
    1500, 1500, 1500, 1500, 1000, 1500
  };
  static uint8_t cantidad = 0;
  static uint32_t ultimaTramaValida = 0;
  static uint32_t ultimoControl = 0;
  static uint32_t ultimoPrint = 0;
  static float yaw = 0.0f;
  static float velocidadYaw = 0.0f;

  uint32_t ahora = millis();
  uint16_t nuevos[ppm::MAX_CHANNELS] = {};
  uint8_t nuevaCantidad = 0;
  if (ppm::leer(nuevos, nuevaCantidad) && nuevaCantidad >= 6) {
    cantidad = nuevaCantidad;
    for (uint8_t i = 0; i < cantidad; ++i) canales[i] = nuevos[i];
    ultimaTramaValida = ahora;
  }

  bool ppmVigente = cantidad >= 6 &&
                    uint32_t(ahora - ultimaTramaValida) <= TIMEOUT_PPM_MS;
  bool solenoideActivo = disparoSolenoide.actualizar(
    ppmVigente, canales[4], ahora, PULSO_SOLENOIDE_MS);
  escribirSolenoide(solenoideActivo);

  if (uint32_t(ahora - ultimoControl) < CONTROL_MS) return;
  float dt = ultimoControl == 0 ? CONTROL_MS / 1000.0f
                                : uint32_t(ahora - ultimoControl) / 1000.0f;
  ultimoControl = ahora;
  dt = campo::limitar(dt, 0.001f, 0.05f);

  bool imuValida = imuDisponible && leerMPU(yaw, velocidadYaw);
  bool driversOK = digitalRead(PIN_EN_DIAG) == HIGH;
  int objetivo[4] = {};

  int8_t ordenRumbo = selectorRumbo.actualizar(ppmVigente, canales[5]);
  if (ordenRumbo > 0) controlRumbo.fijar(0.0f);
  if (ordenRumbo < 0) controlRumbo.fijar(180.0f);

  if (!ppmVigente || !imuValida || !driversOK) {
    if (imuValida) controlRumbo.fijar(yaw);
    detenerMotores();
  } else {
    float stickGiro = omni::eje(canales[0], 1000, CENTRO, 2000, ZONA_MUERTA);
    float avanceCampo = omni::eje(canales[2], 1000, CENTRO, 2000, ZONA_MUERTA);
    float lateralCampo = omni::eje(canales[3], 1000, CENTRO, 2000, ZONA_MUERTA);

    controlRumbo.mover(stickGiro, dt, VELOCIDAD_REFERENCIA_DEG_S);
    float giro = controlRumbo.calcular(
      yaw, velocidadYaw, dt, KP_RUMBO, KI_RUMBO, KD_RUMBO);

    float avanceRobot, lateralRobot;
    campo::aCoordenadasRobot(avanceCampo, lateralCampo, yaw,
                             avanceRobot, lateralRobot);
    float salida[4];
    omni::mezclar(avanceRobot, lateralRobot, giro, salida);
    for (int i = 0; i < 4; ++i) {
      objetivo[i] = int(salida[i] * 255.0f * DUTY_MAX);
      aplicarMotor(motores[i], objetivo[i]);
    }
  }

  if (uint32_t(ahora - ultimoPrint) >= 250) {
    ultimoPrint = ahora;
    Serial.printf(
      "PPM=%s IMU=%s DIAG=%s yaw=%.1f ref=%.1f SOL=%s M=%d,%d,%d,%d\n",
      ppmVigente ? "OK" : "FAIL", imuValida ? "OK" : "FAIL",
      driversOK ? "OK" : "FAIL", yaw, controlRumbo.referencia,
      solenoideActivo ? "ON" : "OFF",
      objetivo[0], objetivo[1], objetivo[2], objetivo[3]);
  }
}
