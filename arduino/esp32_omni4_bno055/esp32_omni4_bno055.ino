#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "PPM.h"
#include "ControlOmni.h"
#include "ControlCampo.h"

// I2C usa los pines normales 21/22. Por ello, en esta variante M1_PWM y
// M3_INB se reubican a GPIO 4 y GPIO 2, respectivamente.
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
constexpr uint8_t DIRECCION_BNO055 = BNO055_ADDRESS_A;
constexpr bool BNO_USAR_CRISTAL_EXTERNO = true;
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
constexpr float SIGNO_YAW = -1.0f;
constexpr float VELOCIDAD_REFERENCIA_DEG_S = 120.0f;
constexpr float KP_RUMBO = 0.018f;
constexpr float KI_RUMBO = 0.0005f;
constexpr float KD_RUMBO = 0.0025f;

struct Motor {
  uint8_t ina, inb, pwm;
  bool invertido;
};

Motor motores[4] = {
  {M1_INA, M1_INB, M1_PWM, false},
  {M2_INA, M2_INB, M2_PWM, true},
  {M3_INA, M3_INB, M3_PWM, false},
  {M4_INA, M4_INB, M4_PWM, false}
};

Adafruit_BNO055 bno(55, DIRECCION_BNO055, &Wire);
omni::DisparoPorCambio disparoSolenoide;
campo::SelectorRumbo selectorRumbo;
campo::ControlRumbo controlRumbo;

bool imuDisponible = false;
float ceroBNO = 0.0f;

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

bool inicializarBNO() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) return false;
  bno.setExtCrystalUse(BNO_USAR_CRISTAL_EXTERNO);
  delay(1000);

  sensors_event_t evento;
  if (!bno.getEvent(&evento, Adafruit_BNO055::VECTOR_EULER) ||
      isnan(evento.orientation.x)) return false;

  ceroBNO = evento.orientation.x;
  return true;
}

bool leerBNO(float &yaw, float &velocidadYaw) {
  sensors_event_t orientacion, giro;
  if (!bno.getEvent(&orientacion, Adafruit_BNO055::VECTOR_EULER) ||
      !bno.getEvent(&giro, Adafruit_BNO055::VECTOR_GYROSCOPE) ||
      isnan(orientacion.orientation.x) || isnan(giro.gyro.z)) return false;

  yaw = campo::envolverGrados(
    SIGNO_YAW * (orientacion.orientation.x - ceroBNO));
  velocidadYaw = SIGNO_YAW * giro.gyro.z * 57.2957795131f;
  return true;
}

void setup() {
  Serial.begin(115200);
  escribirSolenoide(false);
  pinMode(PIN_SOLENOIDE, OUTPUT);
  pinMode(PIN_EN_DIAG, INPUT);
  bool pwmOK = configurarMotores();
  ppm::comenzar(PIN_PPM);
  imuDisponible = inicializarBNO();
  controlRumbo.fijar(0.0f);

  Serial.printf("BNO055=%s PWM=%s. Mantener inmovil al arrancar.\n",
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

  bool imuValida = imuDisponible && leerBNO(yaw, velocidadYaw);
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
