#include <Arduino.h>

#include "PPM.h"
#include "ControlOmni.h"

// ESP32 DevKit V1 (ESP32-WROOM-32).
constexpr uint8_t PIN_PPM = 27;

// Pololu Dual VNH5019: cuatro motores y un EN/DIAG compartido.
constexpr uint8_t M1_INA = 5;
constexpr uint8_t M1_INB = 18;
constexpr uint8_t M1_PWM = 22;
constexpr uint8_t M2_INA = 17;
constexpr uint8_t M2_INB = 19;
constexpr uint8_t M2_PWM = 25;
constexpr uint8_t M3_INA = 26;
constexpr uint8_t M3_INB = 21;
constexpr uint8_t M3_PWM = 14;
constexpr uint8_t M4_INA = 32;
constexpr uint8_t M4_INB = 33;
constexpr uint8_t M4_PWM = 23;
constexpr uint8_t PIN_EN_DIAG = 16;

// La salida sólo manda la etapa de potencia; nunca conectar el solenoide al GPIO.
constexpr uint8_t PIN_SOLENOIDE = 13;
constexpr bool SOLENOIDE_ACTIVO_EN_HIGH = true;
constexpr uint32_t PULSO_SOLENOIDE_MS = 150;

constexpr uint32_t PWM_FRECUENCIA = 20000;
constexpr uint8_t PWM_RESOLUCION = 8;
constexpr int CENTRO = 1500;
constexpr int ZONA_MUERTA = 20;
constexpr float DUTY_MAX = 1.00f;
constexpr uint32_t CONTROL_MS = 2;
constexpr uint32_t TIMEOUT_PPM_MS = 100;

struct Motor {
  uint8_t ina;
  uint8_t inb;
  uint8_t pwm;
  bool invertido;
};

Motor motores[4] = {
  {M1_INA, M1_INB, M1_PWM, false},
  {M2_INA, M2_INB, M2_PWM, true},
  {M3_INA, M3_INB, M3_PWM, false},
  {M4_INA, M4_INB, M4_PWM, false}
};

omni::DisparoPorCambio disparoSolenoide;

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

  if (velocidad > 0) {
    digitalWrite(motor.ina, HIGH);
    digitalWrite(motor.inb, LOW);
  } else if (velocidad < 0) {
    digitalWrite(motor.ina, LOW);
    digitalWrite(motor.inb, HIGH);
  } else {
    digitalWrite(motor.ina, LOW);
    digitalWrite(motor.inb, LOW);
  }
  ledcWrite(motor.pwm, abs(velocidad));
}

void configurarMotores() {
  for (auto &motor : motores) {
    pinMode(motor.ina, OUTPUT);
    pinMode(motor.inb, OUTPUT);
    ledcAttach(motor.pwm, PWM_FRECUENCIA, PWM_RESOLUCION);
  }
  detenerMotores();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Fijar primero el nivel inactivo evita un pulso accidental al arrancar.
  escribirSolenoide(false);
  pinMode(PIN_SOLENOIDE, OUTPUT);

  ppm::comenzar(PIN_PPM);
  pinMode(PIN_EN_DIAG, INPUT);
  configurarMotores();

  Serial.println("Robot omnidireccional listo");
  Serial.println("PPM: CH1 giro, CH3 avance, CH4 lateral, CH5 solenoide");
}

void loop() {
  static uint32_t ultimoControl = 0;
  static uint32_t ultimoPrint = 0;
  static uint32_t ultimaTramaValida = 0;
  static uint16_t canales[ppm::MAX_CHANNELS] = {
    1500, 1500, 1500, 1500, 1000, 1500
  };
  static uint8_t cantidad = 0;

  uint32_t ahora = millis();
  uint16_t nuevosCanales[ppm::MAX_CHANNELS] = {0};
  uint8_t nuevaCantidad = 0;

  if (ppm::leer(nuevosCanales, nuevaCantidad) && nuevaCantidad >= 5) {
    cantidad = nuevaCantidad;
    for (uint8_t i = 0; i < cantidad; ++i) canales[i] = nuevosCanales[i];
    ultimaTramaValida = ahora;
  }

  bool ppmVigente = cantidad >= 5 &&
                    uint32_t(ahora - ultimaTramaValida) <= TIMEOUT_PPM_MS;
  bool driversOK = digitalRead(PIN_EN_DIAG) == HIGH;

  bool solenoideActivo = disparoSolenoide.actualizar(
    ppmVigente, canales[4], ahora, PULSO_SOLENOIDE_MS);
  escribirSolenoide(solenoideActivo);

  if (uint32_t(ahora - ultimoControl) < CONTROL_MS) return;
  ultimoControl = ahora;

  int objetivo[4] = {0, 0, 0, 0};
  if (!ppmVigente || !driversOK) {
    detenerMotores();
  } else {
    // El orden proviene de la configuración actual del transmisor/receptor.
    float giro = omni::eje(canales[0], 1000, CENTRO, 2000, ZONA_MUERTA);    // CH1
    float avance = omni::eje(canales[2], 1000, CENTRO, 2000, ZONA_MUERTA); // CH3
    float lateral = omni::eje(canales[3], 1000, CENTRO, 2000, ZONA_MUERTA);// CH4
    float salida[4];
    omni::mezclar(avance, lateral, giro, salida);

    for (int i = 0; i < 4; ++i) {
      objetivo[i] = int(salida[i] * 255.0f * DUTY_MAX);
      aplicarMotor(motores[i], objetivo[i]);
    }
  }

  if (uint32_t(ahora - ultimoPrint) >= 250) {
    ultimoPrint = ahora;
    Serial.printf(
      "CH1=%u CH2=%u CH3=%u CH4=%u CH5=%u | PPM=%s DIAG=%s SOL=%s | M=%d,%d,%d,%d\n",
      canales[0], canales[1], canales[2], canales[3], canales[4],
      ppmVigente ? "OK" : "FAIL", driversOK ? "OK" : "FAIL",
      solenoideActivo ? "ON" : "OFF",
      objetivo[0], objetivo[1], objetivo[2], objetivo[3]);
  }
}
