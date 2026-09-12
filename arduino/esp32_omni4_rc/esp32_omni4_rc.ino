// Borrador: cuatro omni, dos VNH5019 y FS-iA6 por PWM.
#include <Arduino.h>
#include "ControlOmni.h"
#include "ReceptorPWM.h"

const bool SOLO_RECEPTOR = true; // false despues de revisar canales y failsafe
const float DUTY_MAX = 0.35f;
const float GANANCIA_GIRO = 0.5f;
const uint32_t PASO_MS = 10;
const uint32_t PAUSA_INVERSION_MS = 1000;
const int MIN_US[3] = {1000, 1000, 1000};
const int CENTRO_US[3] = {1500, 1500, 1500};
const int MAX_US[3] = {2000, 2000, 2000};
const int SIGNO_RC[3] = {1, 1, 1};
const int ZONA_MUERTA_US = 40;

struct Motor {
  uint8_t ina, inb, pwm, diag;
  int polaridad;
  bool listo = false;
  bool habilitado = false;
  omni::Rampa rampa;
};

// Frente, izquierda, atras, derecha. Los signos se ajustan con las ruedas al aire.
Motor motores[4] = {
  {25, 26, 27, 32, 1}, // Driver A, M1
  {18, 19, 23, 33, 1}, // Driver A, M2
  {4,   5, 14, 13, 1}, // Driver B, M1
  {16, 17, 21, 22, 1}  // Driver B, M2
};
omni::Habilitacion permiso;
bool falla = false;
uint32_t ultimoPaso = 0, ultimoReporte = 0;

void salidaBaja(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void cortarTodos() {
  for (auto &m : motores) {
    salidaBaja(m.diag);
    m.habilitado = false;
    m.rampa.cortar(millis());
    if (m.listo && !ledcWrite(m.pwm, 0)) falla = true;
  }
}

bool aplicar(Motor &m, int pwm) {
  if (pwm == 0) {
    salidaBaja(m.diag);
    m.habilitado = false;
    return ledcWrite(m.pwm, 0);
  }
  digitalWrite(m.ina, pwm > 0 ? HIGH : LOW);
  digitalWrite(m.inb, pwm < 0 ? HIGH : LOW);
  if (!m.habilitado) {
    pinMode(m.diag, INPUT);
    delayMicroseconds(10); // Asentamiento del pull-up de EN/DIAG
    m.habilitado = true;
  }
  if (digitalRead(m.diag) == LOW) return false;
  return ledcWrite(m.pwm, abs(pwm));
}

void setup() {
  Serial.begin(115200);
  for (auto &m : motores) {
    salidaBaja(m.diag);
    salidaBaja(m.pwm);
    salidaBaja(m.ina);
    salidaBaja(m.inb);
  }
  for (auto &m : motores) {
    m.listo = ledcAttach(m.pwm, 20000, 8);
    if (!m.listo || !ledcWrite(m.pwm, 0)) falla = true;
  }
  cortarTodos();
  receptor::iniciar();
  Serial.println(SOLO_RECEPTOR ? "Modo receptor. Motores deshabilitados." : "Control RC. CH5 en OFF para habilitar.");
}

void loop() {
  uint16_t pulsos[4];
  bool valido = receptor::leer(pulsos);
  float ejes[3];
  for (int i = 0; i < 3; ++i)
    ejes[i] = SIGNO_RC[i] * omni::eje(pulsos[i], MIN_US[i], CENTRO_US[i], MAX_US[i], ZONA_MUERTA_US);
  bool centrado = ejes[0] == 0 && ejes[1] == 0 && ejes[2] == 0;
  permiso.actualizar(valido && !falla, pulsos[3], centrado);

  for (auto &m : motores)
    if (m.habilitado && digitalRead(m.diag) == LOW) falla = true;

  if (SOLO_RECEPTOR || !permiso.activo || falla) {
    cortarTodos();
  } else if (uint32_t(millis() - ultimoPaso) >= PASO_MS) {
    ultimoPaso = millis();
    float ruedas[4];
    // CH1: lateral, CH2: avance, CH4: giro.
    omni::mezclar(ejes[1], ejes[0], GANANCIA_GIRO * ejes[2], ruedas);
    for (int i = 0; i < 4; ++i) {
      Motor &m = motores[i];
      int objetivo = lroundf(255 * DUTY_MAX * ruedas[i]) * m.polaridad;
      int pwm = m.rampa.paso(objetivo, ultimoPaso, PAUSA_INVERSION_MS);
      if (!aplicar(m, pwm)) {
        falla = true;
        cortarTodos();
        break;
      }
    }
  }
  if (uint32_t(millis() - ultimoReporte) >= 250) {
    ultimoReporte = millis();
    Serial.printf("CH1=%u CH2=%u CH4=%u CH5=%u valido=%d habilitado=%d falla=%d PWM=%d,%d,%d,%d\n",
                  pulsos[0], pulsos[1], pulsos[2], pulsos[3], valido,
                  !SOLO_RECEPTOR && permiso.activo && !falla, falla,
                  motores[0].rampa.actual, motores[1].rampa.actual,
                  motores[2].rampa.actual, motores[3].rampa.actual);
  }
  delay(1);
}
