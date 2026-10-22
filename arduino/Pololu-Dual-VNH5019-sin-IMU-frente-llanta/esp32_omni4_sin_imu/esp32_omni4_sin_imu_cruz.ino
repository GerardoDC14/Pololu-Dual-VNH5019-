#include <Arduino.h>

#include "PPM.h"
#include "ControlOmni.h"
#include "ControlEntrada.h"
#include "Diagnostico.h"

constexpr uint8_t RC_PINS[6] = {
  27,  // CH1: Giro (Rotation)
  255, // CH2: Unused (GPIO 255 = disabled)
  15,  // CH3: Avance (Throttle)
  34,  // CH4: Lateral (Strafe)
  35,  // CH5: Solenoide
  12   // CH6: se lee para diagnostico; sin accion sobre motores
};
constexpr uint8_t PIN_SOLENOIDE = 13;
constexpr uint8_t PIN_EN_DIAG = 16;

// Motor Pin Definitions
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

// Módulo activo en HIGH: 3.3 V activa; LOW apaga.
constexpr bool SOLENOIDE_ACTIVO_EN_HIGH = true;
constexpr uint32_t PULSO_SOLENOIDE_MS = 150;

constexpr uint32_t TIMEOUT_PPM_MS = 100;
constexpr uint32_t CONTROL_MS = 5;
constexpr uint32_t PWM_FRECUENCIA = 20000;
constexpr uint8_t PWM_RESOLUCION = 8;
constexpr float DUTY_MAX = 0.60f; // límite de prueba; no es un límite de corriente
constexpr int CENTRO = 1500;
constexpr int CENTRO_GIRO = 1500;
constexpr int CENTRO_AVANCE = 1500;
constexpr int CENTRO_LATERAL = 1500;
constexpr int ZONA_MUERTA = 60; // microsegundos alrededor del centro

// Zona muerta de salida de motores.
constexpr int PWM_DEADBAND_CORTE = 15;

struct Motor {
  uint8_t ina, inb, pwm;
  bool invertido;
};

// Motor configuration: Rear Left (M2) set to false
Motor motores[4] = {
  /* F */ {M1_INA, M1_INB, M1_PWM, false},
  /* I */ {M2_INA, M2_INB, M2_PWM, false},
  /* T */ {M3_INA, M3_INB, M3_PWM, false},
  /* D */ {M4_INA, M4_INB, M4_PWM, false}
};

omni::DisparoPorCambio disparoSolenoide;
omni::ArranqueSeguro arranque;
entrada::SwitchPorTramas switchSolenoide;
entrada::FiltroEje filtros[3];
void escribirSolenoide(bool activo) {
  digitalWrite(PIN_SOLENOIDE,
               activo == SOLENOIDE_ACTIVO_EN_HIGH ? HIGH : LOW);
}

bool pwmListo = false;
bool fallaPermanente = false;
const char *causaFalla = "NINGUNA";
void registrarFalla(const char *causa) {
  if (!fallaPermanente) causaFalla = causa;
  fallaPermanente = true;
}
omni::Rampa rampas[4];
constexpr uint32_t PASO_RAMPA_MS = 2;
constexpr uint32_t PAUSA_INVERSION_MS = 100;

void bloquearPWM() {
  diagnostico::salida(PIN_EN_DIAG, LOW);
  if (pwmListo) {
    for (auto &motor : motores) {
      ledcDetach(motor.pwm);
      diagnostico::salida(motor.pwm, LOW);
    }
    pwmListo = false;
  }
}

void detenerMotores() {
  if (fallaPermanente) bloquearPWM();
  for (int i = 0; i < 4; ++i) {
    auto &motor = motores[i];
    if (pwmListo && !ledcWrite(motor.pwm, 0)) registrarFalla("PWM_ESCRITURA");
    digitalWrite(motor.ina, LOW);
    digitalWrite(motor.inb, LOW);
    rampas[i].cortar(millis());
  }
  if (fallaPermanente) bloquearPWM();
}

void aplicarMotor(Motor &motor, int velocidad) {
  velocidad = constrain(velocidad, -255, 255);
  if (motor.invertido) velocidad = -velocidad;
  const int signo = velocidad > 0 ? 1 : (velocidad < 0 ? -1 : 0);
  static int signosAplicados[4] = {};
  const int indice = int(&motor - motores);
  if (!pwmListo || (signo != signosAplicados[indice] && !ledcWrite(motor.pwm, 0))) {
    registrarFalla("PWM_ESCRITURA");
    return;
  }
  signosAplicados[indice] = signo;
  digitalWrite(motor.ina, velocidad > 0 ? HIGH : LOW);
  digitalWrite(motor.inb, velocidad < 0 ? HIGH : LOW);
  if (!ledcWrite(motor.pwm, abs(velocidad))) registrarFalla("PWM_ESCRITURA");
}

bool configurarMotores() {
  bool correcto = true;
  for (auto &motor : motores) {
    diagnostico::salida(motor.ina, LOW);
    diagnostico::salida(motor.inb, LOW);
    diagnostico::salida(motor.pwm, LOW);
    if (!ledcAttach(motor.pwm, PWM_FRECUENCIA, PWM_RESOLUCION)) correcto = false;
    else if (!ledcWrite(motor.pwm, 0)) correcto = false;
  }
  pwmListo = correcto;
  fallaPermanente = !correcto;
  if (!correcto) causaFalla = "CONFIG_PWM";
  if (!correcto) {
    diagnostico::salida(PIN_EN_DIAG, LOW);
    // Desconecta también los PWM que sí se configuraron.
    for (auto &motor : motores) {
      ledcDetach(motor.pwm);
      diagnostico::salida(motor.pwm, LOW);
    }
  }
  detenerMotores();
  return correcto;
}

void aplicarObjetivos(int objetivo[4], uint32_t ahora) {
  static uint32_t ultimoPaso = 0;
  if (uint32_t(ahora - ultimoPaso) < PASO_RAMPA_MS) return;
  ultimoPaso = ahora;
  for (int i = 0; i < 4; ++i) {
    aplicarMotor(motores[i], rampas[i].paso(objetivo[i], ahora, PAUSA_INVERSION_MS));
    if (fallaPermanente) { detenerMotores(); return; }
  }
}

void setup() {
  diagnostico::salida(PIN_SOLENOIDE, SOLENOIDE_ACTIVO_EN_HIGH ? LOW : HIGH);
  escribirSolenoide(false);

  pinMode(PIN_EN_DIAG, INPUT);
  
  bool pwmOK = configurarMotores();
  Serial.begin(115200);
  Serial.println("FIRMWARE=PWM6-RC-DIRECTO-SIN-SENSOR");
  diagnostico::inicio();

  // Initialize reading individual PWM pins
  ppm::comenzar(RC_PINS, 6);
  
  if (!pwmOK) causaFalla = "CONFIG_PWM";
  Serial.printf("PWM=%s. Centrar CH1/CH3/CH4 durante 500 ms para habilitar.\n",
                pwmOK ? "OK" : "FAIL");
}

void loop() {
  static uint16_t canales[ppm::MAX_CHANNELS] = {
    1500, 1500, 1500, 1500, 1000, 1500
  };
  static uint8_t cantidad = 0;
  static uint16_t ejesFiltrados[3] = {1500,1500,1500};
  static uint32_t ultimaTramaUs = 0;
  static bool recepcionIntegra = false;
  static uint8_t canalesTrama = 0;
  static uint32_t ultimoControl = 0;
  static uint32_t ultimoPrint = 0;

  uint32_t ahora = millis();
  uint16_t nuevos[ppm::MAX_CHANNELS] = {};
  uint8_t nuevaCantidad = 0;
  uint32_t marcaUs = 0;
  uint8_t canalesRecibidos = 0;
  if (ppm::leer(nuevos, nuevaCantidad, marcaUs, &canalesRecibidos)) {
    canalesTrama = canalesRecibidos;
    recepcionIntegra = nuevaCantidad >= 6;
    switchSolenoide.actualizar(recepcionIntegra, nuevos[4]);
    // Sólo una trama completa renueva las órdenes y su plazo de vigencia.
    // Las corruptas se descartan sin desplazar canales ni renovar el timeout.
    if (recepcionIntegra) {
      cantidad = nuevaCantidad;
      for (uint8_t i = 0; i < cantidad; ++i) canales[i] = nuevos[i];
      ultimaTramaUs = marcaUs;
      ejesFiltrados[0] = filtros[0].actualizar(canales[0], CENTRO_GIRO, ZONA_MUERTA);
      ejesFiltrados[1] = filtros[1].actualizar(canales[2], CENTRO_AVANCE, ZONA_MUERTA);
      ejesFiltrados[2] = filtros[2].actualizar(canales[3], CENTRO_LATERAL, ZONA_MUERTA);
    }
  }

  bool ppmVigente = cantidad >= 6 &&
                    uint32_t(micros() - ultimaTramaUs) <= TIMEOUT_PPM_MS * 1000UL;

  if (!ppmVigente) for (auto &filtro : filtros) filtro.reiniciar();
  bool solenoideActivo = disparoSolenoide.actualizar(
    ppmVigente && recepcionIntegra && switchSolenoide.listo && arranque.listo && !fallaPermanente && digitalRead(PIN_EN_DIAG) == HIGH,
    switchSolenoide.pulso, ahora, PULSO_SOLENOIDE_MS);
  escribirSolenoide(solenoideActivo);

  if (uint32_t(ahora - ultimoControl) < CONTROL_MS) { delay(1); return; }
  ultimoControl = ahora;

  bool driversOK = digitalRead(PIN_EN_DIAG) == HIGH;
  bool centrado = abs(int(canales[0]) - CENTRO_GIRO) <= ZONA_MUERTA &&
                  abs(int(canales[2]) - CENTRO_AVANCE) <= ZONA_MUERTA &&
                  abs(int(canales[3]) - CENTRO_LATERAL) <= ZONA_MUERTA;
  bool listo = arranque.actualizar(ppmVigente && driversOK && !fallaPermanente,
                                   centrado, ahora);
  int objetivo[4] = {};

  float stickGiro = omni::eje(ejesFiltrados[0], 1000, CENTRO_GIRO, 2000, ZONA_MUERTA);
  float avanceRobot = omni::eje(ejesFiltrados[1], 1000, CENTRO_AVANCE, 2000, ZONA_MUERTA);
  float lateralRobot = omni::eje(ejesFiltrados[2], 1000, CENTRO_LATERAL, 2000, ZONA_MUERTA);
  // RC directo: no hay referencia angular ni correccion automatica.
  if (!listo || !pwmListo || fallaPermanente || centrado) {
    detenerMotores();
  } else {
    float salida[4];
    omni::mezclar(avanceRobot, lateralRobot, stickGiro, salida);
    for (int i = 0; i < 4; ++i) {
      objetivo[i] = int(salida[i] * 255.0f * DUTY_MAX);
      if (abs(objetivo[i]) < PWM_DEADBAND_CORTE) objetivo[i] = 0;
    }
    aplicarObjetivos(objetivo, ahora);
  }
  if (!listo || fallaPermanente) { solenoideActivo = false; escribirSolenoide(false); }
  static bool solAnterior = false;
  if (solenoideActivo != solAnterior) {
    solAnterior = solenoideActivo;
    Serial.printf("EVENTO_SOL t=%lu SOL=%s CH1=%u CH3=%u CH4=%u CH5=%u CH6=%u CH5_EST=%u\n",
      (unsigned long)ahora, solenoideActivo ? "ON" : "OFF",
      unsigned(canales[0]), unsigned(canales[2]), unsigned(canales[3]),
      unsigned(canales[4]), unsigned(canales[5]), unsigned(switchSolenoide.pulso));
  }

  const uint8_t estadoFalla = (!ppmVigente ? 1 : 0) | (!driversOK ? 2 : 0) |
                              (fallaPermanente ? 8 : 0);
  uint32_t tramasOK = 0, tramasBAD = 0;
  uint16_t intervaloBAD = 0;
  ppm::diagnostico(tramasOK, tramasBAD, intervaloBAD);
  static uint8_t anteriorFalla = 255;
  if (estadoFalla != anteriorFalla) {
    anteriorFalla = estadoFalla;
    Serial.printf("EVENTO t=%lu PPM=%s DIAG=%s BLOQUEO=%s CAUSA=%s NCH=%u BAD=%lu ULT_BAD=%u\n",
      (unsigned long)ahora, ppmVigente ? "OK" : "FAIL", driversOK ? "OK" : "FAIL",
      fallaPermanente ? "SI" : "NO", causaFalla,
      unsigned(canalesTrama), (unsigned long)tramasBAD, unsigned(intervaloBAD));
  }
  if (uint32_t(ahora - ultimoPrint) >= 250) {
    ultimoPrint = ahora;
    Serial.printf(
      "t=%lu LISTO=%s CH1=%u CH3=%u CH4=%u CH5=%u CH6=%u "
      "G=%.2f A=%.2f L=%.2f "
      "PPM=%s DIAG=%s CAUSA=%s SOL=%s PWM=%d,%d,%d,%d RX=%lu BAD=%lu ULT_BAD=%u EDAD=%lu NCH=%u TRAMA=%s\n",
      (unsigned long)ahora, listo ? "SI" : "NO",
      unsigned(canales[0]), unsigned(canales[2]), unsigned(canales[3]), unsigned(canales[4]), unsigned(canales[5]),
      stickGiro, avanceRobot, lateralRobot,
      ppmVigente ? "OK" : "FAIL", driversOK ? "OK" : "FAIL",
      causaFalla, solenoideActivo ? "ON" : "OFF",
      rampas[0].actual, rampas[1].actual, rampas[2].actual, rampas[3].actual,
      (unsigned long)tramasOK, (unsigned long)tramasBAD, unsigned(intervaloBAD),
      (unsigned long)(uint32_t(micros() - ultimaTramaUs) / 1000), unsigned(canalesTrama),
      recepcionIntegra ? "OK" : "DESCARTADA");
  }
  delay(1);
}
