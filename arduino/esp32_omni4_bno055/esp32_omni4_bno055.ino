#include <Arduino.h>
#include <Wire.h>

#include "PPM.h"
#include "ControlOmni.h"
#include "ControlCampo.h"
#include "ControlEntrada.h"
#include "Diagnostico.h"
#include "I2CSeguro.h"

// I2C pins
constexpr uint8_t PIN_I2C_SDA = 21; //P18
constexpr uint8_t PIN_I2C_SCL = 22; //P21
constexpr uint8_t DIRECCION_BNO055 = 0x28;

// Internal oscillator for BNO055 breakout modules
constexpr bool BNO_USAR_CRISTAL_EXTERNO = false;

constexpr uint8_t RC_PINS[6] = {
  27,  // CH1: Giro (Rotation) GND Al lado P12
  255, // CH2: Unused (GPIO 255 = disabled) 
  15,  // CH3: Avance (Throttle) SD1
  34,  // CH4: Lateral (Strafe) P33
  35,  // CH5: Solenoide P25
  12   // CH6: Rumbo (Heading mode) SD2
};
constexpr uint8_t PIN_SOLENOIDE = 13; //SD3
constexpr uint8_t PIN_EN_DIAG = 16; //P0

// Motor Pin Definitions
constexpr uint8_t M1_INA = 5; //P16
constexpr uint8_t M1_INB = 18; //P17
constexpr uint8_t M1_PWM = 4; //P2
constexpr uint8_t M2_INA = 17; //P4
constexpr uint8_t M2_INB = 19; //P5
constexpr uint8_t M2_PWM = 25; //P14
constexpr uint8_t M3_INA = 26; //P12
constexpr uint8_t M3_INB = 2; //P15
constexpr uint8_t M3_PWM = 14; //P13
constexpr uint8_t M4_INA = 32; //P26
constexpr uint8_t M4_INB = 33; //P27
constexpr uint8_t M4_PWM = 23; //RX

// Módulo activo en HIGH: 3.3 V activa; LOW apaga.
constexpr bool SOLENOIDE_ACTIVO_EN_HIGH = true;
constexpr uint32_t PULSO_SOLENOIDE_MS = 150;

constexpr uint32_t TIMEOUT_PPM_MS = 100;
// El BNO055 en IMUPLUS entrega datos a 100 Hz: leer más rápido sólo repite muestras.
constexpr uint32_t CONTROL_MS = 10;
constexpr uint32_t PWM_FRECUENCIA = 20000;
constexpr uint8_t PWM_RESOLUCION = 8;
constexpr float DUTY_MAX = 0.60f; // límite de prueba; no es un límite de corriente
constexpr int CENTRO = 1500;
constexpr int CENTRO_GIRO = 1500;
constexpr int CENTRO_AVANCE = 1500;
constexpr int CENTRO_LATERAL = 1500;
// Switch físico arriba transmite el extremo bajo del canal.
constexpr bool INVERTIR_CH6 = true;
constexpr int ZONA_MUERTA = 30; // microsegundos alrededor del centro

// Asignación de canales a ejes (índices de ejesFiltrados[]):
//   CH1 = giro, CH3 = avance, CH4 = lateral. CH5 (solenoide) y CH6 (modo de rumbo) no cambian.
constexpr uint8_t EJE_GIRO = 0;     // CH1
constexpr uint8_t EJE_AVANCE = 1;   // CH3
constexpr uint8_t EJE_LATERAL = 2;  // CH4
// Sentido de cada stick. Poner -1.0f invierte ese eje sin tocar nada más.
constexpr float SIGNO_STICK_GIRO = 1.0f;
constexpr float SIGNO_STICK_AVANCE = 1.0f;
constexpr float SIGNO_STICK_LATERAL = 1.0f;

// Zona muerta de PWM para la TRASLACIÓN (el giro tiene su propio mapeo, ver giroAPwm).
constexpr int PWM_DEADBAND_CORTE = 15;

// --- Corrección de rumbo (giro) ---
// El giro se suma DESPUÉS de la rampa: la rampa (1 cuenta/10 ms) sólo suaviza la traslación.
// MEDIR PWM_MIN_GIRO: cuentas (0-255) con las que el chasis, en el piso, realmente empieza a girar.
constexpr int   PWM_MIN_GIRO = 40;
constexpr int   PWM_MAX_GIRO = 100;           // tope de velocidad de giro
constexpr float BANDA_GIRO_DEG = 1.0f;        // error por debajo del cual no se aplica giro
constexpr uint32_t PAUSA_INVERSION_GIRO_MS = 30; // freno breve antes de invertir una rueda
static_assert(PWM_MAX_GIRO <= int(255 * DUTY_MAX), "PWM_MAX_GIRO excede DUTY_MAX");

constexpr float SIGNO_YAW = -1.0f;
// Signo fijado por evidencia del montaje: dY=-43.6 e iW=+46.2 con
// SIGNO_GYRO_Z=-1. No deducirlo solo de la orientacion fisica de la placa.
constexpr float SIGNO_GYRO_Z = 1.0f;
// Si yaw (heading) y gyro discrepan en signo durante 600 ms se bloquean las salidas:
// con signos opuestos el término D acelera el giro en vez de frenarlo.
constexpr bool VIGILAR_SIGNO_GYRO = true;
constexpr float VELOCIDAD_REFERENCIA_DEG_S = 120.0f;
constexpr float KP_RUMBO = 0.018f;
constexpr float KI_RUMBO = 0.0005f;
constexpr float KD_RUMBO = 0.0025f;

struct Motor {
  uint8_t ina, inb, pwm;
  bool invertido;
};

// Motor configuration: Rear Left (M2) set to false
Motor motores[4] = {
  /* FL */ {M1_INA, M1_INB, M1_PWM, false},
  /* RL */ {M2_INA, M2_INB, M2_PWM, false},
  /* RR */ {M3_INA, M3_INB, M3_PWM, false},
  /* FR */ {M4_INA, M4_INB, M4_PWM, false}
};

omni::DisparoPorCambio disparoSolenoide;
omni::ArranqueSeguro arranque;
entrada::SelectorEstable selectorRumbo;
entrada::SwitchPorTramas switchSolenoide;
entrada::FiltroEje filtros[3];
campo::RumboHibrido rumbo;
campo::ControlRumbo &controlRumbo = rumbo.control;

bool imuDisponible = false;
float ceroBNO = 0.0f;

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
int baseRampa[4] = {};            // salida de la rampa (sólo traslación)
int giroAplicadoPwm = 0;          // último giro sumado, para telemetría
constexpr uint32_t PASO_RAMPA_MS = 10;
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
    baseRampa[i] = 0;
  }
  giroAplicadoPwm = 0;
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

// Traslación: pasa por la rampa (1 cuenta cada PASO_RAMPA_MS, pausa al invertir).
// Giro: se suma después, sin rampa, para que el lazo de rumbo no tenga ~1.5 s de retardo.
void aplicarObjetivos(const int traslacion[4], int giroPwm, uint32_t ahora) {
  static uint32_t ultimoPaso = 0;
  static int ultimoSigno[4] = {};
  static uint32_t ultimoActivo[4] = {};
  if (uint32_t(ahora - ultimoPaso) >= PASO_RAMPA_MS) {
    ultimoPaso = ahora;
    for (int i = 0; i < 4; ++i)
      baseRampa[i] = rampas[i].paso(traslacion[i], ahora, PAUSA_INVERSION_MS);
  }
  const int limite = int(255 * DUTY_MAX);
  giroAplicadoPwm = giroPwm;
  for (int i = 0; i < 4; ++i) {
    int v = constrain(baseRampa[i] + giroPwm, -limite, limite);
    const int s = v > 0 ? 1 : (v < 0 ? -1 : 0);
    if (s != 0) {
      // Invertir una rueda en marcha: primero un instante a 0 (freno a GND del VNH5019).
      if (ultimoSigno[i] != 0 && s != ultimoSigno[i] &&
          uint32_t(ahora - ultimoActivo[i]) < PAUSA_INVERSION_GIRO_MS) {
        v = 0;
      } else {
        ultimoSigno[i] = s;
        ultimoActivo[i] = ahora;
      }
    }
    aplicarMotor(motores[i], v);
    if (fallaPermanente) { detenerMotores(); return; }
  }
}

// giro (-1..1, salida del PID) -> cuentas PWM de rotación pura.
// Parado: PWM_MIN_GIRO..PWM_MAX_GIRO (vence la fricción estática; antes la zona muerta de 15
// cuentas anulaba el PID hasta ~5° de error y el chasis se quedaba 7-9° fuera de rumbo).
// Trasladando las ruedas ya están en movimiento: proporcional, sin mínimo.
int giroAPwm(float giro, float error, float w, bool trasladando) {
  if (fabsf(error) < BANDA_GIRO_DEG && fabsf(w) < 20.0f) return 0;
  const float a = fminf(fabsf(giro), 1.0f);
  if (a < 0.02f) return 0;
  const int pwm = trasladando ? int(a * PWM_MAX_GIRO)
                              : PWM_MIN_GIRO + int(a * (PWM_MAX_GIRO - PWM_MIN_GIRO));
  return giro < 0 ? -pwm : pwm;
}

bool leerHeadingBNO(float &heading) {
  uint8_t b[2];
  if (!bus::leer(DIRECCION_BNO055, 0x1A, b, 2)) return false;
  heading = uint16_t(uint16_t(b[0]) | (uint16_t(b[1]) << 8)) / 16.0f;
  return heading < 360.0f;
}

bool inicializarBNO() {
  if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)) return false;
  Wire.setTimeOut(10);
  Wire.setClock(100000);
  // Esperas limitadas: un sensor ausente no puede dejar setup en un bucle.
  if (!bus::esperarId(DIRECCION_BNO055, 0x00, 0xA0, 1000)) return false;
  if (!bus::escribir(DIRECCION_BNO055, 0x3D, 0x00)) return false; // CONFIG
  delay(25);
  if (!bus::escribir(DIRECCION_BNO055, 0x3F, 0x20)) return false; // reset
  delay(700); // arranque del BNO055 tras reset
  if (!bus::esperarId(DIRECCION_BNO055, 0x00, 0xA0, 1000)) return false;
  delay(50);
  if (!bus::escribir(DIRECCION_BNO055, 0x07, 0x00) || // página cero
      !bus::escribir(DIRECCION_BNO055, 0x3E, 0x00) || // potencia normal
      !bus::escribir(DIRECCION_BNO055, 0x3B, 0x00) || // grados y grados/s
      !bus::escribir(DIRECCION_BNO055, 0x3F, BNO_USAR_CRISTAL_EXTERNO ? 0x80 : 0x00)) return false;
  delay(10);
  if (!bus::escribir(DIRECCION_BNO055, 0x3D, 0x08)) return false; // IMUPLUS
  delay(500);

  uint8_t id = 0, estado[2];
  if (!bus::leer(DIRECCION_BNO055, 0x00, &id, 1) || id != 0xA0 ||
      !bus::leer(DIRECCION_BNO055, 0x39, estado, 2) || estado[0] != 5 || estado[1] != 0) return false;

  // Cero = promedio de 10 muestras (120 ms) en vez de una sola lectura.
  float primera;
  if (!leerHeadingBNO(primera)) return false;
  float suma = 0.0f;
  for (uint8_t i = 0; i < 10; ++i) {
    delay(12);
    float h;
    if (!leerHeadingBNO(h)) return false;
    suma += campo::envolverGrados(h - primera);
  }
  ceroBNO = primera + suma / 10.0f;
  return true;
}

bool leerBNO(float &yaw, float &velocidadYaw) {
  // ID, estado, unidades y modo se validan a 10 Hz; el burst de datos, en cada ciclo.
  // Antes eran 5 transacciones I2C (~2.5 ms a 100 kHz) por ciclo.
  static bool validado = false;
  static uint32_t ultimaValidacion = 0;
  const uint32_t ahora = millis();
  if (!validado || uint32_t(ahora - ultimaValidacion) >= 100) {
    uint8_t id = 0, estado[2], unidades = 0, modo = 0;
    if (!bus::leer(DIRECCION_BNO055, 0x00, &id, 1) || id != 0xA0 ||
        !bus::leer(DIRECCION_BNO055, 0x39, estado, 2) || estado[0] != 5 || estado[1] != 0 ||
        !bus::leer(DIRECCION_BNO055, 0x3B, &unidades, 1) || unidades != 0 ||
        !bus::leer(DIRECCION_BNO055, 0x3D, &modo, 1) || modo != 0x08) return false;
    validado = true;
    ultimaValidacion = ahora;
  }
  uint8_t datos[8];
  if (!bus::leer(DIRECCION_BNO055, 0x14, datos, 8)) return false;
  // Unidades del BNO055 tras reset: Euler 1/16 grado; gyro 1/16 grado/s.
  // datos[0..5] = gyro X,Y,Z (0x14..0x19); datos[6..7] = heading (0x1A..0x1B).
  float heading = uint16_t(uint16_t(datos[6]) | (uint16_t(datos[7]) << 8)) / 16.0f;
  if (heading >= 360.0f) return false;
  yaw = campo::envolverGrados(SIGNO_YAW * (heading - ceroBNO));
  velocidadYaw = SIGNO_GYRO_Z * bus::little(datos + 4) / 16.0f;
  return isfinite(yaw) && isfinite(velocidadYaw);
}

void setup() {
  diagnostico::salida(PIN_SOLENOIDE, SOLENOIDE_ACTIVO_EN_HIGH ? LOW : HIGH);
  escribirSolenoide(false);

  pinMode(PIN_EN_DIAG, INPUT);
  
  bool pwmOK = configurarMotores();
  // Sin buffer de TX, cada printf de ~280 caracteres bloqueaba el lazo ~25 ms a 115200 baud.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  Serial.println("FIRMWARE=PWM6-BNO055-CH6-PUNTUAL-V4-GYRO");
  diagnostico::inicio();

  // Initialize reading individual PWM pins
  ppm::comenzar(RC_PINS, 6);
  
  imuDisponible = inicializarBNO();
  controlRumbo.fijar(0.0f);
  if (!pwmOK) causaFalla = "CONFIG_PWM";
  if (!imuDisponible) registrarFalla("INICIO_IMU");

  Serial.printf("BNO055=%s PWM=%s. Mantener inmovil al arrancar.\n",
                imuDisponible ? "OK" : "FAIL", pwmOK ? "OK" : "FAIL");
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
  static float yaw = 0.0f;
  static float velocidadYaw = 0.0f;
  // Vigilancia de signos yaw/gyro (ventanas de 200 ms)
  static float yawPrevio = 0.0f, sumaDYaw = 0.0f, sumaW = 0.0f;
  static float ultDYaw = 0.0f, ultIW = 0.0f;
  static bool yawPrevioValido = false;
  static uint32_t inicioVentana = 0;
  static uint8_t ventanasMalas = 0;

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
      ejesFiltrados[EJE_GIRO] = filtros[EJE_GIRO].actualizar(canales[0], CENTRO_GIRO, ZONA_MUERTA);
      ejesFiltrados[EJE_AVANCE] = filtros[EJE_AVANCE].actualizar(canales[2], CENTRO_AVANCE, ZONA_MUERTA);
      ejesFiltrados[EJE_LATERAL] = filtros[EJE_LATERAL].actualizar(canales[3], CENTRO_LATERAL, ZONA_MUERTA);
    }
  }

  bool ppmVigente = cantidad >= 6 && ppm::vigentes(TIMEOUT_PPM_MS * 1000UL) &&
                    uint32_t(micros() - ultimaTramaUs) <= TIMEOUT_PPM_MS * 1000UL;

  if (!ppmVigente) for (auto &filtro : filtros) filtro.reiniciar();
  bool solenoideActivo = disparoSolenoide.actualizar(
    ppmVigente && recepcionIntegra && switchSolenoide.listo && arranque.listo && !fallaPermanente && digitalRead(PIN_EN_DIAG) == HIGH,
    switchSolenoide.pulso, ahora, PULSO_SOLENOIDE_MS);
  escribirSolenoide(solenoideActivo);

  if (uint32_t(ahora - ultimoControl) < CONTROL_MS) { delay(1); return; }
  float dt = ultimoControl == 0 ? CONTROL_MS / 1000.0f
                                : uint32_t(ahora - ultimoControl) / 1000.0f;
  ultimoControl = ahora;
  dt = campo::limitar(dt, 0.001f, 0.05f);

  bool imuValida = imuDisponible && !fallaPermanente && leerBNO(yaw, velocidadYaw);
  if (imuDisponible && !fallaPermanente && !imuValida) {
    registrarFalla("LECTURA_IMU");
    Serial.println("FALLA I2C/IMU: salidas bloqueadas hasta reset. Revisar cables y alimentacion.");
  }

  // Coherencia yaw/gyro: la integral de w debe coincidir en signo con el cambio de yaw.
  if (VIGILAR_SIGNO_GYRO && imuValida) {
    if (yawPrevioValido) {
      sumaDYaw += campo::envolverGrados(yaw - yawPrevio);
      sumaW += velocidadYaw * dt;
    }
    yawPrevio = yaw;
    yawPrevioValido = true;
    if (uint32_t(ahora - inicioVentana) >= 200) {
      inicioVentana = ahora;
      ultDYaw = sumaDYaw;
      ultIW = sumaW;
      if (fabsf(sumaW) > 15.0f && fabsf(sumaDYaw) > 5.0f && sumaDYaw * sumaW < 0.0f) {
        if (++ventanasMalas >= 3 && !fallaPermanente) {
          registrarFalla("SIGNO_GYRO");
          Serial.println("FALLA: yaw y gyro tienen signos opuestos. Invertir SIGNO_GYRO_Z "
                         "sin alterar SIGNO_YAW. Salidas bloqueadas hasta reset.");
        }
      } else {
        ventanasMalas = 0;
      }
      sumaDYaw = sumaW = 0.0f;
    }
  } else {
    yawPrevioValido = false;
    sumaDYaw = sumaW = 0.0f;
  }

  bool driversOK = digitalRead(PIN_EN_DIAG) == HIGH;
  bool centrado = abs(int(canales[0]) - CENTRO_GIRO) <= ZONA_MUERTA &&
                  abs(int(canales[2]) - CENTRO_AVANCE) <= ZONA_MUERTA &&
                  abs(int(canales[3]) - CENTRO_LATERAL) <= ZONA_MUERTA;
  bool listo = arranque.actualizar(ppmVigente && imuValida && driversOK && !fallaPermanente,
                                   centrado, ahora);
  int traslacion[4] = {};

  bool cambioModo = false;
  if (!listo || recepcionIntegra)
    cambioModo = selectorRumbo.actualizar(listo, canales[5], ahora);
  else
    selectorRumbo.desde = ahora; // no confirmar una selección durante el hueco
  const int8_t posicionRumbo = INVERTIR_CH6 ? -selectorRumbo.posicion : selectorRumbo.posicion;
  float stickGiro = SIGNO_STICK_GIRO *
    omni::eje(ejesFiltrados[EJE_GIRO], 1000, CENTRO_GIRO, 2000, ZONA_MUERTA);
  float avanceCampo = SIGNO_STICK_AVANCE *          // CH3
    omni::eje(ejesFiltrados[EJE_AVANCE], 1000, CENTRO_AVANCE, 2000, ZONA_MUERTA);
  float lateralCampo = SIGNO_STICK_LATERAL *        // CH4
    omni::eje(ejesFiltrados[EJE_LATERAL], 1000, CENTRO_LATERAL, 2000, ZONA_MUERTA);
  // El neutro CRUDO tiene prioridad sobre cualquier memoria del filtro.
  if (centrado) stickGiro = avanceCampo = lateralCampo = 0.0f;
  float giro = 0.0f;
  int giroPwm = 0;
  if (!listo || !pwmListo || fallaPermanente) {
    if (imuValida) rumbo.detener(yaw);
    detenerMotores();
  } else {
    giro = rumbo.paso(cambioModo, posicionRumbo, yaw, velocidadYaw,
                      avanceCampo, lateralCampo, stickGiro, dt,
                      VELOCIDAD_REFERENCIA_DEG_S, KP_RUMBO, KI_RUMBO, KD_RUMBO);
    float avanceRobot, lateralRobot;
    campo::aCoordenadasRobot(avanceCampo, lateralCampo, yaw,
                             avanceRobot, lateralRobot);
    float salida[4];
    omni::mezclar(avanceRobot, lateralRobot, 0.0f, salida); // sólo traslación
    for (int i = 0; i < 4; ++i) {
      traslacion[i] = int(salida[i] * 255.0f * DUTY_MAX);
      if (abs(traslacion[i]) < PWM_DEADBAND_CORTE) traslacion[i] = 0;
    }
    const bool trasladando = fabsf(avanceCampo) > 0.001f || fabsf(lateralCampo) > 0.001f;
    const float errRumbo = campo::envolverGrados(controlRumbo.referencia - yaw);
    giroPwm = giroAPwm(giro, errRumbo, velocidadYaw, trasladando);
    if (rumbo.reposo || (centrado && !rumbo.alineando)) detenerMotores();
    else aplicarObjetivos(traslacion, giroPwm, ahora);
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
                              (!imuValida ? 4 : 0) | (fallaPermanente ? 8 : 0);
  uint32_t tramasOK = 0, tramasBAD = 0;
  uint16_t intervaloBAD = 0;
  ppm::diagnostico(tramasOK, tramasBAD, intervaloBAD);
  static uint8_t anteriorFalla = 255;
  if (estadoFalla != anteriorFalla) {
    anteriorFalla = estadoFalla;
    Serial.printf("EVENTO t=%lu PPM=%s DIAG=%s IMU=%s BLOQUEO=%s CAUSA=%s NCH=%u BAD=%lu ULT_BAD=%u\n",
      (unsigned long)ahora, ppmVigente ? "OK" : "FAIL", driversOK ? "OK" : "FAIL",
      imuValida ? "OK" : "FAIL", fallaPermanente ? "SI" : "NO", causaFalla,
      unsigned(canalesTrama), (unsigned long)tramasBAD, unsigned(intervaloBAD));
  }
  if (uint32_t(ahora - ultimoPrint) >= 250) {
    ultimoPrint = ahora;
    const char *modo = posicionRumbo > 0 ? "SEL_0" :
                       (posicionRumbo < 0 ? "SEL_180" : "CENTRO");
    Serial.printf(
      "t=%lu LISTO=%s CH1=%u CH3=%u CH4=%u CH5=%u CH6=%u MODO=%s AUTO=%s AUTO_TO=%s HOLD=%s "
      "yaw=%.1f w=%.1f dY=%.1f iW=%.1f ref=%.1f err=%.1f G=%.2f A=%.2f L=%.2f GP=%d "
      "PPM=%s DIAG=%s IMU=%s CAUSA=%s SOL=%s PWM=%d,%d,%d,%d RX=%lu BAD=%lu ULT_BAD=%u EDAD=%lu NCH=%u TRAMA=%s\n",
      (unsigned long)ahora, listo ? "SI" : "NO",
      unsigned(canales[0]), unsigned(canales[2]), unsigned(canales[3]), unsigned(canales[4]), unsigned(canales[5]),
      modo, rumbo.alineando ? "SI" : "NO", rumbo.alineacionAgotada ? "SI" : "NO", rumbo.reposo ? "REPOSO" : "CORRIGE",
      yaw, velocidadYaw, ultDYaw, ultIW, controlRumbo.referencia,
      campo::envolverGrados(controlRumbo.referencia - yaw), stickGiro, avanceCampo, lateralCampo, giroAplicadoPwm,
      ppmVigente ? "OK" : "FAIL", driversOK ? "OK" : "FAIL", imuValida ? "OK" : "FAIL",
      causaFalla, solenoideActivo ? "ON" : "OFF",
      baseRampa[0] + giroAplicadoPwm, baseRampa[1] + giroAplicadoPwm,
      baseRampa[2] + giroAplicadoPwm, baseRampa[3] + giroAplicadoPwm,
      (unsigned long)tramasOK, (unsigned long)tramasBAD, unsigned(intervaloBAD),
      (unsigned long)(uint32_t(micros() - ultimaTramaUs) / 1000), unsigned(canalesTrama),
      recepcionIntegra ? "OK" : "DESCARTADA");
  }
  delay(1);
}
