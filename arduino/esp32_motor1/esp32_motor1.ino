// Control del motor 1 con ESP32 y VNH5019.
// Serial: 115200 baud
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

const uint8_t M1_INA = 25, M1_INB = 26, M1_PWM = 27, M1_DIAG = 32;
const uint8_t M2_INA = 18, M2_INB = 19, M2_PWM = 23, M2_EN = 33;
const uint32_t FRECUENCIA_PWM = 20000;
const uint32_t PASO_RAMPA_MS = 10;
const uint32_t PAUSA_CAMBIO_MS = 1000;

int dutyPorcentaje = 30;
int pwmActual = 0;
int direccion = 0;          // 0: detenido, 1: avance, -1: reversa
int ultimaDireccion = 0;
bool pwmListo = false;
bool falla = false;
bool esperandoCambio = false;
uint32_t ultimoPaso = 0;
uint32_t inicioParo = 0;
char linea[48];
size_t longitud = 0;
bool lineaLarga = false;

void salidaBaja(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void registrarFalla() {
  salidaBaja(M1_DIAG);
  if (pwmListo) ledcWrite(M1_PWM, 0);
  pwmActual = 0;
  direccion = 0;
  esperandoCambio = false;
  falla = true;
  Serial.println("Falla en M1. Salida deshabilitada hasta reset.");
}

bool escribirPWM(int valor) {
  if (!pwmListo || !ledcWrite(M1_PWM, valor)) {
    registrarFalla();
    return false;
  }
  pwmActual = valor;
  return true;
}

void parar() {
  salidaBaja(M1_DIAG);
  if (pwmActual > 0) inicioParo = millis();
  if (!escribirPWM(0)) return;
  direccion = 0;
  esperandoCambio = false;
  Serial.println("M1 detenido.");
}

void mover(int nuevaDireccion) {
  if (falla) {
    Serial.println("M1 bloqueado por falla. Requiere reset.");
    return;
  }
  if (direccion == nuevaDireccion) return;
  salidaBaja(M1_DIAG);
  if (pwmActual > 0) inicioParo = millis();
  if (!escribirPWM(0)) return;
  direccion = nuevaDireccion;
  // Tiempo de espera antes de invertir el giro.
  esperandoCambio = ultimaDireccion != 0 && ultimaDireccion != direccion &&
                     millis() - inicioParo < PAUSA_CAMBIO_MS;
  Serial.println(direccion == 1 ? "M1: avance." : "M1: reversa.");
}

void mostrarAyuda() {
  Serial.println("adelante | atras | parar | duty 0..100 | estado | ayuda");
}

void procesarComando(char *texto) {
  while (isspace(static_cast<unsigned char>(*texto))) ++texto;
  size_t n = strlen(texto);
  while (n > 0 && isspace(static_cast<unsigned char>(texto[n - 1]))) texto[--n] = '\0';
  for (size_t i = 0; i < n; ++i) texto[i] = tolower(static_cast<unsigned char>(texto[i]));
  if (n == 0) return;

  if (strcmp(texto, "parar") == 0 || strcmp(texto, "s") == 0) {
    parar();
  } else if (strcmp(texto, "adelante") == 0) {
    mover(1);
  } else if (strcmp(texto, "atras") == 0) {
    mover(-1);
  } else if (strncmp(texto, "duty ", 5) == 0) {
    char *fin;
    const long valor = strtol(texto + 5, &fin, 10);
    if (fin == texto + 5 || *fin != '\0' || valor < 0 || valor > 100) {
      Serial.println("Duty fuera de formato. Rango: 0-100, entero.");
      return;
    }
    dutyPorcentaje = valor;
    // duty 0 cancela el cambio de sentido pendiente.
    if (dutyPorcentaje == 0) parar();
    Serial.print("Duty (%): ");
    Serial.println(dutyPorcentaje);
  } else if (strcmp(texto, "estado") == 0) {
    Serial.print("Direccion: "); Serial.println(direccion);
    Serial.print("Duty (%): "); Serial.println(dutyPorcentaje);
    Serial.print("PWM aplicado (0-255): "); Serial.println(pwmActual);
    Serial.print("Cambio de sentido pendiente: "); Serial.println(esperandoCambio);
    Serial.print("Falla: "); Serial.println(falla);
  } else if (strcmp(texto, "ayuda") == 0) {
    mostrarAyuda();
  } else {
    Serial.println("Comando no reconocido.");
    mostrarAyuda();
  }
}

void leerSerial() {
  for (int i = 0; i < 64 && Serial.available(); ++i) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineaLarga) Serial.println("Comando excede 47 caracteres.");
      else {
        linea[longitud] = '\0';
        procesarComando(linea);
      }
      longitud = 0;
      lineaLarga = false;
    } else if (!lineaLarga) {
      if (longitud < sizeof(linea) - 1) linea[longitud++] = c;
      else lineaLarga = true;
    }
  }
}

void actualizarMotor() {
  if (falla || direccion == 0) return;
  const uint32_t ahora = millis();
  if (esperandoCambio) {
    if (ahora - inicioParo < PAUSA_CAMBIO_MS) return;
    esperandoCambio = false;
  }
  if (pwmActual == 0) {
    digitalWrite(M1_INA, direccion == 1 ? HIGH : LOW);
    digitalWrite(M1_INB, direccion == -1 ? HIGH : LOW);
    // EN/DIAG usa el pull-up del driver; no manejar como salida HIGH.
    pinMode(M1_DIAG, INPUT);
  }
  if (digitalRead(M1_DIAG) == LOW) {
    registrarFalla();
    return;
  }
  const int objetivo = (dutyPorcentaje * 255 + 50) / 100;
  if (ahora - ultimoPaso >= PASO_RAMPA_MS && pwmActual != objetivo) {
    ultimoPaso = ahora;
    const int siguiente = pwmActual + (pwmActual < objetivo ? 1 : -1);
    if (escribirPWM(siguiente) && siguiente > 0) ultimaDireccion = direccion;
  }
}

void setup() {
  salidaBaja(M1_DIAG);
  salidaBaja(M2_EN);
  salidaBaja(M1_PWM);
  salidaBaja(M2_PWM);
  salidaBaja(M1_INA);
  salidaBaja(M1_INB);
  salidaBaja(M2_INA);
  salidaBaja(M2_INB);
  Serial.begin(115200);
  pwmListo = ledcAttach(M1_PWM, FRECUENCIA_PWM, 8);
  if (!escribirPWM(0)) return;
  Serial.println("M1 detenido. Duty: 30%.");
  mostrarAyuda();
}

void loop() {
  actualizarMotor();
  leerSerial();
  delay(1);
}
