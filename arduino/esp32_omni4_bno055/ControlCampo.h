#pragma once

#include <math.h>
#include <stdint.h>

namespace campo {

inline float limitar(float valor, float minimo, float maximo) {
  return valor < minimo ? minimo : (valor > maximo ? maximo : valor);
}

inline float envolverGrados(float angulo) {
  while (angulo >= 180.0f) angulo -= 360.0f;
  while (angulo < -180.0f) angulo += 360.0f;
  return angulo;
}

// Convierte un vector expresado respecto al campo a los ejes instantaneos
// del chasis. X es avance, Y es izquierda y yaw positivo es antihorario.
inline void aCoordenadasRobot(float avanceCampo, float lateralCampo,
                              float yawGrados, float &avanceRobot,
                              float &lateralRobot) {
  float radianes = yawGrados * 0.01745329251994329577f;
  float c = cosf(radianes);
  float s = sinf(radianes);
  avanceRobot = c * avanceCampo + s * lateralCampo;
  lateralRobot = -s * avanceCampo + c * lateralCampo;
}

struct SelectorRumbo {
  bool inicializado = false;
  int8_t posicionAnterior = 0;

  // Devuelve -1 para seleccionar 180 grados, +1 para cero y 0 sin orden.
  int8_t actualizar(bool valido, uint16_t pulso) {
    if (!valido) {
      inicializado = false;
      return 0;
    }

    int8_t posicion = pulso < 1300 ? -1 : (pulso > 1700 ? 1 : 0);
    if (!inicializado) {
      inicializado = true;
      posicionAnterior = posicion;
      return 0;
    }

    int8_t orden = posicion != posicionAnterior && posicion != 0 ? posicion : 0;
    posicionAnterior = posicion;
    return orden;
  }
};

struct ControlRumbo {
  float referencia = 0.0f;
  float integral = 0.0f;

  void fijar(float grados) {
    referencia = envolverGrados(grados);
    integral = 0.0f;
  }

  void mover(float stick, float dt, float velocidadMaximaGrados) {
    if (fabsf(stick) < 0.001f) return;
    referencia = envolverGrados(referencia + stick * velocidadMaximaGrados * dt);
    integral = 0.0f;
  }

  float calcular(float yaw, float velocidadYaw, float dt,
                 float kp, float ki, float kd) {
    float error = envolverGrados(referencia - yaw);
    if (dt > 0.0f && fabsf(error) < 45.0f) {
      integral = limitar(integral + error * dt, -120.0f, 120.0f);
    } else {
      integral = 0.0f;
    }
    return limitar(kp * error + ki * integral - kd * velocidadYaw,
                   -1.0f, 1.0f);
  }
};

// CH6 ordena alineaciones; en reposo no mantiene un PID que haga girar ruedas.
struct RumboHibrido {
  ControlRumbo control;
  bool alineando = false;
  bool enMovimiento = false;
  bool reposo = true;

  void detener(float yaw) {
    control.fijar(yaw);
    alineando = enMovimiento = false;
    reposo = true;
  }

  float paso(bool cambioModo, int8_t modo, float yaw, float velocidadYaw,
             float avance, float lateral, float stickGiro, float dt,
             float velocidadReferencia, float kp, float ki, float kd) {
    if (cambioModo) {
      if (modo == 0) detener(yaw);
      else {
        control.fijar(modo > 0 ? 0.0f : 180.0f);
        alineando = true;
      }
    }
    bool traslacion = fabsf(avance) > 0.001f || fabsf(lateral) > 0.001f;
    bool giroManual = fabsf(stickGiro) > 0.001f;
    if (giroManual) {
      if (alineando || !enMovimiento) control.fijar(yaw);
      alineando = false;
      control.mover(stickGiro, dt, velocidadReferencia);
      // El objetivo no puede adelantarse más de 40° al rumbo real.
      float adelanto = limitar(envolverGrados(control.referencia - yaw), -40.0f, 40.0f);
      control.referencia = envolverGrados(yaw + adelanto);
    } else if (traslacion && !enMovimiento && !alineando) {
      control.fijar(yaw); // mantener el rumbo al iniciar la traslación
    }
    float error = envolverGrados(control.referencia - yaw);
    bool dentro = fabsf(error) <= 2.0f && fabsf(velocidadYaw) <= 5.0f;
    if (alineando && dentro) alineando = false;
    reposo = !traslacion && !giroManual && !alineando;
    enMovimiento = traslacion || giroManual;
    if (reposo) { control.fijar(yaw); return 0.0f; }
    if (!giroManual && dentro) { control.integral = 0.0f; return 0.0f; }
    return control.calcular(yaw, velocidadYaw, dt, kp, ki, kd);
  }
};

} // namespace campo
