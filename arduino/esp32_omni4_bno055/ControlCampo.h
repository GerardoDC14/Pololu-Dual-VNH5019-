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

// CH6 es una orden puntual: ir a 0/180 por el camino corto y luego liberar.
// CH1 tiene prioridad manual. CH3/CH4 se procesan incluso al alinear.
// En conduccion normal, sostener el ultimo rumbo al trasladarse sin CH1.
struct RumboHibrido {
  static constexpr float BANDA_SALIDA_DEG = 1.5f;
  static constexpr float VEL_LLEGADA_DEG_S = 15.0f;
  static constexpr float ASENTAMIENTO_S = 0.15f;
  static constexpr float ADELANTO_MAX_DEG = 40.0f;
  static constexpr float TIEMPO_ALINEACION_S = 4.0f;
  ControlRumbo control;
  bool alineando = false, corrigiendo = false, reposo = true;
  bool giroAnterior = false;
  bool alineacionAgotada = false;
  float tiempoAlineando = 0.0f, tiempoEnBanda = 0.0f;

  void detener(float yaw) {
    control.fijar(yaw);
    alineando = corrigiendo = giroAnterior = alineacionAgotada = false;
    tiempoAlineando = tiempoEnBanda = 0.0f;
    reposo = true;
  }

  float paso(bool cambioModo, int8_t modo, float yaw, float velocidadYaw,
             float avance, float lateral, float stickGiro, float dt,
             float velocidadReferencia, float kp, float ki, float kd) {
    const bool traslacion = fabsf(avance) > 0.001f || fabsf(lateral) > 0.001f;
    const bool giroManual = fabsf(stickGiro) > 0.001f;
    if (cambioModo) {
      control.fijar(modo > 0 ? 0.0f : (modo < 0 ? 180.0f : yaw));
      alineando = modo != 0;
      alineacionAgotada = false;
      tiempoAlineando = tiempoEnBanda = 0.0f;
    }
    if (giroManual) {
      // CH1 cancela la maniobra automatica inmediatamente, sin dejar de leer RC.
      if (!giroAnterior || alineando || cambioModo) control.fijar(yaw);
      alineando = false;
      control.mover(stickGiro, dt, velocidadReferencia);
      float adelanto = limitar(envolverGrados(control.referencia - yaw),
                               -ADELANTO_MAX_DEG, ADELANTO_MAX_DEG);
      control.referencia = envolverGrados(yaw + adelanto);
    } else if (giroAnterior && !alineando) {
      // Soltar CH1: mantener este nuevo rumbo, independientemente de CH6.
      control.fijar(yaw);
    }
    giroAnterior = giroManual;

    if (alineando) {
      tiempoAlineando += dt;
      const float error = envolverGrados(control.referencia - yaw);
      if (fabsf(error) <= BANDA_SALIDA_DEG &&
          fabsf(velocidadYaw) <= VEL_LLEGADA_DEG_S) tiempoEnBanda += dt;
      else tiempoEnBanda = 0.0f;
      const bool llego = tiempoEnBanda >= ASENTAMIENTO_S;
      if (llego || tiempoAlineando >= TIEMPO_ALINEACION_S) {
        alineacionAgotada = !llego;
        alineando = false;
        // Sin referencia pendiente que provoque otro giro al liberar CH6.
        control.fijar(yaw);
      }
    }
    // CH3/CH4 nunca cancelan CH6; la traslacion y el rumbo se calculan juntos.
    corrigiendo = traslacion || giroManual || alineando;
    reposo = !corrigiendo;
    if (reposo) {
      // PWM cero en neutro; tomar rumbo actual para la siguiente traslacion.
      control.fijar(yaw);
      return 0.0f;
    }
    return control.calcular(yaw, velocidadYaw, dt, kp, ki, kd);
  }
};

} // namespace campo
