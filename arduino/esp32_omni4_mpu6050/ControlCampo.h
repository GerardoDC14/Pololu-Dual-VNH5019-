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

} // namespace campo
