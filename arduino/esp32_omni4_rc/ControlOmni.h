#pragma once
#include <stdint.h>
#include <math.h>

namespace omni {
inline float limitar(float x, float a, float b) {
  return x < a ? a : (x > b ? b : x);
}

// Orden: frente, izquierda, atras, derecha. Traccion tangencial positiva.
inline void mezclar(float avance, float lateral, float giro, float salida[4]) {
  salida[0] = lateral + giro;
  salida[1] = -avance + giro;
  salida[2] = -lateral + giro;
  salida[3] = avance + giro;
  float escala = 1.0f;
  for (int i = 0; i < 4; ++i) escala = fmaxf(escala, fabsf(salida[i]));
  for (int i = 0; i < 4; ++i) salida[i] /= escala;
}

inline float eje(uint16_t pulso, int minimo, int centro, int maximo, int zona) {
  int diferencia = static_cast<int>(pulso) - centro;
  if (diferencia > zona)
    return limitar(float(diferencia - zona) / (maximo - centro - zona), 0, 1);
  if (diferencia < -zona)
    return limitar(float(diferencia + zona) / (centro - minimo - zona), -1, 0);
  return 0;
}

struct Habilitacion {
  bool activo = false;
  bool vioOff = false;
  bool anteriorOn = false;

  void actualizar(bool valido, int pulso, bool centrado) {
    if (!valido) {
      activo = vioOff = anteriorOn = false;
      return;
    }
    if (pulso < 1300) {
      activo = false;
      vioOff = centrado;
      anteriorOn = false;
    } else if (pulso > 1700) {
      if (!anteriorOn) {
        activo = vioOff && centrado;
        vioOff = false;
      }
      anteriorOn = true;
    } else {
      activo = vioOff = anteriorOn = false;
    }
  }
};

struct Rampa {
  int actual = 0;
  int ultimoSigno = 0;
  uint32_t ceroDesde = 0;

  void cortar(uint32_t ahora) {
    if (actual != 0) ceroDesde = ahora;
    actual = 0;
  }

  int paso(int objetivo, uint32_t ahora, uint32_t pausa) {
    objetivo = objetivo < -255 ? -255 : (objetivo > 255 ? 255 : objetivo);
    int signo = objetivo > 0 ? 1 : (objetivo < 0 ? -1 : 0);
    // La inversion primero baja a cero; la pausa empieza al llegar a cero.
    if (actual != 0 && signo != 0 && signo != (actual > 0 ? 1 : -1)) objetivo = 0;
    if (actual == 0 && signo != 0 && ultimoSigno != 0 && signo != ultimoSigno &&
        uint32_t(ahora - ceroDesde) < pausa) return 0;
    int previo = actual;
    if (actual < objetivo) ++actual;
    else if (actual > objetivo) --actual;
    if (actual == 0 && previo != 0) ceroDesde = ahora;
    if (actual != 0) ultimoSigno = actual > 0 ? 1 : -1;
    return actual;
  }
};
}
