#pragma once
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

namespace entrada {
// Actualizar exactamente una vez por trama; una trama inválida rompe la racha.
struct SwitchPorTramas {
  bool listo = false;
  int8_t candidato = -1;
  uint8_t consecutivas = 0;
  uint16_t pulso = 1500;
  void actualizar(bool valida, uint16_t recibido) {
    int8_t nuevo = recibido < 1300 ? 0 : (recibido > 1700 ? 1 : -1);
    if (!valida || nuevo < 0) {
      listo = false; candidato = -1; consecutivas = 0;
      return;
    }
    if (nuevo != candidato) { candidato = nuevo; consecutivas = 1; }
    else if (consecutivas < 3) ++consecutivas;
    // Durante un candidato nuevo se conserva el estado confirmado, sin disparar.
    if (consecutivas >= 3) { pulso = nuevo ? 2000 : 1000; listo = true; }
  }
};
// Se actualiza una vez por trama válida, nunca sobre datos repetidos del loop.
struct FiltroEje {
  bool inicializado = false;
  uint16_t muestras[3] = {};
  uint8_t indice = 0;
  float valor = 1500.0f;

  void reiniciar() { inicializado = false; }
  uint16_t actualizar(uint16_t pulso, int centro, int zona, float alpha = 0.35f) {
    if (!inicializado || abs(int(pulso) - centro) <= zona) {
      valor = abs(int(pulso) - centro) <= zona ? float(centro) : float(pulso);
      for (auto &m : muestras) m = uint16_t(valor);
      indice = 0;
      inicializado = true;
      return uint16_t(valor);
    }
    muestras[indice] = pulso;
    indice = (indice + 1) % 3;
    uint16_t a = muestras[0], b = muestras[1], c = muestras[2];
    uint16_t mediana = a > b ? (b > c ? b : (a > c ? c : a))
                            : (a > c ? a : (b > c ? c : b));
    valor += alpha * (float(mediana) - valor);
    return uint16_t(lroundf(valor));
  }
};

} // namespace entrada
