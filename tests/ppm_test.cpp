#include <cassert>
#include "../arduino/esp32_omni4_rc/PPM.h"

void flancoDespues(uint32_t intervalo) {
  tiempoUs += intervalo;
  ppm::ppmISR();
}

int main() {
  ppm::comenzar(27);
  uint16_t canales[ppm::MAX_CHANNELS] = {};
  uint8_t cantidad = 0;
  assert(!ppm::leer(canales, cantidad));

  flancoDespues(4000); // sincronización inicial
  const uint16_t esperados[5] = {1100, 1200, 1500, 1800, 2000};
  for (uint16_t pulso : esperados) flancoDespues(pulso);
  flancoDespues(4000); // publica la trama anterior

  assert(ppm::leer(canales, cantidad));
  assert(cantidad == 5);
  for (int i = 0; i < 5; ++i) assert(canales[i] == esperados[i]);
  assert(!ppm::leer(canales, cantidad));

  // Los intervalos fuera de 800..2200 us no se guardan como canales.
  flancoDespues(700);
  flancoDespues(1500);
  flancoDespues(4000);
  assert(ppm::leer(canales, cantidad));
  assert(cantidad == 1 && canales[0] == 1500);
}
