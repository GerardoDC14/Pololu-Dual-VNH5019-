#include <cassert>
#include "../arduino/esp32_omni4_rc/ReceptorPWM.h"

void pulso(int canal, uint32_t inicio, uint32_t ancho) {
  auto &c = receptor::canales[canal];
  tiempoUs = inicio; nivel = true; receptor::flanco(&c);
  tiempoUs += ancho; nivel = false; receptor::flanco(&c);
}
void trama(uint32_t inicio, int ancho=1500) {
  for (int i=0;i<4;++i) pulso(i,inicio,ancho);
}
int main() {
  receptor::iniciar();
  uint16_t p[4];
  assert(!receptor::leer(p));
  trama(1000); trama(21000); trama(41000);
  assert(!receptor::leer(p));
  trama(61000); assert(receptor::leer(p));
  for (auto v:p) assert(v==1500);
  tiempoUs += 100001; assert(!receptor::leer(p));
  trama(181000); assert(!receptor::leer(p));
  trama(201000); trama(221000); trama(241000); assert(receptor::leer(p));
  pulso(0,261000,2500); assert(!receptor::leer(p));
  trama(281000); trama(301000); trama(321000); assert(receptor::leer(p));
  trama(322000); assert(!receptor::leer(p)); // Periodo fuera de rango
  for (auto &c: receptor::canales) c = receptor::Canal{};
  receptor::iniciar();
  uint32_t inicio = UINT32_MAX-40000;
  for (int i=0;i<4;++i) trama(inicio + uint32_t(i*20000));
  assert(receptor::leer(p)); // Desbordamiento de micros()
}
