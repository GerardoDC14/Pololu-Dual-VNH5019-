#include <cassert>
#include <cmath>
#include "../arduino/esp32_omni4_bno055/ControlCampo.h"

bool cerca(float a, float b, float tolerancia = 1e-4f) {
  return std::fabs(a - b) < tolerancia;
}

int main() {
  assert(cerca(campo::envolverGrados(190), -170));
  assert(cerca(campo::envolverGrados(-190), 170));

  float avance, lateral;
  campo::aCoordenadasRobot(1, 0, 0, avance, lateral);
  assert(cerca(avance, 1) && cerca(lateral, 0));
  campo::aCoordenadasRobot(1, 0, 90, avance, lateral);
  assert(cerca(avance, 0) && cerca(lateral, -1));
  campo::aCoordenadasRobot(0, 1, -90, avance, lateral);
  assert(cerca(avance, -1) && cerca(lateral, 0));

  campo::SelectorRumbo selector;
  assert(selector.actualizar(true, 1500) == 0);
  assert(selector.actualizar(true, 1900) == 1);
  assert(selector.actualizar(true, 1900) == 0);
  assert(selector.actualizar(true, 1500) == 0);
  assert(selector.actualizar(true, 1100) == -1);
  assert(selector.actualizar(false, 1100) == 0);
  assert(selector.actualizar(true, 1900) == 0); // reconexion: no orden espuria

  campo::ControlRumbo control;
  control.fijar(0);
  control.mover(1, 0.5f, 120);
  assert(cerca(control.referencia, 60));
  float salida = control.calcular(50, 0, 0.01f, 0.02f, 0, 0);
  assert(cerca(salida, 0.2f));
  control.fijar(180);
  assert(cerca(control.referencia, -180));
  salida = control.calcular(170, 0, 0.01f, 0.02f, 0, 0);
  assert(cerca(salida, 0.2f)); // error corto: -180 - 170 = +10 grados
}
