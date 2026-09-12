#include <cassert>
#include <cmath>
#include "../arduino/esp32_omni4_rc/ControlOmni.h"

bool cerca(float a, float b) { return std::fabs(a - b) < 1e-5f; }
int main() {
  float w[4];
  omni::mezclar(1, 0, 0, w);
  assert(cerca(w[0], 0) && cerca(w[1], -1) && cerca(w[2], 0) && cerca(w[3], 1));
  omni::mezclar(0, 1, 0, w);
  assert(cerca(w[0], 1) && cerca(w[1], 0) && cerca(w[2], -1) && cerca(w[3], 0));
  omni::mezclar(0, 0, 1, w);
  for (float v : w) assert(cerca(v, 1));
  omni::mezclar(1, .5f, .5f, w);
  assert(cerca(w[0], 2.0f/3) && cerca(w[1], -1.0f/3) && cerca(w[2], 0) && cerca(w[3], 1));
  for (int x = -10; x <= 10; ++x)
    for (int y = -10; y <= 10; ++y)
      for (int z = -10; z <= 10; ++z) {
        omni::mezclar(x/10.f, y/10.f, z/10.f, w);
        for (float v : w) assert(std::fabs(v) <= 1.00001f);
        assert(cerca(w[0] + w[2], w[1] + w[3]));
      }
  assert(omni::eje(1500,1000,1500,2000,40)==0);
  assert(omni::eje(1540,1000,1500,2000,40)==0);
  assert(omni::eje(2000,1000,1500,2000,40)==1);
  assert(omni::eje(900,1000,1500,2000,40)==-1);

  omni::Habilitacion h;
  h.actualizar(true,2000,true); assert(!h.activo);
  h.actualizar(true,1000,true); h.actualizar(true,2000,false); assert(!h.activo);
  h.actualizar(true,2000,true); assert(!h.activo);
  h.actualizar(true,1000,true); h.actualizar(true,2000,true); assert(h.activo);
  h.actualizar(true,2000,false); assert(h.activo);
  h.actualizar(false,2000,false); assert(!h.activo);
  h.actualizar(true,2000,true); assert(!h.activo);
  h.actualizar(true,1000,true); h.actualizar(true,2000,true); assert(h.activo);
  h.actualizar(true,1500,true); assert(!h.activo);

  omni::Rampa r;
  for (int t=10;t<=770;t+=10) r.paso(77,t,1000);
  assert(r.actual==77);
  for (int t=780;t<=1540;t+=10) r.paso(-77,t,1000);
  assert(r.actual==0 && r.ceroDesde==1540);
  assert(r.paso(-77,2539,1000)==0);
  assert(r.paso(-77,2540,1000)==-1);
  r.cortar(2550); assert(r.actual==0);
  r.cortar(2700); assert(r.ceroDesde==2550);
  assert(r.paso(77,3549,1000)==0);
  assert(r.paso(77,3550,1000)==1);
  r.cortar(UINT32_MAX-100);
  assert(r.paso(-77,898,1000)==0);
  assert(r.paso(-77,899,1000)==-1);
}
