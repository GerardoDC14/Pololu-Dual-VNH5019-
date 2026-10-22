#include <cassert>
#include <cmath>
#include <initializer_list>
#include "../arduino/esp32_omni4_bno055/ControlCampo.h"
#include "../arduino/esp32_omni4_bno055/ControlEntrada.h"
#include "../arduino/esp32_omni4_bno055/PPM.h"
float paso(campo::RumboHibrido &r, int modo, float yaw, float a=0, float l=0, float g=0, bool cambio=false) {
 return r.paso(cambio,modo,yaw,0,a,l,g,.01f,120,.018f,.0005f,.0025f);
}
void pulso(unsigned ch, unsigned ancho=1500) {
 nivel=true; tiempoUs+=100; pwm::handleInterrupt(ch);
 nivel=false; tiempoUs+=ancho; pwm::handleInterrupt(ch);
}
int main() {
 // CH6 mantenido en un extremo NO impone rumbo sin una orden nueva.
 for (int modo : {-1,0,1}) {
   campo::RumboHibrido r; r.detener(25);
   paso(r,modo,25,1);
   assert(r.control.referencia == 25.f && !r.reposo);
   assert(paso(r,modo,30,1)<0); // deriva: corregir hacia 25
   assert(paso(r,modo,30)==0 && r.reposo);
   paso(r,modo,30,0,1); assert(!r.reposo);
   assert(paso(r,modo,30)==0 && r.reposo);
   paso(r,modo,30,0,0,1); assert(!r.reposo);
   paso(r,modo,45,1); // soltar CH1 y seguir CH3: sostener el rumbo nuevo
   assert(r.control.referencia==45.f && !r.alineando);
   assert(paso(r,modo,50,1)<0);
   assert(paso(r,modo,50)==0 && r.reposo);
 }
 campo::RumboHibrido r; r.detener(40);
 assert(paso(r,1,40,0,0,0,true)<0 && r.alineando); // ir a cero
 paso(r,1,20,1); assert(r.alineando); // CH3 NO cancela
 paso(r,1,10,0,1); assert(r.alineando); // CH4 NO cancela
 paso(r,1,0); assert(r.alineando); // no terminar por una sola muestra
 for(int i=0;i<16;i++) paso(r,1,0);
 assert(!r.alineando && r.reposo && !r.alineacionAgotada);
 // Mantener CH6 arriba, girar con CH1 y soltar: no regresar a 0.
 paso(r,1,0,0,0,1); paso(r,1,60,1);
 assert(r.control.referencia==60.f);
 assert(paso(r,1,65,1)<0); // estabilizar en 60, no en 0
 paso(r,1,65); assert(r.reposo);
 // Envolvimiento: 170 -> 180 debe ser +10, nunca -350.
 assert(paso(r,-1,170,0,0,0,true)>0 && r.alineando);
 assert(paso(r,-1,-175)<0); // sobrepasar: error corto -5
 for(int i=0;i<16;i++) paso(r,-1,-180,1);
 assert(!r.alineando && r.control.referencia==-180.f);
 paso(r,-1,-180,0,0,-1); paso(r,-1,-120,1);
 assert(r.control.referencia==-120.f); // libre despues de 180 tambien
 // Llegada requiere estabilidad; una lectura lenta aislada no basta.
 campo::RumboHibrido estable; estable.detener(20);
 paso(estable,1,20,0,0,0,true);
 for(int i=0;i<10;i++) paso(estable,1,0);
 paso(estable,1,4); // salir de banda reinicia asentamiento
 for(int i=0;i<10;i++) paso(estable,1,0);
 assert(estable.alineando);
 estable.paso(false,1,0,30,0,0,0,.01f,120,.018f,.0005f,.0025f);
 assert(estable.alineando && estable.tiempoEnBanda==0.f);
 for(int i=0;i<16;i++) paso(estable,1,0);
 assert(!estable.alineando && !estable.alineacionAgotada);
 // CH1 interrumpe CH6; centro cancela; nunca se acumulan vueltas.
 paso(r,1,-120,0,0,0,true); assert(r.alineando);
 paso(r,1,-100,0,0,1); assert(!r.alineando);
 paso(r,-1,20,0,0,0,true); assert(r.alineando);
 paso(r,0,20,0,0,0,true); assert(!r.alineando && r.reposo);
 paso(r,-1,0,0,0,0,true);
 for(int i=0;i<410;i++) paso(r,-1,0);
 assert(!r.alineando && r.reposo && r.alineacionAgotada);
 // Un timeout tampoco deja un objetivo pendiente al seguir trasladandose.
 paso(r,-1,0,1); assert(r.control.referencia==0.f);
 paso(r,-1,5,1); assert(r.control.referencia==0.f);
 r.detener(0); assert(!r.alineacionAgotada);
 entrada::FiltroEje f; f.actualizar(2000,1500,30);
 assert(f.actualizar(1500,1500,30)==1500);
 entrada::SelectorEstable sel;
 assert(!sel.actualizar(true,1000,0));
 assert(!sel.actualizar(true,2000,10));
 assert(!sel.actualizar(true,2000,89));
 assert(sel.actualizar(true,2000,90)); assert(sel.posicion==1);
 pwm::comenzar(); assert(!pwm::vigentes(100000));
 uint16_t datos[6]; uint8_t n; uint32_t marca;
 pulso(0); assert(!pwm::leer(datos,n,marca));
 for (unsigned ch : {2,3,4,5}) pulso(ch);
 assert(pwm::leer(datos,n,marca)); assert(pwm::vigentes(100000));
 for (int i=0;i<20;i++) for(unsigned ch : {2,3,4,5}) pulso(ch);
 assert(!pwm::vigentes(100000));
 pulso(0); assert(pwm::vigentes(100000));
 pulso(3,700); assert(!pwm::vigentes(100000));
 pulso(3); assert(pwm::vigentes(100000));
 tiempoUs+=100001; assert(!pwm::vigentes(100000));
 tiempoUs=0xffffe000U; pwm::comenzar();
 for(unsigned ch : {0,2,3,4,5}) pulso(ch);
 assert(pwm::vigentes(100000)); tiempoUs+=100001; assert(!pwm::vigentes(100000));
}
