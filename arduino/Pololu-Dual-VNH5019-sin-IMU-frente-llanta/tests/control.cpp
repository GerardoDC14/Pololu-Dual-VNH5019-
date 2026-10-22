#include <initializer_list>
#include <cassert>
#include <cstdio>
#include "../esp32_omni4_sin_imu/esp32_omni4_sin_imu.ino"
void tick(int ch1=1500,int ch3=1500,int ch4=1500,int ch6=1500,bool rx=true) {
 tiempoMs+=20; tiempoUs=tiempoMs*1000;
 if(rx) {
  const int ch[]={ch1,1500,ch3,ch4,1000,ch6};
  for(int i=0;i<6;++i) pwm::pwmChannels[i]=ch[i];
  pwm::publicadaUs=tiempoUs; pwm::pendiente=true; ++pwm::tramasValidas;
 }
 loop();
}
void stopped() { for(auto &r:rampas) assert(r.actual==0); }
int main() {
 nivel=true; setup();
 for(int i=0;i<30;++i) tick();
 assert(arranque.listo && pwmListo && !fallaPermanente);
 const int signs[3][4]={{1,1,1,1},{0,-1,0,1},{1,0,-1,0}};
 for(int axis=0;axis<3;++axis) {
  for(int pos : {1000,1500,2000}) {
   for(int i=0;i<220;++i) tick(axis==0?2000:1500,axis==1?2000:1500,axis==2?2000:1500,pos);
   for(int j=0;j<4;++j) assert(rampas[j].actual==signs[axis][j]*153);
  }
  tick(); stopped();
 }
 for(int pos : {1000,1500,2000}) {tick(1500,1500,1500,pos);stopped();}
 for(int i=0;i<200;++i) tick(2000);
 for(int i=0;i<6;++i) tick(2000,1500,1500,1500,false);
 stopped(); assert(!arranque.listo);
 for(int i=0;i<30;++i) tick();
 assert(arranque.listo);
 for(int i=0;i<200;++i) tick(2000);
 nivel=false; tick(2000); stopped(); assert(!arranque.listo);
 puts("OK: arranque sin sensor, tres ejes RC, CH6 sin giro automatico, parada neutral, timeout y DIAG.");
}
