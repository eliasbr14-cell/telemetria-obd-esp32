// Teste no PC do núcleo OBD do firmware: codifica -> decodifica -> compara
#include "../telemetria_esp32/obd.h"
#include <cstdio>
#include <cmath>
int main(){
  int falhas=0, casos=0;
  struct {uint8_t pid; float lo, hi, tol;} faixas[] = {
    {0x0C,0,8000,0.125f},{0x0D,0,200,0.5f},{0x05,-40,130,0.5f},{0x0B,10,105,0.5f},{0x04,0,100,100/255.f/2},
    {0x11,0,100,100/255.f/2},{0x06,-25,25,100/128.f/2},{0x0F,-20,60,0.5f},{0x2F,0,100,100/255.f/2},{0x42,9,15,0.0005f}};
  for (auto& f: faixas){ float maxerr=0;
    for(int i=0;i<=1000;i++){ float v=f.lo+(f.hi-f.lo)*i/1000.f; uint8_t d[2]={0,0};
      obd::codificar(f.pid,v,d); float r=obd::decodificar(f.pid,d); float e=fabsf(r-v); if(e>maxerr)maxerr=e; casos++;
      if(e>f.tol+1e-3f){falhas++;} }
    printf("PID 0x%02X  erro max = %.5f  (tolerancia = %.5f)\n",f.pid,maxerr,f.tol);}
  // fluxo completo da ECU emulada
  uint8_t req[8], resp[8]; obd::montarRequisicao(0x01,0x0C,req);
  bool ok=obd::responderComoECU(req,45.0f,false,resp) && obd::respostaValida(0x7E8,resp,0x0C);
  printf("Resposta RPM emulada: %02X %02X %02X %02X %02X -> %.2f rpm (esperado %.2f)\n",resp[0],resp[1],resp[2],resp[3],resp[4],
         obd::decodificar(0x0C,&resp[3]), obd::valorEmulado(0x0C,45.0f));
  obd::montarRequisicao(0x03,0,req); obd::responderComoECU(req,45,true,resp); char s[6]; obd::dtcTexto(resp[3],resp[4],s);
  printf("DTC emulado: %s\n",s); if(strcmp(s,"P0300")) falhas++;
  printf("%d casos, %d falhas, fluxo ECU %s\n",casos,falhas,ok?"OK":"FALHOU");
  return falhas||!ok;
}
