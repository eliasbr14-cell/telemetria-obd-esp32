// obd.h — Núcleo OBD-II independente de hardware (decodificação SAE J1979,
// montagem de quadros ISO 15765-4 e ECU emulada para o modo autoteste).
// Este arquivo não depende do ESP32: é compilado também no PC para testes.
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

namespace obd {

const uint32_t ID_REQ_FUNCIONAL = 0x7DF;
const uint32_t ID_RESP_ECU      = 0x7E8;

struct DefPID {
  uint8_t     pid;
  const char* chave;
  uint8_t     nBytes;
};

// Mesma ordem de varredura usada no software de simulação
static const DefPID PIDS[] = {
  {0x0C, "rpm", 2},               {0x0D, "velocidade", 1},
  {0x05, "temp_arrefecimento", 1},{0x0B, "map", 1},
  {0x04, "carga_motor", 1},       {0x11, "borboleta", 1},
  {0x06, "stft_b1", 1},           {0x0F, "temp_admissao", 1},
  {0x2F, "nivel_combustivel", 1}, {0x42, "tensao_modulo", 2},
};
static const int N_PIDS = sizeof(PIDS) / sizeof(PIDS[0]);

// Converte os bytes A,B da resposta em grandeza física (SAE J1979)
inline float decodificar(uint8_t pid, const uint8_t* d) {
  const float A = d[0], B = d[1];
  switch (pid) {
    case 0x04: case 0x11: case 0x2F: return A * 100.0f / 255.0f;   // %
    case 0x05: case 0x0F:            return A - 40.0f;               // °C
    case 0x06:                       return (A - 128.0f) * 100.0f / 128.0f; // %
    case 0x0B: case 0x0D:            return A;                       // kPa, km/h
    case 0x0C:                       return (256.0f * A + B) / 4.0f; // rpm
    case 0x42:                       return (256.0f * A + B) / 1000.0f; // V
  }
  return NAN;
}

// Operação inversa — usada pela ECU emulada
inline int codificar(uint8_t pid, float v, uint8_t* out) {
  auto u8 = [](float x) -> uint8_t { long r = lroundf(x); return (uint8_t)(r < 0 ? 0 : r > 255 ? 255 : r); };
  auto u16 = [&](float x, uint8_t* o) { long r = lroundf(x); if (r < 0) r = 0; if (r > 65535) r = 65535;
                                        o[0] = (r >> 8) & 0xFF; o[1] = r & 0xFF; };
  switch (pid) {
    case 0x04: case 0x11: case 0x2F: out[0] = u8(v * 255.0f / 100.0f); return 1;
    case 0x05: case 0x0F:            out[0] = u8(v + 40.0f); return 1;
    case 0x06:                       out[0] = u8(v * 128.0f / 100.0f + 128.0f); return 1;
    case 0x0B: case 0x0D:            out[0] = u8(v); return 1;
    case 0x0C:                       u16(v * 4.0f, out); return 2;
    case 0x42:                       u16(v * 1000.0f, out); return 2;
  }
  return 0;
}

// Monta o payload de 8 bytes de uma requisição Modo 01 (quadro único ISO-TP)
inline void montarRequisicao(uint8_t modo, uint8_t pid, uint8_t* p) {
  memset(p, 0, 8);
  if (modo == 0x01) { p[0] = 0x02; p[1] = 0x01; p[2] = pid; }
  else              { p[0] = 0x01; p[1] = modo; }
}

// Valida a resposta: modo+0x40 e PID correspondentes
inline bool respostaValida(uint32_t id, const uint8_t* d, uint8_t pid) {
  return id >= 0x7E8 && id <= 0x7EF && d[1] == 0x41 && d[2] == pid;
}

// DTC de 2 bytes -> texto ("P0300")
inline void dtcTexto(uint8_t b0, uint8_t b1, char* s) {
  const char letras[] = "PCBU";
  snprintf(s, 6, "%c%d%X%02X", letras[(b0 >> 6) & 3], (b0 >> 4) & 3, b0 & 0x0F, b1);
}

// ---------------------------------------------------------------------------
// ECU emulada (somente modo AUTOTESTE): valores sintéticos variando no tempo
// ---------------------------------------------------------------------------
inline float valorEmulado(uint8_t pid, float t) {
  const float ciclo = fmodf(t, 60.0f) / 60.0f;              // ciclo de 60 s
  const float vel = ciclo < 0.2f ? 0 : ciclo < 0.5f ? (ciclo - 0.2f) / 0.3f * 80.0f
                  : ciclo < 0.8f ? 80.0f : (1.0f - ciclo) / 0.2f * 80.0f;
  switch (pid) {
    case 0x0C: return vel < 1 ? 850.0f : 1200.0f + vel * 30.0f;
    case 0x0D: return vel;
    case 0x05: return 90.0f - 65.0f * expf(-t / 240.0f);
    case 0x0B: return vel < 1 ? 32.0f : 45.0f + 0.4f * vel;
    case 0x04: return vel < 1 ? 18.0f : 30.0f + 0.3f * vel;
    case 0x11: return vel < 1 ? 0.0f : 15.0f + 0.2f * vel;
    case 0x06: return 3.0f * sinf(t * 2.5f);
    case 0x0F: return 30.0f;
    case 0x2F: return 62.0f - t * 0.005f;
    case 0x42: return 14.0f + 0.1f * sinf(t / 7.0f);
  }
  return 0;
}

// Gera a resposta da ECU emulada para uma requisição recebida.
// Retorna true se houver resposta a enviar (ID 0x7E8, payload em 'resp').
inline bool responderComoECU(const uint8_t* req, float t, bool falhaAtiva, uint8_t* resp) {
  memset(resp, 0x55, 8);
  if (req[1] == 0x01) {
    uint8_t d[2] = {0, 0};
    int n = codificar(req[2], valorEmulado(req[2], t), d);
    if (n == 0) return false;               // PID não suportado: sem resposta
    resp[0] = 2 + n; resp[1] = 0x41; resp[2] = req[2]; resp[3] = d[0]; if (n == 2) resp[4] = d[1];
    return true;
  }
  if (req[1] == 0x03) {                     // DTCs: P0300 quando a falha está ativa
    if (falhaAtiva) { resp[0] = 4; resp[1] = 0x43; resp[2] = 1; resp[3] = 0x03; resp[4] = 0x00; }
    else            { resp[0] = 2; resp[1] = 0x43; resp[2] = 0; }
    return true;
  }
  return false;
}

} // namespace obd
