/*
 * telemetria_esp32.ino — Telemetria veicular OBD-II via CAN (TWAI) + MQTT
 * TCC Engenharia da Computação — Centro Universitário IBMR
 *
 * Hardware: ESP32 + transceptor SN65HVD230
 *   ESP32 GPIO5 (TX) -> CTX/TXD do SN65HVD230
 *   ESP32 GPIO4 (RX) -> CRX/RXD do SN65HVD230
 *   3V3 -> VCC   |   GND -> GND (e GND do conector OBD, pinos 4/5)
 *   CANH -> pino 6 do OBD-II   |   CANL -> pino 14 do OBD-II
 *
 * MODOS DE OPERAÇÃO (defina MODO_AUTOTESTE abaixo):
 *   1 = AUTOTESTE: não precisa do carro. O controlador TWAI opera em modo
 *       NO_ACK com auto-recepção: cada quadro transmitido volta pelo RX.
 *       Uma ECU emulada no próprio firmware responde às requisições, e o
 *       quadro de resposta percorre o mesmo caminho (TWAI -> transceptor ->
 *       TWAI) antes de ser decodificado. Valida firmware, driver e ligação
 *       ESP32 <-> SN65HVD230. (Sem o transceptor, ligue GPIO5 ao GPIO4.)
 *   0 = VEÍCULO: modo normal, requisições reais à ECU (filtro 0x7E8–0x7EF).
 *
 * Bibliotecas: núcleo ESP32 para Arduino (2.x ou 3.x) e PubSubClient (Nick O'Leary).
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <sys/time.h>
#include "driver/twai.h"
#include "obd.h"

// ======================== CONFIGURAÇÃO ========================
#define MODO_AUTOTESTE 1

const char* WIFI_SSID     = "SUA_REDE";
const char* WIFI_SENHA    = "SUA_SENHA";
const char* MQTT_BROKER   = "192.168.0.10";   // IP do PC que roda o Mosquitto
const int   MQTT_PORTA    = 1883;
const char* TOPICO_BASE   = "telemetria/polo";

#define PINO_TX GPIO_NUM_5
#define PINO_RX GPIO_NUM_4
const uint32_t TIMEOUT_RESPOSTA_MS = 100;    // espera máx. pela ECU por PID
const uint32_t PERIODO_DTC_MS      = 2000;   // consulta de falhas (Modo 03)
const uint32_t FALHA_EMULADA_EM_MS = 30000;  // autoteste: P0300 aparece após 30 s
// ==============================================================

WiFiClient wifi;
PubSubClient mqtt(wifi);
uint32_t seq = 0, ultimoDtc = 0;
uint32_t nReq = 0, nResp = 0, nTimeout = 0;

static double agoraEpoch() {                 // segundos (com fração) via NTP
  struct timeval tv; gettimeofday(&tv, nullptr);
  return tv.tv_sec + tv.tv_usec / 1e6;
}

bool iniciarCAN() {
#if MODO_AUTOTESTE
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PINO_TX, PINO_RX, TWAI_MODE_NO_ACK);
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#else
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PINO_TX, PINO_RX, TWAI_MODE_NORMAL);
  // Aceita apenas IDs 0x7E8–0x7EF (respostas de diagnóstico). Filtro de 11 bits
  // fica nos bits 31..21 do registrador; bits '1' na máscara = "não importa".
  twai_filter_config_t  f = { .acceptance_code = (0x7E8u << 21),
                              .acceptance_mask = ~(0x7F8u << 21),
                              .single_filter   = true };
#endif
  g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) { Serial.println("[CAN] falha ao instalar driver"); return false; }
  if (twai_start() != ESP_OK)                    { Serial.println("[CAN] falha ao iniciar");        return false; }
  Serial.printf("[CAN] TWAI 500 kbit/s iniciado em modo %s\n", MODO_AUTOTESTE ? "AUTOTESTE" : "VEICULO");
  return true;
}

void transmitir(uint32_t id, const uint8_t* dados) {
  twai_message_t m; memset(&m, 0, sizeof(m));
  m.identifier = id;
  m.data_length_code = 8;
  memcpy(m.data, dados, 8);
#if MODO_AUTOTESTE
  m.self = 1;                                // auto-recepção (self reception request)
#endif
  twai_transmit(&m, pdMS_TO_TICKS(50));
}

// Envia uma requisição e aguarda a resposta correspondente.
// No AUTOTESTE, a própria função faz o papel da ECU quando "ouve" a requisição.
bool consultar(uint8_t modo, uint8_t pid, uint8_t* resp) {
  uint8_t req[8]; obd::montarRequisicao(modo, pid, req);
  transmitir(obd::ID_REQ_FUNCIONAL, req);
  nReq++;
  uint32_t limite = millis() + TIMEOUT_RESPOSTA_MS;
  while ((int32_t)(limite - millis()) > 0) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) != ESP_OK) continue;
#if MODO_AUTOTESTE
    if (rx.identifier == obd::ID_REQ_FUNCIONAL) {          // ECU emulada "ouviu" a requisição
      uint8_t r[8];
      bool falha = millis() > FALHA_EMULADA_EM_MS;
      if (obd::responderComoECU(rx.data, millis() / 1000.0f, falha, r)) transmitir(obd::ID_RESP_ECU, r);
      continue;
    }
#endif
    if (rx.identifier < 0x7E8 || rx.identifier > 0x7EF) continue;
    if (rx.data[1] != (uint8_t)(modo + 0x40)) continue;
    if (modo == 0x01 && rx.data[2] != pid) continue;
    memcpy(resp, rx.data, 8);
    nResp++;
    return true;
  }
  nTimeout++;
  return false;
}

void verificarBarramento() {
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) {
    Serial.println("[CAN] BUS-OFF detectado, iniciando recuperação");
    twai_initiate_recovery();
    delay(100);
    twai_start();
  }
}

void conectarRede() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_SENHA);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) delay(250);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] conectado, IP %s\n", WiFi.localIP().toString().c_str());
      configTime(0, 0, "pool.ntp.org", "time.google.com");   // relógio p/ cálculo de latência
    }
  }
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
    if (mqtt.connect("esp32-telemetria")) Serial.println("[MQTT] conectado ao broker");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Telemetria OBD-II | ESP32 + SN65HVD230 ===");
  iniciarCAN();
  mqtt.setServer(MQTT_BROKER, MQTT_PORTA);
  mqtt.setBufferSize(640);
  conectarRede();
}

void loop() {
  verificarBarramento();
  conectarRede();
  mqtt.loop();

  const uint32_t t0 = micros();
  char json[600];
  int n = snprintf(json, sizeof(json), "{\"seq\":%lu,\"dados\":{", (unsigned long)++seq);
  for (int i = 0; i < obd::N_PIDS; i++) {
    const obd::DefPID& p = obd::PIDS[i];
    uint8_t r[8];
    if (consultar(0x01, p.pid, r)) {
      float v = obd::decodificar(p.pid, &r[3]);
      n += snprintf(json + n, sizeof(json) - n, "%s\"%s\":%.3f", i ? "," : "", p.chave, v);
    } else {
      n += snprintf(json + n, sizeof(json) - n, "%s\"%s\":null", i ? "," : "", p.chave);
    }
  }
  const float cicloMs = (micros() - t0) / 1000.0f;
  n += snprintf(json + n, sizeof(json) - n, "},\"ciclo_ms\":%.3f,\"ts\":%.3f}", cicloMs, agoraEpoch());

  Serial.println(json);
  if (mqtt.connected()) {
    char topico[64]; snprintf(topico, sizeof(topico), "%s/dados", TOPICO_BASE);
    mqtt.publish(topico, json);
  }

  // Diagnóstico: leitura de DTCs (Modo 03)
  if (millis() - ultimoDtc >= PERIODO_DTC_MS) {
    ultimoDtc = millis();
    uint8_t r[8];
    if (consultar(0x03, 0, r)) {
      char dj[160]; int m = snprintf(dj, sizeof(dj), "{\"ts\":%.3f,\"dtcs\":[", agoraEpoch());
      const int qtd = r[2] > 2 ? 2 : r[2];
      for (int i = 0; i < qtd; i++) {
        char c[6]; obd::dtcTexto(r[3 + 2 * i], r[4 + 2 * i], c);
        m += snprintf(dj + m, sizeof(dj) - m, "%s\"%s\"", i ? "," : "", c);
      }
      snprintf(dj + m, sizeof(dj) - m, "]}");
      Serial.println(dj);
      if (mqtt.connected()) {
        char topico[64]; snprintf(topico, sizeof(topico), "%s/dtc", TOPICO_BASE);
        mqtt.publish(topico, dj);
      }
    }
    Serial.printf("[STATS] req=%lu resp=%lu timeout=%lu\n",
                  (unsigned long)nReq, (unsigned long)nResp, (unsigned long)nTimeout);
  }
}
