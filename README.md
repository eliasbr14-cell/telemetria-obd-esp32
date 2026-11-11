# Telemetria e diagnóstico veicular de baixo custo com ESP32 e protocolo CAN

Código-fonte e dados do Trabalho de Conclusão de Curso **"Telemetria e diagnóstico veicular de baixo custo com
ESP32 e protocolo CAN: validação em ambiente simulado e em bancada"** — Engenharia da Computação,
Centro Universitário IBMR (2026).

Autores: Elias Borges Viana e Arthur Silva de Cerqueira · Orientador: Prof. Francisco José Rodrigues da Silva Junior

O sistema lê parâmetros OBD-II (Modo 01) e códigos de falha (Modo 03) pelo barramento CAN (ISO 15765-4,
500 kbit/s), publica os dados via MQTT, grava em SQLite e exibe em um dashboard web.

## Estrutura

```
firmware/telemetria_esp32/   firmware Arduino do ESP32 (controlador TWAI + SN65HVD230), modos AUTOTESTE e VEÍCULO
firmware/teste_host/         teste do núcleo OBD do firmware no computador (g++)
software/                    ECU simulada, leitor OBD, servidor (MQTT + SQLite + web), dashboard e scripts de ensaio
resultados/simulacao/        dados brutos dos ensaios em ambiente simulado (E1, E2, E3 e E4)
resultados/bancada_esp32/    dados do ensaio do firmware no ESP32 em bancada (janela de 5 min e ensaio de 60 min)
figuras/                     figuras usadas no TCC
figuras_src/                 fontes HTML/SVG dos diagramas
```

## Ensaios e arquivos de dados

| Ensaio | Onde roda | Script | Arquivos |
|---|---|---|---|
| E1 – exatidão, tempos e latência (240 s) | simulação (python-can) | `software/experimentos.py` | `resultados/simulacao/e1_*.csv`, `resultados.json` |
| E2 – sensibilidade ao tempo de resposta da ECU | simulação | `software/experimentos.py` | `resultados.json` (chave `E2`) |
| E3 – escalabilidade pelo nº de PIDs | simulação | `software/experimentos.py` | `resultados.json` (chave `E3`) |
| E4 – tempo de detecção de falha (30 repetições) | simulação | `software/ensaio_dtc.py` | `resultados/simulacao/e4_dtc.csv` |
| E5 – firmware no ESP32 em bancada (autoteste) | ESP32 + SN65HVD230 + Wi-Fi | `firmware/telemetria_esp32` + `software/servidor.py` | `resultados/bancada_esp32/*.csv`, `esp32_resumo.json` |

Os gráficos são gerados com `python software/graficos.py`.

## 1. Simulação completa no computador (sem ESP32)

1. Instale Python 3.10+ e o broker [Mosquitto](https://mosquitto.org/download/).
2. `pip install -r software/requirements.txt`
3. Dentro de `software/`, em terminais separados:
   ```
   mosquitto -v
   python servidor.py                                  # dashboard em http://localhost:5000
   python simulacao.py --dtc P0300 --dtc-em 30
   ```
4. Para repetir os ensaios: `python experimentos.py` (≈ 8 min), `python ensaio_dtc.py` (≈ 5 min) e `python graficos.py`.

## 2. ESP32 em modo AUTOTESTE (bancada, sem veículo)

1. Arduino IDE com o núcleo **esp32 by Espressif** (testado na versão 3.3.12) e a biblioteca **PubSubClient** 2.8.
2. Ligações: GPIO5 → CTX, GPIO4 → CRX, 3V3 → VCC, GND → GND do SN65HVD230.
3. Em `telemetria_esp32.ino`: `#define MODO_AUTOTESTE 1`, preencha `WIFI_SSID`, `WIFI_SENHA` (rede 2,4 GHz) e
   `MQTT_BROKER` (IP do computador com o Mosquitto). No Mosquitto 2.x, para aceitar conexões da rede, inclua
   `listener 1883` e `allow_anonymous true` no `mosquitto.conf`.
4. Grave a placa, abra o Monitor Serial (115200 baud) e rode `python servidor.py`: os dados aparecem no mesmo dashboard.

## 3. Uso em veículo (modo VEÍCULO)

`#define MODO_AUTOTESTE 0`; CANH → pino 6, CANL → pino 14 e GND → pinos 4/5 do conector OBD-II. O veículo precisa
usar diagnóstico sobre CAN (ISO 15765-4, 500 kbit/s, 11 bits). Esse modo foi implementado e compilado, mas não foi
validado em veículo neste trabalho.

## Licença

MIT — veja `LICENSE`.
