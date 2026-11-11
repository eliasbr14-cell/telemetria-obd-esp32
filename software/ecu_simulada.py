"""
ecu_simulada.py — Emulador de ECU (injeção eletrônica) com interface OBD-II sobre CAN.

Responde a requisições de diagnóstico conforme ISO 15765-4 (CAN 11 bits, 500 kbit/s):
  - Requisição funcional  : ID 0x7DF   |  Requisição física: ID 0x7E0
  - Resposta da ECU motor : ID 0x7E8
  - Modo 01 (dados atuais), Modo 03 (DTCs armazenados), Modo 04 (apagar DTCs)

Os valores físicos são gerados por um modelo simplificado de veículo compacto
1.6 aspirado, percorrendo um ciclo de condução (marcha lenta -> aceleração ->
cruzeiro -> desaceleração). O modelo NÃO pretende reproduzir com exatidão o
comportamento do VW Polo; ele serve para gerar sinais fisicamente coerentes
e conhecidos (valor de referência) para validar a cadeia de telemetria.
"""
import math
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import can

from obd_pids import PIDS, dtc_para_bytes, mascara_pids_suportados

ID_REQ_FUNCIONAL = 0x7DF
ID_REQ_FISICA = 0x7E0
ID_RESP_ECU = 0x7E8

# Relações de transmissão representativas de um hatch compacto 1.6 (5 marchas)
RELACOES = [3.45, 1.94, 1.29, 0.97, 0.80]
DIFERENCIAL = 4.2
RAIO_RODA_M = 0.29        # pneu 185/60 R14 aprox.


def bits_quadro_can(n_dados: int) -> int:
    """Número máximo de bits de um quadro CAN 2.0A (11 bits) com bit stuffing
    no pior caso. Base: 47 bits de overhead + 8*n; stuffing pior caso sobre os
    34 + 8n bits sujeitos a stuffing: floor((34+8n-1)/4)."""
    return 47 + 8 * n_dados + (34 + 8 * n_dados - 1) // 4


def tempo_quadro_s(n_dados: int, bitrate: int = 500_000) -> float:
    return bits_quadro_can(n_dados) / bitrate


@dataclass
class EstadoVeiculo:
    rpm: float = 850.0
    velocidade: float = 0.0
    temp_arrefecimento: float = 25.0
    temp_admissao: float = 28.0
    map: float = 35.0
    carga_motor: float = 20.0
    borboleta: float = 0.0
    stft_b1: float = 0.0
    nivel_combustivel: float = 62.0
    tensao_modulo: float = 14.1
    dtcs: List[str] = field(default_factory=list)


class ModeloVeiculo:
    """Gera um ciclo de condução periódico (período = duracao_ciclo_s)."""

    def __init__(self, duracao_ciclo_s: float = 120.0, semente: int = 42):
        self.T = duracao_ciclo_s
        self.rng = random.Random(semente)
        self.t0 = time.perf_counter()
        self.estado = EstadoVeiculo()
        self._ultimo = self.t0
        self.trava = threading.Lock()

    def _perfil_velocidade(self, tc: float) -> (float, float):
        """Retorna (velocidade alvo km/h, borboleta %) para o instante tc do ciclo."""
        f = tc / self.T
        if f < 0.12:                     # marcha lenta
            return 0.0, 0.0
        if f < 0.40:                     # aceleração até 80 km/h
            x = (f - 0.12) / 0.28
            return 80 * (1 - math.exp(-3 * x)) / (1 - math.exp(-3)), 35 + 30 * (1 - x)
        if f < 0.70:                     # cruzeiro 80 km/h com pequenas variações
            return 80 + 4 * math.sin(2 * math.pi * (f - 0.40) * 6), 18.0
        if f < 0.90:                     # desaceleração (freio motor)
            x = (f - 0.70) / 0.20
            return 80 * (1 - x), 0.0
        return 0.0, 0.0                  # parado novamente

    def atualizar(self):
        with self.trava:
            agora = time.perf_counter()
            dt = agora - self._ultimo
            self._ultimo = agora
            t = agora - self.t0
            tc = t % self.T
            s = self.estado

            v_alvo, borb = self._perfil_velocidade(tc)
            s.velocidade = max(0.0, v_alvo + self.rng.gauss(0, 0.3))
            s.borboleta = max(0.0, min(100.0, borb + self.rng.gauss(0, 0.8))) if borb > 0 else 0.0

            # Marcha escolhida pela velocidade
            if s.velocidade < 3:
                rpm = 850 + self.rng.gauss(0, 15)
            else:
                limites = [20, 38, 55, 72, 999]
                marcha = next(i for i, lim in enumerate(limites) if s.velocidade < lim)
                v_ms = s.velocidade / 3.6
                rpm = v_ms / (2 * math.pi * RAIO_RODA_M) * 60 * RELACOES[marcha] * DIFERENCIAL
                rpm = max(rpm, 900) + self.rng.gauss(0, 10)
            s.rpm = max(0.0, min(6500.0, rpm))

            # Carga e MAP coerentes com a borboleta (motor aspirado: MAP <= ~100 kPa)
            s.carga_motor = max(12.0, min(100.0, 15 + 0.95 * s.borboleta + self.rng.gauss(0, 1)))
            s.map = max(20.0, min(101.0, 28 + 0.72 * s.borboleta + self.rng.gauss(0, 0.8)))

            # Aquecimento do motor: aproximação exponencial a 90 °C (termostato)
            tau = 240.0
            s.temp_arrefecimento += (90.0 - s.temp_arrefecimento) * (1 - math.exp(-dt / tau))
            s.temp_admissao = 28 + 6 * (1 - math.exp(-t / 300)) + self.rng.gauss(0, 0.2)

            # Ajuste de mistura oscilando em torno de 0 (malha fechada da sonda lambda)
            s.stft_b1 = 3 * math.sin(2 * math.pi * t / 2.5) + self.rng.gauss(0, 0.6)

            # Consumo: ~0,02 %/s com carga média (acelerado para ser visível)
            s.nivel_combustivel = max(0.0, s.nivel_combustivel - dt * (0.002 + 0.0002 * s.carga_motor))

            s.tensao_modulo = (13.9 if s.rpm > 700 else 12.4) + 0.15 * math.sin(t / 7) + self.rng.gauss(0, 0.02)

    def valor(self, chave: str) -> float:
        with self.trava:
            return getattr(self.estado, chave)

    def injetar_dtc(self, codigo: str):
        with self.trava:
            if codigo not in self.estado.dtcs:
                self.estado.dtcs.append(codigo)

    def apagar_dtcs(self):
        with self.trava:
            self.estado.dtcs.clear()


class ECUSimulada(threading.Thread):
    """Nó CAN que escuta requisições OBD-II e responde como a ECU do motor."""

    def __init__(self, bus: can.BusABC, modelo: ModeloVeiculo,
                 atraso_ms: float = 10.0, jitter_ms: float = 3.0,
                 simular_tempo_barramento: bool = True, semente: int = 7):
        super().__init__(daemon=True, name="ECUSimulada")
        self.bus = bus
        self.modelo = modelo
        self.atraso_ms = atraso_ms
        self.jitter_ms = jitter_ms
        self.simular_tempo_barramento = simular_tempo_barramento
        self.rng = random.Random(semente)
        self._parar = threading.Event()
        # Registro de verdade de referência: (t_envio, pid, valor_fisico_real, bytes)
        self.registro_verdade: List[tuple] = []
        self.respostas_enviadas = 0

    def parar(self):
        self._parar.set()

    def _responder(self, dados: List[int]):
        payload = [len(dados)] + dados
        payload += [0x55] * (8 - len(payload))          # padding (ISO 15765-4)
        if self.simular_tempo_barramento:
            time.sleep(tempo_quadro_s(8))
        self.bus.send(can.Message(arbitration_id=ID_RESP_ECU, data=payload, is_extended_id=False))
        self.respostas_enviadas += 1

    def run(self):
        while not self._parar.is_set():
            msg = self.bus.recv(timeout=0.05)
            self.modelo.atualizar()
            if msg is None or msg.arbitration_id not in (ID_REQ_FUNCIONAL, ID_REQ_FISICA):
                continue
            if self.simular_tempo_barramento:
                time.sleep(tempo_quadro_s(8))  # tempo da requisição trafegando no barramento
            n, modo = msg.data[0], msg.data[1]
            # Tempo de processamento da ECU (parâmetro de simulação)
            atraso = max(0.0, self.atraso_ms + self.rng.uniform(-self.jitter_ms, self.jitter_ms)) / 1000
            time.sleep(atraso)

            if modo == 0x01 and n >= 2:
                pid = msg.data[2]
                if pid in (0x00, 0x20, 0x40):
                    self._responder([0x41, pid] + mascara_pids_suportados(pid))
                elif pid == 0x01:
                    dtcs = self.modelo.estado.dtcs
                    a = (0x80 if dtcs else 0x00) | min(len(dtcs), 0x7F)
                    self._responder([0x41, 0x01, a, 0x07, 0x65, 0x00])
                elif pid in PIDS:
                    p = PIDS[pid]
                    valor_real = self.modelo.valor(p.chave)
                    b = p.codificar(valor_real)
                    self.registro_verdade.append((time.perf_counter(), pid, valor_real, tuple(b)))
                    self._responder([0x41, pid] + b)
                # PID não suportado: a ECU não responde (comportamento OBD-II padrão)
            elif modo == 0x03:
                dtcs = list(self.modelo.estado.dtcs)[:2]   # quadro único: até 2 DTCs
                corpo = [0x43, len(dtcs)]
                for d in dtcs:
                    corpo += dtc_para_bytes(d)
                self._responder(corpo)
            elif modo == 0x04:
                self.modelo.apagar_dtcs()
                self._responder([0x44])
