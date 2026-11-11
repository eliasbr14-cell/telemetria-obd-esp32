"""
leitor_obd.py — Leitor/decodificador OBD-II + publicador MQTT.

Reproduz em Python a mesma lógica do firmware do ESP32 (pasta firmware/):
  1. envia requisição Modo 01 para cada PID monitorado (ID 0x7DF);
  2. aguarda a resposta da ECU (IDs 0x7E8–0x7EF), validando modo e PID;
  3. decodifica os bytes para grandeza física (fórmulas SAE J1979);
  4. a cada ciclo completo, publica um JSON no tópico MQTT de telemetria;
  5. periodicamente consulta DTCs (Modo 03) e publica no tópico de diagnóstico.

Pode ser usado com qualquer interface do python-can:
  - virtual   (simulação em um único processo)
  - socketcan (Linux, ex.: vcan0 ou adaptador USB-CAN real)
  - slcan     (adaptadores seriais USB-CAN)
"""
import json
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import can

from obd_pids import PIDS, PIDS_MONITORADOS, bytes_para_dtc

ID_REQ_FUNCIONAL = 0x7DF


@dataclass
class Leitura:
    pid: int
    valor: Optional[float]
    bytes_brutos: tuple
    t_req: float
    t_resp: Optional[float]

    @property
    def rtt_ms(self):
        return None if self.t_resp is None else (self.t_resp - self.t_req) * 1000


@dataclass
class Estatisticas:
    requisicoes: int = 0
    respostas: int = 0
    timeouts: int = 0
    respostas_invalidas: int = 0
    leituras: List[Leitura] = field(default_factory=list)


class LeitorOBD:
    def __init__(self, bus: can.BusABC, timeout_s: float = 0.1):
        self.bus = bus
        self.timeout_s = timeout_s
        self.stats = Estatisticas()

    def _enviar(self, dados: List[int]):
        payload = [len(dados)] + dados
        payload += [0x00] * (8 - len(payload))
        self.bus.send(can.Message(arbitration_id=ID_REQ_FUNCIONAL, data=payload, is_extended_id=False))

    def _aguardar(self, modo_resp: int, pid: Optional[int]) -> Optional[can.Message]:
        limite = time.perf_counter() + self.timeout_s
        while True:
            restante = limite - time.perf_counter()
            if restante <= 0:
                return None
            msg = self.bus.recv(timeout=restante)
            if msg is None:
                return None
            if not (0x7E8 <= msg.arbitration_id <= 0x7EF):
                continue                     # tráfego que não é resposta de diagnóstico
            d = msg.data
            if d[1] != modo_resp or (pid is not None and d[2] != pid):
                self.stats.respostas_invalidas += 1
                continue
            return msg

    def ler_pid(self, pid: int) -> Leitura:
        p = PIDS[pid]
        self.stats.requisicoes += 1
        t_req = time.perf_counter()
        self._enviar([0x01, pid])
        msg = self._aguardar(0x41, pid)
        if msg is None:
            self.stats.timeouts += 1
            leitura = Leitura(pid, None, (), t_req, None)
        else:
            t_resp = time.perf_counter()
            self.stats.respostas += 1
            brutos = tuple(msg.data[3:3 + p.n_bytes])
            leitura = Leitura(pid, p.decodificar(list(brutos)), brutos, t_req, t_resp)
        self.stats.leituras.append(leitura)
        return leitura

    def ler_dtcs(self) -> Optional[List[str]]:
        self._enviar([0x03])
        msg = self._aguardar(0x43, None)
        if msg is None:
            return None
        n = msg.data[2]
        return [bytes_para_dtc(msg.data[3 + 2 * i], msg.data[4 + 2 * i]) for i in range(n)]

    def apagar_dtcs(self) -> bool:
        self._enviar([0x04])
        return self._aguardar(0x44, None) is not None


def executar_ciclos(leitor: LeitorOBD, cliente_mqtt, topico_base: str,
                    duracao_s: float, periodo_dtc_s: float = 2.0,
                    pids: List[int] = PIDS_MONITORADOS, ao_publicar=None):
    """Laço principal: varre os PIDs, publica JSON por ciclo e consulta DTCs."""
    seq = 0
    t_fim = time.perf_counter() + duracao_s
    t_ultimo_dtc = 0.0
    while time.perf_counter() < t_fim:
        t_ini_ciclo = time.perf_counter()
        dados: Dict[str, Optional[float]] = {}
        for pid in pids:
            l = leitor.ler_pid(pid)
            dados[PIDS[pid].chave] = None if l.valor is None else round(l.valor, 3)
        seq += 1
        pacote = {
            "seq": seq,
            "ts": time.time(),                       # instante de publicação (epoch, s)
            "ciclo_ms": round((time.perf_counter() - t_ini_ciclo) * 1000, 3),
            "dados": dados,
        }
        cliente_mqtt.publish(f"{topico_base}/dados", json.dumps(pacote), qos=0)
        if ao_publicar:
            ao_publicar(pacote)

        if time.perf_counter() - t_ultimo_dtc >= periodo_dtc_s:
            t_ultimo_dtc = time.perf_counter()
            dtcs = leitor.ler_dtcs()
            if dtcs is not None:
                cliente_mqtt.publish(f"{topico_base}/dtc",
                                     json.dumps({"ts": time.time(), "dtcs": dtcs}), qos=1)
    return seq
