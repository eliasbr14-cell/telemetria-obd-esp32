"""
ensaio_dtc.py — Ensaio E4: tempo de detecção de falha (DTC) em instantes variados.

Repete N vezes: inicia ECU simulada + leitor (mesmas condições do E1: 10 PIDs,
ECU com 10 ms ± 3 ms, consulta do Modo 03 a cada 2 s), injeta a falha P0300 em
um instante aleatório e mede o tempo até o leitor decodificar o código na
resposta do Modo 03. A medição é feita no leitor (antes do MQTT).
Saída: ../resultados/simulacao/e4_dtc.csv e resumo no terminal.
"""
import csv
import os
import random
import statistics as st
import threading
import time

import can

from ecu_simulada import ECUSimulada, ModeloVeiculo
from leitor_obd import LeitorOBD
from obd_pids import PIDS_MONITORADOS

SAIDA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resultados", "simulacao")
N = int(os.environ.get("N_REP", 30))
PERIODO_DTC = 2.0


def uma_execucao(k, rng):
    canal = f"e4_{k}"
    bus_e = can.Bus(interface="virtual", channel=canal, receive_own_messages=False)
    bus_l = can.Bus(interface="virtual", channel=canal, receive_own_messages=False)
    modelo = ModeloVeiculo(semente=42 + k)
    ecu = ECUSimulada(bus_e, modelo, atraso_ms=10.0, jitter_ms=3.0, semente=7 + k)
    ecu.start()
    leitor = LeitorOBD(bus_l)
    t_ini = time.perf_counter()
    atraso_inj = rng.uniform(3.0, 7.0)          # instante aleatório da falha
    marca = {}

    def injetar():
        marca["t"] = time.perf_counter()
        modelo.injetar_dtc("P0300")
    threading.Timer(atraso_inj, injetar).start()

    t_ult_dtc = 0.0
    deteccao = None
    while time.perf_counter() - t_ini < 15 and deteccao is None:
        for pid in PIDS_MONITORADOS:
            leitor.ler_pid(pid)
        if time.perf_counter() - t_ult_dtc >= PERIODO_DTC:
            t_ult_dtc = time.perf_counter()
            dtcs = leitor.ler_dtcs()
            if dtcs and "P0300" in dtcs and "t" in marca:
                deteccao = time.perf_counter() - marca["t"]
    ecu.parar(); time.sleep(0.1)
    bus_e.shutdown(); bus_l.shutdown()
    return atraso_inj, deteccao


def main():
    rng = random.Random(2026)
    linhas = []
    for k in range(1, N + 1):
        inj, det = uma_execucao(k, rng)
        linhas.append((k, inj, det))
        print(f"execução {k:2d}: falha aos {inj:5.2f} s -> detectada em {det:.3f} s", flush=True)
    with open(os.path.join(SAIDA, "e4_dtc.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["execucao", "instante_injecao_s", "tempo_deteccao_s"])
        for k, inj, det in linhas:
            w.writerow([k, f"{inj:.4f}", f"{det:.4f}"])
    d = [x[2] for x in linhas]
    print(f"N={len(d)} média={st.mean(d):.3f} s desvio={st.stdev(d):.3f} s "
          f"mín={min(d):.3f} s mediana={st.median(d):.3f} s máx={max(d):.3f} s")


if __name__ == "__main__":
    main()
