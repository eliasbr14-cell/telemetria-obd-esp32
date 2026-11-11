"""
experimentos.py — Ensaios de validação do sistema em ambiente simulado.

E1  Ensaio principal (2 ciclos de condução, 240 s): exatidão da decodificação,
    tempo de resposta CAN, tempo de ciclo, latência MQTT ponta a ponta,
    perda de mensagens e tempo de detecção de falha (DTC).
E2  Sensibilidade ao tempo de resposta da ECU (0, 10, 25 e 50 ms).
E3  Escalabilidade pelo número de PIDs monitorados (1, 3, 5 e 10).

Requer um broker MQTT em localhost:1883 (ex.: Mosquitto).
Saída: pasta ../resultados/simulacao (CSV + JSON com métricas brutas).
"""
import bisect
import csv
import json
import os
import platform
import statistics as st
import sys
import threading
import time

import can
import paho.mqtt.client as mqtt

from ecu_simulada import ECUSimulada, ModeloVeiculo, bits_quadro_can, tempo_quadro_s
from leitor_obd import LeitorOBD, executar_ciclos
from obd_pids import PIDS, PIDS_MONITORADOS
from servidor import Armazenamento, iniciar_assinante_mqtt

SAIDA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resultados", "simulacao")
os.makedirs(SAIDA, exist_ok=True)


def pct(v, p):
    v = sorted(v)
    k = (len(v) - 1) * p / 100
    f = int(k)
    return v[f] + (v[min(f + 1, len(v) - 1)] - v[f]) * (k - f)


def resumo(v):
    return {"n": len(v), "media": st.mean(v), "desvio": st.pstdev(v), "min": min(v),
            "p50": pct(v, 50), "p95": pct(v, 95), "p99": pct(v, 99), "max": max(v)}


def montar(canal, atraso_ms, jitter_ms, semente=42):
    bus_e = can.Bus(interface="virtual", channel=canal, receive_own_messages=False)
    bus_l = can.Bus(interface="virtual", channel=canal, receive_own_messages=False)
    modelo = ModeloVeiculo(semente=semente)
    ecu = ECUSimulada(bus_e, modelo, atraso_ms=atraso_ms, jitter_ms=jitter_ms)
    ecu.start()
    return bus_e, bus_l, modelo, ecu


def ensaio_principal(duracao=240.0, t_falha=100.0):
    print(f"[E1] ensaio principal: {duracao:.0f} s", flush=True)
    db = os.path.join(SAIDA, "e1_telemetria.db")
    if os.path.exists(db):
        os.remove(db)
    arm = Armazenamento(db)
    recebidos, dtc_rec = [], []

    def ao_receber(topico, p, ts_rec):
        if topico.endswith("/dados"):
            recebidos.append((p["seq"], p["ts"], ts_rec))
        else:
            dtc_rec.append((p["ts"], ts_rec, p["dtcs"]))

    sub = iniciar_assinante_mqtt("localhost", 1883, "tcc/e1", arm, [], ao_receber)
    time.sleep(1)
    bus_e, bus_l, modelo, ecu = montar("e1", 10.0, 3.0)

    t_inj = {}

    def injetar():
        t_inj["wall"] = time.time()
        modelo.injetar_dtc("P0300")
    threading.Timer(t_falha, injetar).start()

    pub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="e1-pub")
    pub.connect("localhost", 1883)
    pub.loop_start()
    leitor = LeitorOBD(bus_l)
    t0 = time.perf_counter()
    publicados = executar_ciclos(leitor, pub, "tcc/e1", duracao)
    time.sleep(2)
    ecu.parar(); pub.loop_stop(); sub.loop_stop()

    # ---- exatidão: casa cada leitura com o valor real enviado pela ECU
    verd = {}
    for t, pid, v, b in ecu.registro_verdade:
        verd.setdefault(pid, []).append((t, v, b))
    tempos = {pid: [x[0] for x in lst] for pid, lst in verd.items()}
    erros = {pid: [] for pid in PIDS_MONITORADOS}
    serie = []   # para gráficos
    for l in leitor.stats.leituras:
        if l.valor is None:
            continue
        i = bisect.bisect_right(tempos[l.pid], l.t_resp) - 1   # último valor enviado antes da recepção
        t_v, v_real, b = verd[l.pid][i]
        assert tuple(b) == l.bytes_brutos, "bytes recebidos diferem dos enviados"
        erros[l.pid].append(l.valor - v_real)
        serie.append((l.t_resp - t0, l.pid, v_real, l.valor))

    tab_exat = []
    for pid in PIDS_MONITORADOS:
        p = PIDS[pid]
        e = [abs(x) for x in erros[pid]]
        tab_exat.append({"pid": f"0x{pid:02X}", "grandeza": p.descricao, "unidade": p.unidade,
                         "amostras": len(e), "resolucao": p.resolucao,
                         "erro_medio_abs": st.mean(e), "erro_max_abs": max(e),
                         "limite_teorico": p.resolucao / 2,
                         "dentro_limite": all(x <= p.resolucao / 2 + 1e-9 for x in e)})

    rtt = [l.rtt_ms for l in leitor.stats.leituras if l.rtt_ms is not None]
    lat = [(r - p) * 1000 for _, p, r in recebidos]
    seqs = sorted(s for s, _, _ in recebidos)
    ciclos = [r[0] for r in arm.db.execute("SELECT ciclo_ms FROM leituras").fetchall()]

    deteccao = None
    for ts_pub, ts_rec, cods in dtc_rec:
        if "P0300" in cods and "wall" in t_inj:
            deteccao = (ts_rec - t_inj["wall"]) * 1000
            break

    res = {
        "duracao_s": duracao, "ciclos_publicados": publicados, "mensagens_recebidas": len(recebidos),
        "perdidas": publicados - len(set(seqs)), "duplicadas": len(seqs) - len(set(seqs)),
        "requisicoes": leitor.stats.requisicoes, "respostas": leitor.stats.respostas,
        "timeouts": leitor.stats.timeouts, "respostas_invalidas": leitor.stats.respostas_invalidas,
        "rtt_can_ms": resumo(rtt), "ciclo_ms": resumo(ciclos), "latencia_mqtt_ms": resumo(lat),
        "taxa_atualizacao_hz": publicados / duracao,
        "deteccao_dtc_ms": deteccao, "consultas_dtc": len(dtc_rec),
        "exatidao": tab_exat,
    }
    with open(os.path.join(SAIDA, "e1_serie.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_s", "pid", "valor_real", "valor_decodificado"])
        for t, pid, vr, vd in serie:
            w.writerow([f"{t:.4f}", f"0x{pid:02X}", f"{vr:.4f}", f"{vd:.4f}"])
    with open(os.path.join(SAIDA, "e1_rtt.csv"), "w") as f:
        f.write("rtt_ms\n" + "\n".join(f"{x:.4f}" for x in rtt))
    with open(os.path.join(SAIDA, "e1_latencia.csv"), "w") as f:
        f.write("latencia_ms\n" + "\n".join(f"{x:.4f}" for x in lat))
    bus_e.shutdown(); bus_l.shutdown()
    return res


class PubNulo:
    def publish(self, *a, **k):
        pass


def ensaio_curto(canal, atraso, jitter, pids, duracao):
    bus_e, bus_l, modelo, ecu = montar(canal, atraso, jitter)
    leitor = LeitorOBD(bus_l)
    n = executar_ciclos(leitor, PubNulo(), "x", duracao, periodo_dtc_s=1e9, pids=pids)
    ecu.parar(); time.sleep(0.1)
    rtt = [l.rtt_ms for l in leitor.stats.leituras if l.rtt_ms is not None]
    bus_e.shutdown(); bus_l.shutdown()
    quadros_s = 2 * leitor.stats.respostas / duracao
    carga = quadros_s * bits_quadro_can(8) / 500_000 * 100
    return {"atraso_ecu_ms": atraso, "n_pids": len(pids), "ciclos": n,
            "ciclo_medio_ms": duracao * 1000 / n, "taxa_por_pid_hz": n / duracao,
            "leituras_por_s": leitor.stats.respostas / duracao,
            "rtt_medio_ms": st.mean(rtt), "timeouts": leitor.stats.timeouts,
            "carga_barramento_pct": carga}


def main():
    info = {"python": sys.version.split()[0], "so": platform.platform(),
            "processador": platform.processor() or platform.machine(),
            "nucleos": os.cpu_count(), "data": time.strftime("%Y-%m-%d %H:%M:%S"),
            "tempo_quadro_8B_us": tempo_quadro_s(8) * 1e6, "bits_quadro_8B": bits_quadro_can(8)}
    resultados = {"ambiente": info}
    resultados["E1"] = ensaio_principal()
    print("[E2] sensibilidade ao tempo de resposta da ECU", flush=True)
    resultados["E2"] = [ensaio_curto(f"e2_{a}", a, a * 0.2, PIDS_MONITORADOS, 30) for a in (0, 10, 25, 50)]
    print("[E3] escalabilidade por número de PIDs", flush=True)
    resultados["E3"] = [ensaio_curto(f"e3_{n}", 10, 3, PIDS_MONITORADOS[:n], 20) for n in (1, 3, 5, 10)]
    with open(os.path.join(SAIDA, "resultados.json"), "w") as f:
        json.dump(resultados, f, indent=2, ensure_ascii=False)
    print(json.dumps(resultados, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
