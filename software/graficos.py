"""graficos.py — Gera as figuras de resultados a partir de ../resultados."""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.ticker import FuncFormatter
VIRG = FuncFormatter(lambda x, p: f"{x:g}".replace(".", ","))
f2 = lambda x: f"{x:.2f}".replace(".", ",")

R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resultados", "simulacao")
FIGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figuras")
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.grid": True, "grid.alpha": .3,
                     "savefig.dpi": 220, "savefig.bbox": "tight"})
AZUL, CINZA, LAR = "#1f5f8b", "#555555", "#d9822b"

res = json.load(open(os.path.join(R, "resultados.json")))

# 1) Série temporal: valor real (ECU) x valor decodificado
s = pd.read_csv(os.path.join(R, "e1_serie.csv"))
fig, ax = plt.subplots(3, 1, figsize=(8.6, 7.2), sharex=True)
for a, (pid, rot) in zip(ax, [("0x0C", "Rotação (rpm)"), ("0x0D", "Velocidade (km/h)"),
                               ("0x05", "Temp. arrefecimento (°C)")]):
    d = s[s.pid == pid]
    a.plot(d.t_s, d.valor_real, color=CINZA, lw=2.4, alpha=.45, label="valor real enviado pela ECU")
    a.plot(d.t_s, d.valor_decodificado, color=AZUL, lw=.9, label="valor decodificado pelo leitor")
    a.set_ylabel(rot)
ax[0].legend(loc="lower center", fontsize=8, frameon=True, ncol=2)
ax[-1].set_xlabel("tempo de ensaio (s)")
fig.savefig(os.path.join(FIGS, "fig_serie_temporal.png")); plt.close(fig)

# 2) Distribuições de tempo: RTT CAN e latência MQTT
rtt = pd.read_csv(os.path.join(R, "e1_rtt.csv")).rtt_ms
lat = pd.read_csv(os.path.join(R, "e1_latencia.csv")).latencia_ms
fig, ax = plt.subplots(1, 2, figsize=(9.6, 3.6))
ax[0].hist(rtt[rtt <= 20], bins=48, color=AZUL); ax[0].set_xlim(7, 20)
n_fora = int((rtt > 20).sum())
ax[0].text(0.98, 0.62, f"{n_fora} valores > 20 ms fora da escala\n(máx. {f2(rtt.max())} ms)", transform=ax[0].transAxes, ha="right", fontsize=7.5, color=CINZA)
ax[0].axvline(rtt.mean(), color=LAR, ls="--", label=f"média {f2(rtt.mean())} ms")
ax[0].axvline(np.percentile(rtt, 95), color=CINZA, ls=":", label=f"p95 {f2(np.percentile(rtt, 95))} ms")
ax[0].set(xlabel="tempo requisição→resposta CAN (ms)", ylabel="nº de requisições", title="(a) Tempo de resposta por PID")
ax[0].legend(frameon=False, fontsize=8)
ax[1].hist(lat[lat <= 2], bins=50, color=AZUL); ax[1].set_xlim(0, 2); ax[1].xaxis.set_major_formatter(VIRG)
n_fora2 = int((lat > 2).sum())
ax[1].text(0.98, 0.62, f"{n_fora2} valores > 2 ms fora da escala\n(máx. {f2(lat.max())} ms)", transform=ax[1].transAxes, ha="right", fontsize=7.5, color=CINZA)
ax[1].axvline(lat.mean(), color=LAR, ls="--", label=f"média {f2(lat.mean())} ms")
ax[1].axvline(np.percentile(lat, 95), color=CINZA, ls=":", label=f"p95 {f2(np.percentile(lat, 95))} ms")
ax[1].set(xlabel="latência publicação→persistência (ms)", ylabel="nº de mensagens", title="(b) Latência MQTT ponta a ponta")
ax[1].legend(frameon=False, fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(FIGS, "fig_distribuicoes.png")); plt.close(fig)

# 3) Sensibilidade (E2) e escalabilidade (E3)
e2 = pd.DataFrame(res["E2"]); e3 = pd.DataFrame(res["E3"])
fig, ax = plt.subplots(1, 2, figsize=(9.6, 3.6))
ax[0].plot(e2.atraso_ecu_ms, e2.ciclo_medio_ms, "o-", color=AZUL, label="tempo de ciclo medido")
teo = 10 * (e2.atraso_ecu_ms + 2 * 0.27)
ax[0].plot(e2.atraso_ecu_ms, teo, "s--", color=CINZA, label="modelo: N·(t_ECU + 2·t_quadro)")
ax[0].set(xlabel="tempo de resposta da ECU (ms)", ylabel="tempo de ciclo, 10 PIDs (ms)", title="(a) Sensibilidade ao tempo da ECU")
ax2 = ax[0].twinx(); ax2.plot(e2.atraso_ecu_ms, e2.taxa_por_pid_hz, "^:", color=LAR, label="atualização por PID (eixo dir.)"); ax2.set_ylabel("atualização por PID (Hz)", color=LAR)
ax2.grid(False); ax2.spines["right"].set_visible(True)
h1, l1 = ax[0].get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
ax[0].legend(h1 + h2, l1 + l2, loc="center right", fontsize=7.5, frameon=True)
ax[1].bar(e3.n_pids.astype(str), e3.taxa_por_pid_hz, color=AZUL)
for i, v in enumerate(e3.taxa_por_pid_hz):
    ax[1].text(i, v + 1, f"{v:.1f} Hz".replace(".", ","), ha="center", fontsize=8)
ax[1].set(xlabel="nº de PIDs monitorados", ylabel="taxa de atualização por PID (Hz)", title="(b) Escalabilidade (t_ECU = 10 ms)")
fig.tight_layout(); fig.savefig(os.path.join(FIGS, "fig_sensibilidade.png")); plt.close(fig)
print("figuras geradas")
