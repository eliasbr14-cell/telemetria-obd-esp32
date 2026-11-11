"""
servidor.py — Camadas de processamento/armazenamento e apresentação.

  * assina os tópicos MQTT  <base>/dados  e  <base>/dtc
  * persiste cada mensagem em SQLite (tabelas leituras e dtcs)
  * calcula a latência de ponta a ponta (instante de recepção - instante de publicação)
  * expõe o dashboard web e uma API:
        GET /                -> dashboard (dashboard/index.html)
        GET /api/stream      -> Server-Sent Events com os dados em tempo real
        GET /api/historico   -> últimas N leituras em JSON
        GET /api/estatisticas-> latência e contagem de mensagens

Uso:
    python servidor.py --broker localhost --porta-web 5000
"""
import argparse
import json
import os
import queue
import sqlite3
import threading
import time

import paho.mqtt.client as mqtt
from flask import Flask, Response, jsonify, send_from_directory

DIR = os.path.dirname(os.path.abspath(__file__))

ESQUEMA = """
CREATE TABLE IF NOT EXISTS leituras (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    seq INTEGER, ts_pub REAL, ts_rec REAL, latencia_ms REAL, ciclo_ms REAL,
    rpm REAL, velocidade REAL, temp_arrefecimento REAL, map REAL, carga_motor REAL,
    borboleta REAL, stft_b1 REAL, temp_admissao REAL, nivel_combustivel REAL, tensao_modulo REAL
);
CREATE TABLE IF NOT EXISTS dtcs (
    id INTEGER PRIMARY KEY AUTOINCREMENT, ts_pub REAL, ts_rec REAL, codigos TEXT
);
"""
CAMPOS = ["rpm", "velocidade", "temp_arrefecimento", "map", "carga_motor", "borboleta",
          "stft_b1", "temp_admissao", "nivel_combustivel", "tensao_modulo"]


class Armazenamento:
    def __init__(self, caminho):
        self.db = sqlite3.connect(caminho, check_same_thread=False)
        self.db.executescript(ESQUEMA)
        self.trava = threading.Lock()

    def inserir_leitura(self, p, ts_rec):
        d = p.get("dados", {})
        lat = (ts_rec - p["ts"]) * 1000
        with self.trava:
            self.db.execute(
                f"INSERT INTO leituras (seq, ts_pub, ts_rec, latencia_ms, ciclo_ms, {','.join(CAMPOS)}) "
                f"VALUES (?,?,?,?,?,{','.join('?' * len(CAMPOS))})",
                [p["seq"], p["ts"], ts_rec, lat, p.get("ciclo_ms")] + [d.get(c) for c in CAMPOS])
            self.db.commit()
        return lat

    def inserir_dtc(self, p, ts_rec):
        with self.trava:
            self.db.execute("INSERT INTO dtcs (ts_pub, ts_rec, codigos) VALUES (?,?,?)",
                            [p["ts"], ts_rec, ",".join(p["dtcs"])])
            self.db.commit()


def criar_app(armazenamento: Armazenamento, assinantes: list):
    app = Flask(__name__, static_folder=None)

    @app.get("/")
    def index():
        return send_from_directory(os.path.join(DIR, "dashboard"), "index.html")

    @app.get("/api/stream")
    def stream():
        q = queue.Queue(maxsize=200)
        assinantes.append(q)

        def gerar():
            try:
                while True:
                    try:
                        evento, dados = q.get(timeout=15)
                        yield f"event: {evento}\ndata: {dados}\n\n"
                    except queue.Empty:
                        yield ": keep-alive\n\n"
            finally:
                assinantes.remove(q)
        return Response(gerar(), mimetype="text/event-stream",
                        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    @app.get("/api/historico")
    def historico():
        with armazenamento.trava:
            cur = armazenamento.db.execute(
                f"SELECT seq, ts_pub, latencia_ms, {','.join(CAMPOS)} FROM leituras ORDER BY id DESC LIMIT 300")
            linhas = cur.fetchall()
        cols = ["seq", "ts", "latencia_ms"] + CAMPOS
        return jsonify([dict(zip(cols, l)) for l in reversed(linhas)])

    @app.get("/api/estatisticas")
    def estatisticas():
        with armazenamento.trava:
            n, lat_med, lat_max = armazenamento.db.execute(
                "SELECT COUNT(*), AVG(latencia_ms), MAX(latencia_ms) FROM leituras").fetchone()
        return jsonify({"mensagens": n, "latencia_media_ms": lat_med, "latencia_max_ms": lat_max})

    return app


def iniciar_assinante_mqtt(broker, porta, topico_base, armazenamento, assinantes, ao_receber=None):
    cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"servidor-{os.getpid()}")

    def on_connect(c, userdata, flags, rc, props=None):
        c.subscribe([(f"{topico_base}/dados", 0), (f"{topico_base}/dtc", 1)])

    def on_message(c, userdata, msg):
        ts_rec = time.time()
        p = json.loads(msg.payload)
        if msg.topic.endswith("/dados"):
            lat = armazenamento.inserir_leitura(p, ts_rec)
            p["latencia_ms"] = round(lat, 3)
            evento = "dados"
        else:
            armazenamento.inserir_dtc(p, ts_rec)
            evento = "dtc"
        texto = json.dumps(p)
        for q in list(assinantes):
            try:
                q.put_nowait((evento, texto))
            except queue.Full:
                pass
        if ao_receber:
            ao_receber(msg.topic, p, ts_rec)

    cliente.on_connect = on_connect
    cliente.on_message = on_message
    cliente.connect(broker, porta, keepalive=30)
    cliente.loop_start()
    return cliente


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default="localhost")
    ap.add_argument("--porta-mqtt", type=int, default=1883)
    ap.add_argument("--topico", default="telemetria/polo")
    ap.add_argument("--banco", default=os.path.join(DIR, "telemetria.db"))
    ap.add_argument("--porta-web", type=int, default=5000)
    a = ap.parse_args()

    arm = Armazenamento(a.banco)
    assinantes = []
    iniciar_assinante_mqtt(a.broker, a.porta_mqtt, a.topico, arm, assinantes)
    app = criar_app(arm, assinantes)
    print(f"Dashboard em http://localhost:{a.porta_web}")
    app.run(host="0.0.0.0", port=a.porta_web, threaded=True)


if __name__ == "__main__":
    main()
