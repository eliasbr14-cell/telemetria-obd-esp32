"""
simulacao.py — Executa a ECU simulada e o leitor OBD-II conectados ao mesmo
barramento CAN e publica a telemetria no broker MQTT.

Exemplos:
    # barramento virtual (Windows/Linux/macOS), publica em localhost:1883
    python simulacao.py

    # Linux com SocketCAN virtual (ECU e leitor podem estar em processos/máquinas distintas)
    sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
    python simulacao.py --interface socketcan --canal vcan0

    # injeta a falha P0300 (falha de ignição aleatória) aos 60 s
    python simulacao.py --dtc P0300 --dtc-em 60
"""
import argparse
import threading
import time

import can
import paho.mqtt.client as mqtt

from ecu_simulada import ECUSimulada, ModeloVeiculo
from leitor_obd import LeitorOBD, executar_ciclos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="virtual")
    ap.add_argument("--canal", default="obd")
    ap.add_argument("--broker", default="localhost")
    ap.add_argument("--porta-mqtt", type=int, default=1883)
    ap.add_argument("--topico", default="telemetria/polo")
    ap.add_argument("--duracao", type=float, default=3600, help="segundos")
    ap.add_argument("--atraso-ecu-ms", type=float, default=10.0)
    ap.add_argument("--dtc", default=None, help="código de falha a injetar, ex. P0300")
    ap.add_argument("--dtc-em", type=float, default=60.0)
    a = ap.parse_args()

    bus_ecu = can.Bus(interface=a.interface, channel=a.canal, bitrate=500_000, receive_own_messages=False)
    bus_leitor = can.Bus(interface=a.interface, channel=a.canal, bitrate=500_000, receive_own_messages=False)

    modelo = ModeloVeiculo()
    ecu = ECUSimulada(bus_ecu, modelo, atraso_ms=a.atraso_ecu_ms)
    ecu.start()

    if a.dtc:
        threading.Timer(a.dtc_em, modelo.injetar_dtc, args=[a.dtc]).start()

    cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="leitor-obd-sim")
    cliente.connect(a.broker, a.porta_mqtt, keepalive=30)
    cliente.loop_start()

    leitor = LeitorOBD(bus_leitor)
    print("Simulação iniciada. Ctrl+C para encerrar.")
    try:
        executar_ciclos(leitor, cliente, a.topico, a.duracao,
                        ao_publicar=lambda p: print(f"#{p['seq']:5d}  rpm={p['dados']['rpm']:7.1f}  "
                                                    f"vel={p['dados']['velocidade']:5.1f}  "
                                                    f"temp={p['dados']['temp_arrefecimento']:5.1f}  "
                                                    f"ciclo={p['ciclo_ms']:.1f} ms"))
    except KeyboardInterrupt:
        pass
    finally:
        ecu.parar()
        cliente.loop_stop()
        bus_ecu.shutdown()
        bus_leitor.shutdown()


if __name__ == "__main__":
    main()
