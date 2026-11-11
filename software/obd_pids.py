"""
obd_pids.py — Definição dos PIDs OBD-II (Modo 01) utilizados no projeto.

As fórmulas de conversão seguem a norma SAE J1979 / ISO 15031-5.
Cada PID possui:
  - nome / unidade / faixa física
  - n_bytes: quantidade de bytes de dados (A, B, ...) na resposta
  - decodificar(bytes) -> valor físico      (usado pelo leitor / ESP32)
  - codificar(valor)   -> bytes             (usado pela ECU simulada)
"""
from dataclasses import dataclass
from typing import Callable, List


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass(frozen=True)
class PID:
    codigo: int
    chave: str          # nome curto usado no JSON / banco de dados
    descricao: str
    unidade: str
    minimo: float
    maximo: float
    n_bytes: int
    resolucao: float    # menor variação representável (quantização)
    decodificar: Callable[[List[int]], float]
    codificar: Callable[[float], List[int]]


def _u8(v):
    return [int(_clamp(round(v), 0, 255))]


def _u16(v):
    x = int(_clamp(round(v), 0, 65535))
    return [(x >> 8) & 0xFF, x & 0xFF]


PIDS = {
    0x04: PID(0x04, "carga_motor", "Carga calculada do motor", "%", 0, 100, 1, 100 / 255,
              lambda d: d[0] * 100 / 255,
              lambda v: _u8(v * 255 / 100)),
    0x05: PID(0x05, "temp_arrefecimento", "Temperatura do líquido de arrefecimento", "°C", -40, 215, 1, 1,
              lambda d: d[0] - 40,
              lambda v: _u8(v + 40)),
    0x06: PID(0x06, "stft_b1", "Ajuste de combustível de curto prazo (banco 1)", "%", -100, 99.2, 1, 100 / 128,
              lambda d: (d[0] - 128) * 100 / 128,
              lambda v: _u8(v * 128 / 100 + 128)),
    0x0B: PID(0x0B, "map", "Pressão absoluta do coletor de admissão (MAP)", "kPa", 0, 255, 1, 1,
              lambda d: d[0],
              lambda v: _u8(v)),
    0x0C: PID(0x0C, "rpm", "Rotação do motor", "rpm", 0, 16383.75, 2, 0.25,
              lambda d: (256 * d[0] + d[1]) / 4,
              lambda v: _u16(v * 4)),
    0x0D: PID(0x0D, "velocidade", "Velocidade do veículo", "km/h", 0, 255, 1, 1,
              lambda d: d[0],
              lambda v: _u8(v)),
    0x0F: PID(0x0F, "temp_admissao", "Temperatura do ar de admissão", "°C", -40, 215, 1, 1,
              lambda d: d[0] - 40,
              lambda v: _u8(v + 40)),
    0x11: PID(0x11, "borboleta", "Posição absoluta da borboleta", "%", 0, 100, 1, 100 / 255,
              lambda d: d[0] * 100 / 255,
              lambda v: _u8(v * 255 / 100)),
    0x2F: PID(0x2F, "nivel_combustivel", "Nível de combustível no tanque", "%", 0, 100, 1, 100 / 255,
              lambda d: d[0] * 100 / 255,
              lambda v: _u8(v * 255 / 100)),
    0x42: PID(0x42, "tensao_modulo", "Tensão do módulo de controle (bateria/alternador)", "V", 0, 65.535, 2, 0.001,
              lambda d: (256 * d[0] + d[1]) / 1000,
              lambda v: _u16(v * 1000)),
}

# Ordem de varredura (polling) utilizada pelo leitor
PIDS_MONITORADOS = [0x0C, 0x0D, 0x05, 0x0B, 0x04, 0x11, 0x06, 0x0F, 0x2F, 0x42]


def mascara_pids_suportados(base: int) -> List[int]:
    """Gera a resposta dos PIDs 0x00/0x20/0x40 (bitmask de PIDs suportados)."""
    bits = 0
    for p in list(PIDS.keys()) + [0x01, 0x20, 0x40]:
        if base < p <= base + 0x20:
            bits |= 1 << (32 - (p - base))
    # sinaliza que o próximo bloco existe
    if base + 0x20 in (0x20, 0x40):
        bits |= 1
    return [(bits >> 24) & 0xFF, (bits >> 16) & 0xFF, (bits >> 8) & 0xFF, bits & 0xFF]


# ----------------------------------------------------------------------------
# DTCs (Modo 03) — codificação de 2 bytes conforme SAE J2012
# ----------------------------------------------------------------------------
_LETRAS = "PCBU"


def dtc_para_bytes(codigo: str) -> List[int]:
    """'P0300' -> [0x03, 0x00]"""
    letra = _LETRAS.index(codigo[0].upper())
    d1 = int(codigo[1], 16)
    resto = int(codigo[2:], 16)
    b0 = (letra << 6) | (d1 << 4) | ((resto >> 8) & 0x0F)
    b1 = resto & 0xFF
    return [b0, b1]


def bytes_para_dtc(b0: int, b1: int) -> str:
    letra = _LETRAS[(b0 >> 6) & 0x03]
    d1 = (b0 >> 4) & 0x03
    return f"{letra}{d1}{b0 & 0x0F:X}{b1:02X}"
