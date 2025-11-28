#!/usr/bin/env python3
import argparse
import binascii
import json
import os
import socket
import sys
import threading
import time
import subprocess
import tempfile
from pathlib import Path

try:
    import serial  # pyserial
except ImportError:
    serial = None


# Mappa device -> endpoint fisico/emulato

DEVICE_ENDPOINTS = {
    "nucleo": "COM3",              
    "disco":  "tcp:localhost:3456", 
}


# Config compilatore 

# clang o wasi-clang in PATH
CLANG_BIN = "clang"   # o "wasi-clang"
CLANG_TARGET = "wasm32-unknown-unknown"

# wamrc di WAMR in PATH (per generare .aot)
WAMRC_BIN = "wamrc"


# Transport 

class Transport:
    def __init__(self, ser=None, sock=None):
        self.ser = ser
        self.sock = sock

    def close(self):
        if self.ser is not None:
            self.ser.close()
        if self.sock is not None:
            self.sock.close()

    def flush_input(self):
        if self.ser is not None:
            self.ser.reset_input_buffer()
        if self.sock is not None:
            self.sock.settimeout(0.0)
            try:
                while True:
                    data = self.sock.recv(1024)
                    if not data:
                        break
            except (BlockingIOError, socket.timeout):
                pass
            finally:
                self.sock.settimeout(0.1)

    def write_line(self, text: str):
        data = (text + "\n").encode("ascii")
        self.write(data)

    def write(self, data: bytes):
        if self.ser is not None:
            self.ser.write(data)
            self.ser.flush()
        elif self.sock is not None:
            self.sock.sendall(data)

    def read_line(self, timeout: float = 1.0):
        deadline = time.time() + timeout
        buf = bytearray()
        while time.time() < deadline:
            b = None
            try:
                if self.ser is not None:
                    chunk = self.ser.read(1)
                    if not chunk:
                        continue
                    b = chunk
                elif self.sock is not None:
                    self.sock.settimeout(0.1)
                    chunk = self.sock.recv(1)
                    if not chunk:
                        if not buf:
                            return None
                        break
                    b = chunk
            except socket.timeout:
                continue

            if b is None:
                continue

            buf += b
            if b == b"\n":
                break

        if not buf:
            return None

        line = buf.decode("ascii", errors="ignore").rstrip("\r\n")
        print("<<", line)
        return line


def open_transport(port: str) -> Transport:
    if port.startswith("tcp:"):
        rest = port[4:]
        if ":" in rest:
            host, p = rest.split(":", 1)
        else:
            host, p = "localhost", rest
        tcp_port = int(p)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, tcp_port))
        s.settimeout(0.1)
        return Transport(sock=s)
    else:
        if serial is None:
            raise RuntimeError("pyserial non installato")
        ser = serial.Serial(port, baudrate=115200, timeout=0.1)
        return Transport(ser=ser)


def read_until_prefix(transport: Transport, prefixes, timeout: float):
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = transport.read_line(timeout=deadline - time.time())
        if line is None:
            continue
        for p in prefixes:
            if line.startswith(p):
                return line
    return None



# Funzioni di compilazione

# Compila un file C in un modulo .wasm

def compile_to_wasm(source_c: str, out_wasm: str):
    cmd = [
        CLANG_BIN,
        f"--target={CLANG_TARGET}",
        "-O3",
        "-nostdlib",
        "-Wl,--no-entry",
        "-Wl,--initial-memory=65536",
        "-Wl,--max-memory=65536",
        "-Wl,--stack-first",
        "-Wl,-z,stack-size=2048",
        source_c,
        "-o",
        out_wasm,
    ]
    print("Compilo C -> WASM:", " ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        return {
            "ok": False,
            "error": "errore compilazione C->WASM",
            "stderr": res.stderr,
            "stdout": res.stdout,
        }
    return {"ok": True, "wasm_path": out_wasm}


# Compila un modulo .wasm in .aot

def compile_to_aot(wasm_path: str, out_aot: str):
    cmd = [
        WAMRC_BIN,
        "--target=thumbv7em",
        "--cpu=cortex-m4",
        "--target-abi=gnu",
        "-o", out_aot,
        wasm_path,
    ]
    print("Compilo WASM -> AOT:", " ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        return {
            "ok": False,
            "error": "errore compilazione WASM->AOT",
            "stderr": res.stderr,
            "stdout": res.stdout,
        }
    return {"ok": True, "aot_path": out_aot}


# Operazioni verso l'agent 

def gw_deploy(device_port: str, module_id: str, wasm_or_aot_path: str):
    if not os.path.isfile(wasm_or_aot_path):
        return {"ok": False, "error": f"file non trovato: {wasm_or_aot_path}"}

    with open(wasm_or_aot_path, "rb") as f:
        data = f.read()

    size = len(data)
    crc32 = binascii.crc32(data) & 0xFFFFFFFF
    crc_hex = f"{crc32:08x}"

    t = open_transport(device_port)
    try:
        t.flush_input()
        line = f"LOAD module_id={module_id} size={size} crc32={crc_hex}"
        print(">>", line)
        t.write_line(line)

        resp = read_until_prefix(t, ["LOAD_READY", "LOAD_ERR"], timeout=3.0)
        if resp is None:
            return {"ok": False, "error": "timeout in attesa di LOAD_READY/LOAD_ERR"}
        if resp.startswith("LOAD_ERR"):
            return {"ok": False, "error": resp}

        print(f">> [BINARY] {size} bytes")
        t.write(data)

        resp2 = read_until_prefix(t, ["LOAD_OK", "LOAD_ERR"], timeout=3.0)
        if resp2 is None:
            return {"ok": False, "error": "timeout in attesa di LOAD_OK/LOAD_ERR"}
        if resp2.startswith("LOAD_ERR"):
            return {"ok": False, "error": resp2}
        return {"ok": True, "detail": resp2}
    finally:
        t.close()


def gw_start(device_port: str, module_id: str, func_name: str,
             func_args: str, wait_result: bool, result_timeout: float):
    t = open_transport(device_port)
    try:
        t.flush_input()
        if func_args:
            line = (
                f"START module_id={module_id} "
                f"func={func_name} args=\"{func_args}\""
            )
        else:
            line = (
                f"START module_id={module_id} "
                f"func={func_name}"
            )
        print(">>", line)
        t.write_line(line)

        while True:
            resp = read_until_prefix(
                t, ["START_OK", "RESULT", "ERROR"], timeout=3.0
            )
            if resp is None:
                return {"ok": False,
                        "error": "timeout in attesa di START_OK/RESULT/ERROR"}

            if resp.startswith("ERROR"):
                return {"ok": False, "error": resp}

            if resp.startswith("RESULT"):
                if ("status=NO_MODULE" in resp
                        or "status=BUSY" in resp
                        or "status=NO_FUNC" in resp):
                    return {"ok": False, "error": resp}
                continue

            break  # START_OK

        if not wait_result:
            return {"ok": True, "detail": "START_OK"}

        resp2 = read_until_prefix(t, ["RESULT"], timeout=result_timeout)
        if resp2 is None:
            return {"ok": False, "error": "timeout in attesa di RESULT"}
        return {"ok": True, "detail": resp2}
    finally:
        t.close()


def gw_stop(device_port: str, module_id: str, result_timeout: float):
    t = open_transport(device_port)
    try:
        t.flush_input()
        line = f"STOP module_id={module_id}"
        print(">>", line)
        t.write_line(line)

        resp = read_until_prefix(t, ["STOP_OK", "RESULT", "ERROR"], timeout=3.0)
        if resp is None:
            return {"ok": False,
                    "error": "timeout in attesa di STOP_OK/RESULT/ERROR"}

        if resp.startswith("RESULT") or resp.startswith("ERROR"):
            return {"ok": True, "detail": resp}

        if "status=PENDING" not in resp:
            return {"ok": True, "detail": resp}

        resp2 = read_until_prefix(t, ["RESULT"], timeout=result_timeout)
        if resp2 is None:
            return {"ok": False, "error": "timeout in attesa di RESULT (stop)"}
        return {"ok": True, "detail": resp2}
    finally:
        t.close()


def gw_status(device_port: str):
    t = open_transport(device_port)
    try:
        t.flush_input()
        line = "STATUS"
        print(">>", line)
        t.write_line(line)

        resp = read_until_prefix(t, ["STATUS", "ERROR", "RESULT"], timeout=2.0)
        if resp is None:
            return {"ok": False, "error": "timeout in attesa di STATUS"}
        return {"ok": True, "detail": resp}
    finally:
        t.close()


# build_and_deploy 
# Modalità: wasm oppure aot
#   wasm: compila C -> wasm e deploya il wasm
#   aot:  compila C -> wasm, poi wasm -> aot, deploya l'aot

def gw_build_and_deploy(device_port: str, module_id: str,
                        source_path: str, mode: str):
   
    source_path = os.path.abspath(source_path)
    if not os.path.isfile(source_path):
        return {"ok": False, "error": f"sorgente C non trovato: {source_path}"}

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir_p = Path(tmpdir)
        wasm_path = str(tmpdir_p / f"{module_id}.wasm")
        res_wasm = compile_to_wasm(source_path, wasm_path)
        if not res_wasm.get("ok"):
            return {"ok": False, "step": "compile_wasm", **res_wasm}

        deploy_path = wasm_path
        extra = {"wasm_path": wasm_path}

        if mode == "aot":
            aot_path = str(tmpdir_p / f"{module_id}.aot")
            res_aot = compile_to_aot(wasm_path, aot_path)
            if not res_aot.get("ok"):
                return {"ok": False, "step": "compile_aot", **res_aot}
            deploy_path = aot_path
            extra["aot_path"] = aot_path

        res_dep = gw_deploy(device_port, module_id, deploy_path)
        return {"step": "deploy", **extra, **res_dep}


# Server TCP del gateway

def handle_client(conn, addr):
    try:
        buf = bytearray()
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk
            if b"\n" in chunk:
                break

        if not buf:
            return

        try:
            req = json.loads(buf.decode("utf-8").strip())
        except Exception as e:
            resp = {"ok": False, "error": f"json non valido: {e}"}
            conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
            return

        device = req.get("device")
        if device not in DEVICE_ENDPOINTS:
            resp = {"ok": False, "error": f"device sconosciuto: {device}"}
            conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
            return

        port = DEVICE_ENDPOINTS[device]
        cmd = req.get("cmd")

        if cmd == "deploy":
            resp = gw_deploy(
                port,
                req["module_id"],
                req["wasm_path"],
            )
        elif cmd == "start":
            resp = gw_start(
                port,
                req["module_id"],
                req["func_name"],
                req.get("func_args", ""),
                bool(req.get("wait_result", False)),
                float(req.get("result_timeout", 10.0)),
            )
        elif cmd == "stop":
            resp = gw_stop(
                port,
                req["module_id"],
                float(req.get("result_timeout", 10.0)),
            )
        elif cmd == "status":
            resp = gw_status(port)
        elif cmd == "build_and_deploy":
            mode = req.get("mode", "wasm")
            resp = gw_build_and_deploy(
                port,
                req["module_id"],
                req["source_path"],
                mode,
            )
        else:
            resp = {"ok": False, "error": f"comando sconosciuto: {cmd}"}

        conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
    finally:
        conn.close()


def run_gateway(listen_host: str, listen_port: int):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((listen_host, listen_port))
        s.listen(5)
        print(f"Gateway in ascolto su {listen_host}:{listen_port}")
        while True:
            conn, addr = s.accept()
            t = threading.Thread(target=handle_client, args=(conn, addr),
                                 daemon=True)
            t.start()


def main():
    parser = argparse.ArgumentParser(
        description="Gateway per orchestrazione moduli Wasm/AOT su device STM32/Zephyr"
    )
    parser.add_argument("--host", default="0.0.0.0", help="Host di ascolto")
    parser.add_argument("--port", type=int, default=9000, help="Porta di ascolto")
    args = parser.parse_args()
    run_gateway(args.host, args.port)


if __name__ == "__main__":
    main()
