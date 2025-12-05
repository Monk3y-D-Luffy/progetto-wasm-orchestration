#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import time


def send_request(gw_host: str, gw_port: int, payload: dict, timeout: float = 5.0):
    data = (json.dumps(payload) + "\n").encode("utf-8")     # trasforma il dizionario in stringa JSON, aggiunge un terminatore di riga e converte la stringa in bytes
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        s.connect((gw_host, gw_port))
        s.sendall(data)

        buf = bytearray()
        while True:
            try:
                chunk = s.recv(4096)  #legge fino a 4096 byte dalla socket
                if not chunk:
                    break
                buf += chunk
            except socket.timeout:
                break

    if not buf:
        print("!! nessuna risposta dal gateway")
        return None

    try:
        resp = json.loads(buf.decode("utf-8"))  # converte i bytes in stringa e prova a interpretare la stringa come JSON e a convertirla in oggetti Python (tipicamente un dict)
    except Exception as e:
        print("!! risposta non valida dal gateway:", e)
        print(buf.decode("utf-8", errors="ignore"))
        return None

    return resp


def pretty_print_response(resp):
    if resp is None:
        return
    print(json.dumps(resp, indent=2, ensure_ascii=False))



# comandi base

def cmd_deploy(args):
    payload = {
        "cmd": "deploy",
        "device": args.device,
        "module_id": args.module_id,
        "wasm_path": args.wasm,
    }
    t0 = time.perf_counter()
    resp = send_request(args.gw_host, args.gw_port, payload)
    t1 = time.perf_counter()
    latency_ms = (t1 - t0) * 1000.0

    print(f"e2e_latency_ms={latency_ms:.2f}")
    pretty_print_response(resp)


def cmd_start(args):
    payload = {
        "cmd": "start",
        "device": args.device,
        "module_id": args.module_id,
        "func_name": args.func_name,
        "func_args": args.func_args or "",
        "wait_result": bool(args.wait_result),
        "result_timeout": float(args.result_timeout),
    }
    timeout = args.result_timeout + 5.0 if args.wait_result else 10.0
    
    t0 = time.perf_counter()
    resp = send_request(args.gw_host, args.gw_port, payload, timeout=timeout)
    t1 = time.perf_counter()
    latency_ms = (t1 - t0) * 1000.0

    print(f"e2e_latency_ms={latency_ms:.2f}")
    pretty_print_response(resp)


def cmd_stop(args):
    payload = {
        "cmd": "stop",
        "device": args.device,
        "module_id": args.module_id,
        "result_timeout": float(args.result_timeout),
    }
    timeout = args.result_timeout + 5.0

    t0 = time.perf_counter()
    resp = send_request(args.gw_host, args.gw_port, payload, timeout=timeout)
    t1 = time.perf_counter()
    latency_ms = (t1 - t0) * 1000.0

    print(f"e2e_latency_ms={latency_ms:.2f}")
    pretty_print_response(resp)


def cmd_status(args):
    payload = {
        "cmd": "status",
        "device": args.device,
    }
    t0 = time.perf_counter()
    resp = send_request(args.gw_host, args.gw_port, payload)
    t1 = time.perf_counter()
    latency_ms = (t1 - t0) * 1000.0

    print(f"e2e_latency_ms={latency_ms:.2f}")
    pretty_print_response(resp)


def cmd_build_and_deploy(args):
    payload = {
        "cmd": "build_and_deploy",
        "device": args.device,
        "module_id": args.module_id,
        "source_path": args.source,
        "mode": args.mode,
    }
    t0 = time.perf_counter()
    resp = send_request(args.gw_host, args.gw_port, payload, timeout=60.0)
    t1 = time.perf_counter()
    latency_ms = (t1 - t0) * 1000.0

    print(f"e2e_latency_ms={latency_ms:.2f}")
    pretty_print_response(resp)



# main

def main():
    parser = argparse.ArgumentParser(
        description="Host client per gateway orchestrator"
    )
    parser.add_argument(
        "--gw-host", default="localhost", help="Hostname o IP del gateway"
    )
    parser.add_argument(
        "--gw-port", type=int, default=9000, help="Porta TCP del gateway"
    )
    parser.add_argument(
        "--device",
        required=True,
        help="ID logico del device (es. nucleo, disco)",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    # deploy
    p_deploy = subparsers.add_parser("deploy", help="Deploy di un modulo wasm/aot")
    p_deploy.add_argument("--module-id", required=True)
    p_deploy.add_argument("--wasm", required=True, help="File .wasm o .aot")
    p_deploy.set_defaults(func=cmd_deploy)

    # start
    p_start = subparsers.add_parser("start", help="Start di una funzione")
    p_start.add_argument("--module-id", required=True)
    p_start.add_argument("--func-name", required=True)
    p_start.add_argument("--func-args", help='Argomenti "a=1,b=2"')
    p_start.add_argument(
        "--wait-result",
        action="store_true",
        help="Aspetta il RESULT prima di uscire",
    )
    p_start.add_argument(
        "--result-timeout",
        type=float,
        default=10.0,
        help="Timeout attesa RESULT",
    )
    p_start.set_defaults(func=cmd_start)

    # stop
    p_stop = subparsers.add_parser("stop", help="Stop di un job long-running")
    p_stop.add_argument("--module-id", required=True)
    p_stop.add_argument(
        "--result-timeout",
        type=float,
        default=10.0,
        help="Timeout attesa RESULT dopo STOP_OK PENDING",
    )
    p_stop.set_defaults(func=cmd_stop)

    # status
    p_status = subparsers.add_parser("status", help="Stato del device")
    p_status.set_defaults(func=cmd_status)

    # build-and-deploy
    p_build = subparsers.add_parser(
        "build-and-deploy",
        help="Compila un sorgente C in WASM/AOT e fa deploy sul device",
    )
    p_build.add_argument("--module-id", required=True, help="ID logico del modulo")
    p_build.add_argument(
        "--source",
        required=True,
        help="File sorgente C da compilare",
    )
    p_build.add_argument(
        "--mode",
        choices=["wasm", "aot"],
        default="wasm",
        help="Tipo di binario da generare (default: wasm)",
    )
    p_build.set_defaults(func=cmd_build_and_deploy)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
