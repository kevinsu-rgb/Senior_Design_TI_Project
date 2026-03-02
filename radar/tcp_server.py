"""
- Discovery (UDP):
  Host broadcasts:  "RADAR_DISCOVERY_V1?" to DISCOVERY_PORT
  Pi responds: "RADAR_DISCOVERY_V1! name=<NAME>;tcp=<TCP_PORT>;ver=1"

- Streaming (TCP):
  Each message is a single JSON object + "\n".
"""

from __future__ import annotations

import argparse
import json
import queue
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Tuple


DISCOVERY_MAGIC_Q = b"RADAR_DISCOVERY_V1?"
DISCOVERY_MAGIC_A = b"RADAR_DISCOVERY_V1!"
DEFAULT_DISCOVERY_PORT = 47777
DEFAULT_TCP_PORT = 7777


def _now_ms() -> int:
    return int(time.time() * 1000)


@dataclass
class ServerConfig:
    name: str
    tcp_host: str
    tcp_port: int
    discovery_port: int
    allow_broadcast: bool


class RadarTcpServer:
    #TCP server that broadcasts classification events to all connected clients.

    def __init__(self, cfg: ServerConfig):
        self.cfg = cfg
        self._clients: list[Tuple[socket.socket, Tuple[str, int]]] = []
        self._clients_lock = threading.Lock()
        self._event_q: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=10_000)
        self._stop = threading.Event()

        self._tcp_thread = threading.Thread(target=self._tcp_accept_loop, daemon=True)
        self._fanout_thread = threading.Thread(target=self._fanout_loop, daemon=True)
        self._udp_thread = threading.Thread(target=self._udp_discovery_loop, daemon=True)

    def start(self) -> None:
        self._tcp_thread.start()
        self._fanout_thread.start()
        self._udp_thread.start()

    def stop(self) -> None:
        self._stop.set()
        # Close client sockets
        with self._clients_lock:
            for c, _addr in self._clients:
                try:
                    c.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                try:
                    c.close()
                except Exception:
                    pass
            self._clients.clear()

    def publish_event(self, event: Dict[str, Any]) -> None:
        # Queue an event to send
        # Add time field
        event.setdefault("ts_ms", _now_ms())
        try:
            self._event_q.put_nowait(event)
        except queue.Full:
            # Drop if overwhelmed
            pass

    def _udp_discovery_loop(self) -> None:
        # UDP socket that listens for broadcasts
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception:
            pass

        sock.bind(("", self.cfg.discovery_port))
        sock.settimeout(0.5)

        while not self._stop.is_set():
            # always reply unless stop
            try:
                # load data, and source IP/port of the reply
                data, (src_ip, src_port) = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            if not data:
                continue

            # Check if it's a valid discovery query
            if data.strip() != DISCOVERY_MAGIC_Q:
                continue

            resp = (
                DISCOVERY_MAGIC_A
                + b" "
                + f"name={self.cfg.name};tcp={self.cfg.tcp_port};ver=1".encode("utf-8")
            )

            # Reply to the sender
            try:
                sock.sendto(resp, (src_ip, src_port))
            except Exception:
                pass

        try:
            sock.close()
        except Exception:
            pass

    def _tcp_accept_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.cfg.tcp_host, self.cfg.tcp_port))
        srv.listen(5)
        srv.settimeout(0.5)

        while not self._stop.is_set():
            try:
                client, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            # low-latency
            try:
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except Exception:
                pass

            with self._clients_lock:
                self._clients.append((client, addr))

            # send a hello
            hello = {
                "type": "hello",
                "ts_ms": _now_ms(),
                "name": self.cfg.name,
                "tcp_port": self.cfg.tcp_port,
                "ver": 1,
            }
            try:
                client.sendall((json.dumps(hello) + "\n").encode("utf-8"))
            except Exception:
                self._drop_client(client)

        try:
            srv.close()
        except Exception:
            pass

    def _fanout_loop(self) -> None:
        while not self._stop.is_set():
            # loop that waits for events and sends to all clients with a .5s timeout
            try:
                event = self._event_q.get(timeout=0.5)
            except queue.Empty:
                continue

            payload = (json.dumps(event, separators=(",", ":")) + "\n").encode("utf-8")

            # Send to all clients; add to dead list and drop for any that error
            dead: list[socket.socket] = []
            
            # lock while accessing clients list since it can be modified by the TCP accept loop
            with self._clients_lock:
                for c, _addr in self._clients:
                    try:
                        c.sendall(payload)
                    except Exception:
                        dead.append(c)

                # update clients list to drop any dead ones
                if dead:
                    new_clients = []
                    for client_socket, addr in self._clients:
                        if client_socket in dead:
                            continue
                        new_clients.append((client_socket, addr))

                    self._clients = new_clients

            # close dead clients
            for c in dead:
                try:
                    c.close()
                except Exception:
                    pass

    def _drop_client(self, client: socket.socket) -> None:
        with self._clients_lock:
            new_clients = []
            for client_socket, addr in self._clients:
                if client_socket == client:
                    continue
                new_clients.append((client_socket, addr))

            self._clients = new_clients
        try:
            client.close()
        except Exception:
            pass



def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--name", default=socket.gethostname(), help="Device name shown in discovery")
    p.add_argument("--tcp-host", default="0.0.0.0", help="Bind address for TCP server")
    p.add_argument("--tcp-port", type=int, default=DEFAULT_TCP_PORT)
    p.add_argument("--discovery-port", type=int, default=DEFAULT_DISCOVERY_PORT)
    
    p.add_argument("--cfg-path", default="config.cfg", help="Radar config file path passed to send_cfg")
    p.add_argument("--cli-port", default="/dev/ttyACM0", help="Radar CLI UART port")
    p.add_argument("--data-port", default="/dev/ttyACM0", help="Radar data UART port")
    p.add_argument("--cli-baud", type=int, default=115200, help="Radar CLI baud rate")
    p.add_argument("--data-baud", type=int, default=1250000, help="Radar data baud rate")

    # Keep the old demo option available for quick network testing
    p.add_argument("--demo", action="store_true", help="Publish fake classification events once per second")

    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = ServerConfig(
        name=args.name,
        tcp_host=args.tcp_host,
        tcp_port=args.tcp_port,
        discovery_port=args.discovery_port,
        allow_broadcast=True,
    )

    print(args)

    srv = RadarTcpServer(cfg)
    srv.start()

    status_q: queue.Queue[str] = queue.Queue()

    if args.demo:
        print("Running in demo mode. Publishing fake events every second.")
        # Demo loop: emits a fake event every second so you can test the pipeline
        try:
            seq = 0
            while True:
                srv.publish_event(
                    {
                        "type": "demo",
                        "class": "NONE" if (seq % 2) else "FALL",
                    }
                )
                seq += 1
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            srv.stop()
    else:
        print("Starting radar reader thread.")
        import read2  # local import to avoid dependency if just running demo

        read2.send_cfg(args.cfg_path, args.cli_baud, args.cli_port, args.data_port)

        uart_t = threading.Thread(
            target=read2.read_uart, 
            daemon=True,
            args=("", args.data_port, args.data_baud),
            name="radar-read-uart"
        )
        pred_t = threading.Thread(
            target=read2.predict, 
            daemon=True, 
            name="radar-predict", 
            kwargs={"status_out_queue": status_q}
        )

        uart_t.start()
        pred_t.start()

        
        print("Starting event publisher loop.")
        try:
            while True:
                try:
                    status = status_q.get(timeout=0.5)
                except queue.Empty:
                    print("empty")
                    event = {
                        "type": "status",
                        "class": "NOT_CLASSIFIED",
                    }
                    srv.publish_event(event)
                    continue
                event = {
                    "type": "status",
                    "class": status,
                }
                srv.publish_event(event)
        except KeyboardInterrupt:
            pass
        finally:
            srv.stop()
    


if __name__ == "__main__":
    main()