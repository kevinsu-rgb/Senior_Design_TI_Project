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


# Helper to load config from file or fallback to defaults
def _load_server_config_from_file() -> ServerConfig:
    import device_config

    file_cfg = {}
    try:
        loaded = device_config.load_config()
        if isinstance(loaded, dict):
            file_cfg = loaded
    except Exception:
        file_cfg = {}

    return ServerConfig(
        name=file_cfg.get("radar_name") or socket.gethostname(),
        tcp_host=file_cfg.get("tcp_host") or "0.0.0.0",
        tcp_port=int(file_cfg.get("tcp_port", DEFAULT_TCP_PORT)),
        discovery_port=int(file_cfg.get("discovery_port", DEFAULT_DISCOVERY_PORT)),
        allow_broadcast=bool(file_cfg.get("allow_broadcast", True)),
        heartbeat_interval_s=float(file_cfg.get("heartbeat_interval_s", 1.0)),
    )


@dataclass
class ServerConfig:
    name: str
    tcp_host: str
    tcp_port: int
    discovery_port: int
    allow_broadcast: bool
    heartbeat_interval_s: float


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
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)

    def start(self) -> None:
        self._tcp_thread.start()
        self._fanout_thread.start()
        self._udp_thread.start()
        self._heartbeat_thread.start()

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

    def _heartbeat_loop(self) -> None:
        while not self._stop.is_set():
            self.publish_event(
                {
                    "type": "heartbeat",
                    "status": "UNKNOWN",
                }
            )

            if self._stop.wait(self.cfg.heartbeat_interval_s):
                break

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


# Background thread to refresh config from file
def refresh_cfg_loop(cfg: ServerConfig, stop_event: threading.Event, interval_s: float = 2.0) -> None:
    import device_config

    while not stop_event.is_set():
        try:
            loaded = device_config.load_config()
            if isinstance(loaded, dict):
                radar_name = loaded.get("radar_name")
                if isinstance(radar_name, str) and radar_name.strip():
                    cfg.name = radar_name.strip()

                tcp_host = loaded.get("tcp_host")
                if isinstance(tcp_host, str) and tcp_host.strip():
                    cfg.tcp_host = tcp_host.strip()

                tcp_port = loaded.get("tcp_port")
                if tcp_port is not None:
                    cfg.tcp_port = int(tcp_port)

                discovery_port = loaded.get("discovery_port")
                if discovery_port is not None:
                    cfg.discovery_port = int(discovery_port)

                allow_broadcast = loaded.get("allow_broadcast")
                if allow_broadcast is not None:
                    cfg.allow_broadcast = bool(allow_broadcast)

                heartbeat_interval_s = loaded.get("heartbeat_interval_s")
                if heartbeat_interval_s is not None:
                    cfg.heartbeat_interval_s = float(heartbeat_interval_s)
        except Exception as e:
            print(f"Failed to refresh config from JSON: {e}")

        if stop_event.wait(interval_s):
            break

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Radar TCP server")
    p.add_argument("--demo", action="store_true", help="Run in demo mode")
    return p.parse_args()

def main() -> None:
    args = parse_args()
    cfg = _load_server_config_from_file()

    srv = RadarTcpServer(cfg)
    srv.start()

    status_q: queue.Queue[str] = queue.Queue()
    cfg_refresh_t = threading.Thread(
        target=refresh_cfg_loop,
        args=(cfg, srv._stop),
        daemon=True,
        name="config-refresh",
    )
    cfg_refresh_t.start()

    if args.demo:
        print("Running in demo mode. Publishing fake events every second.")
        # Demo loop: emits a fake event every second so you can test the pipeline
        try:
            seq = 0
            while True:
                srv.publish_event(
                    {
                        "type": "status",
                        "status": "STANDING" if (seq % 2) else "STANDING",
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

        try:
            read2.send_cfg(cfg.cfg_path, cfg.cli_baud, cfg.cli_port, cfg.data_port)
        except Exception as e:
            while (True):
                print(f"Failed to send config from {cfg.cfg_path} to radar on {cfg.cli_port} at baud {cfg.cli_baud}. Check connection and config file.")
                event = {
                    "type": "status",
                    "status": "RADAR_CFG_ERROR",
                }
                srv.publish_event(event)
                time.sleep(5.0)
            

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
        prev_status = "UNKNOWN"
        try:
            while True:
                try:
                    status = status_q.get(timeout=0.5)
                    prev_status = status
                except queue.Empty:
                    print("empty")
                    event = {
                        "type": "status",
<<<<<<< HEAD
                        "status": prev_status,
=======
                        "status": "UNKNOWN",
                        "name": cfg.name,
>>>>>>> 7ddc379 (change name is broken, recconnect is working)
                    }
                    srv.publish_event(event)
                    continue
                event = {
                    "type": "status",
                    "status": status,
                    "name": cfg.name,
                }
                srv.publish_event(event)
        except KeyboardInterrupt:
            pass
        finally:
            srv.stop()
    


if __name__ == "__main__":
    main()
