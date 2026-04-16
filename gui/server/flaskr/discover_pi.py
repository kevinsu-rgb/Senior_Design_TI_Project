import socket
import time
import json
import threading
import queue
from typing import Dict, Any, Iterator, Optional

DISCOVERY_MAGIC_Q = b"RADAR_DISCOVERY_V1?"
DISCOVERY_MAGIC_A = b"RADAR_DISCOVERY_V1!"
DISCOVERY_PORT = 47777

def discover_pis(timeout_s=0.6):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.settimeout(timeout_s)

    # Bind so we can receive replies
    s.bind(("", 0))

    # Broadcast discovery
    s.sendto(DISCOVERY_MAGIC_Q, ("255.255.255.255", DISCOVERY_PORT))

    devices = []
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        # Wait for replies until timeout
        try:
            # load data, and source IP/port of the reply
            data, (ip, _port) = s.recvfrom(2048)
        except socket.timeout:
            break

        data = data.strip()
        if not data.startswith(DISCOVERY_MAGIC_A):
            continue

        # Parse: "RADAR_DISCOVERY_V1! name=...;tcp=...;ver=1"
        try:
            payload = data.split(b" ", 1)[1].decode("utf-8", errors="replace")
            
            fields = {}
            for part in payload.split(";"):
                if "=" not in part:
                    continue
                key, value = part.split("=", 1)
                fields[key] = value

            devices.append({
                "ip": ip,
                "name": fields.get("name", ip),
                "tcp_port": int(fields.get("tcp", "7777")),
                "ver": fields.get("ver", ""),
            })
        except Exception:
            continue

    s.close()

    # De-dupe by (ip, tcp_port)
    uniq = {}
    for d in devices:
        uniq[(d["ip"], d["tcp_port"])] = d
    return list(uniq.values())



def connect_and_stream(ip: str, tcp_port: int, timeout_s: float = 3.0) -> Iterator[Dict[str, Any]]:
    # Connect to the Pi TCP stream and yield JSON events. This processes one line at a time, so it can be used without buffering the entire response.
    sock = socket.create_connection((ip, tcp_port), timeout=timeout_s)
    
    try:
        # Use a file wrapper for clean line-based reads
        f = sock.makefile("r", encoding="utf-8", newline="\n")
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                # Skip malformed lines rather than killing the stream
                continue
    finally:
        try:
            sock.close()
        except Exception:
            pass


def choose_device(devices: list[dict]) -> Optional[dict]:
    if not devices:
        return None

    print("Multiple devices found:")
    for i, d in enumerate(devices):
        print(f"  [{i}] {d['name']} at {d['ip']}:{d['tcp_port']} (ver={d['ver']})")

    while True:
        sel = input("Select device index: ").strip()
        if sel.isdigit():
            idx = int(sel)
            if 0 <= idx < len(devices):
                return devices[idx]
        print("Invalid selection. Try again.")


def _stream_device(dev: dict, status_queue: queue.Queue, events_lock: threading.Lock, active_devices: dict, active_lock: threading.Lock):
    # Connect to one device and print events as they arrive. This is designed to run in a separate thread for each device.
    name = dev.get("name", "unknown")
    ip = dev.get("ip")
    port = dev.get("tcp_port")
    key = (ip, port)
    prefix = f"[{name} {ip}:{port}]"

    count = 0
    try:
        while True:
            if count > 5:
                print(f"{prefix} too many connection failures, giving up.")
                break
            try:
                for evt in connect_and_stream(ip, port):
                    print(f"{prefix} {evt}")
                    with events_lock:
                        status_queue.put((ip, evt))
                print(f"{prefix} stream ended. Reconnecting in 1s...")
                time.sleep(1.0)
                count += 1

            except KeyboardInterrupt:
                raise
            except OSError as e:
                # If the server goes away, retry after a short delay
                print(f"{prefix} connection error: {e}. Reconnecting in 1s...")
                time.sleep(1.0)
                count += 1
            except Exception as e:
                print(f"{prefix} unexpected error: {e}. Reconnecting in 1s...")
                time.sleep(1.0)
                count += 1
    finally:
        with active_lock:
            active_devices.pop(key, None)
        print(f"{prefix} removed from active devices.")

def _stream_all(devices: list[dict], status_queue, active_devices: Optional[dict] = None, active_lock: Optional[threading.Lock] = None):
    events_lock = threading.Lock()

    if active_devices is None:
        active_devices = {}
    if active_lock is None:
        active_lock = threading.Lock()

    for dev in devices:
        ip = dev.get("ip")
        port = dev.get("tcp_port")
        key = (ip, port)

        with active_lock:
            existing = active_devices.get(key)
            if existing is not None and existing.is_alive():
                continue

            t = threading.Thread(
                target=_stream_device,
                args=(dev, status_queue, events_lock, active_devices, active_lock),
                daemon=True,
            )
            active_devices[key] = t
            t.start()

    return active_devices


def stream_with_periodic_discovery(status_queue: queue.Queue, discovery_interval_s: float = 5.0, timeout_s: float = 1.0):
    active_devices: Dict[tuple, threading.Thread] = {}
    active_lock = threading.Lock()

    while True:
        try:
            devices = discover_pis(timeout_s=timeout_s)
            _stream_all(devices, status_queue, active_devices=active_devices, active_lock=active_lock)
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"Periodic discovery error: {e}")

        time.sleep(discovery_interval_s)

def main():
    devices = discover_pis(timeout_s=1.0)
    print(devices)
    time.sleep(4)  # Small delay to ensure all responses are printed before proceeding
    if not devices:
        print("No devices found. Make sure the Pi server is running and you are on the same LAN.")
        raise SystemExit(1)

    # If only one device is found, connect to it.
    # If multiple are found, connect to all of them concurrently.
    if len(devices) == 1:
        selected = devices
    else:
        print("Multiple devices found. Connecting to all.")
        for d in devices:
            print(f"  - {d['name']} at {d['ip']}:{d['tcp_port']} (ver={d['ver']})")
        selected = devices

    status_queue = queue.Queue()
    _stream_all(selected, status_queue)

    # Keep main thread alive so daemon threads can run
    try:
        while True:
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()