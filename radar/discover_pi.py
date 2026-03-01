import socket
import time
import json
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


if __name__ == "__main__":
    devices = discover_pis(timeout_s=1.0)
    if not devices:
        print("No devices found. Make sure the Pi server is running and you are on the same LAN.")
        raise SystemExit(1)

    dev = choose_device(devices)
    if dev is None:
        print("No device selected.")
        raise SystemExit(1)

    print(f"Connecting to {dev['name']} at {dev['ip']}:{dev['tcp_port']}...")
    try:
        for evt in connect_and_stream(dev["ip"], dev["tcp_port"]):
            # Print events as they arrive
            print(evt)
    except KeyboardInterrupt:
        pass
    except OSError as e:
        print(f"Connection error: {e}")
        raise SystemExit(2)