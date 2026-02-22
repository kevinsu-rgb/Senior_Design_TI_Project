import json
import time
import queue
import serial


def _extract_status(line: str):
    text = line.strip()
    if not text:
        return None

    if text.startswith("{"):
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return None
        status = payload.get("status")
        if isinstance(status, str) and status.strip():
            return status.strip().upper()
        return None

    # Fallback: if sender transmits plain labels per line.
    return text.upper()


def read_statuses_from_serial(com_port: str, baud_rate: int, status_queue):
    while True:
        ser = None
        try:
            print(f"[BT] Opening {com_port} @ {baud_rate}")
            ser = serial.Serial(com_port, baud_rate, timeout=1)
            print(f"[BT] Connected to {com_port}")

            while True:
                raw = ser.readline()
                if not raw:
                    continue

                decoded = raw.decode("utf-8", errors="ignore")
                status = _extract_status(decoded)
                if not status:
                    continue

                try:
                    status_queue.put(status, block=False)
                except queue.Full:
                    pass
        except Exception as exc:
            print(f"[BT] Serial read failed: {exc}. Retrying in 2s...")
            time.sleep(2)
        finally:
            try:
                ser.close()
            except Exception:
                pass
