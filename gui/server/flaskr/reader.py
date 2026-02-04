import os
import sys
import time
from contextlib import suppress
from pathlib import Path
import queue as pyqueue
import struct

import numpy as np
import serial
import torch

from . import queue

MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"

HEATMAP_TLV = 304
NUM_RANGE_BINS = 128
NUM_AZIMUTH_BINS = 8
HEATMAP_BYTES = 4 * NUM_RANGE_BINS * NUM_AZIMUTH_BINS

# keep only the most recent heatmap frame
heatmap_queue: "pyqueue.Queue[np.ndarray]" = pyqueue.Queue(maxsize=1)


def _ensure_repo_root_on_path() -> Path:
    # add the repo root to the path
    repo_root = Path(__file__).resolve().parents[3]
    repo_root_str = str(repo_root)
    if repo_root_str not in sys.path:
        sys.path.insert(0, repo_root_str)
    return repo_root


def _load_model():
    repo_root = _ensure_repo_root_on_path()

    from radar.nn import NeuralNetwork

    model_path = repo_root / "radar" / "model.pth"
    model = NeuralNetwork()
    model.load_state_dict(torch.load(model_path, map_location="cpu"))
    model.eval()
    return model


def _read_cfg_lines(cfg_path: Path) -> list[str]:
    # prune blank lines and comments and keep ordering
    lines = []
    with cfg_path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("%"):
                continue
            lines.append(line)
    return lines


def send_cfg_single_com(port: str, cfg_path: str, initial_baud: int = 115200) -> int:
    repo_root = _ensure_repo_root_on_path()
    resolved_cfg = Path(cfg_path)
    if not resolved_cfg.is_absolute():
        resolved_cfg = (repo_root / resolved_cfg).resolve()

    cfg_lines = _read_cfg_lines(resolved_cfg)

    baud = int(initial_baud)
    # use short timeouts to not hang waiting on responses
    cli = serial.Serial(port, baud, timeout=0.1, write_timeout=0.5)
    try:
        with suppress(Exception):
            cli.reset_output_buffer()
        with suppress(Exception):
            cli.reset_input_buffer()

        # activate cli
        with suppress(Exception):
            cli.write(b"\n\n")
        time.sleep(0.05)

        # send chars 1 by 1
        char_delay_s = float(os.getenv("RADAR_CLI_CHAR_DELAY_S", "0.001"))
        line_delay_s = float(os.getenv("RADAR_CLI_LINE_DELAY_S", "0.02"))

        for line in cfg_lines:
            payload = (line + "\n").encode("utf-8", errors="ignore")
            for b in payload:
                with suppress(Exception):
                    cli.write(bytes([b]))
                time.sleep(char_delay_s)
            time.sleep(line_delay_s)

            # check for responses
            with suppress(Exception):
                n = cli.in_waiting
                if n:
                    _ = cli.read(n)

            # baudRate
            parts = line.split()
            if parts and parts[0] == "baudRate" and len(parts) >= 2:
                try:
                    baud = int(parts[1])
                    print(f"[reader] CLI baudRate -> {baud}")
                    time.sleep(0.05)
                    cli.baudrate = baud
                    time.sleep(0.05)
                except Exception as e:
                    print(f"[reader] Failed to switch baud: {e}")
            elif parts and parts[0].lower().startswith("sensorstart"):
                print("[reader] sensorStart sent")

        return baud
    finally:
        with suppress(Exception):
            cli.close()


def start_pipeline_with_config(
    port: str = None,
    cfg_path: str = None,
    initial_baud: int = 115200,
    default_stream_baud: int = 1250000,
):
    repo_root = _ensure_repo_root_on_path()

    if port is None:
        port = os.getenv("RADAR_PORT", "COM5")
    if cfg_path is None:
        cfg_path = os.getenv("RADAR_CFG", str(repo_root / "radar" / "config.cfg"))

    final_baud = default_stream_baud
    try:
        print(f"[reader] Sending cfg {cfg_path} on {port} @ {initial_baud}")
        final_baud = send_cfg_single_com(port=port, cfg_path=cfg_path, initial_baud=initial_baud)
        print(f"[reader] Config sent. Switching to stream @ {final_baud}")
    except Exception as e:
        print(f"[reader] Config send failed: {e}. Trying to stream anyway @ {default_stream_baud}")
        final_baud = default_stream_baud

    time.sleep(float(os.getenv("RADAR_STREAM_START_DELAY_S", "0.3")))

    # listen for heatmaps
    read_uart_heatmap(port=port, baud=final_baud)


def read_uart_heatmap(port: str = None, baud: int = None):
    if port is None:
        port = os.getenv("RADAR_PORT", "COM5")
    if baud is None:
        baud = int(os.getenv("RADAR_BAUD", "1250000"))

    ser = serial.Serial(port, baudrate=baud, timeout=0.2)
    print(f"[reader] Listening for heatmap on {port} @ {baud}")

    buffer = bytearray()
    alpha = float(os.getenv("HEATMAP_BG_ALPHA", "0.05"))
    background_avg = np.zeros((NUM_RANGE_BINS, NUM_AZIMUTH_BINS), dtype=np.float32)

    while True:
        bytecount = ser.in_waiting
        if bytecount > 0:
            buffer.extend(ser.read(bytecount))

        magic_index = buffer.find(MAGIC_WORD)
        if magic_index == -1:
            # prevent unbounded growth if sync is lost
            if len(buffer) > 2_000_000:
                del buffer[:-64]
            continue

        if magic_index > 0:
            del buffer[:magic_index]

        if len(buffer) < 40:
            continue

        # frame header = 40 bytes; totalPacketLen = bytes 12:16
        total_packet_len = int.from_bytes(buffer[12:16], byteorder="little")
        num_tlvs = int.from_bytes(buffer[32:36], byteorder="little")

        if total_packet_len <= 0:
            # drop magic word
            del buffer[:8]
            continue

        if len(buffer) < total_packet_len:
            continue

        tlv_offset = 40
        for _ in range(num_tlvs):
            tlv_type = int.from_bytes(buffer[tlv_offset : tlv_offset + 4], "little")
            tlv_length = int.from_bytes(buffer[tlv_offset + 4 : tlv_offset + 8], "little")
            tlv_data = buffer[tlv_offset + 8 : tlv_offset + 8 + tlv_length]
            tlv_offset += 8 + tlv_length

            if tlv_type != HEATMAP_TLV:
                continue

            if len(tlv_data) < HEATMAP_BYTES:
                continue

            current_frame = np.frombuffer(tlv_data[:HEATMAP_BYTES], dtype=np.uint32).reshape(
                NUM_RANGE_BINS, NUM_AZIMUTH_BINS
            )
            current_frame = current_frame.astype(np.float32)

            # background subtraction
            background_avg = ((1.0 - alpha) * background_avg) + (alpha * current_frame)
            heatmap_2d = np.maximum(current_frame - background_avg, 0)

            # keep most recent
            try:
                heatmap_queue.put_nowait(heatmap_2d)
            except pyqueue.Full:
                with suppress(pyqueue.Empty):
                    _ = heatmap_queue.get_nowait()
                with suppress(pyqueue.Full):
                    heatmap_queue.put_nowait(heatmap_2d)

        # Consume full frame
        del buffer[:total_packet_len]


def predict_loop():
    model = _load_model()
    class_names = ["sitting", "standing"]

    print("[reader] ML inference loop started")

    while True:
        heatmap_raw = heatmap_queue.get()

        with torch.no_grad():
            input_tensor = torch.from_numpy(heatmap_raw).to(dtype=torch.float32)
            t_min, t_max = input_tensor.min(), input_tensor.max()
            input_tensor = (input_tensor - t_min) / (t_max - t_min + 1e-8)
            input_tensor = input_tensor.view(1, 1, NUM_RANGE_BINS, NUM_AZIMUTH_BINS)

            logits = model(input_tensor)
            p_idx = int(torch.argmax(logits, dim=1).item())
            status = class_names[p_idx] if 0 <= p_idx < len(class_names) else "unknown"

        # always keep newest prediction
        msg = {"status": status, "ml_idx": p_idx}
        try:
            queue.put_nowait(msg)
        except pyqueue.Full:
            with suppress(Exception):
                _ = queue.get_nowait()
            with suppress(Exception):
                queue.put_nowait(msg)
