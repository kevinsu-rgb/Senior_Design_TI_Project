import json
import queue
import threading
from pathlib import Path
from typing import Any, Dict, Optional

CONFIG_PATH = Path("radar_config.json")

_command_q: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=1_000)
_command_stop = threading.Event()
_command_thread: Optional[threading.Thread] = None
_command_lock = threading.Lock()
_runtime_cfg = None


def load_config():
    if not CONFIG_PATH.exists():
        return {}
    with CONFIG_PATH.open("r", encoding="utf-8") as f:
        return json.load(f)



def save_config(data):
    with CONFIG_PATH.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)



def _process_command(cmd: Dict[str, Any]) -> None:
    global _runtime_cfg

    if not isinstance(cmd, dict):
        return

    cmd_type = cmd.get("type")
    if cmd_type == "update_radar_identity":
        config = load_config()
        if not isinstance(config, dict):
            config = {}

        radar_name = cmd.get("radar_name")
        radar_id = cmd.get("radar_id")

        if isinstance(radar_name, str) and radar_name.strip():
            radar_name = radar_name.strip()
            config["radar_name"] = radar_name
            if _runtime_cfg is not None:
                _runtime_cfg.name = radar_name

        if isinstance(radar_id, str) and radar_id.strip():
            config["radar_id"] = radar_id.strip()

        save_config(config)
        return

    save_config(cmd)



def _command_loop() -> None:
    while not _command_stop.is_set():
        try:
            cmd = _command_q.get(timeout=0.5)
        except queue.Empty:
            continue

        try:
            _process_command(cmd)
        except Exception as e:
            print(f"Failed to process command: {e}")



def start_command_worker(runtime_cfg=None) -> None:
    global _command_thread, _runtime_cfg

    with _command_lock:
        _runtime_cfg = runtime_cfg

        if _command_thread is not None and _command_thread.is_alive():
            return

        _command_stop.clear()
        _command_thread = threading.Thread(target=_command_loop, daemon=True)
        _command_thread.start()



def stop_command_worker() -> None:
    _command_stop.set()



def enqueue_command(cmd: Dict[str, Any]) -> bool:
    try:
        _command_q.put_nowait(cmd)
        return True
    except queue.Full:
        return False


if __name__ == "__main__":
    print("Starting device_config command worker...")

    start_command_worker()

    try:
        while True:
            # Keep process alive to handle incoming commands
            threading.Event().wait(1.0)
    except KeyboardInterrupt:
        print("Stopping device_config command worker...")
        stop_command_worker()