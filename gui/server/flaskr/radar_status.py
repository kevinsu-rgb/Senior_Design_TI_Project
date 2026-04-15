import os
from pathlib import Path
import threading
import time
from flask_socketio import emit

from . import bt_status
from . import socketio, status_queue

import logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

background_task_started = False
state_lock = threading.Lock()
start_time = time.time()
OFFLINE_TIMEOUT_S = 5.0

# Per-radar state keyed by radar_ip so multiple radars don't overwrite each other.
# Each value contains only what the UI needs.
radars_state = {}

# command to send to radars, set by UI and read by discover_pi when connecting/streaming
external_command = None
external_command_lock = threading.Lock()


def _now_hms():
    return time.strftime("%H:%M:%S", time.localtime())


def _get_radar_state(radar_ip: str) -> dict:
    # Caller must hold state_lock.
    st = radars_state.get(radar_ip)
    if st is None:
        st = {
            "name" : "Radar " + radar_ip,
            "latest_live_status": "unknown",
            "display_status": "unknown",
            "previous_logged_status": "unknown",
            "fault_latched": False,
            "activity_log": [],
            "people_count": 1,
            "is_connected": False,
            "last_packet_time": 0.0,
            "connected_since": None,
        }
        radars_state[radar_ip] = st
    return st

def set_external_command(cmd: str | None):
    global external_command
    with external_command_lock:
        external_command = cmd

def get_external_command():
    with external_command_lock:
        return external_command

def _build_update_payload(radar_ip: str) -> dict:
    # Caller needs to hold state_lock.
    st = _get_radar_state(radar_ip)
    is_connected = bool(st.get("is_connected", False))
    return {
        "name": st.get("name", "Radar " + radar_ip),
        "radar_ip": radar_ip,
        "is_connected": is_connected,
        "status": st.get("display_status", "unknown") if is_connected else "offline",
        "people_count": int(st.get("people_count", 0)),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
        "activity_log": st.get("activity_log", [])[-5:],
        "fault_latched": bool(st.get("fault_latched", False)),
        "server_start_time": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(start_time)),
        "connected_since": st.get("connected_since"),
    }


def clear_fault(radar_ip: str):
    with state_lock:
        if radar_ip not in radars_state:
            return {"ok": False, "message": "Radar not found"}, 404

        st = _get_radar_state(radar_ip)
        st["fault_latched"] = False
        st["display_status"] = st.get("latest_live_status")
        st["previous_logged_status"] = st["display_status"]
        st.setdefault("activity_log", []).append({"time": _now_hms(), "event": "Fault cleared"})

    socketio.emit("radar_status_update", {"updates": _build_update_payload(radar_ip)})
    return {"ok": True, "fault_latched": False}, 200

# also need to handle radar going offline
def background_thread():
    global status_queue

    while True:

        with state_lock:
            now = time.time()
            for radar_ip, st in radars_state.items():
                last_packet_time = float(st.get("last_packet_time", 0.0))
                is_connected = bool(st.get("is_connected", False))

                if is_connected and last_packet_time and (now - last_packet_time > OFFLINE_TIMEOUT_S):
                    st["is_connected"] = False
                    st.setdefault("activity_log", []).append({"time": _now_hms(), "event": "Radar disconnected"})
                    socketio.emit("radar_status_update", {"updates": _build_update_payload(radar_ip)})

        # Drain the queue and update per-radar state.
        while not status_queue.empty():
            item = status_queue.get_nowait()

            ip = None
            evt = None

            # New format: (ip, evt)
            if isinstance(item, tuple) and len(item) >= 2:
                ip = item[0]
                evt = item[1]

                log.info(f"Received status update from {ip}: {evt}")
                
            else:
                continue

            if not ip:
                continue

            newest_status = None
            name = None
            is_heartbeat = False

            if isinstance(evt, dict):
                if evt.get("type") == "heartbeat":
                    is_heartbeat = True
                    newest_status = None
                    name = evt.get("name")
                else:
                    newest_status = evt.get("status")
                    name = evt.get("name")
            elif isinstance(evt, str):
                newest_status = evt
            else:
                newest_status = None

            if newest_status is not None:
                newest_status = str(newest_status).lower()

            with state_lock:
                st = _get_radar_state(ip)
                was_connected = bool(st.get("is_connected", False))
                st["last_packet_time"] = time.time()
                
                
                if name is not None:
                    st["name"] = name

                if not was_connected:
                    st["is_connected"] = True
                    st["connected_since"] = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(st["last_packet_time"]))
                    st.setdefault("activity_log", []).append({"time": _now_hms(), "event": "Radar connected"})

                log.info(f"Processing status for {name}: newest_status={newest_status}, display_status={st.get('display_status')}, fault_latched={st.get('fault_latched')}")

                if is_heartbeat:
                    socketio.emit("radar_status_update", {"updates": _build_update_payload(ip)})
                    continue

                if newest_status is None:
                    continue

                previous_live_status = st.get("latest_live_status")
                st["latest_live_status"] = newest_status

                if not st.get("fault_latched", False):
                    st["display_status"] = newest_status

                    if newest_status != st["previous_logged_status"]:
                        st["activity_log"].append(
                            {
                                "time": _now_hms(),
                                "event": f"Status changed: {st['previous_logged_status']} → {newest_status}",
                            }
                        )
                        st["previous_logged_status"] = newest_status

                    if newest_status == "falling":
                        st["fault_latched"] = True

                elif newest_status != previous_live_status:
                    log.info(f"Fault latched for {ip}; keeping display_status={st.get('display_status')} while latest_live_status={newest_status}")

                socketio.emit("radar_status_update", {"updates": _build_update_payload(ip)})

        socketio.sleep(0.05)  # Sleep briefly to avoid busy loop when queue is empty.
                


@socketio.on("connect")
def test_connect():
    print("Client connected")
    global background_task_started
    emit("my response", {"data": "Connected"})

    if not background_task_started:
        socketio.start_background_task(background_thread)
        status_source = os.getenv("RADAR_STATUS_SOURCE", "tcp").lower().strip()

        if status_source == "bluetooth":
            bt_com_port = os.getenv("BT_STATUS_COM_PORT", "COM8")
            bt_baud = int(os.getenv("BT_STATUS_BAUD", "115200"))
            socketio.start_background_task(
                bt_status.read_statuses_from_serial,
                bt_com_port,
                bt_baud,
                status_queue,
            )
        elif status_source == "tcp":
            from . import discover_pi
            socketio.start_background_task(
                discover_pi.stream_with_periodic_discovery,
                status_queue,
                discovery_interval_s=2.0,
                command=get_external_command(),
            )

        else:
            from . import reader

            radar_port = os.getenv("RADAR_PORT", "/dev/ttyACM0")
            path = Path(__file__).resolve().parent / "configs" / "config.cfg"
            reader.send_cfg(path, 115200, radar_port, radar_port)
            socketio.start_background_task(reader.read_uart, "", radar_port, 1250000)
            socketio.start_background_task(reader.predict)
        background_task_started = True


@socketio.on("disconnect")
def test_disconnect():
    print("Client disconnected")
