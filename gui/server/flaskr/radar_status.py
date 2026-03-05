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

# Per-radar state keyed by radar_ip so multiple radars don't overwrite each other.
# Each value contains only what the UI needs.
radars_state = {}


def _now_hms():
    return time.strftime("%H:%M:%S", time.gmtime())


def _uptime_string() -> str:
    uptime = int(time.time() - start_time)
    return f"{uptime//86400}d {(uptime%86400)//3600}h {(uptime%3600)//60}m {uptime%60}s"


def _get_radar_state(radar_ip: str) -> dict:
    # Caller must hold state_lock.
    st = radars_state.get(radar_ip)
    if st is None:
        st = {
            "latest_live_status": "unknown",
            "display_status": "unknown",
            "previous_logged_status": "unknown",
            "fault_latched": False,
            "activity_log": [],
            "people_count": 1,
        }
        radars_state[radar_ip] = st
    return st


def _build_update_payload(radar_ip: str) -> dict:
    # Caller needs to hold state_lock.
    st = _get_radar_state(radar_ip)
    return {
        "radar_ip": radar_ip,
        "is_connected": True,
        "status": st.get("display_status", "unknown"),
        "people_count": int(st.get("people_count", 0)),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "activity_log": st.get("activity_log", [])[-5:],
        "uptime": _uptime_string(),
        "fault_latched": bool(st.get("fault_latched", False)),
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

# TODO: uptime doesnt update with this logic, maybe put uptime on the client side and just send server start time?
# also need to handle radar going offline
def background_thread():
    global status_queue

    while True:

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

            if isinstance(evt, dict):
                newest_status = evt.get("status")
            elif isinstance(evt, str):
                newest_status = evt
            else:
                newest_status = None

            if newest_status is not None:
                newest_status = str(newest_status).lower()
            if newest_status is None:
                continue

            with state_lock:
                st = _get_radar_state(ip)

                log.info(f"Processing status for {ip}: newest_status={newest_status}, display_status={st.get('display_status')}, fault_latched={st.get('fault_latched')}")

                if newest_status == st.get("latest_live_status"):
                    # No change, skip.
                    continue
                else:
                    st["latest_live_status"] = newest_status

                    if st.get("fault_latched", False):
                        # If fault is latched, display status does not change until cleared.
                        continue
                    else:
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
                 discover_pi._stream_all,
                 discover_pi.discover_pis(timeout_s=1.0),
                    status_queue,
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
