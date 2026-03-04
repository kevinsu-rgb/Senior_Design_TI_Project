import os
from pathlib import Path
import threading
import time
from flask_socketio import emit

from . import bt_status
from . import socketio, status_queue

background_task_started = False
state_lock = threading.Lock()
start_time = time.time()
latest_live_status = "unknown"
display_status = "unknown"
previous_logged_status = "unknown"
fault_latched = False
activity_log = []


def _now_hms():
    return time.strftime("%H:%M:%S", time.gmtime())


def _uptime_string():
    uptime = int(time.time() - start_time)
    return f"{uptime//86400}d {(uptime%86400)//3600}h {(uptime%3600)//60}m {uptime%60}s"


def _build_update_payload(radar_id: int):
    with state_lock:
        return {
            "radar_id": radar_id,
            "is_connected": True,
            "status": display_status,
            "people_count": 1,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "activity_log": activity_log[-5:],
            "uptime": _uptime_string(),
            "fault_latched": fault_latched,
        }


def clear_fault(radar_id: int):
    global fault_latched, display_status, previous_logged_status

    if radar_id != 1:
        return {"ok": False, "message": "Radar not found"}, 404

    with state_lock:
        fault_latched = False
        display_status = latest_live_status
        previous_logged_status = display_status
        activity_log.append({"time": _now_hms(), "event": "Fault cleared"})

    socketio.emit("radar_status_update", {"updates": _build_update_payload()})
    return {"ok": True, "fault_latched": False}, 200


def background_thread():
    global status_queue
    global latest_live_status, display_status, previous_logged_status, fault_latched
    
    while True:
        newest_status = None
        while not status_queue.empty():
            item = status_queue.get_nowait()
            # New format: (ip, evt)
            if isinstance(item, tuple) and len(item) >= 2:
                ip = item[0]
                evt = item[1]
                if isinstance(evt, dict):
                    newest_status = evt.get("status")
                elif isinstance(evt, str):
                    newest_status = evt
                else:
                    newest_status = None

                if newest_status is not None:
                    newest_status = str(newest_status).lower()

                continue

            # Legacy format: plain string
            if isinstance(item, str):
                newest_status = item.lower()


        if newest_status is not None:
            with state_lock:
                latest_live_status = newest_status
                if not fault_latched:
                    if newest_status != previous_logged_status:
                        activity_log.append(
                            {
                                "time": _now_hms(),
                                "event": f"Status changed: {previous_logged_status} → {newest_status}",
                            }
                        )
                        previous_logged_status = newest_status

                    display_status = newest_status
                    if newest_status == "falling":
                        fault_latched = True

        socketio.emit(
            "radar_status_update",
            # TODO: use radarid.
            {"updates": _build_update_payload(1)},
        )
        # Emit at a stable cadence so clients do not get flooded.
        socketio.sleep(1)


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
