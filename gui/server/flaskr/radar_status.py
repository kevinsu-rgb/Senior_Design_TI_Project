import os
from pathlib import Path
import time
from flask_socketio import emit

from . import bt_status
from . import socketio, status_queue

background_task_started = False


def background_thread():
    global status_queue

    uptime = 0
    activity_log = []
    prev_status = "Unknown"
    start_time = time.time()
    status = "Unknown"

    fallStarted = True
    i = 0
    while True:
        while not status_queue.empty():
            status = status_queue.get_nowait().lower()

        people_count = 1
        if status != prev_status:
            activity_log.append(
                {
                    "time": time.strftime("%H:%M:%S", time.gmtime()),
                    "event": f"Status changed: {prev_status} → {status}",
                }
            )
            prev_status = status

        uptime = int(time.time() - start_time)

        socketio.emit(
            "radar_status_update",
            {
                "updates": {
                    "radar_id": 1,
                    "is_connected": True,
                    "status": status,
                    "people_count": people_count,
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "activity_log": activity_log[-5:],
                    "uptime": f"{uptime//86400}d {(uptime%86400)//3600}h {(uptime%3600)//60}m {uptime%60}s",
                }
            },
        )
        socketio.sleep(0)


@socketio.on("connect")
def test_connect():
    global background_task_started
    emit("my response", {"data": "Connected"})

    if not background_task_started:
        socketio.start_background_task(background_thread)
        status_source = os.getenv("RADAR_STATUS_SOURCE", "local").lower().strip()

        if status_source == "bluetooth":
            bt_com_port = os.getenv("BT_STATUS_COM_PORT", "COM7")
            bt_baud = int(os.getenv("BT_STATUS_BAUD", "115200"))
            socketio.start_background_task(
                bt_status.read_statuses_from_serial,
                bt_com_port,
                bt_baud,
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
