from flask_socketio import emit
import random
import time

from . import reader
from . import socketio, queue

background_task_started = False


def background_thread():
    global queue

    uptime = 0
    activity_log = []
    prev_status = "falling"
    start_time = time.time()
    while True:
        # TODO: HOOK UP THE NEW READ.PY WITH ML MODEL WITH THIS QUEUE
        status = queue.get_nowait() if not queue.empty() else prev_status
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

        socketio.sleep(0.5)


@socketio.on("connect")
def test_connect():
    global background_task_started
    emit("my response", {"data": "Connected"})

    if not background_task_started:
        # ignore this absolute path the paths are weird in electron, we can probably just put this in a .env or something
        reader.send_cfg(
            "COM7",
            "C:/Users/jaidenmagnan/github/Senior_Design_TI_Project/gui/server/flaskr/configs/AOP_6m_default.cfg",
            "COM8",
        )
        socketio.start_background_task(background_thread)
        socketio.start_background_task(reader.read_uart, "COM7", "COM8")
        background_task_started = True


@socketio.on("disconnect")
def test_disconnect():
    print("Client disconnected")
