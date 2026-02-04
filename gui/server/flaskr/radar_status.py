import os
import time
from flask_socketio import emit

from . import reader
from . import socketio, queue

background_task_started = False


def background_thread():
    global queue

    uptime = 0
    activity_log = []
    prev_status = "unknown"
    prev_ml_idx = None
    start_time = time.time()
    while True:
        # ML inference dict format: {"status": "sitting"|"standing", ...}
        status_update = None
        # Drain queue to most recent update
        while not queue.empty():
            status_update = queue.get_nowait()

        status = prev_status
        ml_idx = prev_ml_idx
        if isinstance(status_update, dict):
            status = status_update.get("status", prev_status)
            ml_idx = status_update.get("ml_idx", prev_ml_idx)
        elif isinstance(status_update, str):
            status = status_update
        people_count = 1

        if status != prev_status:
            activity_log.append(
                {
                    "time": time.strftime("%H:%M:%S", time.gmtime()),
                    "event": f"Status changed: {prev_status} → {status}",
                }
            )
            prev_status = status
            prev_ml_idx = ml_idx

        uptime = int(time.time() - start_time)

        socketio.emit(
            "radar_status_update",
            {
                "updates": {
                    "radar_id": 1,
                    "is_connected": True,
                    "status": status,
                    "ml_idx": ml_idx,
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
        socketio.start_background_task(background_thread)

        # send cfg and start streaming automatically on first GUI connect
        radar_port = os.getenv("RADAR_PORT", "COM5")
        socketio.start_background_task(reader.start_pipeline_with_config, radar_port)
        socketio.start_background_task(reader.predict_loop)
        background_task_started = True


@socketio.on("disconnect")
def test_disconnect():
    print("Client disconnected")
