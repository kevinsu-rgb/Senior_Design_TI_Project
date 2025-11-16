from flask_socketio import emit
import random
import time
from . import socketio

background_task_started = False

def background_thread():
    uptime = 0
    activity_log = [{}]
    prev_status = 'falling'
    while True:
        status = random.choice(['standing', 'falling'])
        people_count = random.randint(0, 5) if status == 'standing' else 1
        uptime += 1

        if status != prev_status:
            activity_log.append({
                "time": time.strftime("%H:%M:%S", time.gmtime()),
                "event": f"Status changed: {prev_status} → {status}"
            })
            prev_status = status
        
        socketio.emit('radar_status_update',{
            "updates": {
                "radar_id": 1,
                "is_connected": True,
                "status": status,
                "people_count": people_count,
                "timestamp": "2024-06-01T12:00:00Z",
                "activity_log": activity_log[-5:],
                "uptime": f"{uptime//86400}d {(uptime%86400)//3600}h {(uptime%3600)//60}m {uptime%60}s",
            }
        })
        
        socketio.sleep(1) 

@socketio.on('connect')
def test_connect():
    global background_task_started
    emit('my response', {'data': 'Connected'})
    
    if not background_task_started:
        socketio.start_background_task(background_thread)
        background_task_started = True

@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected')


