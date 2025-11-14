from flask import Blueprint

bp = Blueprint('radar', __name__, url_prefix='/api/radar')

@bp.route('/activity/<id>', methods=['GET'])
def radar_activity(id):
    return {
            "activity": [
                { "time": "12:30:30", "event": "Fall detected" },
    		    { "time": "12:12:12", "event": "People detected: 1 → 2" },
    		    { "time": "11:11:11", "event": "People detected: 0 → 1" },
    		    { "time": "10:10:10", "event": "People detected: 1 → 0" },
    		    { "time": "00:00:30", "event": "Heartbeat received" }
            ],
        }

@bp.route('/list', methods=['GET'])
def radar_list():
    radar_ids = [1, 3, 5, 6]
    radars = []
    for radar_id in radar_ids:
        radars.append({
            "radar_id": radar_id,
            "is_connected": True,
            "name": "Radar " + chr(64 + radar_id),
            "status": "standing",
            "people_count": "2",
            "timestamp": "2024-06-01T12:00:00Z",
            "uptime": "30d 12h 30m 30s",
        })

    return {"radars": radars}