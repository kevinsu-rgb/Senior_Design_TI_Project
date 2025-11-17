from flask import Blueprint

bp = Blueprint('radar', __name__, url_prefix='/api/radar')

@bp.route('/list', methods=['GET'])
def radar_list():
    radar_ids = [1]
    radars = []
    for radar_id in radar_ids:
        radars.append({
            "radar_id": radar_id,
            "name": "Radar " + chr(64 + radar_id),
            "timestamp": "2024-06-01T12:00:00Z",
        })

    return {"radars": radars}