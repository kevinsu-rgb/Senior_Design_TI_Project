from flask import Blueprint

from . import radar_status

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
            "status": "unknown",
            "fault_latched": False,
        })

    return {"radars": radars}


@bp.route('/<int:radar_id>/clear-fault', methods=['POST'])
def clear_fault(radar_id):
    body, status_code = radar_status.clear_fault(radar_id)
    return body, status_code