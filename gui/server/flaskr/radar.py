from flask import Blueprint

from . import discover_pi
from . import radar_status

bp = Blueprint('radar', __name__, url_prefix='/api/radar')

@bp.route('/list', methods=['GET'])
def radar_list():
    devices = discover_pi.discover_pis(timeout_s=1.0)
    radars = []
    for i, dev in enumerate(devices):
        radars.append({
            "radar_id": i + 1,
            "name": dev.get("name", "unknown"),
            "radar_ip": dev.get("ip"),
            "tcp_port": dev.get("tcp_port"),
            "timestamp": "2024-06-01T12:00:00Z",
            "status": "unknown",
            "fault_latched": False,
        })
    print(f"Discovered radars: {radars}", flush=True)
    return {"radars": radars}


@bp.route('/<string:radar_ip>/clear-fault', methods=['POST'])
def clear_fault(radar_ip):
    body, status_code = radar_status.clear_fault(radar_ip)
    return body, status_code

@bp.route('/name', methods=['POST'])
def set_name():
    from flask import request
    data = request.get_json()
    radar_status.set_name(data.get("name"), data.get("radar_ip"))
    return {"ok": True, "message": f"Name set to '{data.get('name')}'"}