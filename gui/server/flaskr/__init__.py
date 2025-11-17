import os

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO
import queue

socketio = SocketIO()

queue = queue.Queue()

def create_app(test_config=None):
    app = Flask(__name__, instance_relative_config=True)
    app.config.from_mapping(
        SECRET_KEY='dev',
        DATABASE=os.path.join(app.instance_path, 'flaskr.sqlite'),
    )

    CORS(app)

    from . import db
    db.init_app(app)

    from . import radar
    app.register_blueprint(radar.bp)

    socketio.init_app(app, cors_allowed_origins="*")
    from . import radar_status

    if test_config is None:
        app.config.from_pyfile('config.py', silent=True)
    else:
        app.config.from_mapping(test_config)

    try:
        os.makedirs(app.instance_path)
    except OSError:
        pass

    return app