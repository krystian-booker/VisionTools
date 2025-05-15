import os
import threading

from web import create_app
from orchestrator.manager import start_roscore, start_rosbridge, start_flir, start_video_server

app = create_app()

def start_orchestrator():
    """
    Launch orchestrator services once at startup.
    """
    services = [
        ('roscore', start_roscore),
        ('rosbridge', start_rosbridge),
        ('flir', start_flir),
        ('video_server', start_video_server)
    ]
    for name, fn in services:
        success, msg = fn()
        print(f"{name}: {msg}")

if __name__ == '__main__':
    # Start orchestrator services in a background daemon thread
    orchestrator_thread = threading.Thread(target=start_orchestrator, daemon=True)
    orchestrator_thread.start()

    # Launch Flask app
    port = int(os.getenv('PORT', 3000))
    app.run(host='0.0.0.0', port=port)
