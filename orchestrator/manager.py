# orchestrator/manager.py

import subprocess
import time
import socket
import os
from pathlib import Path

# Config location
config_home = Path(os.getenv("XDG_CONFIG_HOME", Path.home() / ".config"))
config_dir  = config_home / "frcVisionTools"
config_dir.mkdir(parents=True, exist_ok=True)
CAMERAS_FILE = config_dir / "cameras.yaml"

# keep track of running subprocesses
processes = {}

# point this at your static/web entrypoint
WEB_APP_DIR = Path(__file__).parent.parent / 'web'

def get_local_ip():
    """Return first non-internal IPv4 or 'localhost'."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        return s.getsockname()[0]
    except Exception:
        return 'localhost'
    finally:
        s.close()

def _launch(name, cmd, cwd=None, delay=1):
    """Helper to start a subprocess if not already running."""
    if name in processes and processes[name].poll() is None:
        return False, f"{name} already running"
    p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    processes[name] = p
    time.sleep(delay)
    if p.poll() is not None:
        return False, f"Failed to start {name}"
    return True, f"{name} started"

def _terminate(name, timeout=5):
    """Helper to stop a subprocess and wait (up to timeout) for it to actually exit."""
    p = processes.get(name)
    if not p or p.poll() is not None:
        return False, f"{name} not running"

    p.terminate()
    try:
        p.wait(timeout=timeout)
        # cleanup entry so launch can reuse the slot
        del processes[name]
        return True, f"{name} stopped"
    except subprocess.TimeoutExpired:
        p.kill()
        del processes[name]
        return False, f"{name} did not stop gracefully"

def start_roscore():
    return _launch('roscore', ['roscore'], delay=2)

def stop_roscore():
    return _terminate('roscore')

def start_rosbridge():
    return _launch('rosbridge',
                   ['roslaunch', 'rosbridge_server', 'rosbridge_websocket.launch'])

def stop_rosbridge():
    return _terminate('rosbridge')

def start_flir():
    return _launch('flir', [
        'roslaunch',
        'flir_camera_node',
        'flir_cameras.launch',
        f'cameras_file:={CAMERAS_FILE}'
    ])

def stop_flir():
    return _terminate('flir', timeout=5)

def restart_flir():
    stopped, stop_msg = stop_flir()
    # ignore “not running” errors
    if not stopped and "not running" in stop_msg:
        stop_msg = f"{stop_msg} (okay)"
    elif not stopped:
        # some other failure: include it in the message but still try to start
        stop_msg = f"stop error: {stop_msg}"

    started, start_msg = start_flir()
    # overall success = whether start succeeded
    combined_msg = {'stop': stop_msg, 'start': start_msg}
    return started, combined_msg

def start_video_server(port: int = 8080,
                       address: str = '0.0.0.0',
                       width: int = 640,
                       height: int = 480,
                       fps: int = 30,
                       quality: int = 50,
                       transport: str = 'compressed'):
    """
    Launch web_video_server with sensible defaults for UI-friendly streaming:
      - compressed transport (so we don’t hit the Image vs. CompressedImage MD5 mismatch)
      - 640×480 @ 30 fps at 50% JPEG quality
    """
    cmd = [
        'rosrun', 'web_video_server', 'web_video_server',
        f'_port:={port}',
        f'_address:={address}',
        # depending on your version it may be _default_transport
        f'_default_transport:={transport}',
        f'_width:={width}',
        f'_height:={height}',
        f'_fps:={fps}',
        f'_quality:={quality}',
    ]
    return _launch('video_server', cmd)


def stop_video_server():
    """
    Terminate the web_video_server subprocess if it is running.
    """
    return _terminate('video_server')

def restart_video_server(port: int = 8080, address: str = '0.0.0.0'):
    """
    Helper to restart the video server, returning a dict with 'stop' and 'start' messages.
    """
    stopped, stop_msg = stop_video_server()
    if not stopped and "not running" in stop_msg:
        stop_msg = f"{stop_msg} (okay)"
    elif not stopped:
        stop_msg = f"stop error: {stop_msg}"

    started, start_msg = start_video_server(port, address)
    return started, {'stop': stop_msg, 'start': start_msg}

def get_status():
    """Return a dict of {service_name: 'running'|'stopped'}."""
    return {
        name: ('running' if p and p.poll() is None else 'stopped')
        for name, p in processes.items()
    }
