#!/usr/bin/env python3

import subprocess
import time
import sys
import os

# Dependency check
try:
    from flask import Flask, jsonify
except ImportError:
    print("Error: Flask is not installed. Install it with: pip3 install flask", file=sys.stderr)
    sys.exit(1)

app = Flask(__name__)
processes = {}
WEB_APP_DIR = os.path.join(os.path.dirname(__file__), 'web')  # adjust if your web folder is elsewhere


def start_roscore():
    if 'roscore' in processes and processes['roscore'].poll() is None:
        return False, "roscore already running"
    p = subprocess.Popen(['roscore'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    processes['roscore'] = p
    time.sleep(2)
    if p.poll() is not None:
        return False, "Failed to start roscore"
    return True, "roscore started"


def stop_roscore():
    p = processes.get('roscore')
    if not p or p.poll() is not None:
        return False, "roscore not running"
    p.terminate()
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        p.kill()
    return True, "roscore stopped"


def start_rosbridge():
    if 'rosbridge' in processes and processes['rosbridge'].poll() is None:
        return False, "rosbridge already running"
    p = subprocess.Popen(
        ['roslaunch', 'rosbridge_server', 'rosbridge_websocket.launch'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    processes['rosbridge'] = p
    time.sleep(1)
    if p.poll() is not None:
        return False, "Failed to start rosbridge"
    return True, "rosbridge started"


def stop_rosbridge():
    p = processes.get('rosbridge')
    if not p or p.poll() is not None:
        return False, "rosbridge not running"
    p.terminate()
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        p.kill()
    return True, "rosbridge stopped"


def start_flir():
    if 'flir' in processes and processes['flir'].poll() is None:
        return False, "FLIR node already running"
    p = subprocess.Popen(
        ['roslaunch', 'flir_camera_node', 'flir_cameras.launch'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    processes['flir'] = p
    time.sleep(1)
    if p.poll() is not None:
        return False, "Failed to start FLIR node"
    return True, "FLIR node started"


def stop_flir():
    p = processes.get('flir')
    if not p or p.poll() is not None:
        return False, "FLIR node not running"
    p.terminate()
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        p.kill()
    return True, "FLIR node stopped"


def start_web():
    if 'web' in processes and processes['web'].poll() is None:
        return False, "Web server already running"
    # Assumes 'npm start' in WEB_APP_DIR
    p = subprocess.Popen(
        ['sudo', '-E', 'npm', 'start'],
        cwd=WEB_APP_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    processes['web'] = p
    time.sleep(1)
    if p.poll() is not None:
        return False, "Failed to start web server"
    return True, "Web server started"


def stop_web():
    p = processes.get('web')
    if not p or p.poll() is not None:
        return False, "Web server not running"
    p.terminate()
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        p.kill()
    return True, "Web server stopped"


# HTTP Endpoints
@app.route('/start_roscore', methods=['POST'])
def http_start_roscore():
    ok, msg = start_roscore()
    return jsonify({'success': ok, 'message': msg})

@app.route('/stop_roscore', methods=['POST'])
def http_stop_roscore():
    ok, msg = stop_roscore()
    return jsonify({'success': ok, 'message': msg})

@app.route('/start_rosbridge', methods=['POST'])
def http_start_rosbridge():
    ok, msg = start_rosbridge()
    return jsonify({'success': ok, 'message': msg})

@app.route('/stop_rosbridge', methods=['POST'])
def http_stop_rosbridge():
    ok, msg = stop_rosbridge()
    return jsonify({'success': ok, 'message': msg})

@app.route('/start_flir', methods=['POST'])
def http_start_flir():
    ok, msg = start_flir()
    return jsonify({'success': ok, 'message': msg})

@app.route('/stop_flir', methods=['POST'])
def http_stop_flir():
    ok, msg = stop_flir()
    return jsonify({'success': ok, 'message': msg})

@app.route('/start_web', methods=['POST'])
def http_start_web():
    ok, msg = start_web()
    return jsonify({'success': ok, 'message': msg})

@app.route('/stop_web', methods=['POST'])
def http_stop_web():
    ok, msg = stop_web()
    return jsonify({'success': ok, 'message': msg})

@app.route('/status', methods=['GET'])
def http_status():
    status = {name: ('running' if p and p.poll() is None else 'stopped')
              for name, p in processes.items()}
    return jsonify(status)

if __name__ == '__main__':
    # Startup sequence
    for fn, label in [(start_roscore, "roscore"),
                      (start_rosbridge, "rosbridge"),
                      (start_flir, "FLIR node"),
                      (start_web, "web server")]:
        print(f"Starting {label}...")
        ok, msg = fn()
        print(msg)

    # Launch REST API
    app.run(host='0.0.0.0', port=5000)

    # Cleanup on exit
    for name, p in processes.items():
        if p and p.poll() is None:
            p.terminate()
