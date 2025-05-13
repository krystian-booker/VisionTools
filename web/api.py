# web/api.py
import os
from flask import Blueprint, jsonify, request, current_app
from pathlib import Path
import yaml

# Ensure specific strings are always double-quoted in YAML
class QuotedString(str): pass

def quoted_str_representer(dumper, data):
    return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='"')

yaml.add_representer(QuotedString, quoted_str_representer)

api_bp = Blueprint('api', __name__)

# Path to YAML config
config_home = Path(os.getenv("XDG_CONFIG_HOME", Path.home() / ".config"))
config_dir  = config_home / "frcVisionTools"
config_dir.mkdir(parents=True, exist_ok=True)
CAMERAS_FILE = config_dir / "cameras.yaml"

def ensure_dir_exists():
    CAMERAS_FILE.parent.mkdir(parents=True, exist_ok=True)

@api_bp.route('/cameras', methods=['GET'])
def get_cameras():
    try:
        ensure_dir_exists()
        if not CAMERAS_FILE.exists():
            default = {'camera_config': {'cameras': []}}
            CAMERAS_FILE.write_text(yaml.dump(default), encoding='utf-8')
        contents = CAMERAS_FILE.read_text(encoding='utf-8')
        data = yaml.safe_load(contents)
        return jsonify(data)
    except Exception as e:
        current_app.logger.error(f"Error loading cameras.yaml: {e}")
        return jsonify(error='Could not load camera config'), 500

@api_bp.route('/cameras', methods=['POST'])
def add_camera():
    data = request.get_json() or {}
    serial     = data.get('serial')
    role       = data.get('role')
    video_mode = data.get('video_mode')
    name       = data.get('name', '')
    framerate  = data.get('framerate', 30.0)
    exposure   = data.get('exposure', 9996.41)
    gain       = data.get('gain', 18.4)

    if not serial or not role or not video_mode:
        return jsonify(error='serial, role and video_mode are required'), 400

    try:
        ensure_dir_exists()
        if not CAMERAS_FILE.exists():
            CAMERAS_FILE.write_text(
                yaml.dump({'camera_config': {'cameras': []}}),
                encoding='utf-8'
            )

        contents = CAMERAS_FILE.read_text(encoding='utf-8')
        cfg = yaml.safe_load(contents) or {}
        cfg.setdefault('camera_config', {}).setdefault('cameras', [])

        new_cam = {
            'serial':     QuotedString(str(serial)),
            'name':       QuotedString(str(name)),
            'role':       QuotedString(str(role)),
            'video_mode': QuotedString(str(video_mode)),
            'framerate':  float(framerate),
            'exposure':   float(exposure),
            'gain':       float(gain),
        }
        cfg['camera_config']['cameras'].append(new_cam)
        CAMERAS_FILE.write_text(yaml.dump(cfg), encoding='utf-8')
        return jsonify(new_cam), 201

    except Exception as e:
        current_app.logger.error(f"Error saving new camera: {e}")
        return jsonify(error='Could not save camera configuration'), 500

@api_bp.route('/cameras/<serial>', methods=['PUT'])
def update_camera(serial):
    data       = request.get_json() or {}
    new_serial = data.get('serial', serial)
    role       = data.get('role')
    video_mode = data.get('video_mode')
    name       = data.get('name', '')

    if not role or not video_mode:
        return jsonify(error='role and video_mode are required'), 400

    ensure_dir_exists()
    if not CAMERAS_FILE.exists():
        return jsonify(error='No cameras file found'), 404

    # load existing config
    cfg = yaml.safe_load(CAMERAS_FILE.read_text(encoding='utf-8')) or {}
    cameras = cfg.setdefault('camera_config', {}).setdefault('cameras', [])

    for cam in cameras:
        if cam.get('serial') == serial:
            cam['serial']     = QuotedString(str(new_serial))
            cam['name']       = QuotedString(str(name))
            cam['role']       = QuotedString(str(role))
            cam['video_mode'] = QuotedString(str(video_mode))

            if 'framerate' in data:
                cam['framerate'] = float(data['framerate'])
            if 'exposure' in data:
                cam['exposure']  = float(data['exposure'])
            if 'gain' in data:
                cam['gain']      = float(data['gain'])

            # write with yaml.dump so our QuotedString representer is honored
            CAMERAS_FILE.write_text(yaml.dump(cfg), encoding='utf-8')
            return jsonify({
                'serial': str(cam['serial']),
                'name':   str(cam['name']),
                'role':   str(cam['role']),
                'video_mode': str(cam['video_mode']),
                'framerate': cam.get('framerate'),
                'exposure':  cam.get('exposure'),
                'gain':      cam.get('gain'),
            })

    return jsonify(error=f'Camera {serial} not found'), 404

@api_bp.route('/cameras/<serial>', methods=['DELETE'])
def delete_camera(serial):
    try:
        ensure_dir_exists()
        if not CAMERAS_FILE.exists():
            return jsonify(error='No cameras file found'), 404

        contents = CAMERAS_FILE.read_text(encoding='utf-8')
        cfg = yaml.safe_load(contents) or {}
        cams = cfg.get('camera_config', {}).get('cameras', [])
        new_cams = [c for c in cams if c.get('serial') != serial]

        if len(new_cams) == len(cams):
            return jsonify(error='Camera not found'), 404

        cfg['camera_config']['cameras'] = new_cams
        CAMERAS_FILE.write_text(yaml.dump(cfg), encoding='utf-8')
        return jsonify(serial=serial)

    except Exception as e:
        current_app.logger.error(f"Error deleting camera: {e}")
        return jsonify(error='Could not delete camera'), 500
