# web/views.py
from flask import Blueprint, render_template
from orchestrator import get_local_ip

views_bp = Blueprint('views', __name__)

@views_bp.route('/')
def dashboard():
    local_ip = get_local_ip()
    return render_template(
        'dashboard.html',
        active='dashboard',
        localIP=local_ip
    )

@views_bp.route('/camera-setup')
def camera_setup():
    return render_template(
        'cameraSetup.html',
        active='camera-setup'
    )

@views_bp.route('/camera-calibration')
def camera_calibration():
    return render_template(
        'cameraCalibration.html',
        active='camera-calibration'
    )

@views_bp.route('/about')
def about():
    return render_template(
        'about.html',
        active='about'
    )
