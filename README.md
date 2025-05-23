# Installation

Run the setup script:

```bash
bash setup.sh
```

---

## Usage

This package uses a single launch file, `flir_camera.launch.py`, to start one or more FLIR camera drivers. All camera settings are defined in a YAML config file, passed in as a launch argument.

---

### Launching the Camera

To launch the camera driver, provide the path to your YAML config file using the `camera_config_file` argument.

Example (using the included `single_chameleon_template.yaml`):

```bash
ros2 launch flir_camera_bringup flir_camera.launch.py camera_config_file:=$(ros2 pkg prefix flir_camera_bringup)/share/flir_camera_bringup/config/single_chameleon_template.yaml
```

You can modify this command to use any of the provided templates or your own config.

---

### Configuration Files

The `config/` directory includes example YAML files:

- `single_chameleon_template.yaml` — Template for one FLIR Chameleon camera  
- `dual_chameleon_template.yaml` — Template for two FLIR Chameleon cameras  
- `single_blackfly_s.yaml` — For one FLIR Blackfly S camera  
- `dual_blackfly_s_triggered.yaml` — For two Blackfly S cameras with triggering  

---

### Creating a Custom Config

1. Copy a template file:

   ```bash
   cp config/single_blackfly_s.yaml config/my_custom_config.yaml
   ```

2. Edit `my_custom_config.yaml` with your camera’s serial number and desired parameters.

3. Launch using your custom config:

   ```bash
   ros2 launch flir_camera_bringup flir_camera.launch.py camera_config_file:=$(find-pkg-share flir_camera_bringup)/config/my_custom_config.yaml
   ```

---

Each camera must have an entry under `cameras:` in the YAML file, with fields like `name`, `serial_number`, `camera_type`, and `ros_parameters`.
