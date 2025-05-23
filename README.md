# Installation
1.  `bash setup.sh`


## Usage

This package uses a unified launch file, `flir_camera.launch.py`, to start the FLIR camera driver(s). All configurations for the camera(s) are defined in a YAML file, which is passed to the launch file as an argument.

### Launching the Camera(s)

To launch the camera driver, you will need to specify the path to your YAML configuration file using the `camera_config_file` argument.

Below is an example command that launches the driver using the `single_chameleon_template.yaml` configuration file. This command is confirmed to work. You can adapt this command to use other provided examples or your own custom configuration file.

Example command:

```bash
ros2 launch flir_camera_bringup flir_camera.launch.py camera_config_file:=$(ros2 pkg prefix flir_camera_bringup)/share/flir_camera_bringup/config/single_chameleon_template.yaml
```

See the "Configuration Files" section below for more details on available examples and how to customize them.

### Configuration Files

The `config/` directory within this package contains example YAML configuration files:

*   `dual_blackfly_s_triggered.yaml`: For two FLIR Blackfly S cameras with triggering enabled.
*   `dual_chameleon_template.yaml`: A template for two FLIR Chameleon cameras.
*   `single_blackfly_s.yaml`: For a single FLIR Blackfly S camera.
*   `single_chameleon_template.yaml`: A template for a single FLIR Chameleon camera.

**To use these examples:**

1.  Copy one of the template files (e.g., `cp config/single_blackfly_s.yaml config/my_custom_config.yaml`).
2.  Edit `my_custom_config.yaml` to match your specific camera serial numbers and desired parameters.
3.  Launch the driver using your custom configuration file:

    ```bash
    ros2 launch flir_camera_bringup flir_camera.launch.py camera_config_file:=$(find-pkg-share flir_camera_bringup)/config/my_custom_config.yaml
    ```

You can also create your own YAML configuration file from scratch, following the structure of the examples. Each camera you want to launch should have an entry under the `cameras:` list in the YAML file, specifying its `name`, `serial_number`, `camera_type`, and `ros_parameters`.
