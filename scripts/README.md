# Scripts for YOLOv5 Training, Conversion, and ROS Node Management

## Purpose

This directory contains Python scripts that form the core of the machine learning pipeline and ROS 2 node interaction:
- Training YOLOv5 models.
- Converting trained models to formats suitable for edge deployment (e.g., OpenVINO).
- Starting and stopping ROS 2 nodes.

These scripts are primarily orchestrated by the backend server (`backend/app.py` which uses `run_training_pipeline.py` and `ros_node_manager.py`), but some can also be run standalone for testing or specific tasks, provided the necessary environment (like OpenVINO or ROS 2) is configured.

## Key Scripts

### 1. `run_training_pipeline.py`

-   **Purpose**: This is the main orchestrator script for the YOLOv5 training and conversion pipeline. It manages the entire workflow from setting up training directories, initiating YOLOv5 training, and finally triggering model conversion. It is typically invoked by the backend server.
-   **Command-Line Arguments**:
    -   `--data_yaml` (required): Path to the `data.yaml` file defining the dataset.
    -   `--output_base_dir` (default: `./training_output`): Base directory where all run-specific output folders will be created.
    -   `--model_type` (default: `yolov5n.pt`): Specifies the YOLOv5 model architecture to train.
    -   `--epochs` (default: `100`): Number of training epochs.
    -   `--imgsz` (default: `416`): Input image size for training.
    -   `--device` (default: `cpu`): Training device (`cpu` or GPU ID like `0`).
    -   `--conversion_target` (optional): Target format for model conversion (e.g., `oak_d_lite`). If not provided, only training is performed.
    -   `--run_name` (optional): Custom name for the training run.
-   **Example Standalone Usage**:
    ```bash
    python scripts/run_training_pipeline.py \
        --data_yaml /path/to/your/dataset/data.yaml \
        --model_type yolov5s.pt \
        --epochs 50 \
        --conversion_target oak_d_lite \
        --run_name my_custom_run
    ```
-   **Workflow**:
    1.  Creates output directories under `<output_base_dir>/<run_name>/`.
    2.  Calls `train_yolov5/train.py` for model training.
    3.  Locates `best.pt` from training output.
    4.  If `--conversion_target` is specified, calls `convert_manager.py`.

### 2. `train_yolov5/train.py`

-   **Purpose**: Wrapper for Ultralytics YOLOv5 training functionality.
-   **Key Arguments (as used by `run_training_pipeline.py`)**:
    -   `--data`, `--weights`, `--epochs`, `--imgsz`, `--device`, `--project`, `--name`.
-   **Note**: Relies on the `ultralytics` Python package.

### 3. `convert_manager.py`

-   **Purpose**: Dispatches model conversion tasks to specific converter scripts based on the `--target` argument.
-   **Command-Line Arguments**:
    -   `--pt_path` (required): Path to the input `.pt` model file.
    -   `--target` (required): Desired conversion target (e.g., `oak_d_lite`).
    -   `--output_dir` (optional): Output directory.
-   **Current Targets**:
    -   `oak_d_lite`: Dispatches to `openvino_converter.py`.

### 4. `openvino_converter.py`

-   **Purpose**: Converts a YOLOv5 `.pt` model to ONNX, then to OpenVINO IR (`.xml`, `.bin`), and finally compiles to a MyriadX `.blob` file.
-   **Command-Line Arguments**:
    -   `--pt_path` (required): Path to the input `.pt` model.
    -   `--output_dir` (optional): Output directory.
-   **Crucial Note on OpenVINO Environment**:
    -   This script **requires** a properly configured OpenVINO environment. The Model Optimizer (`mo`) and Compile Tool (`compile_tool`) must be in `PATH`.
    -   Before running this script (or the backend server that calls it), source the OpenVINO `setupvars.sh` script:
        ```bash
        source /opt/intel/openvino_YYYY.X.X/setupvars.sh 
        # Adjust path and version to your OpenVINO installation.
        ```

### 5. `backend/database_setup.py` (Located in `backend/`)

-   **Purpose**: While located in the `backend` directory, this script is crucial for initializing the SQLite database (`ros_nodes.sqlite`) used by `ros_node_manager.py`. It creates the necessary tables and populates initial ROS node configurations.
-   **Execution**: It's called automatically when the backend Flask server (`backend/app.py`) starts. It can also be run manually for direct database setup: `python backend/database_setup.py`.

### 6. `backend/ros_node_manager.py` (Located in `backend/`)

-   **Purpose**: This script, though part of the backend, contains the core Python logic for starting and stopping ROS 2 nodes using `subprocess.Popen`. It interacts with the `ros_nodes.sqlite` database to get node configurations and update their runtime status (PID, last start/stop times).
-   **Functions**:
    -   `start_node()`: Constructs and executes `ros2 launch` or `ros2 run` commands.
    -   `stop_node()`: Sends SIGINT/SIGTERM signals to running ROS nodes.
    -   `restart_node()`: Combines stop and start logic.
    -   `check_node_process_status()`: Verifies if a PID is active.
    -   `update_all_node_statuses()`: Proactively updates DB for 'running' nodes whose PIDs are no longer active.
-   **Note**: This script requires a properly configured ROS 2 environment (e.g., `jazzy`) to be sourced in the environment where the backend server (which calls these functions) is running. The `start_all.sh` script handles this.

## Dependencies

-   **`ultralytics`**: For `train_yolov5/train.py`. Install via `pip install ultralytics`.
-   **OpenVINO Toolkit**: For `openvino_converter.py`. System-level installation required.
-   **ROS 2 (e.g., Jazzy)**: For `ros_node_manager.py` to launch/run ROS nodes. System-level installation required.
-   Standard Python libraries: `argparse`, `pathlib`, `subprocess`, `sqlite3`, `json`, `shlex`.

Ensure all dependencies are installed and relevant environments (OpenVINO, ROS 2) are configured before running scripts that rely on them, or before starting the main application via `start_all.sh`.
