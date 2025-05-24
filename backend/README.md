# Backend Server (Flask)

## Purpose

This directory contains the Flask-based backend server application. It provides a RESTful API to:
1.  Manage the YOLOv5 training and model conversion pipeline, allowing users to initiate jobs, monitor their status, and view real-time logs.
2.  Manage the lifecycle (start, stop, restart, enable/disable) and configuration of ROS 2 nodes.

The backend interacts with scripts in the `scripts/` directory for these core functionalities and uses an SQLite database to persist ROS node configurations and states.

## Core Components

-   **`app.py`**: The main Flask application file. It defines API endpoints, handles requests, and orchestrates calls to other modules.
-   **`database_setup.py`**: Responsible for initializing the SQLite database (`ros_nodes.sqlite`) used for ROS node management. It creates tables and populates initial node configurations. This is run automatically on `app.py` startup.
-   **`ros_node_manager.py`**: Contains the Python logic for starting, stopping, and restarting ROS 2 nodes using `subprocess.Popen`. It interacts with the `ros_nodes.sqlite` database.
-   **`requirements.txt`**: Lists the Python dependencies for the backend (e.g., Flask, Flask-CORS).
-   **`current_training.log`**: A log file where stdout and stderr from the YOLO training pipeline (`run_training_pipeline.py`) are redirected.

## Database (`ros_nodes.sqlite`)

-   **Purpose**: Stores configurations and runtime states for manageable ROS 2 nodes.
-   **Default Location**: `~/.my_robot_app_data/ros_nodes.sqlite` (This path is defined in `database_setup.py` and used by `app.py`).
-   **Initialization**: Automatically created and initialized by `database_setup.py` (called from `app.py`) if it doesn't exist when the backend server starts.
-   **`ros_nodes` Table Schema Overview**:
    -   `id`: INTEGER PRIMARY KEY AUTOINCREMENT
    -   `node_name`: TEXT NOT NULL UNIQUE (e.g., "flir_camera_node")
    -   `description`: TEXT (e.g., "FLIR Blackfly S GigE camera driver")
    -   `package_name`: TEXT NOT NULL (e.g., "flir_spinnaker_ros2")
    -   `launch_file_or_executable`: TEXT NOT NULL (e.g., "flir_camera.launch.py" or "usb_cam_node_exe")
    -   `node_type`: TEXT NOT NULL CHECK (`launch` or `run`)
    -   `ros_args`: TEXT (JSON string for structured arguments, or plain string for simple args, e.g., `{"camera_index": 0}` or `param_name:=value`)
    -   `is_enabled`: INTEGER NOT NULL DEFAULT 0 (0 for false, 1 for true)
    -   `status`: TEXT NOT NULL DEFAULT 'stopped' (e.g., 'stopped', 'running', 'error', 'starting', 'stopping')
    -   `pid`: INTEGER (Process ID of the running node, NULL if not running)
    -   `last_started`: TEXT (ISO 8601 timestamp, UTC)
    -   `last_stopped`: TEXT (ISO 8601 timestamp, UTC)
    -   `log_file`: TEXT (Path to a specific log file for this node, if applicable - currently not auto-managed by `ros_node_manager.py` for individual nodes)

## API Endpoints

The backend server exposes the following API endpoints. The base URL is typically `http://localhost:5000`.

### Training Pipeline Endpoints

#### 1. `POST /train_and_convert`
-   **Purpose**: Initiates a new YOLOv5 training and optional model conversion job.
-   **Request Body (JSON)**:
    -   `data_yaml_path` (string, required): Absolute path to the `data.yaml` file.
    -   `model_type` (string, optional): E.g., `yolov5n.pt`. Default: `yolov5n.pt`.
    -   `epochs` (integer, optional): Default: `100`.
    -   `imgsz` (integer, optional): Image size. Default: `416`.
    -   `device` (string, optional): `cpu` or GPU ID (e.g., `0`). Default: `cpu`.
    -   `conversion_target` (string, optional): E.g., `oak_d_lite`. If `null`, only training.
    -   `run_name` (string, optional): Custom name for the run. Default: timestamp-based.
-   **Responses**:
    -   **202 Accepted**: Job started successfully. Returns job details including PID.
        ```json
        {
            "status": "started", 
            "message": "Training pipeline initiated.", 
            "job_pid": 12345, 
            "log_file": "/path/to/backend/current_training.log",
            "job_details": { "status": "running", "pid": 12345, ... }
        }
        ```
    -   **409 Conflict**: If a training job is already running.
    -   **400 Bad Request**: Missing parameters or invalid JSON.
    -   **500 Internal Server Error**: Server-side issues.
-   **Behavior**: Starts `scripts/run_training_pipeline.py` asynchronously. Only one training job can run at a time. Logs are written to `backend/current_training.log`.

#### 2. `GET /training_status`
-   **Purpose**: Retrieves the status of the current or last-run training job.
-   **Response (JSON)**: Details the job's `status` ('idle', 'running', 'initiating', 'completed', 'failed', 'error'), `pid`, log file path, start/end times, model paths, and any error messages.
-   **Behavior**: If a job is 'running', its PID liveness is checked. If the PID is gone, the status is updated by parsing `current_training.log`.

#### 3. `GET /training_log_stream`
-   **Purpose**: Provides a Server-Sent Event (SSE) stream for real-time logs from the training job.
-   **Response Type**: `text/event-stream`.
-   **Behavior**: Streams `current_training.log`. Sends a `job_terminated` event when the job is no longer 'running'.

#### 4. `GET /config_options`
-   **Purpose**: Provides frontend with configuration options for the training UI.
-   **Response (JSON)**: Includes `model_types`, `conversion_targets`, and `default_training_parameters`.

### ROS Node Management Endpoints

#### 5. `GET /ros/nodes`
-   **Purpose**: Retrieves a list of all configured ROS nodes from the database.
-   **Response (JSON)**: An array of node objects, each matching the `ros_nodes` table schema.
-   **Behavior**: Before fetching, this endpoint calls an internal function (`update_all_node_statuses`) that checks any node marked 'running' and updates its status to 'error' if its PID is no longer active. This ensures the returned statuses are relatively up-to-date.

#### 6. `POST /ros/nodes/<int:node_id>/enable`
-   **Purpose**: Enables a specific ROS node (sets `is_enabled = 1`).
-   **Response (JSON)**: Success/error message and the updated node data.
    -   **200 OK**: On success.
    -   **404 Not Found**: If `node_id` does not exist.
    -   **500 Internal Server Error**: For database errors.

#### 7. `POST /ros/nodes/<int:node_id>/disable`
-   **Purpose**: Disables a specific ROS node (sets `is_enabled = 0`).
-   **Response (JSON)**: Similar to `/enable`.

#### 8. `POST /ros/nodes/<int:node_id>/start`
-   **Purpose**: Starts a specific, enabled ROS node.
-   **Response (JSON)**:
    -   **200 OK**: If node started successfully or was already running and confirmed active. Example:
        ```json
        {
            "status": "success", 
            "message": "Node xyz_node started successfully.", 
            "pid": 54321, 
            "node_id": 1, 
            "node_name": "xyz_node",
            "last_started": "YYYY-MM-DDTHH:MM:SS.ffffffZ"
        }
        ```
    -   **404 Not Found**: If `node_id` does not exist.
    -   **409 Conflict**: If node is not enabled, or if already running (and PID confirmed active).
    -   **400 Bad Request**: If `ros_args` are malformed.
    -   **500 Internal Server Error**: For database or subprocess errors.
-   **Behavior**: Calls `ros_node_manager.start_node()`. Updates node `status`, `pid`, `last_started` in DB.

#### 9. `POST /ros/nodes/<int:node_id>/stop`
-   **Purpose**: Stops a specific, running ROS node.
-   **Response (JSON)**:
    -   **200 OK**: If node stopped successfully, was already stopped, or termination signals were sent (even if not immediately confirmed). Example:
        ```json
        {
            "status": "success", 
            "message": "Node xyz_node stopped successfully.", 
            "node_id": 1,
            "last_stopped": "YYYY-MM-DDTHH:MM:SS.ffffffZ"
        }
        ```
    -   **404 Not Found**: If `node_id` does not exist.
    -   **500 Internal Server Error**: For database or process signal errors.
-   **Behavior**: Calls `ros_node_manager.stop_node()`. Attempts graceful termination (SIGINT, then SIGTERM). Updates node `status`, clears `pid`, sets `last_stopped` in DB.

#### 10. `POST /ros/nodes/<int:node_id>/restart`
-   **Purpose**: Restarts a specific, enabled ROS node (effectively calls stop then start).
-   **Response (JSON)**: The response from the `start` part of the operation.
    -   **200 OK**: If restart sequence initiated successfully and node started.
    -   **404 Not Found**: If `node_id` does not exist (checked during stop attempt).
    -   **Other codes**: Reflects errors from stop or start phases (e.g., 409 if not enabled, 500).
-   **Behavior**: Calls `ros_node_manager.restart_node()`.

## Setup & Running

1.  **Navigate to `backend/` directory**.
2.  **Install Dependencies**: `pip install -r requirements.txt`.
3.  **Environment for Scripts**:
    *   **OpenVINO**: If using model conversion, ensure the OpenVINO environment is sourced in your terminal before starting the Flask server (e.g., `source /opt/intel/openvino_YYYY.X.X/setupvars.sh`).
    *   **ROS 2**: For ROS node management, ensure your target ROS 2 distribution (e.g., Jazzy) is sourced (e.g., `source /opt/ros/jazzy/setup.bash`).
    The `start_all.sh` script in the project root attempts to handle ROS 2 sourcing.
4.  **Run the Flask Server**:
    ```bash
    python app.py
    ```
    The server typically starts on `http://localhost:5000`. The ROS node database is initialized automatically on first startup at `~/.my_robot_app_data/ros_nodes.sqlite`.

## Job and Process Management Notes

-   **Training Pipeline**: Only one training job can be active at a time. Its state is tracked in memory (`current_job` global variable in `app.py`) and logs are sent to `backend/current_training.log`.
-   **ROS Nodes**:
    -   Multiple ROS nodes can be managed (started/stopped) concurrently.
    -   Their states (`status`, `pid`, timestamps) are persisted in the SQLite database.
    -   The backend attempts to track actual process liveness via PIDs, but this can have limitations (e.g., PID reuse if not handled carefully, though `os.kill(pid,0)` is fairly reliable for existence check).
    -   The `GET /ros/nodes` endpoint includes a "lazy update" mechanism to correct DB status for 'running' nodes whose PIDs are no longer found.
    -   Launched ROS nodes are started in new process sessions (`start_new_session=True` in `subprocess.Popen`) to help them persist independently of the Flask server's own workers if the server were to be managed by a more complex setup (like Gunicorn workers). However, if the main Flask `app.py` process itself is killed, these child processes might still be terminated by the OS depending on process group leadership.
