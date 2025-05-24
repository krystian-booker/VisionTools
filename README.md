# YOLOv5 Training, Conversion, and ROS Node Management Pipeline

## Project Overview

This project provides a comprehensive solution for:
1.  Training YOLOv5 object detection models.
2.  Converting trained models into formats suitable for edge deployment (e.g., OpenVINO for Oak-D Lite).
3.  Managing the lifecycle (start, stop, restart, enable/disable) of ROS 2 nodes.

It features a unified web-based user interface built with React and Material UI, powered by a Flask backend, to manage training processes, configure and control ROS nodes, and view real-time logs and status updates.

## Directory Structure

-   **`backend/`**: Flask-based web server providing APIs for training and ROS node management.
    -   `app.py`: Main Flask application file.
    -   `database_setup.py`: Initializes the SQLite database for ROS nodes.
    -   `ros_node_manager.py`: Contains logic for starting/stopping ROS nodes.
    -   `requirements.txt`: Python dependencies for the backend.
    -   `current_training.log`: Log file for the YOLO training pipeline.
-   **`frontend/`**: React-based single-page application for the user interface.
    -   `public/index.html`: The main HTML entry point.
    -   `src/`: Contains all React components, Redux state management, and screens.
        -   `App.js`: Main application component, sets up routing/layout.
        -   `index.js`: Renders the React app into the DOM.
        -   `store.js`: Redux store configuration.
        -   `components/`: Reusable UI components (e.g., for ROS nodes, training forms).
        -   `features/`: Redux slices for different application domains (e.g., `rosNodesSlice.js`, `trainingSlice.js`).
        -   `screens/`: Top-level components for different UI sections (e.g., `TrainingScreen.js`).
    -   `package.json`: Node.js dependencies and scripts for the frontend.
-   **`scripts/`**: Python scripts for core training and model conversion logic.
    -   `train_yolov5/`: Houses the training script adapted from Ultralytics YOLOv5.
    -   `run_training_pipeline.py`: Orchestrates YOLO training and conversion.
    -   `convert_manager.py`: Manages dispatch to specific model converters.
    -   `openvino_converter.py`: Handles OpenVINO conversion.
-   **`training_output/`**: Default base directory where all training runs, logs, and converted models are saved (created by the training pipeline).
-   **`start_all.sh`**: Unified shell script to start both the backend and frontend development server.
-   **`~/.my_robot_app_data/ros_nodes.sqlite`**: Default location for the SQLite database that stores ROS node configurations and their runtime states. This path can be overridden via environment variables if needed (though not explicitly implemented in current `database_setup.py`).

## Core Features

-   **YOLOv5 Model Training**:
    -   Leverages Ultralytics' YOLOv5 for training custom object detection models.
    -   Configuration via a web UI for dataset paths, model types, epochs, image size, etc.
-   **Model Conversion**:
    -   Supports conversion of trained PyTorch (`.pt`) models to OpenVINO Intermediate Representation (IR) and then to a `.blob` file compatible with Oak-D Lite devices.
    -   Extensible design for future conversion targets.
-   **ROS 2 Node Management**:
    -   Persistent storage of ROS node configurations (package, launch file/executable, arguments, enabled status) in an SQLite database.
    -   Web UI to list, enable/disable, start, stop, and restart configured ROS 2 nodes.
    -   Runtime status tracking (PID, last start/stop times).
-   **Web User Interface (React & Material UI)**:
    -   Centralized dashboard for initiating and monitoring training/conversion jobs.
    -   Real-time log output from the training process via Server-Sent Events (SSE).
    -   Dedicated section for managing ROS 2 nodes.
    -   Responsive design elements for a clean user experience.
-   **Unified Startup**:
    -   `start_all.sh` script to conveniently launch the backend Flask server and the frontend React development server.

## Setup Instructions

### Prerequisites

-   Python 3.8+
-   `pip` (Python package installer)
-   Node.js (v16+ recommended) and `npm` (Node package manager)
-   Git (for cloning the repository)
-   **ROS 2 Jazzy Jubiliee** (or your target ROS 2 distribution): Ensure it is installed and its environment can be sourced. The `start_all.sh` script attempts to source `/opt/ros/jazzy/setup.bash`.
-   **OpenVINO Toolkit**: Required for converting models for Oak-D Lite. Ensure it is installed and its environment is properly configured (e.g., by sourcing the `setupvars.sh` script). Refer to the official OpenVINO documentation for installation details.

### Installation Steps

1.  **Clone the Repository**:
    ```bash
    git clone <repository_url>
    cd <repository_name> 
    ```
    (Replace `<repository_url>` and `<repository_name>` accordingly)

2.  **Install Backend Dependencies**:
    ```bash
    cd backend
    pip install -r requirements.txt
    cd .. 
    ```
    Key backend dependencies include `Flask`, `Flask-CORS`. `sqlite3` is part of the Python standard library.

3.  **Install Frontend Dependencies**:
    ```bash
    cd frontend
    npm install
    cd ..
    ```
    This will install React, Redux Toolkit, Material UI, Axios, and other necessary packages defined in `frontend/package.json`.

4.  **Database Initialization**:
    The ROS node database (`ros_nodes.sqlite`) is automatically initialized (including table creation and population with default nodes) when the backend server starts for the first time. It will be located at `~/.my_robot_app_data/ros_nodes.sqlite` by default.

5.  **OpenVINO & ROS Environment**:
    The `start_all.sh` script attempts to source the ROS 2 environment. For model conversion to Oak-D Lite `.blob` format, your OpenVINO environment also needs to be active in the terminal where `start_all.sh` is run, as the backend invokes scripts using OpenVINO tools.
    Example for manual OpenVINO setup if not part of your global ROS environment:
    ```bash
    # Before running start_all.sh, in the same terminal:
    source /opt/intel/openvino_2023.x.x/setupvars.sh 
    # Adjust path to your OpenVINO installation
    ```

## Running the Application

The recommended way to start the application for development is using the unified startup script:

1.  **Ensure Prerequisites are Met**: Python, Node.js/npm, ROS 2, OpenVINO (if conversion needed).
2.  **Make `start_all.sh` Executable** (if needed):
    ```bash
    chmod +x start_all.sh
    ```
3.  **Run the Startup Script**:
    From the project root directory:
    ```bash
    ./start_all.sh
    ```
    This script will:
    *   Source the ROS 2 environment.
    *   Start the Flask backend server (typically on `http://localhost:5000`). Logs to `backend.log`.
    *   Start the React frontend development server (typically on `http://localhost:3000`). Logs to `frontend.log`.

4.  **Access the Application**:
    Open your web browser and navigate to `http://localhost:3000` (or the port shown by the React dev server).

## How to Use

1.  **Training UI**:
    *   Navigate to the "YOLOv5 Model Training & Conversion" section of the web UI.
    *   The UI will load configuration options from the backend (e.g., model types, conversion targets, default parameters).
    *   **Fill in the Form**:
        *   **Path to data.yaml**: Provide the *absolute path* to your YOLOv5 dataset configuration file. This path must be accessible by the machine running the backend server.
        *   Select the YOLOv5 model type, training epochs, image size, and training device.
        *   Choose a conversion target (e.g., `oak_d_lite`) or "None" to only train the model.
        *   Optionally, provide a custom run name.
    *   Click "Start Training".
    *   The UI will display the job status and real-time logs from the backend.
    *   Once completed, paths to the best `.pt` model and the converted model (if applicable) will be shown.

2.  **ROS Node Management UI**:
    *   Navigate to the "ROS Node Status & Management" section.
    *   A list of pre-configured ROS nodes from the database will be displayed with their current status (e.g., 'running', 'stopped', 'error').
    *   **Enable/Disable**: Use the toggle switch to enable or disable a node. A node must be enabled to be started.
    *   **Actions**:
        *   **Start**: Starts an enabled node that is currently stopped.
        *   **Stop**: Stops a running node.
        *   **Restart**: Stops and then starts an enabled node.
    *   The UI provides feedback on the status of these operations.

## Future Enhancements / To-Do

-   **File Uploads**: Allow uploading `data.yaml` for training.
-   **Dataset Management**: Basic UI for managing datasets on the server.
-   **More Conversion Targets**: Extend `convert_manager.py`.
-   **Robust Job Management**: Implement a proper job queue for multiple training requests.
-   **Security**: Add authentication and more robust input validation.
-   **Configuration File**: Centralize configurations.
-   **Dockerization**: Package for easier deployment.
-   **ROS Node Log Viewing**: Stream logs from running ROS nodes to the UI.
-   **ROS Node Configuration UI**: Allow adding/editing ROS node configurations directly from the UI instead of just initial DB population.
-   **Theming/Styling**: Further refine Material UI theming.The root `README.md` has been updated in the previous turn (Assistant Turn 61).

I will now proceed to update `scripts/README.md`. I'll first read its current content to see what changes are needed.
