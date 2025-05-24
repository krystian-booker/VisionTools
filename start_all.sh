#!/usr/bin/env bash
set -e

echo "Sourcing ROS 2 Jazzy environment..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
    echo "ROS 2 Jazzy environment sourced."
else
    echo "Warning: ROS 2 Jazzy global setup script not found at /opt/ros/jazzy/setup.bash. Functionality relying on ROS may be affected."
fi

# Determine the absolute path of the script and the project root
# This assumes start_all.sh is in the project root.
SCRIPT_DIR_REAL_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$SCRIPT_DIR_REAL_PATH"

echo "Project root identified as: $PROJECT_ROOT"

# Optional: Source local ROS 2 workspace if it exists from project root
# This allows for locally built packages to be found by ROS.
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    echo "Sourcing local ROS 2 workspace ($PROJECT_ROOT/install/setup.bash)..."
    source "$PROJECT_ROOT/install/setup.bash"
    echo "Local ROS 2 workspace (install/setup.bash) sourced."
elif [ -f "$PROJECT_ROOT/_ws/install/setup.bash" ]; then # Common alternative, e.g. if using vcs tool
    echo "Sourcing local ROS 2 workspace ($PROJECT_ROOT/_ws/install/setup.bash)..."
    source "$PROJECT_ROOT/_ws/install/setup.bash"
    echo "Local ROS 2 workspace (_ws/install/setup.bash) sourced."
else
    echo "No local ROS 2 workspace (install/setup.bash or _ws/install/setup.bash) found to source."
fi


BACKEND_DIR="$PROJECT_ROOT/backend"
FRONTEND_DIR="$PROJECT_ROOT/frontend"

BACKEND_LOG="$PROJECT_ROOT/backend.log"
BACKEND_PID_FILE="$PROJECT_ROOT/backend.pid"
FRONTEND_LOG="$PROJECT_ROOT/frontend.log"
FRONTEND_PID_FILE="$PROJECT_ROOT/frontend.pid"

# Ensure log/pid directories are not accidentally created in / if PROJECT_ROOT is empty
if [ -z "$PROJECT_ROOT" ] || [ "$PROJECT_ROOT" == "/" ]; then
    echo "Error: Project root could not be determined or is '/'. Exiting for safety."
    exit 1
fi

# Clean up old pid files if they exist
rm -f "$BACKEND_PID_FILE"
rm -f "$FRONTEND_PID_FILE"

echo ""
echo "Starting Backend Server..."
if [ ! -d "$BACKEND_DIR" ]; then
    echo "Error: Backend directory '$BACKEND_DIR' not found."
    exit 1
fi
cd "$BACKEND_DIR"

# Ensure python3 is used if python might be python2
if command -v python3 &> /dev/null; then
    PYTHON_EXEC=python3
else
    PYTHON_EXEC=python
fi
echo "Using Python executable: $PYTHON_EXEC"

if [ ! -f "app.py" ]; then
    echo "Error: backend/app.py not found in $BACKEND_DIR."
    cd "$PROJECT_ROOT"
    exit 1
fi

# Start backend
$PYTHON_EXEC app.py > "$BACKEND_LOG" 2>&1 &
BACKEND_PID=$!
echo $BACKEND_PID > "$BACKEND_PID_FILE"
echo "Backend server starting in background. PID: $BACKEND_PID. Log: $BACKEND_LOG"
cd "$PROJECT_ROOT"

# Give backend a moment to start (optional, but can be helpful)
echo "Waiting a few seconds for backend to initialize..."
sleep 3 

echo ""
echo "Starting Frontend Development Server (React)..."
if [ ! -d "$FRONTEND_DIR" ]; then
    echo "Error: Frontend directory '$FRONTEND_DIR' not found."
    # Kill backend if frontend can't start to avoid leaving it orphaned
    echo "Stopping backend server (PID: $BACKEND_PID) due to frontend setup error."
    kill $BACKEND_PID || echo "Backend (PID: $BACKEND_PID) already stopped or failed to stop."
    rm -f "$BACKEND_PID_FILE"
    exit 1
fi
cd "$FRONTEND_DIR"

# Check if npm is available
if ! command -v npm &> /dev/null; then
    echo "Error: npm command not found. Please install Node.js and npm."
    echo "Stopping backend server (PID: $BACKEND_PID) due to missing npm."
    kill $BACKEND_PID || echo "Backend (PID: $BACKEND_PID) already stopped or failed to stop."
    rm -f "$BACKEND_PID_FILE"
    cd "$PROJECT_ROOT"
    exit 1
fi

# Check for package.json to ensure it's a Node project
if [ ! -f "package.json" ]; then
    echo "Error: frontend/package.json not found. Cannot start React development server."
    echo "Ensure you are in the correct directory and have initialized your React project."
    echo "Stopping backend server (PID: $BACKEND_PID)."
    kill $BACKEND_PID || echo "Backend (PID: $BACKEND_PID) already stopped or failed to stop."
    rm -f "$BACKEND_PID_FILE"
    cd "$PROJECT_ROOT"
    exit 1
fi

# Start frontend (React dev server)
npm start > "$FRONTEND_LOG" 2>&1 &
FRONTEND_PID=$!
echo $FRONTEND_PID > "$FRONTEND_PID_FILE"
echo "Frontend server starting in background. PID: $FRONTEND_PID. Log: $FRONTEND_LOG"
cd "$PROJECT_ROOT"

echo ""
echo "================================================"
echo "All services initiated."
echo "------------------------------------------------"
echo "Backend Server:"
echo "  - PID: $BACKEND_PID (saved in $BACKEND_PID_FILE)"
echo "  - Log: $BACKEND_LOG"
echo "  - Expected URL (API): http://localhost:5000"
echo "------------------------------------------------"
echo "Frontend Development Server:"
echo "  - PID: $FRONTEND_PID (saved in $FRONTEND_PID_FILE)"
echo "  - Log: $FRONTEND_LOG"
echo "  - Access UI at: http://localhost:3000 (typical React dev port)"
echo "------------------------------------------------"
echo "ROS Node Management:"
echo "  - ROS nodes are NOT started automatically by this script."
echo "  - Please manage ROS nodes (start/stop/enable/disable) via the web UI or API"
echo "    once the backend and frontend are fully initialized."
echo "================================================"
echo ""
echo "To stop services, you may need to manually kill processes using their PIDs,"
echo "or use a dedicated stop script (if available)."
echo "Example: kill $BACKEND_PID $FRONTEND_PID"
echo ""
echo "Note: It might take a few moments for the servers to be fully ready."
echo "Check logs if you encounter issues."
