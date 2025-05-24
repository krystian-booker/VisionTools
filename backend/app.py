import sys # For sys.executable
from flask import Flask, request, jsonify, Response
from flask_cors import CORS
import subprocess
import os
import pathlib
import shlex # For safely joining command parts for logging
from datetime import datetime # For job start time
import time # For time.sleep()

# --- Database Setup ---
# Assuming database_setup.py is in the same directory (backend/)
from database_setup import initialize_database, DEFAULT_DB_PATH, get_db_connection

app = Flask(__name__)
CORS(app) # Enable CORS for all routes

# Initialize the database
# This ensures it's done once when the app starts.
# This path will be used by database_setup.initialize_database
# and can be used by other parts of the app for DB connections.
DATABASE_FILE = DEFAULT_DB_PATH 
try:
    initialize_database(DATABASE_FILE) 
    app.logger.info(f"Using ROS Node database at: {DATABASE_FILE.resolve()}")
except Exception as e:
    app.logger.error(f"CRITICAL: Failed to initialize ROS Node database at {DATABASE_FILE.resolve()}: {e}")
    # Depending on the application's needs, you might want to exit here
    # or allow the app to run in a degraded state if the DB is not critical for all functions.
    # For now, we log the error and continue; some endpoints might fail if DB is needed.

# Determine the base directory of the backend app
# This is typically where app.py is located.
BACKEND_DIR = pathlib.Path(__file__).parent.resolve()
# Path to the scripts directory, assuming it's a sibling of the backend directory
SCRIPTS_DIR = (BACKEND_DIR.parent / "scripts").resolve()

# --- Global Job State Management (for training pipeline) ---
TRAINING_LOG_FILE = BACKEND_DIR / "current_training.log" # Log file in the backend directory
current_job = {
    'status': 'idle',    # 'idle', 'running', 'completed', 'failed'
    'pid': None,
    'log_file': str(TRAINING_LOG_FILE.resolve()), # Store resolved path
    'start_time': None,
    'end_time': None,
    'best_pt_path': None,
    'converted_model_path': None,
    'error_message': None # Store error message if job fails
}
# Note: A background thread/task could monitor the PID for completion/failure
# and update 'status', 'end_time', and parse results from TRAINING_LOG_FILE.
# For this task, we'll only update status to 'running' and 'idle' (implicitly via new job start).
# from flask import Response # Already imported
# import time # Already imported
# os is already imported for os.kill, will be used for os.path.exists and os.SEEK_END

# import sqlite3 # Already imported
# A separate '/status' endpoint would be needed to check on the job.

# --- ROS Node Management Imports ---
# Assuming ros_node_manager.py is in the same directory (backend/)
from ros_node_manager import start_node, stop_node, restart_node, update_all_node_statuses # Import new function


# This function was for parsing direct stdout, not applicable for log file parsing in status check.
# Will be replaced/complemented by parse_final_output_from_log
# def parse_pipeline_output(stdout_str: str) -> dict:
#     """
#     Parses the stdout of run_training_pipeline.py to find
#     FINAL_BEST_PT_PATH and FINAL_CONVERTED_MODEL_PATH.
#     """
#     results = {
#         "best_pt_path": None,
#         "converted_model_path": None,
#     }
#     for line in stdout_str.strip().split('\n'):
#         if line.startswith("FINAL_BEST_PT_PATH:"):
#             path = line.split("FINAL_BEST_PT_PATH:", 1)[1].strip()
#             results["best_pt_path"] = path if path != "None" else None
#         elif line.startswith("FINAL_CONVERTED_MODEL_PATH:"):
#             path = line.split("FINAL_CONVERTED_MODEL_PATH:", 1)[1].strip()
#             results["converted_model_path"] = path if path != "None" else None
#     return results

def parse_final_output_from_log(log_file_path_str: str) -> dict:
    """
    Parses the training log file to find FINAL_BEST_PT_PATH and FINAL_CONVERTED_MODEL_PATH.
    Returns a dictionary with these paths or None, and an error message if applicable.
    """
    results = {
        'best_pt_path': None,
        'converted_model_path': None,
        'error': None
    }
    try:
        log_file_path = pathlib.Path(log_file_path_str)
        if not log_file_path.is_file():
            results['error'] = f"Log file not found: {log_file_path_str}"
            return results

        with open(log_file_path, 'r') as f:
            lines = f.readlines()
        
        found_best_pt = False
        found_converted_model = False

        for line in lines:
            line = line.strip()
            if line.startswith("FINAL_BEST_PT_PATH:"):
                path = line.split("FINAL_BEST_PT_PATH:", 1)[1].strip()
                results['best_pt_path'] = path if path != "None" else None
                found_best_pt = True
            elif line.startswith("FINAL_CONVERTED_MODEL_PATH:"):
                path = line.split("FINAL_CONVERTED_MODEL_PATH:", 1)[1].strip()
                results['converted_model_path'] = path if path != "None" else None
                found_converted_model = True
        
        if not found_best_pt:
            # If best_pt_path is not found, it usually indicates a failure in training itself.
            results['error'] = "FINAL_BEST_PT_PATH not found in logs. Training likely failed or was interrupted early."
            # Optionally, capture last few lines for error context
            # results['log_snippet'] = "".join(lines[-10:]) 
            
    except Exception as e:
        results['error'] = f"Error parsing log file: {str(e)}"
    
    return results


@app.route('/')
def hello():
    return "Backend server is running!"

@app.route('/train_and_convert', methods=['POST'])
def train_and_convert():
    try:
        data = request.get_json()
        if not data:
            return jsonify({'status': 'error', 'message': 'No JSON data provided.'}), 400
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Invalid JSON: {str(e)}'}), 400

    # --- Check for Existing Job ---
    # (A more robust check might involve checking if current_job['pid'] is actually running)
    if current_job['status'] == 'running':
        # Check if the PID is still active (basic check, platform dependent nuances)
        if current_job['pid'] is not None:
            # This check is rudimentary. A process could have died and PID reused.
            # On Linux/macOS:
            try:
                os.kill(current_job['pid'], 0) # Check if process exists
                return jsonify({'status': 'error', 'message': f'A training job (PID: {current_job["pid"]}) is already in progress. Please wait or check status.'}), 409 # Conflict
            except OSError:
                app.logger.info(f"Previously running job (PID: {current_job['pid']}) seems to have finished or was interrupted. Allowing new job.")
                current_job['status'] = 'failed' # Mark as failed if it was running but os.kill failed
                current_job['error_message'] = 'Previous job process not found, presumed dead.'
            # On Windows, os.kill(pid, 0) doesn't work the same way.
            # A more complex check or a dedicated process monitoring thread would be needed for robustness.
            # For now, if status is 'running', we are cautious. A /status endpoint would help manually reset.

    # --- Parameter Extraction ---
    data_yaml_path = data.get('data_yaml_path')
    if not data_yaml_path:
        return jsonify({'status': 'error', 'message': 'Missing required parameter: data_yaml_path'}), 400
    
    data_yaml_path = str(pathlib.Path(data_yaml_path).resolve()) # Ensure absolute path

    output_base_dir = data.get('output_base_dir', str((BACKEND_DIR / "./training_output").resolve()))
    model_type = data.get('model_type', 'yolov5n.pt')
    epochs = data.get('epochs', 100)
    imgsz = data.get('imgsz', 416)
    device = data.get('device', 'cpu')
    conversion_target = data.get('conversion_target') # Optional
    run_name = data.get('run_name') # Optional

    # --- Command Construction ---
    pipeline_script_path = SCRIPTS_DIR / "run_training_pipeline.py"
    if not pipeline_script_path.is_file():
        app.logger.error(f'run_training_pipeline.py not found at {pipeline_script_path}')
        return jsonify({'status': 'error', 'message': f'run_training_pipeline.py not found at {pipeline_script_path}'}), 500

    command = [
        sys.executable, # Use the same python interpreter
        "-u", # Unbuffered Python output
        str(pipeline_script_path),
        "--data_yaml", data_yaml_path,
        "--output_base_dir", output_base_dir,
        "--model_type", model_type,
        "--epochs", str(epochs),
        "--imgsz", str(imgsz),
        "--device", device,
    ]
    if conversion_target:
        command.extend(["--conversion_target", conversion_target])
    if run_name:
        command.extend(["--run_name", run_name])

    app.logger.info(f"Attempting to start training job. Command: {' '.join(shlex.quote(str(c)) for c in command)}")
    app.logger.info(f"Log file: {TRAINING_LOG_FILE.resolve()}")

    # --- Asynchronous Subprocess Execution ---
    try:
        # Open log file in write mode to clear previous logs for this job
        with open(TRAINING_LOG_FILE, 'w') as log_file_handle:
            process = subprocess.Popen(
                command,
                stdout=log_file_handle,
                stderr=subprocess.STDOUT, # Redirect stderr to stdout (which goes to log_file_handle)
                cwd=BACKEND_DIR # Execute from backend directory context if needed, though script paths are absolute
            )
        
        # Update Global Job State
        current_job['status'] = 'running'
        current_job['pid'] = process.pid
        current_job['start_time'] = datetime.utcnow().isoformat() + "Z"
        current_job['end_time'] = None
        current_job['best_pt_path'] = None
        current_job['converted_model_path'] = None
        current_job['error_message'] = None
        # log_file is already set globally

        app.logger.info(f"Training pipeline initiated successfully. PID: {process.pid}")
        return jsonify({
            'status': 'started',
            'message': 'Training pipeline initiated.',
            'job_pid': process.pid,
            'log_file': str(TRAINING_LOG_FILE.resolve()),
            'job_details': current_job # Send back the current state
        }), 202 # Accepted

    except FileNotFoundError:
        app.logger.error(f"Script not found or python interpreter error for command: {' '.join(command)}")
        current_job['status'] = 'failed'
        current_job['error_message'] = 'Error executing pipeline script (FileNotFound).'
        return jsonify({'status': 'error', 'message': 'Error executing pipeline script (FileNotFound). Check server logs.'}), 500
    except Exception as e:
        app.logger.error(f"Failed to start training process: {str(e)}")
        current_job['status'] = 'failed'
        current_job['error_message'] = f'Failed to start training process: {str(e)}'
        return jsonify({'status': 'error', 'message': f'Failed to start training process: {str(e)}'}), 500

if __name__ == '__main__':
    # Configure basic logging for the Flask app itself
    # logging.basicConfig(level=logging.INFO) # Already configured by Flask's app.logger
    if not app.debug: # Ensure logging is configured for production if not in debug mode
        # import logging # logging is already imported via Flask's app.logger setup
        import logging # Explicitly import logging for StreamHandler and INFO
        stream_handler = logging.StreamHandler() # Use 'logging.StreamHandler'
        stream_handler.setLevel(logging.INFO)    # Use 'logging.INFO'
        app.logger.addHandler(stream_handler)
    app.logger.setLevel(logging.INFO) # Ensure app logger level is INFO
    
    # Note: Debug mode should be OFF in production
    app.run(debug=True, host='0.0.0.0', port=os.environ.get("FLASK_RUN_PORT", 5000))


# --- ROS Node Management API Endpoints (Continued) ---

@app.route('/ros/nodes', methods=['GET'])
def get_ros_nodes():
    conn = None
    try:
        # Proactively update statuses of nodes marked 'running' if their PIDs are dead
        app.logger.debug("Calling update_all_node_statuses before fetching nodes for GET /ros/nodes.")
        updated_ids = update_all_node_statuses(str(DATABASE_FILE.resolve()))
        if updated_ids:
            app.logger.info(f"Proactive status update: Nodes {updated_ids} were updated from 'running' to 'error' due to missing PIDs.")

        conn = get_db_connection(DATABASE_FILE)
        # Fetch all nodes after potential updates.
        # _dict_factory ensures these are dictionaries.
        nodes_for_response = conn.execute("SELECT * FROM ros_nodes ORDER BY id;").fetchall()
        
        return jsonify(nodes_for_response)
    except sqlite3.Error as e:
        app.logger.error(f"Database error in GET /ros/nodes: {e}")
        return jsonify({'status': 'error', 'message': 'Database error occurred while fetching ROS nodes.'}), 500
    except Exception as ex: # Catch any other unexpected errors
        app.logger.error(f"Unexpected error in GET /ros/nodes: {ex}")
        return jsonify({'status': 'error', 'message': 'An unexpected server error occurred.'}), 500
    finally:
        if conn:
            conn.close()

@app.route('/ros/nodes/<int:node_id>/enable', methods=['POST'])
def enable_ros_node(node_id):
    conn = None
    try:
        conn = get_db_connection(DATABASE_FILE)
        cursor = conn.cursor()
        cursor.execute("UPDATE ros_nodes SET is_enabled = 1 WHERE id = ?;", (node_id,))
        conn.commit()
        if cursor.rowcount == 0:
            return jsonify({'status': 'error', 'message': f'Node with ID {node_id} not found.'}), 404
        
        # Fetch and return the updated node
        updated_node = conn.execute("SELECT * FROM ros_nodes WHERE id = ?;", (node_id,)).fetchone()
        return jsonify({'status': 'success', 'message': f'Node {node_id} enabled.', 'node': updated_node})
    except sqlite3.Error as e:
        app.logger.error(f"Database error in POST /ros/nodes/{node_id}/enable: {e}")
        return jsonify({'status': 'error', 'message': 'Database error occurred.'}), 500
    except Exception as ex: 
        app.logger.error(f"Unexpected error in POST /ros/nodes/{node_id}/enable: {ex}")
        return jsonify({'status': 'error', 'message': 'An unexpected server error occurred.'}), 500
    finally:
        if conn:
            conn.close()

@app.route('/ros/nodes/<int:node_id>/disable', methods=['POST'])
def disable_ros_node(node_id):
    conn = None
    try:
        conn = get_db_connection(DATABASE_FILE)
        cursor = conn.cursor()
        cursor.execute("UPDATE ros_nodes SET is_enabled = 0 WHERE id = ?;", (node_id,))
        conn.commit()
        if cursor.rowcount == 0:
            return jsonify({'status': 'error', 'message': f'Node with ID {node_id} not found.'}), 404
        
        # Fetch and return the updated node
        updated_node = conn.execute("SELECT * FROM ros_nodes WHERE id = ?;", (node_id,)).fetchone()
        return jsonify({'status': 'success', 'message': f'Node {node_id} disabled.', 'node': updated_node})
    except sqlite3.Error as e:
        app.logger.error(f"Database error in POST /ros/nodes/{node_id}/disable: {e}")
        return jsonify({'status': 'error', 'message': 'Database error occurred.'}), 500
    except Exception as ex:
        app.logger.error(f"Unexpected error in POST /ros/nodes/{node_id}/disable: {ex}")
        return jsonify({'status': 'error', 'message': 'An unexpected server error occurred.'}), 500
    finally:
        if conn:
            conn.close()

@app.route('/ros/nodes/<int:node_id>/start', methods=['POST'])
def start_ros_node_endpoint(node_id):
    app.logger.info(f"Received request to start ROS node with ID: {node_id}")
    # DATABASE_FILE is already defined globally in app.py
    result = start_node(node_id, str(DATABASE_FILE.resolve()))
    
    # Extract the http_status_code from the result, default to 500 if not present
    # The start_node function is designed to include 'http_status_code' in its return dict.
    http_status_code = result.pop('http_status_code', 500) 
    
    return jsonify(result), http_status_code

@app.route('/ros/nodes/<int:node_id>/restart', methods=['POST'])
def restart_ros_node_endpoint(node_id):
    app.logger.info(f"Received request to RESTART ROS node with ID: {node_id}")
    # DATABASE_FILE is already defined globally in app.py
    result = restart_node(node_id, str(DATABASE_FILE.resolve()))
    
    # Extract the http_status_code from the result, default to 500 if not present
    # The restart_node function is designed to include 'http_status_code' in its return dict.
    http_status_code = result.pop('http_status_code', 500)
    
    return jsonify(result), http_status_code

@app.route('/ros/nodes/<int:node_id>/stop', methods=['POST'])
def stop_ros_node_endpoint(node_id):
    app.logger.info(f"Received request to stop ROS node with ID: {node_id}")
    # DATABASE_FILE is already defined globally in app.py
    result = stop_node(node_id, str(DATABASE_FILE.resolve()))
    
    # Extract the http_status_code from the result, default to 500 if not present
    # The stop_node function is designed to include 'http_status_code' in its return dict.
    http_status_code = result.pop('http_status_code', 500)
    
    return jsonify(result), http_status_code

# --- Training Pipeline API Endpoints (Existing) ---

@app.route('/training_log_stream')
def training_log_stream():
    def generate_log_content():
        # Check initial job status
        # Ensure current_job itself and its 'status' key exist to prevent KeyErrors
        if not current_job or 'status' not in current_job or current_job['status'] != 'running':
            yield f"data: No active training job or job is not running. Current status: {current_job.get('status', 'N/A')}.\n\n"
            return

        log_path_str = current_job.get('log_file')
        if not log_path_str:
            yield f"data: Log file path not configured for the current job.\n\n"
            return
        
        log_path = pathlib.Path(log_path_str) # Use pathlib for path operations
        if not log_path.exists(): # Use log_path.exists() which is more robust
            yield f"data: Log file {str(log_path)} not found.\n\n"
            return

        yield f"data: Connecting to log stream for job PID: {current_job.get('pid', 'N/A')}. Log file: {str(log_path)}\n\n"
        
        # First, stream existing content
        try:
            with open(log_path, 'r') as f:
                app.logger.info(f"Streaming existing content from {log_path}")
                for line in f: # Read all current lines
                    yield f"data: {line.strip()}\n\n"
                # f.tell() is now at the end of the file, which is what we want for the next open for tailing
                current_pos = f.tell() 
        except Exception as e:
            app.logger.error(f"Error reading existing log content: {e}")
            yield f"data: Error reading existing log content: {str(e)}\n\n"
            return
        
        app.logger.info(f"Finished streaming existing content from {log_path}. Starting to tail for new content.")

        # Tail for new content
        try:
            with open(log_path, 'r') as f:
                f.seek(current_pos) # Start reading from where we left off (end of initial read)
                
                while True:
                    # Check job status for termination
                    # Add a more direct check for 'pid' as well for robustness, 
                    # as status might not be immediately updated if process dies unexpectedly
                    if current_job['status'] != 'running':
                        final_status_message = f"Training job status changed to: {current_job['status']}."
                        if current_job['status'] == 'completed':
                            final_status_message += f" Best model: {current_job.get('best_pt_path', 'N/A')}."
                            if current_job.get('converted_model_path') and current_job.get('converted_model_path') != 'None':
                                final_status_message += f" Converted model: {current_job.get('converted_model_path')}."
                        elif current_job['status'] == 'failed':
                            final_status_message += f" Error: {current_job.get('error_message', 'Unknown error')}."
                        
                        app.logger.info(f"Log stream: {final_status_message} Job PID was {current_job.get('pid', 'N/A')}")
                        yield f"data: {final_status_message}\n\n"
                        yield f"event: job_terminated\ndata: {current_job['status']}\n\n" # Send a specific event
                        return # Exit generator

                    line = f.readline()
                    if not line:
                        time.sleep(0.1) # Wait for new lines
                        # Periodically check if the file path still exists (e.g. if logs are rotated/deleted)
                        if not log_path.exists():
                            app.logger.warning(f"Log file {log_path} disappeared. Stopping stream.")
                            yield f"data: Log file {log_path} disappeared. Stopping stream.\n\n"
                            return
                        continue
                    yield f"data: {line.strip()}\n\n"
        except Exception as e:
            app.logger.error(f"Error during log tailing: {e}")
            yield f"data: Error during log tailing: {str(e)}. Detaching.\n\n"
            return
                
    return Response(generate_log_content(), mimetype='text/event-stream')


@app.route('/training_status', methods=['GET'])
def training_status():
    # Make a copy for the response, but updates based on PID check will modify global current_job
    job_snapshot = current_job.copy()

    if job_snapshot['status'] == 'running' and job_snapshot['pid'] is not None:
        try:
            os.kill(job_snapshot['pid'], 0) # Check if process exists
            # If os.kill succeeds, process is still running.
            # No change to job_snapshot['status'] needed here for this case.
            app.logger.debug(f"Training job PID {job_snapshot['pid']} is still active.")
        except OSError: # Process does not exist (or permission issue, but usually means it's gone)
            app.logger.info(f"Training job PID {job_snapshot['pid']} no longer active. Post-processing logs.")
            # Update global current_job directly.
            current_job['status'] = 'postprocessing' # Intermediate state
            current_job['pid'] = None # Process is confirmed gone
            current_job['end_time'] = datetime.utcnow().isoformat() + "Z"

            # Now parse the log file for final results
            log_results = parse_final_output_from_log(current_job['log_file']) # Use path from current_job
            
            if log_results.get('best_pt_path'): # Success if best_pt_path is found
                current_job['status'] = 'completed'
                current_job['best_pt_path'] = log_results['best_pt_path']
                current_job['converted_model_path'] = log_results['converted_model_path']
                current_job['error_message'] = None # Clear previous errors if any
                app.logger.info(f"Job PID {job_snapshot['pid']} completed successfully. Paths: {log_results['best_pt_path']}, {log_results['converted_model_path']}")
            else:
                current_job['status'] = 'failed'
                current_job['error_message'] = log_results.get('error', 'Failed to parse final output from logs or training script did not complete successfully.')
                app.logger.error(f"Job PID {job_snapshot['pid']} failed. Error: {current_job['error_message']}")
            
            # Update snapshot with the new status from the globally updated current_job
            job_snapshot = current_job.copy()
        except Exception as e:
            # Catch any other unexpected errors during os.kill or subsequent logic
            app.logger.error(f"Unexpected error checking process status for PID {job_snapshot['pid']}: {e}")
            # Potentially mark as failed or unknown, or leave as running for next check
            # For now, we'll assume it might be a transient issue and not change status here
            # to avoid incorrectly marking a running job as failed due to a check error.

    return jsonify(job_snapshot)

@app.route('/config_options', methods=['GET'])
def config_options():
    """
    Provides frontend components with necessary configuration details,
    such as available models and conversion targets.
    """
    model_types = ["yolov5n.pt", "yolov5s.pt", "yolov5m.pt", "yolov5l.pt", "yolov5x.pt"]
    
    # Ideally, conversion_targets would be dynamically fetched from convert_manager.py or a config file.
    # For now, hardcoding based on current and potential future support.
    conversion_targets = ["oak_d_lite", "onnx_only"] # Assuming 'onnx_only' might be a future target
    
    default_training_parameters = {
        "epochs": 100,
        "imgsz": 416,
        "device": "cpu"
    }
    
    config_data = {
        "model_types": model_types,
        "conversion_targets": conversion_targets,
        "default_training_parameters": default_training_parameters
    }
    
    return jsonify(config_data), 200
