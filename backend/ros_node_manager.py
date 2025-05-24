import sqlite3
import subprocess
import os
import shlex
import json
from datetime import datetime, timezone # Ensure timezone for UTC
import logging
from pathlib import Path
import signal # For signal.SIGINT, signal.SIGTERM
import time   # For time.sleep()

# Assuming database_setup.py is in the same directory (backend/)
# This makes get_db_connection available for use within this module.
try:
    from .database_setup import get_db_connection
except ImportError:
    # Fallback for direct execution or if the . is not working in some contexts
    from database_setup import get_db_connection


# Configure logging for this module
logger = logging.getLogger(__name__)
# Basic configuration if run standalone, or rely on app's config if imported
if not logger.hasHandlers():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(module)s - %(levelname)s - %(message)s')


def start_node(node_id: int, db_path_str: str) -> dict:
    """
    Starts a ROS 2 node based on its configuration in the database.

    Args:
        node_id: The ID of the node to start.
        db_path_str: The string path to the SQLite database file.

    Returns:
        A dictionary with status, message, and potentially other details like PID or error code.
        Includes 'http_status_code' for the Flask app to use.
    """
    db_path = Path(db_path_str)
    conn = None
    node = None

    try:
        conn = get_db_connection(db_path)
        node = conn.execute("SELECT * FROM ros_nodes WHERE id = ?;", (node_id,)).fetchone()

        if not node:
            return {'status': 'error', 'message': f'Node with ID {node_id} not found.', 'http_status_code': 404}

        if not node['is_enabled']:
            return {'status': 'error', 'message': f'Node {node["node_name"]} (ID: {node_id}) is not enabled.', 'http_status_code': 409} # 409 Conflict

        # Check current status and PID
        if node['status'] == 'running' and node['pid'] is not None:
            try:
                os.kill(node['pid'], 0)  # Check if process exists
                logger.info(f"Node {node['node_name']} (PID: {node['pid']}) is already reported as running and process exists.")
                return {'status': 'info', 'message': f'Node {node["node_name"]} is already running with PID {node["pid"]}.', 'pid': node['pid'], 'http_status_code': 409} # 409 Conflict
            except OSError:
                logger.warning(f"Node {node['node_name']} (PID: {node['pid']}) was reported as running, but process not found. Clearing PID and attempting restart.")
                cursor = conn.cursor()
                cursor.execute("UPDATE ros_nodes SET status = 'stopped', pid = NULL WHERE id = ?;", (node_id,))
                conn.commit()
                # Refresh node data after update
                node = conn.execute("SELECT * FROM ros_nodes WHERE id = ?;", (node_id,)).fetchone()


        # Construct the command
        command = ['ros2', node['node_type'], node['package_name'], node['launch_file_or_executable']]
        
        # Parse ros_args
        if node['ros_args']:
            parsed_args = None
            try:
                # Attempt to parse as JSON first
                loaded_json_args = json.loads(node['ros_args'])
                if isinstance(loaded_json_args, list):
                    # Filter out non-string elements just in case, though ideally should be all strings
                    parsed_args = [str(arg) for arg in loaded_json_args]
                elif isinstance(loaded_json_args, dict):
                    # Convert dict to list of 'key:=value' strings
                    # This is a common pattern, but specific needs might vary.
                    parsed_args = [f"{k}:={v}" for k, v in loaded_json_args.items()]
                else:
                    # If JSON but not list or dict, treat as a single string arg (unlikely but possible)
                    logger.warning(f"ros_args for node {node['node_name']} was JSON but not a list/dict: {node['ros_args']}. Treating as single string.")
                    parsed_args = shlex.split(str(loaded_json_args))

            except json.JSONDecodeError:
                # Not JSON, so treat as a plain string and use shlex
                logger.debug(f"ros_args for node {node['node_name']} is not JSON. Using shlex: {node['ros_args']}")
                parsed_args = shlex.split(node['ros_args'])
            except Exception as e_args_parse: # Catch any other parsing error
                 logger.error(f"Error parsing ros_args for node {node['node_name']}: '{node['ros_args']}'. Error: {e_args_parse}")
                 return {'status': 'error', 'message': f"Error parsing ros_args: {e_args_parse}", 'http_status_code': 400}

            if parsed_args:
                command.extend(parsed_args)
        
        logger.info(f"Constructed command for node {node['node_name']}: {' '.join(command)}")

        # Environment setup
        ros_env = os.environ.copy()
        # Potentially add/modify environment variables like ROS_DOMAIN_ID, RMW_IMPLEMENTATION, etc.
        # Example: ros_env['ROS_DOMAIN_ID'] = '42'

        # Execute the process
        # Using start_new_session=True to ensure the process is detached
        # and won't be killed if the Flask app (or its worker) dies.
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL, # Redirect stdout to /dev/null
            stderr=subprocess.DEVNULL, # Redirect stderr to /dev/null
            env=ros_env,
            start_new_session=True # Creates a new process group
        )
        
        pid = process.pid
        logger.info(f"Node {node['node_name']} (ID: {node_id}) started with PID: {pid}.")

        # Update database
        cursor = conn.cursor()
        current_utc_time = datetime.now(timezone.utc).isoformat()
        cursor.execute(
            "UPDATE ros_nodes SET status = 'running', pid = ?, last_started = ? WHERE id = ?;",
            (pid, current_utc_time, node_id)
        )
        conn.commit()

        return {
            'status': 'success', 
            'message': f'Node {node["node_name"]} started successfully.', 
            'pid': pid, 
            'node_id': node_id,
            'node_name': node['node_name'],
            'last_started': current_utc_time,
            'http_status_code': 200
        }

    except sqlite3.Error as e:
        logger.error(f"Database error starting node {node_id if node_id else 'Unknown'}: {e}")
        return {'status': 'error', 'message': f'Database error: {e}', 'http_status_code': 500}
    except subprocess.SubprocessError as e: # Catch errors from Popen itself
        logger.error(f"Subprocess error starting node {node_id if node_id else 'Unknown'} (Name: {node['node_name'] if node else 'N/A'}): {e}")
        # Attempt to revert status in DB if node data was fetched
        if conn and node:
            try:
                cursor = conn.cursor()
                cursor.execute("UPDATE ros_nodes SET status = 'error', pid = NULL WHERE id = ?;", (node_id,))
                conn.commit()
            except sqlite3.Error as db_err:
                 logger.error(f"Failed to update node status to 'error' after subprocess failure: {db_err}")
        return {'status': 'error', 'message': f'Failed to start subprocess for node {node["node_name"] if node else ""}: {e}', 'http_status_code': 500}
    except Exception as e:
        logger.error(f"Unexpected error starting node {node_id if node_id else 'Unknown'}: {e}")
        return {'status': 'error', 'message': f'An unexpected server error occurred: {e}', 'http_status_code': 500}
    finally:
        if conn:
            conn.close()

if __name__ == '__main__':
    # This section is for testing the module directly.
    # Ensure you have a database initialized, e.g., by running database_setup.py first.
    
    # Determine the default DB path, assuming this script is in backend/
    # and database_setup.py defines DEFAULT_DB_PATH correctly relative to project structure
    try:
        from database_setup import DEFAULT_DB_PATH
        test_db_path = DEFAULT_DB_PATH
    except ImportError:
        # Fallback if relative import fails (e.g. if structure is different or ran from elsewhere)
        # This assumes database_setup.py is in the same directory for direct execution.
        logger.warning("Could not import DEFAULT_DB_PATH from database_setup, using a local default for testing.")
        test_db_path = Path(__file__).parent / 'ros_nodes.sqlite' # Fallback path

    if not test_db_path.exists():
        logger.error(f"Test database {test_db_path} not found. Please run database_setup.py first.")
    else:
        logger.info(f"Using test database: {test_db_path}")
        
        # Example: Try to start node with ID 1 (assuming it exists and is enabled)
        node_to_start_id = 1 
        logger.info(f"\n--- Attempting to start node with ID: {node_to_start_id} ---")
        result = start_node(node_to_start_id, str(test_db_path.resolve()))
        logger.info(f"Start node result: {result}")

        # Example: Try to start an already "running" node (if the previous was successful)
        if result.get('status') == 'success':
            logger.info(f"\n--- Attempting to start node ID {node_to_start_id} again (should be 'already running' or PID cleared) ---")
            result_again = start_node(node_to_start_id, str(test_db_path.resolve()))
            logger.info(f"Second start attempt result: {result_again}")

        # Example: Try to start a non-existent node
        non_existent_node_id = 999
        logger.info(f"\n--- Attempting to start non-existent node ID: {non_existent_node_id} ---")
        result_non_existent = start_node(non_existent_node_id, str(test_db_path.resolve()))
        logger.info(f"Non-existent node start result: {result_non_existent}")

        # Example: Try to start a disabled node (Manually disable one in DB for this test if needed)
        # Assuming node ID 3 was set as disabled (is_enabled=0) in INITIAL_NODES
        disabled_node_id = 3 
        logger.info(f"\n--- Attempting to start disabled node ID: {disabled_node_id} ---")
        # You might need to ensure node 3 is actually disabled in your DB for this to be a good test.
        # E.g., by running: UPDATE ros_nodes SET is_enabled = 0 WHERE id = 3;
        result_disabled = start_node(disabled_node_id, str(test_db_path.resolve()))
        logger.info(f"Disabled node start result: {result_disabled}")

        # Note: To fully test, you'd also need to manually check if processes are running
        # using 'ps aux | grep ros2' or similar, and verify DB state.
        # This direct execution doesn't include stopping nodes. They will keep running.
        logger.info("\n--- Test sequence finished ---")
        logger.info("Remember to manually stop any ROS nodes started if they are not needed.")


# --- Status Checking and Proactive Updates ---

def check_node_process_status(node: dict) -> str:
    """
    Checks the OS-level status of a process if the node is marked as 'running'.

    Args:
        node: A dictionary representing a node (must contain 'pid' and 'status').

    Returns:
        The actual status: 'running', 'error' (if process not found but should be running),
        or the original node['status'].
    """
    if not node: # Defensive check
        return 'unknown' 
        
    current_db_status = node.get('status')
    pid = node.get('pid')

    if current_db_status == 'running' and pid is not None:
        try:
            os.kill(pid, 0)  # Check if process exists
            return 'running' # Process is alive
        except OSError:
            logger.warning(f"Process check: Node '{node.get('node_name', 'N/A')}' (PID: {pid}) was marked 'running' but process not found.")
            return 'error' # Process not found, though DB says it's running
        except Exception as e: # Other potential errors with os.kill
            logger.error(f"Error checking process status for PID {pid}: {e}")
            return 'unknown' # Status cannot be determined reliably
    return current_db_status # Return original status if not 'running' or no PID


def update_all_node_statuses(db_path_str: str) -> list[int]:
    """
    Checks all nodes currently marked as 'running' in the database.
    If the OS process for a node is not found, its status is updated in the DB
    to 'error', its PID is cleared, and 'last_stopped' is set.

    Args:
        db_path_str: Path to the SQLite database.

    Returns:
        A list of node IDs whose statuses were updated from 'running' to 'error'.
    """
    db_path = Path(db_path_str)
    conn = None
    updated_node_ids = []
    
    logger.debug("Executing update_all_node_statuses...")

    try:
        conn = get_db_connection(db_path)
        # Fetch all nodes that are supposed to be running
        # Using _dict_factory ensures nodes are dicts
        running_nodes = conn.execute("SELECT * FROM ros_nodes WHERE status = 'running' AND pid IS NOT NULL;").fetchall()

        if not running_nodes:
            logger.debug("No nodes currently marked as 'running' with a PID. No status updates needed.")
            return updated_node_ids

        logger.debug(f"Found {len(running_nodes)} nodes marked as 'running' to check.")

        for node_dict in running_nodes:
            # The check_node_process_status function is simple enough that we can inline its core logic here
            # to avoid repeated DB calls if it were more complex. But for consistency, we can use it.
            # However, check_node_process_status is designed for a single node dict, not DB interaction.
            # The logic for updating DB is better here.
            
            pid_to_check = node_dict.get('pid')
            node_id_to_update = node_dict.get('id')
            node_name_for_log = node_dict.get('node_name', f"ID {node_id_to_update}")

            if pid_to_check is None: # Should not happen due to query, but defensive
                logger.warning(f"Node {node_name_for_log} marked running but has no PID. Skipping check, consider manual review.")
                continue

            try:
                os.kill(pid_to_check, 0) # Check if process exists
                # If no error, process is alive. No DB change needed for this node.
                logger.debug(f"Node {node_name_for_log} (PID: {pid_to_check}) is confirmed running.")
            except OSError:
                # Process is not alive, but DB says 'running'. Correct this.
                logger.info(f"Node {node_name_for_log} (PID: {pid_to_check}) was marked 'running' but process not found. Updating status to 'error'.")
                cursor = conn.cursor()
                current_utc_time = datetime.now(timezone.utc).isoformat()
                cursor.execute(
                    "UPDATE ros_nodes SET status = 'error', pid = NULL, last_stopped = ? WHERE id = ?;",
                    (current_utc_time, node_id_to_update)
                )
                conn.commit()
                updated_node_ids.append(node_id_to_update)
            except Exception as e_kill: # Other errors during os.kill
                 logger.error(f"Error checking process status for node {node_name_for_log} (PID {pid_to_check}): {e_kill}. Status not updated.")


        if updated_node_ids:
            logger.info(f"Updated status to 'error' for the following node IDs: {updated_node_ids}")
        else:
            logger.debug("No running nodes found to have unexpectedly stopped.")

    except sqlite3.Error as e:
        logger.error(f"Database error in update_all_node_statuses: {e}")
    except Exception as ex:
        logger.error(f"Unexpected error in update_all_node_statuses: {ex}")
    finally:
        if conn:
            conn.close()
            logger.debug("Database connection closed after update_all_node_statuses.")
    
    return updated_node_ids


def restart_node(node_id: int, db_path_str: str) -> dict:
    """
    Restarts a ROS 2 node by first stopping it and then starting it.

    Args:
        node_id: The ID of the node to restart.
        db_path_str: The string path to the SQLite database file.

    Returns:
        A dictionary indicating the outcome of the restart operation.
    """
    logger.info(f"--- Received request to RESTART node ID: {node_id} ---")

    # Step 1: Stop the node
    logger.info(f"Restart Step 1: Attempting to stop node ID: {node_id}")
    stop_result = stop_node(node_id, db_path_str)
    stop_http_status = stop_result.get('http_status_code', 500) # Default to 500 if not set

    logger.info(f"Stop node result for node ID {node_id}: {stop_result}")

    # Check if stopping failed critically
    if stop_http_status == 404: # Node not found
        logger.error(f"Restart failed: Node ID {node_id} not found during stop attempt.")
        return stop_result # Return the original error from stop_node
    
    if stop_http_status == 500: # Server error during stop
        logger.error(f"Restart failed: Server error occurred while trying to stop node ID {node_id}.")
        return stop_result

    # If stop_http_status is 200 (stopped successfully, was already stopped, or warning), proceed to start.
    # A 'warning' status from stop_node (e.g., termination not confirmed) is still a go for trying to start.
    
    logger.info(f"Restart Step 2: Attempting to start node ID: {node_id} (after stop attempt completed with status: {stop_result.get('status', 'N/A')})")
    start_result = start_node(node_id, db_path_str)
    logger.info(f"Start node result for node ID {node_id} (during restart): {start_result}")
    
    # The result of start_node is the final outcome of the restart.
    # It contains its own http_status_code.
    # We might want to augment the message if the stop operation had a warning.
    if stop_result.get('status') == 'warning' and start_result.get('status') == 'success':
        start_result['message'] = f"Node restarted (stop warning: {stop_result.get('message', '')}; start success: {start_result.get('message', '')})"
    elif stop_result.get('status') == 'info' and 'already stopped' in stop_result.get('message', '') and start_result.get('status') == 'success':
         start_result['message'] = f"Node started (was previously stopped). {start_result.get('message','')}"


    return start_result


def stop_node(node_id: int, db_path_str: str) -> dict:
    """
    Stops a running ROS 2 node based on its configuration in the database.

    Args:
        node_id: The ID of the node to stop.
        db_path_str: The string path to the SQLite database file.

    Returns:
        A dictionary with status, message, and potentially other details.
        Includes 'http_status_code' for the Flask app to use.
    """
    db_path = Path(db_path_str)
    conn = None
    node = None
    pid_to_stop = None

    try:
        conn = get_db_connection(db_path)
        node = conn.execute("SELECT id, node_name, status, pid FROM ros_nodes WHERE id = ?;", (node_id,)).fetchone()

        if not node:
            return {'status': 'error', 'message': f'Node with ID {node_id} not found.', 'http_status_code': 404}

        pid_to_stop = node.get('pid') # Store pid before any DB updates might clear it

        if node['status'] != 'running' or pid_to_stop is None:
            # If already marked as stopped but PID somehow exists, try to clean up DB if process is gone
            if pid_to_stop is not None:
                try:
                    os.kill(pid_to_stop, 0) # Check if process is unexpectedly alive
                    logger.warning(f"Node {node['node_name']} (ID: {node_id}) is marked '{node['status']}' but PID {pid_to_stop} is alive. Attempting to stop.")
                    # Proceed to stop logic
                except OSError: # Process not found
                    logger.info(f"Node {node['node_name']} (ID: {node_id}) is '{node['status']}', and PID {pid_to_stop} is not active. Correcting DB if needed.")
                    if node['status'] != 'stopped' or node['pid'] is not None: # If DB state is inconsistent
                        cursor = conn.cursor()
                        current_utc_time = datetime.now(timezone.utc).isoformat()
                        cursor.execute("UPDATE ros_nodes SET status = 'stopped', pid = NULL, last_stopped = ? WHERE id = ?;", (current_utc_time, node_id))
                        conn.commit()
                    return {'status': 'info', 'message': f'Node {node["node_name"]} was already stopped or not running.', 'http_status_code': 200}
            else: # No PID, and status is not 'running'
                 return {'status': 'info', 'message': f'Node {node["node_name"]} is not running (status: {node["status"]}).', 'http_status_code': 200}


        logger.info(f"Attempting to stop node {node['node_name']} (PID: {pid_to_stop}).")
        process_terminated = False

        # Attempt SIGINT
        try:
            logger.debug(f"Sending SIGINT to PID {pid_to_stop} for node {node['node_name']}.")
            os.kill(pid_to_stop, signal.SIGINT)
            # Wait for termination (e.g., up to 3 seconds)
            for _ in range(6): # 6 * 0.5s = 3s
                time.sleep(0.5)
                os.kill(pid_to_stop, 0) # Check if process exists
            logger.warning(f"Node {node['node_name']} (PID: {pid_to_stop}) did not terminate via SIGINT within 3s.")
        except OSError: # Process already terminated
            process_terminated = True
            logger.info(f"Node {node['node_name']} (PID: {pid_to_stop}) terminated after SIGINT.")
        except Exception as e_sigint: # Other errors during SIGINT (permissions etc.)
            logger.error(f"Error sending SIGINT to PID {pid_to_stop}: {e_sigint}")
            # Decide if we should proceed to SIGTERM or fail here. For now, proceed.

        # Attempt SIGTERM if not terminated
        if not process_terminated:
            try:
                logger.warning(f"Sending SIGTERM to PID {pid_to_stop} for node {node['node_name']}.")
                os.kill(pid_to_stop, signal.SIGTERM)
                # Wait for termination (e.g., up to 2 more seconds)
                for _ in range(4): # 4 * 0.5s = 2s
                    time.sleep(0.5)
                    os.kill(pid_to_stop, 0)
                logger.error(f"Node {node['node_name']} (PID: {pid_to_stop}) did not terminate via SIGTERM within 2s. It might require manual cleanup or SIGKILL.")
                # Even if it doesn't confirm termination, we'll update DB as 'stopped'
                # as we've issued termination signals. A more robust system might have 'error_stopping'.
            except OSError: # Process terminated
                process_terminated = True
                logger.info(f"Node {node['node_name']} (PID: {pid_to_stop}) terminated after SIGTERM.")
            except Exception as e_sigterm:
                logger.error(f"Error sending SIGTERM to PID {pid_to_stop}: {e_sigterm}")
                # Update DB as if stopped, but log the error.

        # Update database
        cursor = conn.cursor()
        current_utc_time = datetime.now(timezone.utc).isoformat()
        cursor.execute(
            "UPDATE ros_nodes SET status = 'stopped', pid = NULL, last_stopped = ? WHERE id = ?;",
            (current_utc_time, node_id)
        )
        conn.commit()
        
        if process_terminated:
             return {
                'status': 'success', 
                'message': f'Node {node["node_name"]} stopped successfully.', 
                'node_id': node_id,
                'last_stopped': current_utc_time,
                'http_status_code': 200
            }
        else: # Signals sent, but termination not confirmed by os.kill check
            return {
                'status': 'warning', # Or 'success' if we consider sending signal as success
                'message': f'Termination signals sent to node {node["node_name"]} (PID: {pid_to_stop}). Termination within timeout not confirmed. Status updated to stopped.',
                'node_id': node_id,
                'last_stopped': current_utc_time,
                'http_status_code': 200 # Still a 200 as the action was taken from server side.
            }

    except sqlite3.Error as e:
        logger.error(f"Database error stopping node {node_id}: {e}")
        return {'status': 'error', 'message': f'Database error: {e}', 'http_status_code': 500}
    except Exception as e:
        logger.error(f"Unexpected error stopping node {node_id}: {e}")
        # Ensure node details are available if error happens after fetching node
        node_name_for_msg = node['node_name'] if node else f"ID {node_id}"
        return {'status': 'error', 'message': f'An unexpected server error occurred while stopping node {node_name_for_msg}: {e}', 'http_status_code': 500}
    finally:
        if conn:
            conn.close()
