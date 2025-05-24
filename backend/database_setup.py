import sqlite3
from pathlib import Path
import os
import json # For ros_args if stored as JSON
import logging

# Configure logging
# Using basicConfig for simplicity in this standalone script.
# In a larger app, this might be part of a broader logging setup.
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

DEFAULT_DB_DIR = Path.home() / '.my_robot_app_data'
DEFAULT_DB_PATH = DEFAULT_DB_DIR / 'ros_nodes.sqlite'

# To add more nodes, simply add new dictionaries to this list.
# Ensure 'node_name' is unique.
# 'ros_args' can be a simple string or a dictionary (which will be stored as JSON).
INITIAL_NODES = [
    {
        'node_name': "flir_camera_node",
        'description': "FLIR Blackfly S GigE camera driver",
        'package_name': "flir_spinnaker_ros2", # Example package name
        'launch_file_or_executable': "flir_camera.launch.py", # Example launch file
        'node_type': "launch",
        'ros_args': "", # Example: could be {'camera_index': 0} or "camera_index:=0"
        'is_enabled': 0,
        'status': 'stopped', # Default status
        'pid': None,
        'last_started': None,
        'last_stopped': None,
        'log_file': None
    },
    {
        'node_name': "usb_camera_node",
        'description': "Standard USB camera driver",
        'package_name': "usb_cam", # Example package name
        'launch_file_or_executable': "usb_cam_node_exe", # Renamed to avoid confusion with node_name, typically it's an executable name
        'node_type': "run",
        'ros_args': "--params-file /path/to/your/usb_cam_params.yaml", # Example actual string
        'is_enabled': 1,
        'status': 'stopped',
        'pid': None,
        'last_started': None,
        'last_stopped': None,
        'log_file': None
    },
    {
        'node_name': "tagslam_node",
        'description': "TagSLAM localization node",
        'package_name': "tagslam_ros2", # Example package name
        'launch_file_or_executable': "tagslam.launch.py", # Example launch file
        'node_type': "launch",
        'ros_args': {'map_name': 'my_office_map', 'use_sim_time': True}, # Example dict for JSON storage
        'is_enabled': 0,
        'status': 'stopped',
        'pid': None,
        'last_started': None,
        'last_stopped': None,
        'log_file': None
    }
]

def _dict_factory(cursor, row):
    """Helper function to return query results as dictionaries."""
    fields = [column[0] for column in cursor.description]
    return {key: value for key, value in zip(fields, row)}

def get_db_connection(db_path: Path) -> sqlite3.Connection:
    """Establishes a connection to the SQLite database."""
    try:
        conn = sqlite3.connect(db_path, detect_types=sqlite3.PARSE_DECLTYPES | sqlite3.PARSE_COLNAMES)
        conn.row_factory = _dict_factory # Return rows as dictionaries
        return conn
    except sqlite3.Error as e:
        logger.error(f"Error connecting to database {db_path}: {e}")
        raise # Re-raise the exception to be handled by the caller

def initialize_database(db_path_override: str | Path | None = None):
    """
    Initializes the SQLite database: creates the directory, database file (if needed),
    the 'ros_nodes' table (if needed), and populates it with initial data if newly created.
    """
    if db_path_override:
        db_path = Path(db_path_override)
    else:
        db_path = DEFAULT_DB_PATH
    
    logger.info(f"Initializing database at: {db_path.resolve()}")

    # Ensure the directory for the database file exists
    try:
        db_path.parent.mkdir(parents=True, exist_ok=True)
        logger.info(f"Ensured database directory exists: {db_path.parent.resolve()}")
    except OSError as e:
        logger.error(f"Error creating database directory {db_path.parent.resolve()}: {e}")
        return # Cannot proceed without the directory

    conn = None # Initialize conn to None for the finally block
    try:
        conn = get_db_connection(db_path)
        cursor = conn.cursor()

        # Check if 'ros_nodes' table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='ros_nodes';")
        table_exists = cursor.fetchone()

        if not table_exists:
            logger.info(f"Table 'ros_nodes' does not exist. Creating table...")
            # Create 'ros_nodes' table
            # Using TEXT for TIMESTAMP fields for simplicity with sqlite3, will store ISO format strings.
            cursor.execute('''
            CREATE TABLE ros_nodes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                node_name TEXT NOT NULL UNIQUE,
                description TEXT,
                package_name TEXT NOT NULL,
                launch_file_or_executable TEXT NOT NULL,
                node_type TEXT NOT NULL CHECK(node_type IN ('launch', 'run')),
                ros_args TEXT,
                is_enabled INTEGER NOT NULL DEFAULT 0,
                status TEXT NOT NULL DEFAULT 'stopped', 
                pid INTEGER,
                last_started TEXT, 
                last_stopped TEXT, 
                log_file TEXT
            )
            ''')
            conn.commit()
            logger.info(f"Table 'ros_nodes' created successfully in {db_path.resolve()}")

            # Populate with initial data since the table was just created
            logger.info("Populating 'ros_nodes' with initial data...")
            for node_data in INITIAL_NODES:
                # Ensure all expected keys are present, providing defaults for optional ones
                # This makes adding to INITIAL_NODES less error-prone
                current_node = {
                    'description': node_data.get('description', ''),
                    'ros_args': node_data.get('ros_args', ''), # Default to empty string
                    'is_enabled': node_data.get('is_enabled', 0),
                    'status': node_data.get('status', 'stopped'),
                    'pid': node_data.get('pid'), # None if not present
                    'last_started': node_data.get('last_started'), # None if not present
                    'last_stopped': node_data.get('last_stopped'), # None if not present
                    'log_file': node_data.get('log_file') # None if not present
                }
                
                # Merge required fields
                current_node.update({
                    'node_name': node_data['node_name'],
                    'package_name': node_data['package_name'],
                    'launch_file_or_executable': node_data['launch_file_or_executable'],
                    'node_type': node_data['node_type'],
                })

                # Convert ros_args dict to JSON string if it's a dict
                if isinstance(current_node['ros_args'], dict):
                    current_node['ros_args'] = json.dumps(current_node['ros_args'])
                
                try:
                    cursor.execute('''
                    INSERT INTO ros_nodes (
                        node_name, description, package_name, launch_file_or_executable, 
                        node_type, ros_args, is_enabled, status, pid, 
                        last_started, last_stopped, log_file
                    ) VALUES (
                        :node_name, :description, :package_name, :launch_file_or_executable, 
                        :node_type, :ros_args, :is_enabled, :status, :pid,
                        :last_started, :last_stopped, :log_file
                    )
                    ''', current_node)
                except sqlite3.IntegrityError as ie:
                    logger.warning(f"Could not insert node '{current_node['node_name']}' due to integrity error (likely already exists or UNIQUE constraint failed): {ie}")
                except sqlite3.Error as db_err:
                    logger.error(f"Database error inserting node '{current_node['node_name']}': {db_err}")

            conn.commit()
            logger.info(f"Successfully populated 'ros_nodes' with initial data.")
        else:
            logger.info(f"Table 'ros_nodes' already exists in {db_path.resolve()}. No action taken on table structure or initial data.")

    except sqlite3.Error as e:
        logger.error(f"An SQLite error occurred: {e}")
        if conn: # Rollback changes if any error occurred during transaction
            conn.rollback()
    except Exception as ex: # Catch any other unexpected errors
        logger.error(f"An unexpected error occurred: {ex}")
        if conn:
            conn.rollback()
    finally:
        if conn:
            conn.close()
            logger.info(f"Database connection to {db_path.resolve()} closed.")

if __name__ == '__main__':
    logger.info(f"Running database_setup.py directly. Initializing ROS Node database at default location: {DEFAULT_DB_PATH.resolve()}")
    initialize_database()

    # Example: Test connection and fetch data to verify (optional)
    # logger.info("Verifying database content...")
    # try:
    #     conn_verify = get_db_connection(DEFAULT_DB_PATH)
    #     nodes = conn_verify.execute("SELECT * FROM ros_nodes").fetchall()
    #     conn_verify.close()
    #     if nodes:
    #         logger.info(f"Found {len(nodes)} nodes in the database:")
    #         for node in nodes:
    #             logger.info(f"  Node: {dict(node)}")
    #     else:
    #         logger.info("No nodes found in the database after initialization (unexpected).")
    # except Exception as e_verify:
    #     logger.error(f"Error verifying database content: {e_verify}")
