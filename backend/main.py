import rclpy
from rclpy.node import Node
from fastapi import FastAPI, Request, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import yaml
import os

app = FastAPI()

# CORS Middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

# Pydantic Model for Camera Configuration
class Camera(BaseModel):
    name: Optional[str] = "New Camera"
    serial: str # Mandatory
    topic: Optional[str] = ""
    debug: Optional[bool] = False
    compute_brightness: Optional[bool] = True
    dump_node_map: Optional[bool] = False
    gain_auto: Optional[str] = 'Off' # Enum: Off, Once, Continuous
    exposure_auto: Optional[str] = 'Off' # Enum: Off, Once, Continuous
    offset_x: Optional[int] = 0
    offset_y: Optional[int] = 0
    image_width: Optional[int] = 2048
    image_height: Optional[int] = 1536
    pixel_format: Optional[str] = 'Mono8'  # Enum: BayerRG8, RGB8, Mono8 etc.
    frame_rate_continuous: Optional[bool] = True
    frame_rate: Optional[float] = 100.0
    trigger_mode: Optional[str] = 'On' # Enum: On, Off
    chunk_mode_active: Optional[bool] = True
    chunk_selector_frame_id: Optional[str] = 'FrameID'
    chunk_enable_frame_id: Optional[bool] = True
    chunk_selector_exposure_time: Optional[str] = 'ExposureTime'
    chunk_enable_exposure_time: Optional[bool] = True
    chunk_selector_gain: Optional[str] = 'Gain'
    chunk_enable_gain: Optional[bool] = True
    chunk_selector_timestamp: Optional[str] = 'Timestamp'
    chunk_enable_timestamp: Optional[bool] = True
    adjust_timestamp: Optional[bool] = True
    gain: Optional[int] = 0 # Kept, ensure it's not fighting gain_auto
    exposure_time: Optional[int] = 9000 # Kept, ensure it's not fighting exposure_auto
    line2_selector: Optional[str] = 'Line2'
    line2_v33enable: Optional[bool] = False
    line3_selector: Optional[str] = 'Line3'
    line3_linemode: Optional[str] = 'Input' # Enum: Input, Output
    trigger_selector: Optional[str] = 'FrameStart' # Enum: FrameStart, FrameEnd etc.
    trigger_source: Optional[str] = 'Line3' # Enum: Line3, Software etc.
    trigger_delay: Optional[int] = 9 # Microseconds
    trigger_overlap: Optional[str] = 'ReadOut' # Enum: ReadOut, Off etc.

# Global variables
ros_node: Optional[Node] = None
ros_connected = False
camera_configs: List[Camera] = []
CONFIG_FILE_PATH = "backend/camera_config.yaml"

# ROS 2 Integration
def check_ros_connection():
    global ros_connected, ros_node
    if not rclpy.ok():
        try:
            rclpy.init()
        except Exception:
            ros_connected = False
            return
    
    if ros_node is None: # Create node only if it doesn't exist
        try:
            ros_node = Node("fastapi_ros_check")
        except Exception:
            ros_connected = False
            if rclpy.ok(): # shutdown rclpy if node creation failed
                 rclpy.shutdown()
            return

    try:
        # Try spinning the node once to check if the context is valid
        rclpy.spin_once(ros_node, timeout_sec=0.1)
        ros_connected = True
    except Exception:
        ros_connected = False
        # If spinning fails, the node might be unusable, so clean it up
        if ros_node:
            ros_node.destroy_node()
        ros_node = None
        if rclpy.ok():
            rclpy.shutdown()


@app.on_event("startup")
async def startup_event():
    try:
        rclpy.init()
        global ros_node
        ros_node = Node("fastapi_ros_node") # Keep a persistent node
        global ros_connected
        ros_connected = True
    except Exception as e:
        print(f"Error initializing RCLPY: {e}")
        ros_connected = False
    load_config_from_yaml()

@app.on_event("shutdown")
async def shutdown_event():
    global ros_node
    if ros_node:
        ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

# Configuration Storage
def load_config_from_yaml():
    global camera_configs
    try:
        if os.path.exists(CONFIG_FILE_PATH):
            with open(CONFIG_FILE_PATH, 'r') as f:
                data = yaml.safe_load(f)
                if data:
                    camera_configs = [Camera(**item) for item in data]
    except FileNotFoundError:
        print(f"Config file not found: {CONFIG_FILE_PATH}")
        camera_configs = []
    except yaml.YAMLError as e:
        print(f"Error parsing YAML config: {e}")
        camera_configs = []

def save_config_to_yaml():
    global camera_configs
    try:
        with open(CONFIG_FILE_PATH, 'w') as f:
            # Pydantic v2 uses model_dump(). Ensure all fields are included by default.
            yaml_data = [config.model_dump() for config in camera_configs]
            yaml.dump(yaml_data, f, indent=4)
    except Exception as e:
        print(f"Error saving YAML config: {e}")


@app.get("/")
async def read_root():
    return {"message": "Backend is running"}

# API Endpoints
@app.get("/status")
async def get_status():
    check_ros_connection()
    return {"connected": ros_connected}

@app.get("/config", response_model=List[Camera])
async def get_config():
    return camera_configs

@app.post("/config")
async def update_config(configs: List[Camera], request: Request):
    global camera_configs
    camera_configs = configs
    save_config_to_yaml()
    return {"message": "Configuration saved successfully"}
