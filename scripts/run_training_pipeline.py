# Copyright (C) 2024 Intel Corporation
# SPDX-License-Identifier: MIT

"""
Orchestrates the YOLOv5 training and model conversion process.
"""

import argparse
import logging
import os
import pathlib
import subprocess
import sys
import datetime

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', stream=sys.stdout)
# Configure all loggers to output to stdout, so `python -u` handles it consistently.
# This also means direct print() statements and logger output will be in the same stream.
logger = logging.getLogger(__name__)

def run_subprocess_realtime(command: list[str], cwd: str | None = None) -> tuple[bool, str, str]:
    """
    Runs a subprocess command, logs its output line by line in real-time,
    and returns (success, full_stdout, full_stderr).
    """
    logger.info(f"Executing command: {' '.join(command)}")
    if cwd:
        logger.info(f"Working directory: {cwd}")
    
    full_stdout_lines = []
    full_stderr_lines = [] # Only used if stderr is not redirected to STDOUT

    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, # Redirect stderr to stdout for simplicity in reading
            text=True,
            bufsize=1,  # Line-buffered
            universal_newlines=True, # Ensure text mode and proper newline handling
            cwd=cwd
        )

        # Real-time logging of stdout (and stderr if redirected)
        if process.stdout:
            for line in iter(process.stdout.readline, ''):
                cleaned_line = line.strip()
                if cleaned_line: # Avoid logging empty lines if desired, but often useful to keep them
                    logger.info(cleaned_line) # Log each line as it comes
                full_stdout_lines.append(line)
            process.stdout.close()
        
        return_code = process.wait()

        full_stdout = "".join(full_stdout_lines)
        # Since stderr is redirected to stdout, full_stderr will be empty if collected separately
        # If not redirecting, a similar loop for process.stderr would be needed.
        full_stderr = "" 

        if return_code == 0:
            logger.info(f"Command executed successfully (Return Code: {return_code}).")
            return True, full_stdout, full_stderr # full_stderr will be empty here
        else:
            logger.error(f"Command failed with return code {return_code}.")
            # full_stdout already contains the error messages if stderr was redirected
            return False, full_stdout, full_stderr

    except FileNotFoundError:
        logger.error(f"Command not found: {command[0]}. Ensure it is in PATH or the path is correct.")
        return False, "", "FileNotFoundError: Command not found"
    except Exception as e:
        logger.error(f"An unexpected error occurred while running the command: {str(e)}")
        return False, "", str(e)

def main():
    parser = argparse.ArgumentParser(description="Run YOLOv5 training and conversion pipeline.")
    parser.add_argument("--data_yaml", type=str, required=True, help="Path to the data.yaml file.")
    parser.add_argument("--output_base_dir", type=str, default="./training_output", help="Base directory for all outputs.")
    parser.add_argument("--model_type", type=str, default="yolov5n.pt", help="YOLOv5 model type (e.g., yolov5n.pt).")
    parser.add_argument("--epochs", type=int, default=100, help="Number of training epochs.")
    parser.add_argument("--imgsz", type=int, default=416, help="Image size for training.")
    parser.add_argument("--device", type=str, default="cpu", help="Training device ('cpu' or GPU ID like '0').")
    parser.add_argument("--conversion_target", type=str, help="Target for model conversion (e.g., 'oak_d_lite'). If not provided, no conversion.")
    parser.add_argument("--run_name", type=str, help="Optional name for the training run. Timestamp-based if not provided.")

    args = parser.parse_args()

    # --- Setup Directories ---
    if args.run_name:
        run_name_str = args.run_name
    else:
        run_name_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    run_dir = pathlib.Path(args.output_base_dir) / run_name_str
    training_subdir = run_dir / "training"
    converted_subdir = run_dir / "converted"

    try:
        run_dir.mkdir(parents=True, exist_ok=True)
        training_subdir.mkdir(parents=True, exist_ok=True)
        converted_subdir.mkdir(parents=True, exist_ok=True)
        logger.info(f"Run directory created: {run_dir.resolve()}")
    except OSError as e:
        logger.error(f"Failed to create directories: {e}")
        sys.exit(1)

    # --- Training ---
    script_dir = pathlib.Path(__file__).parent.resolve()
    train_script_path = script_dir / "train_yolov5" / "train.py" # Path to yolov5/train.py
    
    if not train_script_path.is_file():
        # Attempt to locate train.py within a cloned yolov5 directory if not found directly
        # This assumes a common structure where this script is in 'scripts' and 'yolov5' is a sibling.
        # This part might need adjustment based on the actual project structure.
        logger.warning(f"{train_script_path} not found. Attempting to find yolov5/train.py in parent.")
        train_script_path = script_dir.parent / "yolov5" / "train.py" 
        if not train_script_path.is_file():
             logger.error(f"train.py not found at {script_dir / 'train_yolov5' / 'train.py'} or {script_dir.parent / 'yolov5' / 'train.py'}. Please ensure it's correctly placed.")
             sys.exit(1)
        logger.info(f"Found train.py at: {train_script_path}")


    # The 'project' argument for train.py will be our 'training_subdir'
    # The 'name' argument will be 'exp' by default, creating 'training_subdir/exp'
    train_cmd = [
        sys.executable, str(train_script_path),
        "--data", str(pathlib.Path(args.data_yaml).resolve()),
        "--weights", args.model_type, # Ultralytics train.py uses --weights for model type
        "--epochs", str(args.epochs),
        "--imgsz", str(args.imgsz),
        "--device", args.device,
        "--project", str(training_subdir.resolve()), # Output directory for training runs
        "--name", "exp"  # Subdirectory name within project
    ]
    
    logger.info("Starting YOLOv5 training...")
    # Use the new real-time subprocess runner
    train_success, train_stdout, train_stderr = run_subprocess_realtime(train_cmd)

    if not train_success:
        logger.error("YOLOv5 training failed.")
        sys.exit(1)

    # Locate best.pt
    # Path is project/name/weights/best.pt
    best_pt_path = training_subdir / "exp" / "weights" / "best.pt"
    if not best_pt_path.is_file():
        logger.error(f"best.pt not found at expected location: {best_pt_path.resolve()}")
        # Fallback: search for 'best.pt' in training_subdir if the 'exp' naming changes or if multiple exps (exp, exp2, ...)
        found_best_pts = list(training_subdir.glob("**/weights/best.pt"))
        if found_best_pts:
            best_pt_path = found_best_pts[0]
            logger.info(f"Found best.pt at: {best_pt_path.resolve()}")
        else:
            logger.error(f"Could not find best.pt in any subdirectory of {training_subdir.resolve()}/exp*/weights/.")
            logger.info("Please check the training output to confirm if training completed successfully and where 'best.pt' was saved.")
            sys.exit(1)
            
    logger.info(f"Training completed. Best model: {best_pt_path.resolve()}")
    print(f"Best model path: {best_pt_path.resolve()}")

    # --- Conversion ---
    if args.conversion_target:
        logger.info(f"Starting model conversion for target: {args.conversion_target}...")
        convert_manager_script = script_dir / "convert_manager.py"

        if not convert_manager_script.is_file():
            logger.error(f"convert_manager.py not found at: {convert_manager_script.resolve()}")
            sys.exit(1)

        convert_cmd = [
            sys.executable, str(convert_manager_script.resolve()),
            "--pt_path", str(best_pt_path.resolve()),
            "--target", args.conversion_target,
            "--output_dir", str(converted_subdir.resolve())
        ]

        # Use the new real-time subprocess runner
        convert_success, convert_stdout, convert_stderr = run_subprocess_realtime(convert_cmd)

        if not convert_success:
            logger.error("Model conversion failed.")
            sys.exit(1)
        
        # convert_manager.py should print the final converted model path to its stdout
        # Assuming the last non-empty line of stdout is the path
        final_converted_path = ""
        if convert_stdout:
            lines = [line for line in convert_stdout.strip().split('\n') if line.strip()]
            if lines:
                # Example output: "Converted model ready: /path/to/model.blob"
                last_line = lines[-1]
                if "Converted model ready:" in last_line:
                     final_converted_path = last_line.split("Converted model ready:", 1)[1].strip()
                else: # Fallback if the prefix is missing, take the last line as is
                    final_converted_path = last_line
        
        if final_converted_path and pathlib.Path(final_converted_path).exists():
            logger.info(f"Model conversion successful. Converted model: {final_converted_path}")
            print(f"Converted model path: {final_converted_path}")
        else:
            logger.warning(f"Could not determine converted model path from convert_manager.py output or path does not exist.")
            logger.warning(f"convert_manager.py stdout:\n{convert_stdout}")
            logger.warning(f"convert_manager.py stderr:\n{convert_stderr}")
            # Do not exit, as training might still be considered a success
    else:
        logger.info("No conversion target specified. Skipping conversion.")
        final_converted_path = "None" # Ensure it's defined for the final print

    logger.info("Training pipeline finished.")
    
    # Print final paths for parsing by the backend
    if 'best_pt_path' in locals() and best_pt_path:
        print(f"FINAL_BEST_PT_PATH:{best_pt_path.resolve()}")
    else:
        # This case should ideally not be reached if training was successful
        print("FINAL_BEST_PT_PATH:None") 
        
    if 'final_converted_path' in locals() and final_converted_path:
        print(f"FINAL_CONVERTED_MODEL_PATH:{final_converted_path}") # final_converted_path is already resolved or "None"
    else:
        # This case handles if conversion was skipped or failed to produce a path string
        print("FINAL_CONVERTED_MODEL_PATH:None")

if __name__ == "__main__":
    main()
