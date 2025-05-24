# Copyright (C) 2024 Intel Corporation
# SPDX-License-Identifier: MIT

"""
Manages the conversion of PyTorch models to various target formats
by dispatching to specific converter scripts.
"""

import argparse
import logging
import os
import pathlib
import subprocess
import sys

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- Target Handler Functions ---
# Each function should accept the main args (for pt_path, output_dir, etc.)
# and the specific sub-directory for its output.
# It should return the path to the final converted model file or None if conversion fails.

def convert_for_oak_d_lite(args: argparse.Namespace, target_output_dir: pathlib.Path) -> str | None:
    """
    Handles the conversion process for the 'oak_d_lite' target
    by calling the openvino_converter.py script.
    """
    logger.info(f"Attempting conversion for target: oak_d_lite")
    
    script_dir = pathlib.Path(__file__).parent.resolve()
    openvino_converter_script = script_dir / "openvino_converter.py"

    if not openvino_converter_script.is_file():
        logger.error(f"openvino_converter.py not found at: {openvino_converter_script}")
        return None

    cmd = [
        sys.executable, # Use the same python interpreter
        str(openvino_converter_script),
        "--pt_path", str(args.pt_path),
        "--output_dir", str(target_output_dir) 
    ]

    logger.info(f"Executing command: {' '.join(cmd)}")
    try:
        process = subprocess.run(cmd, capture_output=True, text=True, check=True, cwd=script_dir)
        logger.info("openvino_converter.py executed successfully.")
        # The openvino_converter.py script prints the blob path to its stdout
        blob_path_line = process.stdout.strip().split('\n')[-1] # Get the last line of stdout
        if "Successfully generated blob:" in blob_path_line:
            blob_path = blob_path_line.split("Successfully generated blob:")[1].strip()
            logger.info(f"Converted blob path: {blob_path}")
            return blob_path
        else:
            logger.error(f"Could not parse blob path from openvino_converter.py output: {process.stdout}")
            logger.error(f"openvino_converter.py stderr: {process.stderr}")
            return None

    except subprocess.CalledProcessError as e:
        logger.error(f"Error calling openvino_converter.py for oak_d_lite.")
        logger.error(f"Command: {' '.join(e.cmd)}")
        logger.error(f"Return code: {e.returncode}")
        logger.error(f"Stdout: {e.stdout}")
        logger.error(f"Stderr: {e.stderr}")
        return None
    except FileNotFoundError:
        logger.error(f"Python interpreter or {openvino_converter_script} not found during subprocess call.")
        return None

# --- Target Dispatch Table ---
# To add a new target:
# 1. Define a new function (e.g., `convert_for_my_new_target`) similar to `convert_for_oak_d_lite`.
#    This function will handle the specific conversion steps for your new target.
# 2. Add an entry to the `TARGET_DISPATCH` dictionary below, mapping your target
#    name (e.g., "my_new_target") to your new handler function.

TARGET_DISPATCH = {
    "oak_d_lite": convert_for_oak_d_lite,
    # Example for a future target:
    # "another_target": convert_for_another_target,
}

def main():
    """Main function to parse arguments and dispatch to the correct converter."""
    parser = argparse.ArgumentParser(description="Manage model conversion to different targets.")
    parser.add_argument(
        "--pt_path",
        type=str,
        required=True,
        help="Path to the input .pt model file."
    )
    parser.add_argument(
        "--target",
        type=str,
        required=True,
        choices=list(TARGET_DISPATCH.keys()), # Restrict choices to supported targets
        help=f"The desired conversion target. Supported: {', '.join(TARGET_DISPATCH.keys())}"
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        help="Directory where the converted model and intermediate files will be saved. "
             "Defaults to '<pt_filename_root>_converted/' next to the .pt file."
    )

    args = parser.parse_args()

    pt_file_path = pathlib.Path(args.pt_path)
    if not pt_file_path.is_file():
        logger.error(f"Input .pt file not found: {pt_file_path}")
        sys.exit(1)

    if args.output_dir:
        base_output_dir = pathlib.Path(args.output_dir)
    else:
        base_output_dir = pt_file_path.parent / f"{pt_file_path.stem}_converted"
    
    # Create a subdirectory for the specific target's output
    # e.g. <base_output_dir>/openvino_files for oak_d_lite
    # This keeps outputs organized if multiple targets are run for the same pt file.
    # For 'oak_d_lite', the openvino_converter.py script already creates its own subdirectories (IR),
    # so we pass a directory like 'base_output_dir/oak_d_lite_files' to it.
    # The openvino_converter.py will then create 'base_output_dir/oak_d_lite_files/IR/' etc.
    # Let's name the sub-directory based on the target or a general name if the converter handles it.
    # For 'oak_d_lite' which uses openvino_converter, let's call the passed dir 'openvino_conversion_output'
    # or similar to avoid confusion with the 'IR' subfolder it creates.
    # Or, more simply, we can let openvino_converter.py use its default naming within this target-specific dir.
    
    # Let's make a specific directory for this target's output within base_output_dir
    target_specific_output_dir = base_output_dir / f"{args.target}_output"
    target_specific_output_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"Base output directory set to: {base_output_dir}")
    logger.info(f"Target-specific output directory for '{args.target}': {target_specific_output_dir}")


    # --- Dispatch to target handler ---
    handler_func = TARGET_DISPATCH.get(args.target)
    
    if not handler_func: # Should be caught by argparse choices, but as a safeguard
        logger.error(f"Unsupported target: {args.target}. Supported targets are: {', '.join(TARGET_DISPATCH.keys())}")
        sys.exit(1)

    logger.info(f"Dispatching to handler for target: {args.target}")
    # The handler function will use `target_specific_output_dir` for its outputs.
    final_model_path = handler_func(args, target_specific_output_dir)

    if final_model_path:
        logger.info(f"Successfully converted model for target '{args.target}'.")
        logger.info(f"Final model path: {final_model_path}")
        print(f"Converted model ready: {final_model_path}") # Print final path to stdout
    else:
        logger.error(f"Conversion failed for target '{args.target}'.")
        sys.exit(1)

if __name__ == "__main__":
    main()
