# Copyright (C) 2024 Intel Corporation
# SPDX-License-Identifier: MIT

"""
This script converts a YOLOv5 PyTorch model to the OpenVINO Intermediate Representation (IR)
and then compiles it to a MyriadX blob.

Dependencies:
- ultralytics: pip install ultralytics
- OpenVINO Toolkit: Installation required. Ensure 'mo' (Model Optimizer) and 
  'compile_tool' are in the system PATH.
  This might require sourcing the setupvars.sh script from the OpenVINO installation directory,
  e.g., source /opt/intel/openvino_2023.2.0/setupvars.sh (adjust version as needed)
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

def main():
    """Main function to orchestrate the conversion and compilation process."""
    parser = argparse.ArgumentParser(description="Convert YOLOv5 .pt model to OpenVINO IR and MyriadX blob.")
    parser.add_argument(
        "--pt_path",
        type=str,
        required=True,
        help="Path to the input YOLOv5 .pt model file."
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        help="Directory where the ONNX, IR, and blob files will be saved. "
             "Defaults to a subdirectory in the same location as the .pt file (e.g., model_name_openvino/)."
    )

    args = parser.parse_args()

    pt_path = pathlib.Path(args.pt_path)
    if not pt_path.is_file():
        logger.error(f"Input .pt file not found: {pt_path}")
        sys.exit(1)

    model_name = pt_path.stem
    
    if args.output_dir:
        output_dir = pathlib.Path(args.output_dir)
    else:
        output_dir = pt_path.parent / f"{model_name}_openvino"
    
    output_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"Output directory set to: {output_dir}")

    # --- Check for OpenVINO tools ---
    if not check_command_availability("mo"):
        logger.error("OpenVINO Model Optimizer (mo) not found in PATH. "
                     "Please ensure OpenVINO environment is configured. "
                     "e.g., source /opt/intel/openvino/bin/setupvars.sh")
        sys.exit(1)

    if not check_command_availability("compile_tool"):
        logger.error("OpenVINO Compile Tool (compile_tool) not found in PATH. "
                     "Please ensure OpenVINO environment is configured. "
                     "e.g., source /opt/intel/openvino/bin/setupvars.sh")
        sys.exit(1)

    # --- Load YOLOv5 Model ---
    try:
        from ultralytics import YOLO
        logger.info(f"Loading YOLOv5 model from: {pt_path}")
        model = YOLO(str(pt_path))
        logger.info("YOLOv5 model loaded successfully.")
    except ImportError:
        logger.error("ultralytics library not found. Please install it: pip install ultralytics")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error loading YOLOv5 model: {e}")
        sys.exit(1)

    # --- Export to ONNX ---
    onnx_filename = f"{model_name}.onnx"
    onnx_path = output_dir / onnx_filename
    try:
        logger.info(f"Exporting model to ONNX format at: {onnx_path}")
        # The export path is handled by ultralytics by setting the working directory
        # However, we will explicitly move the file to the desired output_dir
        # Ultralytics typically saves to <original_path_parent>/<original_name>.onnx
        # or if model is from hub to <cwd>/<model_name>.onnx
        # For consistency, we'll export and then move.
        
        # Ultralytics' export function returns the path to the exported ONNX file.
        exported_onnx_path_str = model.export(format='onnx', half=False, simplify=True) # Added half=False and simplify=True for better compatibility
        exported_onnx_path = pathlib.Path(exported_onnx_path_str)

        if exported_onnx_path != onnx_path:
             # If ultralytics saved it elsewhere, move it to our target onnx_path
            if onnx_path.exists():
                onnx_path.unlink() # Remove if it already exists to avoid error during rename
            exported_onnx_path.rename(onnx_path)
            logger.info(f"ONNX model moved to {onnx_path}")
        else:
            logger.info(f"ONNX model saved at {onnx_path}")

    except Exception as e:
        logger.error(f"Error exporting model to ONNX: {e}")
        logger.error("Ensure you have the necessary dependencies for ONNX export, e.g., onnx, onnxruntime.")
        sys.exit(1)
    
    # Placeholder for ONNX to IR and IR to Blob conversion
    # logger.info("ONNX export complete. IR conversion and Blob compilation needed.") # Removed this line

    # --- Convert ONNX to OpenVINO IR ---
    ir_output_dir = output_dir / "IR"
    ir_output_dir.mkdir(parents=True, exist_ok=True)
    # Model Optimizer command
    # Example: mo --input_model model.onnx --output_dir <output_dir_for_ir> --model_name <model_name_for_ir>
    # We need to ensure the model_name for IR doesn't conflict if multiple runs save to the same IR subfolder.
    # Using the original model_name should be fine as IR files include .xml and .bin
    cmd_mo = [
        "mo",
        "--input_model", str(onnx_path),
        "--output_dir", str(ir_output_dir),
        "--model_name", model_name, # This will produce model_name.xml and model_name.bin
        # Add any other necessary MO parameters here, e.g., --data_type, --scale_values etc.
        # For YOLO, common parameters might include --reverse_input_channels (if trained with BGR)
        # and mean/scale values if not part of the ONNX graph.
        # Assuming standard preprocessing is part of the ONNX model for now.
        # For some models, especially from TF, --input_shape might be needed.
        # For YOLOv5 from Ultralytics, it's usually self-contained.
    ]
    logger.info(f"Converting ONNX to OpenVINO IR with command: {' '.join(cmd_mo)}")
    try:
        process_mo = subprocess.run(cmd_mo, capture_output=True, text=True, check=True)
        logger.info("OpenVINO IR conversion successful.")
        logger.debug(f"MO stdout: {process_mo.stdout}")
        ir_xml_path = ir_output_dir / f"{model_name}.xml"
        ir_bin_path = ir_output_dir / f"{model_name}.bin"
        if ir_xml_path.exists() and ir_bin_path.exists():
            logger.info(f"IR files saved: {ir_xml_path}, {ir_bin_path}")
        else:
            logger.error(f"IR files not found after MO execution. stdout: {process_mo.stdout}, stderr: {process_mo.stderr}")
            sys.exit(1)
            
    except subprocess.CalledProcessError as e:
        logger.error(f"OpenVINO Model Optimizer (mo) failed with return code {e.returncode}.")
        logger.error(f"MO stdout: {e.stdout}")
        logger.error(f"MO stderr: {e.stderr}")
        logger.error("Check if the ONNX model is valid and OpenVINO environment is correctly configured.")
        sys.exit(1)
    except FileNotFoundError:
        logger.error("OpenVINO Model Optimizer (mo) command not found. Is OpenVINO installed and setupvars.sh sourced?")
        sys.exit(1)


    # --- Compile IR to MyriadX Blob ---
    blob_filename = f"{model_name}.blob"
    blob_path = output_dir / blob_filename
    # Compile tool command
    # Example: compile_tool -m <path_to_ir.xml> -ip U8 -d MYRIAD -o <path_to_output.blob>
    cmd_compile = [
        "compile_tool",
        "-m", str(ir_xml_path),
        "-ip", "U8",  # Input precision, U8 is common for YOLO on MyriadX
        "-d", "MYRIAD",
        "-o", str(blob_path),
        # Add any other necessary compile_tool parameters here
        # For example, -VPU_NUMBER_OF_SHAVES, -VPU_NUMBER_OF_CMX_SLICES
        # These are often specific to the OAK device model and desired performance.
        # Default values are usually a good starting point.
    ]
    logger.info(f"Compiling IR to MyriadX blob with command: {' '.join(cmd_compile)}")
    try:
        process_compile = subprocess.run(cmd_compile, capture_output=True, text=True, check=True)
        logger.info("MyriadX blob compilation successful.")
        logger.debug(f"Compile tool stdout: {process_compile.stdout}")
        if blob_path.exists():
            logger.info(f"MyriadX blob saved: {blob_path}")
            print(f"Successfully generated blob: {blob_path.resolve()}") # Print final path to stdout
        else:
            logger.error(f"Blob file not found after compile_tool execution. stdout: {process_compile.stdout}, stderr: {process_compile.stderr}")
            sys.exit(1)

    except subprocess.CalledProcessError as e:
        logger.error(f"OpenVINO Compile Tool failed with return code {e.returncode}.")
        logger.error(f"Compile tool stdout: {e.stdout}")
        logger.error(f"Compile tool stderr: {e.stderr}")
        logger.error("Ensure the IR files are valid and the MyriadX target is supported.")
        sys.exit(1)
    except FileNotFoundError:
        logger.error("OpenVINO compile_tool command not found. Is OpenVINO installed and setupvars.sh sourced?")
        sys.exit(1)

    logger.info("OpenVINO conversion and compilation process completed.")


def check_command_availability(command: str) -> bool:
    """Checks if a command is available in the system PATH."""
    try:
        subprocess.run([command, "-h"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        # Some tools might return non-zero on -h, but are still callable.
        # A more robust check might involve 'which' or 'where'
        # For now, attempting to run with -h (or a version command if available) is a basic check.
        # On Windows, where 'which' isn't standard:
        if os.name == 'nt':
            try:
                subprocess.run(['where', command], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return True
            except (subprocess.CalledProcessError, FileNotFoundError):
                return False
        else: # For Linux/macOS
            try:
                subprocess.run(['which', command], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return True
            except (subprocess.CalledProcessError, FileNotFoundError):
                # Fallback if 'which' is not found or command is not in PATH
                # Try running the command directly, as it might be an alias or built-in
                # Some tools like 'mo' might not respond to '-h' with a 0 exit code.
                # A more reliable check for 'mo' specifically might be needed if this fails.
                # For now, this is a general approach.
                try:
                    subprocess.run([command, '--version'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
                    return True
                except (subprocess.CalledProcessError, FileNotFoundError):
                     # Try common help flags if --version fails
                    try:
                        subprocess.run([command, '-h'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        return True # Assume available if it runs without error, even if exit code isn't 0
                    except (FileNotFoundError): # Only catch if command itself is not found
                        return False
                    except subprocess.CalledProcessError: # Command found but -h gave error, assume available
                        return True 


    except FileNotFoundError: # Command itself not found
        return False
    return True # Default assumption if initial check passes without FileNotFoundError

if __name__ == "__main__":
    main()
