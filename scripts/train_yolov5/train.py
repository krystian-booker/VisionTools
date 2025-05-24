#!/usr/bin/env python3
import argparse
from ultralytics import YOLO

def parse_args():
    parser = argparse.ArgumentParser(
        description="Train a YOLOv5 model using Ultralytics"
    )
    parser.add_argument(
        "--data", "-d",
        type=str,
        required=True,
        help="Path to your data.yaml file"
    )
    parser.add_argument(
        "--model", "-m",
        type=str,
        default="yolov5n.pt",
        help="Pretrained YOLOv5 model checkpoint"
    )
    parser.add_argument(
        "--epochs", "-e",
        type=int,
        default=100,
        help="Number of training epochs"
    )
    parser.add_argument(
        "--imgsz", "-i",
        type=int,
        default=416,
        help="Image size"
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cpu",
        help="CUDA device (e.g. '0' or 'cpu')"
    )
    parser.add_argument(
        "--project",
        type=str,
        default="runs/train",
        help="Directory to save training runs"
    )
    parser.add_argument(
        "--name",
        type=str,
        default="exp",
        help="Subdirectory name for the training run"
    )
    return parser.parse_args()

def main():
    args = parse_args()
    # Load model
    model = YOLO(args.model) # In Ultralytics, model type (e.g., yolov5n.pt) is passed to YOLO() constructor
    # Train
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        device=args.device,
        project=args.project, # Pass the project argument
        name=args.name        # Pass the name argument
        # Note: Ultralytics train.py uses --weights for pretrained model,
        # but in YOLO().train() it's implicitly the model loaded in YOLO(args.model)
        # The run_training_pipeline.py correctly uses --weights for train_script_path
        # which maps to --model in this script.
    )

if __name__ == "__main__":
    main()
