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
        default="0",
        help="CUDA device (e.g. '0' or 'cpu')"
    )
    return parser.parse_args()

def main():
    args = parse_args()
    # Load model
    model = YOLO(args.model)
    # Train
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        device=args.device
    )

if __name__ == "__main__":
    main()
