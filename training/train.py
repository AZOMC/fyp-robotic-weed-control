#!/usr/bin/env python3
"""
train.py - train the weed/crop detector.

The deployed model is Model 4 in the report: YOLO26n at 320px on the v3
dataset. The hyperparameters below are the ones that produced it.

    python train.py --data path/to/data.yaml

Dataset layout is standard Ultralytics YOLO format (see README):

    dataset/
      data.yaml          nc: 2, names: ['Crops', 'Weed']
      train/images  train/labels
      valid/images  valid/labels
      test/images   test/labels
"""

import argparse
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="path to data.yaml")
    ap.add_argument("--model", default="yolo26n.pt",
                    help="base weights (yolo26n.pt deployed; yolo11n.pt compared)")
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--imgsz", type=int, default=320,
                    help="320 was chosen over 640: see docs/RESULTS.md")
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--device", default="0", help="'0' for GPU, 'cpu' otherwise")
    ap.add_argument("--project", default="runs")
    ap.add_argument("--name", default="weed_model")
    args = ap.parse_args()

    YOLO(args.model).train(
        data=args.data,
        epochs=args.epochs,
        patience=50,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=8,
        cache=True,
        amp=True,
        optimizer="auto",
        lr0=0.005,
        warmup_epochs=5,
        cos_lr=True,
        label_smoothing=0.1,
        project=args.project,
        name=args.name,
        exist_ok=True,
        save_period=1,
    )


if __name__ == "__main__":
    main()
