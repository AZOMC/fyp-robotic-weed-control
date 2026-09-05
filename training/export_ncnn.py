#!/usr/bin/env python3
"""
export_ncnn.py - export trained weights to NCNN for the Raspberry Pi.

NCNN is the format the Pi actually runs. It beat ONNX at matched
resolution under both power conditions tested (docs/RESULTS.md), and it
is built for ARM CPUs like the Pi 4's BCM2711.

    python export_ncnn.py --weights runs/weed_model/weights/best.pt

Produces best_ncnn_model/ next to the weights. Copy that whole folder to
the Pi and point demo.py at it with --model.
"""

import argparse
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True, help="path to best.pt")
    ap.add_argument("--imgsz", type=int, default=320,
                    help="MUST match the inference size used on the Pi")
    args = ap.parse_args()
    YOLO(args.weights).export(format="ncnn", imgsz=args.imgsz)


if __name__ == "__main__":
    main()
