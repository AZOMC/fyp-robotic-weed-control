#!/usr/bin/env python3
"""
evaluate.py - mAP / precision / recall on the held-out test split.

    python evaluate.py --weights runs/weed_model/weights/best.pt \
                       --data path/to/data.yaml
"""

import argparse
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--split", default="test")
    ap.add_argument("--device", default="0")
    args = ap.parse_args()

    m = YOLO(args.weights)
    r = m.val(data=args.data, split=args.split, device=args.device)
    d = r.results_dict
    print()
    print("  mAP@50      %.4f" % d["metrics/mAP50(B)"])
    print("  mAP@50-95   %.4f" % d["metrics/mAP50-95(B)"])
    print("  precision   %.4f" % d["metrics/precision(B)"])
    print("  recall      %.4f" % d["metrics/recall(B)"])


if __name__ == "__main__":
    main()
