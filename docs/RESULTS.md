# Results

## Detection

Four models were trained. The dataset was rebuilt twice along the way,
which mattered more than the architecture change.

| | Model 1 | Model 2 | Model 3 | Model 4 (deployed) |
|---|---|---|---|---|
| Dataset | 3 merged Roboflow sets | quality-filtered v1 | v2 crops + 2 new weed sources | same as 3 |
| Architecture | YOLOv11n | YOLOv11n | YOLOv11n | **YOLO26n** |
| Input | 640 px | 640 px | 320 px | **320 px** |
| Weed:crop ratio | 9.39 : 1 | 3.47 : 1 | 1.00 : 1 | 1.00 : 1 |
| **mAP@50** | 0.829 | 0.888 | 0.962 | **0.960** |

Balancing the class ratio moved mAP@50 from 0.829 to 0.962. Nothing else
came close to that.

### Deployed model (YOLO26n, 320 px)

| Metric | Value |
|---|---|
| mAP@50 | 0.960 |
| mAP@50-95 | 0.7700 |
| Precision | 0.9336 |
| Recall | 0.9017 |
| F1 | 0.92 @ 0.42 confidence |

Per-class true detection: 99% crop, 98% weed.

### Why not Model 3, which scored higher?

YOLOv11n scored 0.962 against YOLO26n's 0.960 — inside the noise. But
YOLOv11n hallucinated weeds and crops in empty background, while YOLO26n
tightened exactly those errors. For a robot, a background false positive
means the arm drives into bare soil and wastes a 7.6 s cycle. Aggregate mAP
does not price that in, so it was the wrong metric to select on.

## Inference performance

Static benchmark: one fixed validation image, 30 consecutive passes after a
5-pass warm-up, on the Raspberry Pi 4.

| Configuration | Input | Under-voltage | FPS | Clean power | FPS | Gain |
|---|---|---|---|---|---|---|
| YOLOv11n NCNN | 320 px | 180.27 ms | 5.55 | 101.49 ms | 9.85 | +77.6% |
| YOLOv11n NCNN | 640 px | 670.04 ms | 1.49 | 398.34 ms | 2.51 | +68.3% |
| **YOLO26n NCNN** | **320 px** | 171.57 ms | 5.83 | **95.67 ms** | **10.45** | +79.3% |
| YOLO26n ONNX | 320 px | 213.61 ms | 4.68 | 105.23 ms | 9.50 | +103.0% |

Live pipeline, including capture, overlay drawing, JPEG encoding and Flask
streaming:

| Condition | Frames | FPS min | FPS max | Inference min | Inference max |
|---|---|---|---|---|---|
| Under-voltage | 516 | 2.80 | 8.16 | 118.82 ms | 200.47 ms |
| Clean power | 750 | 4.42 | 10.25 | 94.42 ms | 217.40 ms |

Both sessions show a slow first few seconds while the NCNN backend, OpenCV
capture and Flask threading initialise, then settle into a stable band. In
normal operation under clean power, expect FPS at or above 4.42. A sustained
drop below that is outside normal pipeline variance and worth investigating
with `vcgencmd get_throttled` and `vcgencmd measure_temp`.

### NCNN vs ONNX

At matched resolution NCNN won under both conditions, but the margin
narrowed sharply once power was fixed — 24.5% faster under-voltage, 9.1%
faster on clean power. The ONNX runtime was disproportionately sensitive to
the reduced clock speeds caused by throttling.

## Actuation

| Metric | Value |
|---|---|
| Mean cycle time | 7.61 s |
| Plucking share | 53% |
| Binning share | 47% |
| Throughput | ~473 weeds/hour (~7.88/min) |
| Pluck success | 16/16 (100%) |
| Bin landing | 12/16 (75%) |

Four runs of four weeds each. Every target was detected and successfully
gripped. Four failed to land in the bin; three of those trace to the same
root cause — the bounding-box centre is not the plant's root, so the gripper
closes on leaves and the weed works loose during the traverse.

## Raw data

- [`benchmark_results_clean_power.csv`](benchmark_results_clean_power.csv)
- [`benchmark_results_undervoltage.csv`](benchmark_results_undervoltage.csv)
- [`training_results_yolo26n.csv`](training_results_yolo26n.csv) — per-epoch metrics
- [`training_results_yolo11n.csv`](training_results_yolo11n.csv) — per-epoch metrics

## Test conditions

Bench testing: controlled indoor lighting, flat hard surface, artificial
weed targets on bare substrate, no wind. Results should be read as an upper
bound on field performance, not a prediction of it.
