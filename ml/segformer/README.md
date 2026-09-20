# ml/segformer: disaster segmentation

What the ground is like, per pixel, so the ground teams' routes know what they are walking into:
`hazard_mapper` runs this network in flight and turns its output into hazard regions (HSI,
`aero_sense_perception/hsi.py`).

```
ml/segformer/
  make_dataset.py   renders the labelled dataset in its own headless Gazebo (own GZ_PARTITION, no SITL)
  train.py          fine-tunes SegFormer-B0 (nvidia/mit-b0) on it and scores it
  models/segformer_b0_disaster/   the trained weights (committed) and metrics.json
  datasets/         generated, not committed (rebuild with make_dataset.py)
```

## Input and classes

The thermal camera sees 57 deg, the RGB camera 127 deg from the same mount, so one sample is the
thermal image plus the centre of the RGB image resampled onto it: four channels at 256x192
(`aero_sense_perception/disaster.py`, used by both training and the node). Classes: background,
road, intact, damaged, collapsed, water, vehicle.

Labels come from a gz segmentation camera with the thermal camera's lens, over a copy of the world
in which each include carries a Label plugin (`URI_CLASSES`); the world file itself is untouched.

## Build and train

```bash
source /opt/ros/humble/setup.bash && source ~/uav_ws/install/setup.bash && source install/setup.bash
python3 ml/segformer/make_dataset.py --frames 1200      # ~2 s a frame; --start N resumes
python3 ml/segformer/train.py --epochs 40               # GPU if CUDA is up, else CPU (~65 s an epoch)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/aero_sense_perception -q
```

## What it scores

1200 frames (150 validation), 40 epochs on the CPU: mIoU **0.9385**, pixel accuracy 0.9751.

| class | background | road | intact | damaged | collapsed | water | vehicle |
|---|---|---|---|---|---|---|---|
| IoU | 0.949 | 0.968 | 0.940 | 0.931 | 0.936 | 0.973 | 0.873 |

Validation frames come from the same simulated town as training, from poses the drone flies
(15-35 m, up to 15 deg of tilt). It measures the network on this simulation, not on real imagery.
In flight it costs 45 ms a frame at 1 Hz on two CPU threads (`docs/VERIFICATION.md`).
