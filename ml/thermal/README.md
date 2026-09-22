# ml/thermal: the thermal person detector

YOLOv8n trained twice: on HIT-UAV's real UAV LWIR, then on this world's people rendered through the
drone's own 256x192 thermal core. It runs beside the blob detector (`detector.py`), not instead of
it; see "Results" for why.

```
ml/thermal/
  hit_uav.py         HIT-UAV -> one-class YOLO (Person kept; vehicles dropped; DontCare painted out)
  make_dataset.py    this world's people through the thermal camera (ml/make_dataset.py's scenes)
  train.py           stage hit_uav (COCO yolov8n -> models/yolov8n_hit_uav.pt),
                     stage world (that -> ../models/yolov8n_thermal/yolov8n_thermal.pt)
  compare.py         blob detector vs YOLO vs "blob proposes, YOLO scores", on the same frames
  test_thermal.py
  models/            stage 1 weights and curves, compare.*.json (committed)
  datasets/, runs/   generated, not committed
```

## Build it

```bash
source /opt/ros/humble/setup.bash && source ~/uav_ws/install/setup.bash && source install/setup.bash
curl -LO https://github.com/suojiashun/HIT-UAV-Infrared-Thermal-Dataset/releases/download/v1.2.1/HIT-UAV.zip
unzip HIT-UAV.zip "HIT-UAV/normal_json/*" "HIT-UAV/LICENSE" -d ml/thermal/datasets && rm HIT-UAV.zip
python3 ml/thermal/hit_uav.py                      # 2029 / 290 / 579 frames, 8533 / 1168 / 2611 people
python3 ml/thermal/train.py hit_uav                # ~45 min on an RTX 2050
python3 ml/thermal/make_dataset.py --scenes 300    # ~20 s a scene, own GZ_PARTITION
python3 ml/thermal/make_dataset.py --scenario      # the scenario's 23 casualties: the test set
python3 ml/thermal/train.py world
python3 ml/thermal/compare.py                      # and --data .../world_people --split val
```

HIT-UAV (CC0) is 2,898 frames from 60-130 m, 640x512, white-hot with the camera's own gain; people
are a median 12x19 px.

## The image the network sees

`aero_sense_perception.thermal_yolo.to_image`: kelvin through a fixed 290-307 K window onto 0-255,
white-hot like HIT-UAV (water 291 K near black, ground 298 K mid-grey, a body from 306 K near white).
A fixed window rather than per-frame gain, so an empty frame is not stretched into contrast. Training
and the drone both call it; change the window there and retrain.

## The rendered set

`ml/make_dataset.py`'s scenes, conditions and heights (8-35 m), with the thermal camera and a
segmentation camera sharing its lens. Differences from the RGB set:

- Skin between 305.5 and 311 K (the live casualties' range), not a fixed 308 K.
- Views aimed within 0.4 x height of someone: the RGB set's reach was for a 127 deg lens and left
  most 57 deg frames empty. Every eighth view still looks 45-90 m away at nobody.
- Boxes from 2 visible pixels, not 6: a wading head or a hand from 25 m is 2-5 px and plainly hot.
  Anyone smaller is in `seg/`, and a detection on them is neither a hit nor a false positive.
- `kelvin/` keeps each frame as the drone gets it, so the blob detector is scored on the same frames.
- `Sim.move` is overridden: the base class dates "after the move" from the newest RGB frame, and with
  no RGB camera every photograph was a stale frame from before the move.

## Results (2026-09-22)

Stage 1 (HIT-UAV, stopped by patience at epoch 69, best 49): HIT-UAV test split P 0.853, R 0.772,
mAP50 0.856, mAP50-95 0.410. Stage 2 (4,192 rendered frames, 100 epochs, 1.5 h): rendered val
mAP50 0.994, R 0.98.

`compare.py`, the scenario's 23 casualties from 8 heights (184 frames, 132 people boxed):

| | recall | false positives | stray |
|---|---|---|---|
| blob detector (today) | 0.886 | 55 | 0 |
| YOLOv8n, HIT-UAV only | 0.356 | 66 | 47 |
| YOLOv8n, both stages | 0.871 | 4 | 0 |
| blob proposes, YOLO scores | 0.856 | 51 | 0 |

Every miss of the blob detector but one is V09, deceased at ambient (no thermal method sees them;
RGB does). The blob detector's false positives are all a second blob on the same person, which the
tracker folds in; YOLO gives one box a person. On the 608 rendered validation frames (1,014 people)
it is 0.999 against 0.990. The blob detector's 124 stray false positives there are all one torn frame
(`s0168_v03`: half the frame at the L16 ceiling, a rendering fault, a row of "detections"); YOLO
ignored that frame but has 7 strays of its own in other frames, six at the frame's edge and one in
a frame with nothing warmer than 298 K.

HIT-UAV alone does not transfer (0.356): real LWIR with camera gain is not Gazebo's 2.6 K steps.
The fine-tuned model matches the blob detector but does not beat it, and cannot here: nothing in this
world but a person is warmer than 301 K. Where it would earn its place is real footage (engines,
animals, sun-baked roofs), which is why it keeps the HIT-UAV stage. It runs beside the blob
detector (`thermal_yolo:=true`) and nothing onboard acts on it; the flight is in docs/VERIFICATION.md.
