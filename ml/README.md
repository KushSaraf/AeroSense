# ml: the RGB person detector

Stock YOLO11n (COCO) finds nobody in the drone's RGB camera from search height and almost nobody
from SWOOP's 10 m close look. It learned people from the side, and from 30 m a person is 8-14
pixels seen from straight above. This folder builds a dataset of what the drone actually sees and
fine-tunes YOLO11n on it.

```
ml/
  make_dataset.py        renders the dataset in its own headless Gazebo (own GZ_PARTITION, no SITL)
  train.py               fine-tunes YOLO11n on it and scores stock and fine-tuned models
  evaluate.py            recall by height and condition, and false positives, for any model
  test_make_dataset.py   checks the labelling
  models/
    yolo11n_aerial/      the fine-tuned weights (committed), training curves and settings
    evals/               evaluate.py's reports, one JSON per model and split
    runs/                ultralytics training runs, not committed
  datasets/              generated, not committed (rebuild with make_dataset.py)
    aerial_people/
      images/{train,val}/   RGB frames, 960x600 JPEG, exactly the drone's camera (sensors.yaml, medium)
      labels/{train,val}/   YOLO boxes: `0 cx cy w h`, normalised, one line per visible person
      meta/                 per frame: camera pose, height, tilt; each person's condition, character,
                            pose, world position and visible pixels
      previews/             frames with the boxes and conditions drawn on, to look at
      data.yaml             for ultralytics
      summary.json          frames, boxes per condition, frames per height
    scenario_test/          the test set: the scenario's own 23 casualties (make_dataset.py --scenario)
```

## Build the dataset

```bash
source /opt/ros/humble/setup.bash && source ~/uav_ws/install/setup.bash && source install/setup.bash
python3 tools/make_people.py --all && colcon build --base-paths src && source install/setup.bash
python3 ml/make_dataset.py --scenes 250           # ~20 s a scene; --start N resumes
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest ml -q
```

Each scene puts 6-12 people round a random spot in one sector of the disaster world, each in one
of these conditions, and photographs them 16 times from 8-35 m with up to 15 deg of tilt:

| Condition | Where | What shows |
|---|---|---|
| open_lying, open_seated, open_standing | either sector, dry ground | the whole person |
| legs_under_rubble | earthquake | the upper body, the legs under a pile of slabs and brick |
| feet_out_of_rubble | earthquake | the feet |
| hand_out_of_rubble | earthquake | a forearm and hand |
| buried | earthquake | nothing: a hard negative |
| wading | flood, knee to chest deep | head, shoulders, the rest under water |
| car_roof, terrace | flood | the whole person, on a stranded car or a house's roof |
| window | flood | head and shoulders out of a first-floor window |

People are the scenario's own posed characters (4 of them, every pose), and rubble, cars and houses
are built exactly as the scenario builds them (`victim_models`, `layout_world.perch_spot`). The
scenario's 23 casualties are never in the dataset: they are the test set.

Boxes come from a segmentation camera beside the RGB camera. Each person's body carries its own
label, so a box is exactly the pixels of that person the camera sees. People with fewer than 6
visible pixels get no box. Two of every 16 views look at the disaster 45-90 m away from the people,
so the dataset has frames of rubble, cars and water with nobody in them.

## Test set

`python3 ml/make_dataset.py --scenario` photographs the scenario's 23 casualties exactly where the
mission meets them, each from 8 heights between 8 and 34 m, into `datasets/scenario_test/`
(split `test`). No model ever trains on these people or on these rubble piles.

## Train and evaluate

```bash
python3 ml/train.py                     # 60 epochs at 960 px; GPU if CUDA works, else CPU
python3 ml/evaluate.py ml/models/yolo11n_aerial/yolo11n_aerial.pt --data ml/datasets/scenario_test --split test
```

Training starts from COCO YOLO11n, one class, at the frame's full 960 px (shrinking to 640 loses
10-pixel people), with any rotation and vertical flips, because a camera looking straight down has
no up. `train.py` publishes the best weights to `models/yolo11n_aerial/` and scores stock and
fine-tuned YOLO11n on the validation split and the test set. A person counts as found when a
detection overlaps their box (IoU 0.3) or its centre falls inside it; anything else is a false
positive.

After a suspend the NVIDIA driver can leave CUDA unusable (`cuInit` returns 999) until
`sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm`.

## Results

The dataset as rendered (`--scenes 250`, seed 2026): 4,000 frames (3,488 train, 512 val), 13,795
people boxed, 648 frames with nobody in them, median 26 visible pixels a person. The test set:
184 frames of the scenario's casualties, 384 boxes (the two buried casualties show nothing).

Scored at 960 px (`models/evals/*.json`). Stock YOLO11n keeps only COCO's person class. Stray false
positives are detections not beside any person; the rest are a second box on someone already found
(a torso and legs split by rubble), which lands where they are and which the tracker folds into them.

| Model | Confidence | Split | People found | below 15 m | 15-25 m | 25-40 m | False positives (stray) |
|---|---|---|---|---|---|---|---|
| stock YOLO11n | 0.25 | validation, 512 frames | 0.3 % of 2,011 | 2 % | 0 % | 0 % | 11 (10) |
| stock YOLO11n | 0.25 | scenario casualties, 184 frames | 0.5 % of 384 | 2 % | 0 % | 0 % | 2 (2) |
| **yolo11n_aerial** | 0.25 | validation | **87.2 %** | 90 % | 90 % | 85 % | 292 (77) |
| **yolo11n_aerial** | 0.25 | scenario casualties | **77.1 %** | 84 % | 76 % | 75 % | 121 (55) |
| yolo11n_aerial | 0.45 | validation | 67.1 % | 74 % | 73 % | 61 % | 25 (7) |
| yolo11n_aerial | 0.45 | scenario casualties | 51.0 % | 60 % | 51 % | 47 % | 15 (6) |

Trained 60 epochs in 1.9 h on an RTX 2050 (batch 8, 960 px); validation mAP50 0.76 at the end
(`models/yolo11n_aerial/results.csv`). On the scenario's casualties at 0.25: V09, dead and at
ambient so invisible to the thermal camera, is found in 96 % of the frames that show it. The
weakest are the smallest: V20 and V23 leaning out of windows (0 of 5, 1 of 4), V21's hand
(2 of 8), V18 seated with legs under rubble (33 %).
