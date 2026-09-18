# ml: the RGB person detector

Stock YOLO11n (COCO) finds nobody in the drone's RGB camera from search height and almost nobody
from SWOOP's 10 m close look. It learned people from the side, and from 30 m a person is 8-14
pixels seen from straight above. This folder builds a dataset of what the drone actually sees and
fine-tunes YOLO11n on it.

```
ml/
  make_dataset.py        renders the dataset in its own headless Gazebo (own GZ_PARTITION, no SITL)
  test_make_dataset.py   checks the labelling
  datasets/              generated, not committed (rebuild with make_dataset.py)
    aerial_people/
      images/{train,val}/   RGB frames, 960x600 JPEG, exactly the drone's camera (sensors.yaml, medium)
      labels/{train,val}/   YOLO boxes: `0 cx cy w h`, normalised, one line per visible person
      meta/                 per frame: camera pose, height, tilt; each person's condition, character,
                            pose, world position and visible pixels
      previews/             frames with the boxes and conditions drawn on, to look at
      data.yaml             for ultralytics
      summary.json          frames, boxes per condition, frames per height
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
