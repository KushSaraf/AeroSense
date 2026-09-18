"""The dataset generator's labelling: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest ml -q"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import make_dataset as md  # noqa: E402


def test_a_box_is_exactly_the_visible_pixels_of_one_person():
    labels = np.zeros((40, 60), np.uint8)
    labels[10:14, 20:30] = 1                     # person 1: 40 px
    labels[30:32, 5:7] = 2                       # person 2: 4 px, a speck
    found = md.boxes(labels, [{"label": 1}, {"label": 2}, {"label": 3}])
    assert found == [{"label": 1, "visible_px": 40, "box": [20, 10, 30, 14]}]


def test_only_the_person_carries_a_label_not_their_rubble():
    person = {"id": "T1", "character": "man", "pose": "supine", "visibility": "partial", "exposed": "upper_body",
              "state": "lying", "temperature_k": 308.0, "motion": "none"}
    sdf = md.person_sdf("t", person, 7)
    assert sdf.count("<label>7</label>") == 1
    body = sdf[sdf.index('<visual name="body">'):]
    assert body.index("<label>7</label>") < body.index("</visual>")


def test_waving_arm_is_labelled_as_the_same_person():
    person = {"id": "T2", "character": "woman", "pose": "standing_waving", "visibility": "full",
              "state": "lying", "temperature_k": 308.0, "motion": "waving"}
    assert md.person_sdf("t", person, 3).count("<label>3</label>") == 2


def test_the_camera_never_flies_inside_the_radio_mast():
    placer = md.Placer()
    x, y, radius, height = max(placer.structures, key=lambda s: s[3])      # the 44 m mast
    assert not placer.airspace(x, y, 20.0)
    assert placer.airspace(x, y, height + 5.0)
