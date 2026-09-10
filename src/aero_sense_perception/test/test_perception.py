"""Phase 6 checks: what the detector finds, where it says it is, and whether it keeps its name."""
import math

import numpy as np
import pytest

from aero_sense_perception import detector, geolocate
from aero_sense_perception.tracker import Tracker
from aero_sense_perception.victim_detector import load_config

CFG = load_config()
DET = CFG["detector"]
AMBIENT_K = 298.0


def frame_with(hot_spots, shape=(256, 320)):
    """An ambient LWIR frame with square warm patches: (u, v, size, kelvin)."""
    kelvin = np.full(shape, AMBIENT_K, dtype=np.float32)
    for u, v, size, temp in hot_spots:
        kelvin[v:v + size, u:u + size] = temp
    return kelvin


def test_finds_a_warm_body_and_ignores_ambient_ground():
    blobs = detector.detect(frame_with([(100, 80, 5, 309.0)]), **DET)
    assert len(blobs) == 1
    assert blobs[0].peak_k == pytest.approx(309.0) and blobs[0].area_px == 25
    assert 100 <= blobs[0].u <= 105 and 80 <= blobs[0].v <= 85


def test_ignores_specks_and_whole_hot_surfaces():
    blobs = detector.detect(frame_with([(10, 10, 1, 310.0), (40, 40, 80, 310.0)]), **DET)
    assert blobs == ()


def test_confidence_grows_with_temperature_and_saturates():
    cool, warm = (detector.detect(frame_with([(50, 50, 6, t)]), **DET)[0] for t in (305.0, 312.0))
    assert cool.confidence < warm.confidence == 1.0


def test_the_deceased_casualty_is_invisible_to_thermal():
    """295 K against 298 K ground: this is why RGB shape has to join later."""
    assert detector.detect(frame_with([(60, 60, 6, 295.0)]), **DET) == ()


def looking_down_from(height_m):
    """Camera at `height_m` over the origin, optical axis straight down, and a 90 deg pinhole."""
    k = [160.0, 0.0, 160.0, 0.0, 160.0, 128.0, 0.0, 0.0, 1.0]
    rotation = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])   # optical z -> map -z
    return k, np.array([0.0, 0.0, height_m]), rotation


def test_centre_pixel_lands_under_the_camera():
    k, position, rotation = looking_down_from(25.0)
    point = geolocate.project(160.0, 128.0, k, position, rotation)
    assert point == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)


def test_offset_pixel_lands_at_the_right_ground_distance():
    k, position, rotation = looking_down_from(20.0)
    point = geolocate.project(160.0 + 160.0, 128.0, k, position, rotation)   # 45 deg off axis
    assert point[0] == pytest.approx(20.0, abs=1e-6) and point[2] == pytest.approx(0.0, abs=1e-6)


def test_a_ray_that_never_meets_the_ground_has_no_position():
    k, position, _ = looking_down_from(20.0)
    up = np.eye(3)                                    # optical z -> map +z, pointing at the sky
    assert geolocate.project(160.0, 128.0, k, position, up) is None


def tracker():
    return Tracker(**CFG["tracker"])


def test_a_single_hot_frame_is_not_a_casualty():
    assert tracker().update([((10.0, 5.0, 0.0), 0.8, 309.0)], 1.0) == ()


def test_repeated_looks_confirm_one_casualty_with_a_stable_id():
    t = tracker()
    for i in range(CFG["tracker"]["confirm_hits"]):
        confirmed = t.update([((10.0 + 0.3 * i, 5.0, 0.0), 0.8, 309.0)], float(i))
    assert len(confirmed) == 1
    first = confirmed[0]
    again = t.update([((10.2, 5.1, 0.0), 0.8, 309.0)], 9.0)
    assert again[0].track_id == first.track_id and again[0].hits == first.hits + 1
    assert again[0].confidence > first.confidence          # agreement builds certainty
    assert again[0].confidence < 1.0                       # but never certainty itself


def test_casualties_far_apart_stay_separate():
    t = tracker()
    for i in range(CFG["tracker"]["confirm_hits"]):
        confirmed = t.update([((0.0, 0.0, 0.0), 0.9, 309.0), ((40.0, 0.0, 0.0), 0.9, 308.0)], float(i))
    assert len({c.track_id for c in confirmed}) == 2


def test_a_found_casualty_is_never_forgotten_but_stops_being_current():
    """The drone flies on; the casualty stays where it is and stays on the list."""
    t = tracker()
    for i in range(CFG["tracker"]["confirm_hits"]):
        t.update([((0.0, 0.0, 0.0), 0.9, 309.0)], float(i))
    much_later = CFG["tracker"]["forget_after_s"] + 10
    assert len(t.confirmed(now_s=much_later)) == 1
    assert t.current(now_s=much_later) == ()
