"""Phase 6 checks: what the detector finds, where it says it is, and whether it keeps its name."""
import math

import numpy as np
import pytest

from aero_sense_perception import detector, geolocate
from aero_sense_perception.tracker import Tracker
from aero_sense_perception.victim_detector import load_config

CFG = load_config()
HFOV_RAD = 0.9948
#: the detector as it runs close to the ground (4 px minimum), where a speck is not a body part
DET = {**{k: v for k, v in CFG["detector"].items() if k != "min_blob_m2"}, "min_blob_px": 4}
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


def test_the_smallest_blob_is_a_hand_at_any_height():
    """The minimum is ground area, so a hand-sized patch counts from any height: one pixel from
    search altitude, several from a SWOOP close look, a lot more just off the ground."""
    area_m2 = CFG["detector"]["min_blob_m2"]
    assert 0.005 <= area_m2 <= 0.012                                     # about a forearm and hand
    at = {h: detector.min_blob_px(area_m2, h, HFOV_RAD, 256) for h in (2.0, 10.0, 30.0, 60.0)}
    assert at[60.0] == at[30.0] == 1 and at[10.0] > at[30.0] and at[2.0] > at[10.0]
    gsd_10 = 2 * 10.0 * math.tan(HFOV_RAD / 2) / 256
    assert at[10.0] * gsd_10 ** 2 <= area_m2 < (at[10.0] + 1) * gsd_10 ** 2
    one_px = {**DET, "min_blob_px": at[30.0]}
    assert len(detector.detect(frame_with([(10, 10, 1, 309.0)]), **one_px)) == 1   # a hand from 30 m


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


def seen(position, confidence, peak_k, exposure=1.0, surround_k=293.0):
    """One look at a casualty, as the detector hands it to the tracker."""
    return (position, confidence, peak_k, exposure, surround_k)


def test_a_single_hot_frame_is_not_a_casualty():
    assert tracker().update([seen((10.0, 5.0, 0.0), 0.8, 309.0)], 1.0) == ()


def test_repeated_looks_confirm_one_casualty_with_a_stable_id():
    t = tracker()
    for i in range(CFG["tracker"]["confirm_hits"]):
        confirmed = t.update([seen((10.0 + 0.3 * i, 5.0, 0.0), 0.8, 309.0)], float(i))
    assert len(confirmed) == 1
    first = confirmed[0]
    again = t.update([seen((10.2, 5.1, 0.0), 0.8, 309.0)], 9.0)
    assert again[0].track_id == first.track_id and again[0].hits == first.hits + 1
    assert again[0].confidence > first.confidence          # agreement builds certainty
    assert again[0].confidence < 1.0                       # but never certainty itself


def test_casualties_far_apart_stay_separate():
    t = tracker()
    for i in range(CFG["tracker"]["confirm_hits"]):
        confirmed = t.update([seen((0.0, 0.0, 0.0), 0.9, 309.0), seen((40.0, 0.0, 0.0), 0.9, 308.0)], float(i))
    assert len({c.track_id for c in confirmed}) == 2


def test_a_found_casualty_is_never_forgotten_but_stops_being_current():
    """The drone flies on; the casualty stays where it is and stays on the list."""
    t = tracker()
    for i in range(CFG["tracker"]["confirm_hits"]):
        t.update([seen((0.0, 0.0, 0.0), 0.9, 309.0)], float(i))
    much_later = CFG["tracker"]["forget_after_s"] + 10
    assert len(t.confirmed(now_s=much_later)) == 1
    assert t.current(now_s=much_later) == ()



WEAK = CFG["tracker"]["new_track_confidence"] / 2
STRONG = 0.9


def test_a_weak_look_never_starts_a_casualty():
    t = tracker()
    for i in range(CFG["tracker"]["confirm_hits"] + 2):
        t.update([seen((10.0, 5.0, 0.0), WEAK, 305.0)], float(i))
    assert t.tracks == ()


def test_weak_looks_keep_a_casualty_going():
    t = tracker()
    t.update([seen((10.0, 5.0, 0.0), STRONG, 309.0)], 0.0)
    for i in range(1, CFG["tracker"]["confirm_hits"]):
        confirmed = t.update([seen((10.5, 5.0, 0.0), WEAK, 305.0)], float(i))
    assert len(confirmed) == 1 and confirmed[0].hits == CFG["tracker"]["confirm_hits"]


def test_neighbours_seen_together_each_get_their_own_look():
    """A at 0 m, B at 9 m. Nearest-first sends both looks to B (4.4 m and 1 m off it); BYTE's
    optimal matching gives A the look 4.6 m off it and B the other."""
    t = tracker()
    t.update([seen((0.0, 0.0, 0.0), STRONG, 309.0), seen((9.0, 0.0, 0.0), STRONG, 309.0)], 0.0)
    t.update([seen((4.6, 0.0, 0.0), STRONG, 309.0), seen((8.0, 0.0, 0.0), STRONG, 309.0)], 1.0)
    assert [tr.hits for tr in t.tracks] == [2, 2]


def test_a_body_split_by_rubble_stays_one_casualty():
    t = tracker()
    for i in range(CFG["tracker"]["confirm_hits"]):
        confirmed = t.update([seen((0.0, 0.0, 0.0), STRONG, 309.0), seen((4.3, 0.0, 0.0), STRONG, 309.0)],
                             float(i))
    assert len(t.tracks) == 1 and len(confirmed) == 1



def test_two_tracks_that_settle_on_one_casualty_become_one():
    """First looks 7 m apart start two tracks; later looks in between pull them together, and the
    older name survives."""
    t = tracker()
    t.update([seen((0.0, 0.0, 0.0), STRONG, 309.0)], 0.0)
    t.update([seen((7.0, 0.0, 0.0), STRONG, 309.0)], 1.0)
    for i in range(2, 8):
        confirmed = t.update([seen((5.5, 0.0, 0.0), STRONG, 309.0)], float(i))
    assert [tr.track_id for tr in t.tracks] == ["V-001"]
    assert confirmed[0].hits == 8



def test_a_casualty_seen_thousands_of_times_keeps_its_confidence():
    """After ~1400 looks 0.6 ** (hits - 1) underflowed to 0: working the single-look strength back
    out of the confidence divided by it and crashed the detector mid-mission (flight_nav_none)."""
    t = tracker()
    for i in range(2000):
        confirmed = t.update([seen((0.0, 0.0, 0.0), 0.73 if i % 2 else 0.31, 309.0)], float(i))
    assert confirmed[0].hits == 2000 and confirmed[0].strength == 0.73
    assert confirmed[0].confidence == pytest.approx(0.99)


# -- SWOOP leads: faint heat that is not yet a casualty ------------------------------------------

from aero_sense_perception import suspects  # noqa: E402

SUS = {k: v for k, v in CFG["suspects"].items() if k not in ("associate_radius_m", "min_height_m", "min_looks")}
LEPTON_HFOV = 0.9948


def lepton_frame(kelvin=298.0):
    return np.full((120, 160), kelvin, dtype=np.float32)


def test_a_hand_too_small_to_be_a_casualty_is_still_a_lead():
    frame = lepton_frame()
    frame[50:52, 70] = 305.8                                   # two pixels of skin from 30 m

    assert detector.detect(frame, **DET) == ()
    (lead,) = suspects.find(frame, 30.0, LEPTON_HFOV, **SUS)
    assert 0.05 <= lead.probability < 0.5 and lead.contrast_k == pytest.approx(7.8, abs=0.01)


def test_an_arm_above_cold_flood_water_rates_higher_than_on_warm_ground():
    water, ground = lepton_frame(291.0), lepton_frame(298.0)
    water[60, 80:82] = ground[60, 80:82] = 305.8

    (in_water,) = suspects.find(water, 30.0, LEPTON_HFOV, **SUS)
    (on_ground,) = suspects.find(ground, 30.0, LEPTON_HFOV, **SUS)
    assert in_water.probability > on_ground.probability


def test_a_whole_body_is_a_certain_lead():
    frame = lepton_frame()
    frame[40:44, 40:46] = 308.4

    (lead,) = suspects.find(frame, 30.0, LEPTON_HFOV, **SUS)
    assert lead.probability == pytest.approx(1.0)


def test_warm_roads_and_heat_below_the_sensor_noise_are_not_leads():
    road = lepton_frame()
    road[:, 60:100] = 301.0                                    # an 8 m asphalt road
    faint = lepton_frame()
    faint[40:43, 40] = 299.0                                   # a buried casualty's 1 K bloom

    assert suspects.find(road, 30.0, LEPTON_HFOV, **SUS) == ()
    assert suspects.find(faint, 30.0, LEPTON_HFOV, **SUS) == ()


def test_the_same_patch_seen_again_is_one_lead_keeping_its_best_look():
    leads = suspects.merge((), [((10.0, 5.0, 0.0), 0.2, 305.8, 7.8)], radius_m=6.0)
    leads = suspects.merge(leads, [((11.0, 5.0, 0.0), 0.4, 306.0, 8.0), ((40.0, 5.0, 0.0), 0.1, 304.0, 6.0)], 6.0)
    leads = suspects.merge(leads, [((10.5, 5.0, 0.0), 0.1, 304.0, 6.0)], 6.0)

    assert [lead.lead_id for lead in leads] == ["S-001", "S-002"]
    assert leads[0].probability == 0.4 and leads[0].looks == 3     # looks do not compound


def test_ground_between_cold_collapsed_walls_is_not_a_lead():
    """Buildings read ambient (293 K), cooler than the ground, so a gali between them is a narrow
    warm strip. It is still only ground."""
    frame = lepton_frame(293.0)                                # rubble and walls
    frame[40:80, 70:73] = 298.0                                # a 60 cm strip of earth between them

    assert suspects.find(frame, 30.0, LEPTON_HFOV, **SUS) == ()


# -- RGB people (rgb.py): leads from any height, casualties from low down ---------------------------

RGB = CFG["rgb"]
LEAD_MIN_M = CFG["suspects"]["min_height_m"]


def rgb_casualties(seen, height_m):
    from aero_sense_perception import rgb
    return rgb.casualties(seen, height_m, LEAD_MIN_M, RGB["confirm_confidence"], RGB["confirm_max_height_m"],
                          AMBIENT_K)


def test_an_rgb_person_is_a_lead_from_search_height_but_not_from_the_pad():
    from aero_sense_perception import rgb
    seen = [((10.0, 5.0, 0.0), 0.4), ((30.0, 5.0, 0.0), RGB["lead_confidence"] - 0.01)]
    assert [look[:2] for look in rgb.looks(seen, 30.0, LEAD_MIN_M, RGB["lead_confidence"], AMBIENT_K)] \
        == [((10.0, 5.0, 0.0), 0.4)]
    assert rgb.looks(seen, 0.3, LEAD_MIN_M, RGB["lead_confidence"], AMBIENT_K) == []


def test_only_a_confident_close_look_makes_an_rgb_casualty():
    seen = [((10.0, 5.0, 0.0), 0.8), ((30.0, 5.0, 0.0), RGB["confirm_confidence"] - 0.01)]
    assert [c[0] for c in rgb_casualties(seen, 10.0)] == [(10.0, 5.0, 0.0)]
    assert rgb_casualties(seen, 30.0) == []                  # from search height it is only a lead


def test_someone_only_rgb_sees_is_confirmed_and_triaged_as_showing_no_heat():
    from aero_sense_perception import triage
    tracker = Tracker(**CFG["tracker"])
    for t in range(CFG["tracker"]["confirm_hits"]):
        confirmed = tracker.update(rgb_casualties([((20.0, 40.0, 0.0), 0.7)], 10.0), float(t))
    assert len(confirmed) == 1
    track = confirmed[0]
    assessment = triage.assess(triage.Observation(peak_k=track.peak_k, surround_k=AMBIENT_K, ambient_k=AMBIENT_K,
                                                  exposure=track.exposure, structure_distance_m=math.inf))
    assert assessment.priority == "P3" and "no live thermal signature" in assessment.rationale


def test_a_body_both_cameras_see_keeps_its_thermal_reading():
    tracker = Tracker(**CFG["tracker"])
    tracker.update([((20.0, 40.0, 0.0), 1.0, 309.0, 0.8, AMBIENT_K)], 0.0)            # thermal
    tracker.update(rgb_casualties([((20.5, 40.0, 0.0), 0.7)], 10.0), 0.5)             # RGB, same body
    (track,) = tracker.tracks
    assert track.hits == 2 and track.peak_k == 309.0 and track.exposure == 0.8


def test_rgb_people_far_off_nadir_are_left_out():
    from aero_sense_perception import rgb
    k = [240.0, 0.0, 480.0, 0.0, 240.0, 300.0, 0.0, 0.0, 1.0]           # the 960x600, 127 deg RGB camera
    down = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # optical z straight down
    assert rgb.off_nadir_deg(480, 300, k, down) == pytest.approx(0.0, abs=1e-6)
    assert rgb.off_nadir_deg(900, 300, k, down) > CFG["rgb"]["max_off_nadir_deg"]      # the frame's edge
    assert rgb.off_nadir_deg(480 + 240 * math.tan(math.radians(30)), 300, k, down) == pytest.approx(30.0)


def test_the_thermal_model_sees_a_body_white_and_the_ground_grey():
    """thermal_yolo.to_image, the one way kelvin reaches the thermal YOLO in training and onboard:
    white-hot like HIT-UAV, water dark, ground mid-grey, a live body near white, nothing wrapping."""
    from aero_sense_perception import thermal_yolo
    image = thermal_yolo.to_image(np.array([[285.0, 291.0, 298.0, 306.0, 550.0]]))
    assert image.shape == (1, 5, 3) and image.dtype == np.uint8
    grey = image[0, :, 0].tolist()
    assert grey[0] == 0 and grey[-1] == 255                  # clipped, not wrapped
    assert grey[1] < 30 and 100 < grey[2] < 140 and grey[3] > 230
    assert (image[..., 0] == image[..., 2]).all()


def test_each_camera_the_yolo_node_serves_has_its_settings():
    """rgb_detector reads a perception.yaml section per camera: a missing key would only show when
    the node starts in a flight."""
    from aero_sense_perception.rgb_detector import CAMERAS
    for section, _topic, floor in CAMERAS.values():
        assert {"model", "imgsz", "rate_hz", "max_off_nadir_deg", floor} <= set(CFG[section])
