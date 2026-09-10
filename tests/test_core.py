"""Checks for the pure-logic core. Run: python3 -m pytest tests -q  (or python3 tests/test_core.py)."""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from aerosense import geo, planner, risk, world_model  # noqa: E402
from aerosense.comms import StoreAndForwardLink  # noqa: E402

INTRINSICS = (500.0, 500.0, 320.0, 240.0)
ALT = 15.0
GROUND_AHEAD = ALT / math.tan(geo.CAMERA_TILT_RAD)
CENTRE_DEPTH = ALT / math.sin(geo.CAMERA_TILT_RAD)


# -- geo ----------------------------------------------------------------------

def test_centre_pixel_hits_ground_ahead_of_level_drone():
    p = geo.pixels_to_ned([320], [240], [CENTRE_DEPTH], INTRINSICS, (0, 0, -ALT), (0, 0, 0))[0]
    assert np.allclose(p, (GROUND_AHEAD, 0, 0), atol=1e-6)


def test_yaw_east_projects_east():
    p = geo.pixels_to_ned([320], [240], [CENTRE_DEPTH], INTRINSICS,
                          (0, 0, -ALT), (0, 0, math.pi / 2))[0]
    assert np.allclose(p, (0, GROUND_AHEAD, 0), atol=1e-6)


def test_image_right_is_body_right():
    p = geo.pixels_to_ned([420], [240], [CENTRE_DEPTH], INTRINSICS, (0, 0, -ALT), (0, 0, 0))[0]
    assert p[1] > 1.0


def test_ned_gz_roundtrip():
    assert geo.gz_to_ned(*geo.ned_to_gz(3.0, 4.0, -5.0)) == (3.0, 4.0, -5.0)


# -- risk ---------------------------------------------------------------------

def test_survivor_in_fire_is_critical_safe_one_is_not():
    _, sev, dng = risk.hazard_exposure({"fire": 0.0})
    assert risk.risk_level(risk.risk_score(0.9, sev, dng, 0.3)) == "CRITICAL"
    assert risk.risk_level(risk.risk_score(0.9, 0.0, 0.0, 1.0)) == "MEDIUM"


def test_weak_detection_stays_low_even_in_danger():
    assert risk.risk_level(risk.risk_score(0.2, 1.0, 1.0, 0.0)) == "LOW"


def test_hazard_exposure_picks_worst():
    assert risk.hazard_exposure({"fire": 20.0, "flood": 0.0})[0] == "flood"
    assert risk.hazard_exposure({})[0] == "none"


# -- planner ------------------------------------------------------------------

def _empty(n=10):
    return np.zeros((n, n), dtype=bool)


def test_astar_routes_through_gap_in_wall():
    obstacle = _empty()
    obstacle[:, 5] = True
    obstacle[8, 5] = False
    route = planner.plan_route(planner.cost_grid(obstacle, _empty(), _empty()), (0, 0), (0, 9))
    assert route is not None and (8, 5) in route[0]


def test_unreachable_goal_has_zero_accessibility():
    obstacle = _empty()
    obstacle[:, 5] = True
    route = planner.plan_route(planner.cost_grid(obstacle, _empty(), _empty()), (0, 0), (0, 9))
    assert route is None and planner.accessibility(route, (0, 0), (0, 9)) == 0.0


def test_fire_clearance_blocks_nearby_cells():
    fire = _empty(12)
    fire[5, 5] = True
    cost = planner.cost_grid(_empty(12), _empty(12), fire)
    assert np.isinf(cost[5, 8]) and np.isfinite(cost[5, 9])


def test_flood_lowers_accessibility():
    water = np.ones((10, 10), dtype=bool)
    route = planner.plan_route(planner.cost_grid(_empty(), water, _empty()), (0, 0), (0, 9))
    assert 0.15 < planner.accessibility(route, (0, 0), (0, 9)) < 0.3


def test_goal_on_obstacle_cell_is_still_reachable():
    obstacle = _empty()
    obstacle[4, 4] = True
    route = planner.plan_route(planner.cost_grid(obstacle, _empty(), _empty()), (0, 0), (4, 4))
    assert route is not None and route[0][-1] == (4, 4)


# -- comms --------------------------------------------------------------------

def test_link_buffers_during_outage_and_flushes_in_order():
    got = []
    link = StoreAndForwardLink(got.append, outages=[(10, 20)])
    link.send({"type": "survivor", "id": 1}, 5)
    link.send({"type": "telemetry", "n": 1}, 12)
    link.send({"type": "survivor", "id": 2}, 13)
    link.send({"type": "telemetry", "n": 2}, 14)
    assert len(got) == 1 and link.buffered == 2       # telemetry coalesced
    link.send({"type": "telemetry", "n": 3}, 25)
    assert got == [{"type": "survivor", "id": 1}, {"type": "survivor", "id": 2},
                   {"type": "telemetry", "n": 2}, {"type": "telemetry", "n": 3}]
    assert link.buffered == 0


def test_forced_link_loss():
    got = []
    link = StoreAndForwardLink(got.append)
    link.set_forced_down(True)
    assert not link.send({"type": "hazard"}, 0) and got == []
    link.set_forced_down(False)
    assert link.send({"type": "hazard"}, 1) and len(got) == 2


# -- world model --------------------------------------------------------------

def test_depth_points_build_height_map_and_obstacles():
    wm = world_model.add_depth_points(world_model.empty_map(), [[10.2, 0.3, -6.0], [10.4, 0.1, 0.0]])
    i, j = world_model.to_cell(10.2, 0.3)
    assert wm.height[i, j] == 6.0 and world_model.obstacle_mask(wm)[i, j]
    assert np.isnan(world_model.empty_map().height).all()


def test_survivor_detections_merge_nearby_and_split_far():
    wm = world_model.empty_map()
    for t, (n, e) in enumerate([(20, 5), (21, 5), (20.5, 6), (40, -10)]):
        wm = world_model.add_survivor(wm, n, e, 0.6, t == 1, float(t))
    first, second = wm.survivors
    assert first.hits == 3 and first.confirmed and first.thermal
    assert second.hits == 1 and not second.confirmed


def test_fire_needs_repeated_evidence():
    pts = [[30.5, 0.5, 0.0]]
    once = world_model.add_hazard_points(world_model.empty_map(), "fire", pts)
    twice = world_model.add_hazard_points(once, "fire", pts)
    assert not world_model.hazard_masks(once)["fire"].any()
    assert world_model.hazard_masks(twice)["fire"].any()
    assert world_model.hazard_distances(twice, 30.5, 4.5)["fire"] == 4.0


def test_survivor_is_not_its_own_collapse_hazard():
    wm = world_model.add_depth_points(world_model.empty_map(), [[20.5, 5.5, -1.8]])
    assert "collapse" not in world_model.hazard_distances(wm, 20.5, 5.5)


def test_encoded_map_covers_whole_grid():
    code = world_model.encode_map(world_model.empty_map())
    assert len(code) == world_model.SHAPE[0] * world_model.SHAPE[1] and set(code) == {"0"}


if __name__ == "__main__":
    tests = [f for name, f in sorted(globals().items()) if name.startswith("test_")]
    for t in tests:
        t()
    print(f"{len(tests)} checks passed")
