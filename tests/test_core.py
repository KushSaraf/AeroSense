"""Checks for the pure-logic core. Run: python3 -m pytest tests -q  (or python3 tests/test_core.py)."""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from aerosense import dashboard, flight, geo, mission, planner, risk, world_model  # noqa: E402
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
    looked_up = world_model.heights_at(wm, [10.2, 50.0, 999.0], [0.3, 0.0, 0.0])
    assert looked_up[0] == 6.0 and np.isnan(looked_up[1]) and len(looked_up) == 2


def test_closer_look_overrides_far_height_and_far_look_is_ignored():
    cell_point = lambda h: [[20.5, 5.5, -h]]
    far = world_model.add_depth_points(world_model.empty_map(), cell_point(3.0), [25.0])
    near = world_model.add_depth_points(far, cell_point(0.1), [8.0])
    i, j = world_model.to_cell(20.5, 5.5)
    assert far.height[i, j] == 3.0 and near.height[i, j] == 0.1
    again_far = world_model.add_depth_points(near, cell_point(3.0), [25.0])
    assert again_far.height[i, j] == 0.1
    similar = world_model.add_depth_points(near, cell_point(1.5), [9.0])     # within tie: max
    assert similar.height[i, j] == 1.5


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


# -- mission logic ------------------------------------------------------------

class _State:
    def __init__(self, n, e, alt):
        self.n, self.e, self.d = n, e, -alt


def test_lawnmower_alternates_lane_direction():
    wps = mission.lawnmower((10.0, 30.0), (-5.0, 5.0), 10.0)
    assert wps == ((10.0, -5.0), (10.0, 5.0), (20.0, 5.0), (20.0, -5.0), (30.0, -5.0), (30.0, 5.0))


def test_tall_obstacle_on_track_triggers_climb_and_clear_track_returns():
    wm = world_model.add_depth_points(world_model.empty_map(), [[36.5, 0.5, -12.0]])
    tallest = mission.tallest_ahead(wm, _State(36.5, -10.0, 15.0), (36.5, 20.0))
    assert tallest == 12.0
    assert mission.avoid_altitude(tallest, mission.SEARCH_ALT_M) == 20.0
    assert mission.tallest_ahead(wm, _State(36.5, 10.0, 20.0), (36.5, 20.0)) == 0.0
    assert mission.avoid_altitude(0.0, 20.0) == mission.SEARCH_ALT_M


def test_low_obstacle_does_not_trigger_climb():
    assert mission.avoid_altitude(3.0, mission.SEARCH_ALT_M) == mission.SEARCH_ALT_M


def test_climb_target_is_stepped_so_creeping_height_does_not_retrigger():
    first = mission.avoid_altitude(9.2, mission.SEARCH_ALT_M)
    assert first == 20.0
    assert mission.avoid_altitude(9.6, first) == first          # same step: no new climb


def test_outage_argument_validation():
    assert mission.parse_outage("45:85") == (45.0, 85.0)
    for bad in ("85:45", "abc", "1:2:3", "-5:10"):
        try:
            mission.parse_outage(bad)
        except Exception:
            continue
        raise AssertionError(f"accepted bad outage {bad!r}")


def test_pose_is_interpolated_at_capture_time_with_yaw_wrap():
    attitudes = ((0.0, 0.0, 0.0, 3.1), (1.0, 0.2, 0.0, -3.1))        # yaw crosses +-pi
    positions = ((0.0, 0.0, 0.0, -15.0), (1.0, 4.0, 0.0, -15.0))
    ned, (roll, _, yaw) = flight.interpolate_pose(attitudes, positions, 0.5)
    assert ned == (2.0, 0.0, -15.0) and abs(roll - 0.1) < 1e-9 and abs(abs(yaw) - math.pi) < 0.01
    assert flight.interpolate_pose(attitudes, positions, 1.5) is None
    assert flight.interpolate_pose(attitudes[:1], positions, 0.5) is None
    held, _ = flight.interpolate_pose(attitudes, positions, 1.05)     # just past newest sample
    assert held == (4.0, 0.0, -15.0)


def test_near_field_keeps_only_close_tall_points():
    points = [[40.0, 0.0, -12.0],     # tall, 10 m away: keep
              [40.0, 0.0, -2.0],      # close but low: ground/debris, not trusted
              [70.0, 0.0, -12.0]]     # tall but 40 m away: pose error too large
    kept = mission.near_field_obstacles(points, (30.0, 0.0))
    assert kept.tolist() == [[40.0, 0.0, -12.0]]


def test_param_echo_comparison_tolerates_float32():
    assert flight.param_matches(4.0, float(np.float32(4.0)))
    assert flight.param_matches(0.3, float(np.float32(0.3)))
    assert not flight.param_matches(4.0, 10.0)


def test_hard_bank_or_missing_pose_is_not_mapped():
    assert mission.is_mappable(((0, 0, -15), (0.05, -0.05, 1.0)))
    assert not mission.is_mappable(((0, 0, -15), (0.0, math.radians(20), 1.0)))
    assert not mission.is_mappable(None)


# -- dashboard ----------------------------------------------------------------

def test_ground_station_replaces_survivor_list_and_ranks_it():
    ground = dashboard.GroundStation()
    ground.deliver({"type": "survivors", "items": [{"id": 1, "score": 0.3}, {"id": 5, "score": 0.4}]})
    ground.deliver({"type": "survivors", "items": [{"id": 1, "score": 0.9}, {"id": 2, "score": 0.8}]})
    ground.deliver({"type": "bogus"})
    snap = ground.snapshot()
    assert [s["id"] for s in snap["survivors"]] == [1, 2] and snap["rx_count"] == 2


def test_survivor_tracks_that_drift_together_are_merged():
    wm = world_model.add_survivor(world_model.empty_map(), 30.0, 5.0, 0.5, True, 0.0)
    wm = world_model.add_survivor(wm, 34.0, 5.0, 0.6, False, 1.0)       # 4 m away: new track
    assert len(wm.survivors) == 2
    wm = world_model.add_survivor(wm, 31.5, 5.0, 0.5, True, 2.0)        # drags #1 to 30.75
    wm = world_model.add_survivor(wm, 32.5, 5.0, 0.5, True, 3.0)        # now within 3 m of #2
    assert len(wm.survivors) == 1
    (only,) = wm.survivors
    assert only.id == 1 and only.hits == 4 and only.thermal


def test_link_endpoint_validates_input():
    link = StoreAndForwardLink(lambda m: None)
    client = dashboard.create_app(dashboard.GroundStation(), link).test_client()
    assert client.post("/api/link", json={"down": "yes"}).status_code == 400
    assert client.post("/api/link", data="not json").status_code == 400
    assert client.post("/api/link", json={"down": True}).status_code == 200 and link.forced_down
    assert client.get("/api/state").get_json()["sim_link_forced_down"] is True


if __name__ == "__main__":
    tests = [f for name, f in sorted(globals().items()) if name.startswith("test_")]
    for t in tests:
        t()
    print(f"{len(tests)} checks passed")
