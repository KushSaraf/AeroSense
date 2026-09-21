"""The ground link: where it drops, what it carries when weak, and what goes first on reconnect."""
from types import SimpleNamespace

from aero_sense_mission import comms
from aero_sense_mission.mission_manager import SCENARIO_AREAS
from aero_sense_mission.search_pattern import Area, lawnmower

ZONE = {"test": Area(0.0, 0.0, 10.0, 10.0)}


def test_inside_a_dead_zone_the_link_is_down():
    assert comms.link_state(5.0, 5.0, zones=ZONE) == comms.OFFLINE


def test_near_a_dead_zone_the_link_is_weak_and_far_from_it_fine():
    assert comms.link_state(15.0, 5.0, zones=ZONE) == comms.DEGRADED
    assert comms.link_state(40.0, 5.0, zones=ZONE) == comms.CONNECTED


def test_a_drone_hovering_on_the_boundary_does_not_flap():
    assert comms.link_state(11.0, 5.0, zones=ZONE, was_offline=True) == comms.OFFLINE
    assert comms.link_state(11.0, 5.0, zones=ZONE, was_offline=False) == comms.DEGRADED
    assert comms.link_state(14.0, 5.0, zones=ZONE, was_offline=True) == comms.DEGRADED


def test_cutting_the_link_by_hand_works_anywhere():
    assert comms.link_state(40.0, 5.0, forced_down=True, zones=ZONE) == comms.OFFLINE


def test_a_weak_link_carries_telemetry_but_not_video():
    assert comms.passes(comms.DEGRADED, comms.LATEST)
    assert comms.passes(comms.DEGRADED, comms.QUEUE)
    assert not comms.passes(comms.DEGRADED, comms.DROP)
    assert not comms.passes(comms.OFFLINE, comms.LATEST)


def test_holding_keeps_the_newest_telemetry_every_event_and_no_video():
    held = {}
    held = comms.hold(held, "pose", comms.LATEST, "old")
    held = comms.hold(held, "pose", comms.LATEST, "new")
    held = comms.hold(held, "events", comms.QUEUE, "a")
    held = comms.hold(held, "events", comms.QUEUE, "b")
    after = comms.hold(held, "rgb", comms.DROP, "frame")
    assert after == {"pose": "new", "events": ("a", "b")}
    assert after is held                      # dropping changes nothing


def test_holding_does_not_touch_what_was_held_before():
    before = {"events": ("a",)}
    comms.hold(before, "events", comms.QUEUE, "b")
    assert before == {"events": ("a",)}


def test_on_reconnect_casualties_go_first_then_events_in_order_then_telemetry():
    held = {"pose": "p", "events": ("a", "b"), "mission_state": "s", "victims": "v"}
    assert comms.flush_order(held) == [
        ("victims", "v"), ("mission_state", "s"), ("events", "a"), ("events", "b"), ("pose", "p")]


def test_undelivered_casualties_are_counted_by_priority():
    victims = [SimpleNamespace(victim_id=i, priority=p) for i, p in
               (("V-1", "P1"), ("V-2", "P1"), ("V-3", "P3"), ("V-4", ""))]
    assert comms.undelivered(victims, {"V-1"}) == {"P1": 1, "P2": 0, "P3": 1}


def test_the_dead_zone_is_inside_the_earthquake_sector_and_the_search_flies_through_it():
    area, zone = SCENARIO_AREAS["earthquake"], comms.NO_NETWORK_ZONES["north_east_blocks"]
    assert area.min_x <= zone.min_x and zone.max_x <= area.max_x
    assert area.min_y <= zone.min_y and zone.max_y <= area.max_y
    legs_inside = {y for x, y in lawnmower(area, 25.0) if zone.min_y <= y <= zone.max_y}
    assert len(legs_inside) >= 2


def test_the_backlog_goes_out_at_a_rate_not_all_at_once():
    """A link that has just come back is not a healthy one."""
    tokens = comms.refill(0.0, 1.0, comms.SEND_RATE_HZ[comms.CONNECTED])
    assert tokens == comms.SEND_BURST                       # a second of waiting fills the bucket
    assert comms.refill(0.0, 0.1, comms.SEND_RATE_HZ[comms.CONNECTED]) == 2.5
    # a weak link drains more slowly than a strong one, and neither goes backwards
    assert comms.refill(0.0, 0.1, comms.SEND_RATE_HZ[comms.DEGRADED]) == 0.5
    assert comms.refill(3.0, -5.0, comms.SEND_RATE_HZ[comms.CONNECTED]) == 3.0


def test_what_a_paced_flush_did_not_reach_is_still_held_in_order():
    policies = {"victims": comms.LATEST, "events": comms.QUEUE}
    held = comms.hold(comms.hold(comms.hold(
        {}, "events", comms.QUEUE, "one"), "events", comms.QUEUE, "two"),
        "victims", comms.LATEST, "the list")

    ordered = comms.flush_order(held)
    assert ordered[0] == ("victims", "the list")            # casualties first, always

    rest = comms.regroup(ordered[1:], policies)             # only the first one got through
    assert rest == {"events": ("one", "two")}
    assert comms.flush_order(rest) == [("events", "one"), ("events", "two")]
