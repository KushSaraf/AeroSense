"""SWOOP: which faint leads the drone descends to, how low it may go, and what it concludes."""
from types import SimpleNamespace

from aero_sense_mission import swoop
from aero_sense_perception.structure_map import Structure


def report(victim_id, x, y, confidence=0.5):
    return SimpleNamespace(victim_id=victim_id, confidence=confidence, position=SimpleNamespace(x=x, y=y))


MAST = Structure("mast", "radio_tower", -40.0, 60.0, 6.7, 44.2)
HOUSE = Structure("house", "aero_sense_building_rowhouse_3f_cream", 0.0, 0.0, 6.4, 13.4)


def test_over_open_ground_it_descends_to_the_floor():
    assert swoop.verify_altitude((HOUSE,), 50.0, 50.0, floor_m=10.0, ceiling_m=30.0) == 10.0


def test_beside_a_building_it_stays_clear_above_its_roof():
    altitude = swoop.verify_altitude((HOUSE,), 8.0, 0.0, floor_m=10.0, ceiling_m=30.0)

    assert altitude > HOUSE.height_m + 5.0 and altitude < 30.0


def test_beside_the_mast_nothing_below_cruise_is_clear():
    assert swoop.verify_altitude((MAST,), -40.0, 52.0, floor_m=10.0, ceiling_m=30.0) is None


def test_it_goes_to_the_nearest_lead_likely_enough_and_not_yet_explained():
    leads = (report("S-001", 100.0, 0.0, 0.3), report("S-002", 10.0, 0.0, 0.04),
             report("S-003", 20.0, 0.0, 0.06), report("S-004", 30.0, 0.0, 0.9), report("S-005", 5.0, 5.0, 0.5))
    casualties = (report("V-001", 31.0, 1.0),)                 # S-004 is already a casualty
    visited = [(4.0, 4.0)]                                     # S-005 has been verified

    lead = swoop.next_lead(leads, casualties, visited, (0.0, 0.0), min_probability=0.05, skip_radius_m=8.0)

    assert lead.victim_id == "S-003"                           # S-002 is under 5 %


def test_nothing_left_to_verify():
    assert swoop.next_lead((), (), [], (0.0, 0.0), 0.05, 8.0) is None


def test_a_casualty_confirmed_at_the_spot_proves_the_lead_and_none_rules_it_out():
    casualties = (report("V-001", 50.0, 50.0), report("V-002", 3.0, 1.0))

    assert swoop.proven(casualties, 0.0, 0.0, radius_m=8.0).victim_id == "V-002"
    assert swoop.proven(casualties, -30.0, 0.0, radius_m=8.0) is None


def test_leads_outside_the_search_area_are_not_its_business():
    """The first flight descended on the command base's warm pad four times before searching."""
    area = SimpleNamespace(min_x=-180.0, min_y=10.0, max_x=-20.0, max_y=92.0)
    leads = (report("S-001", 0.0, -110.0, 0.75), report("S-002", -100.0, 50.0, 0.1))

    lead = swoop.next_lead(leads, (), [], (0.0, -110.0), 0.05, 8.0, area)

    assert lead.victim_id == "S-002"
