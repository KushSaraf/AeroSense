"""What the drone's gas readings mean: NIOSH exposure limits, and chemical hazard regions."""
from aero_sense_perception import gas


def test_each_gas_is_judged_against_its_own_niosh_limits():
    assert gas.severity("NH3", 24.0) == ""                 # under the REL (TWA 25)
    assert gas.severity("NH3", 25.0) == "MODERATE"
    assert gas.severity("NH3", 35.0) == "HIGH"             # the short-term limit
    assert gas.severity("NH3", 300.0) == "CRITICAL"        # IDLH
    # H2S has only a ceiling REL: any exceedance is already HIGH
    assert gas.severity("H2S", 9.9) == "" and gas.severity("H2S", 10.0) == "HIGH"
    assert gas.severity("CO", 1199.0) == "HIGH" and gas.severity("CO", 1200.0) == "CRITICAL"


def test_clean_air_makes_no_region_and_touching_bad_air_makes_one():
    grid = gas.GasGrid(cell_m=10.0)
    grid.add(5.0, 5.0, {"NH3": 3.0})                       # measurable, under the REL
    assert grid.regions() == []

    grid.add(15.0, 5.0, {"NH3": 60.0})
    grid.add(25.0, 5.0, {"NH3": 120.0, "H2S": 0.0})        # the next cell along: the same region
    [region] = grid.regions()

    assert region.severity == "HIGH" and region.species == ("NH3",)
    assert region.peak_ppm == {"NH3": 120.0} and region.area_m2 == 200.0


def test_a_cell_keeps_the_worst_it_has_seen_and_its_worst_gas_decides():
    grid = gas.GasGrid(cell_m=10.0)
    grid.add(5.0, 5.0, {"NH3": 400.0})
    grid.add(6.0, 6.0, {"NH3": 30.0})                      # a later, cleaner pass does not undo it
    grid.add(7.0, 7.0, {"H2S": 12.0})

    [region] = grid.regions()
    assert region.severity == "CRITICAL" and region.species == ("H2S", "NH3")
