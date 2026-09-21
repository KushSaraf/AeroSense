"""The simulator's gas: a Gaussian plume, and what the drone's sensors make of it."""
import math

import pytest
import yaml
from pathlib import Path

from aero_sense_bringup import plume

EAST_WIND = {"speed_mps": 3.0, "from_deg": 270.0, "stability": "B"}      # from the west, blowing east
GAS_FILE = Path(__file__).resolve().parents[2] / "aero_sense_gazebo" / "config" / "gas.yaml"


def _source(z=0.0, rate=100.0, species="NH3"):
    return {"species": species, "x": 0.0, "y": 0.0, "z": z, "rate_g_per_s": rate}


def test_a_ground_level_leak_matches_the_textbook_centreline():
    """For H = 0 at z = 0 the reflected plume is C = Q / (pi u sy sz): the formula is the one we
    think it is, and the ppm conversion is right."""
    x = 200.0
    sy, sz = plume.BRIGGS_RURAL["B"][0](x), plume.BRIGGS_RURAL["B"][1](x)
    grams_per_m3 = 100.0 / (math.pi * 3.0 * sy * sz)
    expected_ppm = grams_per_m3 * 1000.0 * 24.45 / 17.03

    assert plume.concentration_ppm(_source(), EAST_WIND, x, 0.0, 0.0) == pytest.approx(expected_ppm)


def test_nothing_upwind_and_less_off_the_centreline():
    source = _source(z=10.0)
    assert plume.concentration_ppm(source, EAST_WIND, -50.0, 0.0, 10.0) == 0.0          # upwind
    centre = plume.concentration_ppm(source, EAST_WIND, 100.0, 0.0, 10.0)
    left, right = (plume.concentration_ppm(source, EAST_WIND, 100.0, y, 10.0) for y in (15.0, -15.0))
    assert left == pytest.approx(right)                                                # symmetric across
    assert centre > left > 0.0


def test_the_wind_blows_the_way_a_met_bearing_says():
    """from_deg is where it comes FROM: a south-easterly carries the gas north-west."""
    east, north = plume.downwind(135.0)
    assert east == pytest.approx(-math.sqrt(0.5)) and north == pytest.approx(math.sqrt(0.5))


def test_a_sensor_reads_nothing_below_its_floor_and_pins_at_its_top():
    assert plume.reading(0.4, (1.0, 300.0)) == (0.0, False)          # under the MiCS-6814 NH3 floor
    assert plume.reading(64.8, (1.0, 300.0)) == (64.8, False)
    assert plume.reading(900.0, (1.0, 300.0)) == (300.0, True)       # the true value may be higher


def test_the_ammonia_leak_is_readable_from_cruise_altitude_downwind():
    """The scenario has to be something the drone can find at 30 m, or gas sensing is dead weight
    on every flight: downwind of the hall the plume has spread up to the search altitude."""
    scenario = yaml.safe_load(GAS_FILE.read_text())
    wind = scenario["wind"]
    [leak] = [s for s in scenario["sources"] if s["species"] == "NH3"]
    east, north = plume.downwind(wind["from_deg"])
    x, y = leak["x"] + 100.0 * east, leak["y"] + 100.0 * north                        # 100 m downwind

    ppm = plume.concentration_ppm(leak, wind, x, y, 30.0)

    assert 25.0 <= ppm < 300.0          # over NH3's REL (25 ppm), inside the MiCS-6814's range
