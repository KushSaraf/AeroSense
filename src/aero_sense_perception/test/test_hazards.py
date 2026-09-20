"""Disaster segmentation's input, and HSI: pixels on the ground, cells, severity, regions."""
import numpy as np

from aero_sense_perception import disaster, hsi
from aero_sense_perception.disaster import CLASS_INDEX


def cell_of(x0, y0, cls, n=40):
    """n pixels of one class spread over the 5 m cell whose corner is (x0, y0)."""
    xy = np.column_stack([np.linspace(x0 + 0.1, x0 + 4.9, n), np.linspace(y0 + 0.1, y0 + 4.9, n)])
    return xy, np.full(n, CLASS_INDEX[cls])


def test_a_block_of_rubble_is_one_critical_structural_region():
    grid = hsi.HazardGrid()
    for x0 in (0, 5, 10):
        grid.add(*cell_of(x0, 0, "collapsed"))
    [region] = grid.regions()
    assert (region.severity, region.type, region.area_m2, region.hsi) == ("CRITICAL", "structural", 75.0, 1.0)
    assert region.centroid == (7.5, 2.5)


def test_a_road_half_under_rubble_is_high_and_a_clear_road_is_no_hazard():
    grid = hsi.HazardGrid()
    grid.add(*cell_of(0, 0, "road"))
    grid.add(*cell_of(0, 0, "collapsed"))
    grid.add(*cell_of(50, 50, "road"))
    [region] = grid.regions()
    assert region.severity == "HIGH" and region.hsi == 0.5


def test_flood_water_is_a_flood_hazard_and_separate_patches_are_separate_regions():
    grid = hsi.HazardGrid()
    grid.add(*cell_of(0, 0, "water"))
    grid.add(*cell_of(20, 0, "water"))
    regions = grid.regions()
    assert len(regions) == 2 and {r.type for r in regions} == {"flood"} and {r.severity for r in regions} == {"HIGH"}
    assert len({r.hazard_id for r in regions}) == 2


def test_a_cell_seen_by_too_few_pixels_is_not_judged():
    grid = hsi.HazardGrid()
    grid.add(*cell_of(0, 0, "collapsed", n=hsi.MIN_SAMPLES - 1))
    assert grid.regions() == []


def test_pixels_land_under_a_nadir_camera():
    k = [100.0, 0, 50.0, 0, 100.0, 40.0, 0, 0, 1]
    down = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)     # optical z along map -z
    xy, ok = hsi.ground_points([50.0, 150.0], [40.0, 40.0], k, (10.0, 20.0, 30.0), down, max_range_m=20.0)
    assert ok.tolist() == [True, False]                               # the second lands 30 m out
    assert np.allclose(xy[0], (10.0, 20.0))


def test_the_thermal_view_is_the_centre_of_the_rgb_frame():
    rgb = np.tile(np.arange(960, dtype=np.float32)[None, :, None], (600, 1, 3))   # value = column
    view = disaster.rgb_in_thermal_view(rgb, 2.2166, (256, 192), 0.9948)
    assert view.shape == (192, 256, 3)
    scale = disaster.focal_px(960, 2.2166) / disaster.focal_px(256, 0.9948)
    assert abs(view[96, 128, 0] - 480) < 1.5                          # centre on centre
    assert abs(view[96, 0, 0] - (480 - 128 * scale)) < 1.5            # the edge, scaled about it
    x = disaster.network_input(np.zeros((192, 256, 3), np.uint8), np.full((192, 256), 305.0))
    assert x.shape == (4, 192, 256) and np.isclose(x[3, 0, 0], 0.5)
