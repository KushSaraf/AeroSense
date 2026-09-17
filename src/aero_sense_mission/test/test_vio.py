"""Fitting OpenVINS's frame onto the map, and measuring how far it drifted."""
import math

import numpy as np
import pytest

from aero_sense_mission import vio

TRUTH = np.array([(0.0, -110.0), (0.0, -80.0), (-40.0, -60.0), (-120.0, 20.0), (-60.0, 35.0)])


def test_a_turned_and_shifted_track_fits_back_exactly():
    fit = (math.radians(73.0), 12.5, -110.0)
    source = vio.rotate(TRUTH - (fit[1], fit[2]), -fit[0])       # what OpenVINS would report
    yaw, tx, ty = vio.fit_yaw_translation(source, TRUTH)
    assert (math.degrees(yaw), tx, ty) == pytest.approx((73.0, 12.5, -110.0), abs=1e-9)
    assert np.allclose(vio.apply(source, (yaw, tx, ty)), TRUTH)


def test_drift_is_measured_against_the_distance_flown():
    estimate = TRUTH + np.array([(0, 0), (0, 0), (0, 0), (0, 0), (3.0, 4.0)])
    report = vio.drift_report(estimate, TRUTH)
    flown = sum(math.dist(a, b) for a, b in zip(TRUTH, TRUTH[1:]))
    assert report["final_m"] == pytest.approx(5.0) and report["max_m"] == pytest.approx(5.0)
    assert report["final_percent_of_flown"] == pytest.approx(500.0 / flown)


def test_one_point_is_not_enough_to_fit():
    with pytest.raises(ValueError):
        vio.fit_yaw_translation(TRUTH[:1], TRUTH[:1])
