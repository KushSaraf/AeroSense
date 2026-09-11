"""Triage must rank a casualty by what the drone measured, and in the order a responder needs."""
import math

import pytest

from aero_sense_perception import triage

AMBIENT = 293.0


def observe(**overrides):
    base = dict(peak_k=308.0, surround_k=AMBIENT, ambient_k=AMBIENT, exposure=1.0,
                structure_distance_m=math.inf)
    base.update(overrides)
    return triage.Observation(**base)


def test_open_ground_with_a_healthy_signature_is_the_least_urgent():
    """The complaint that started this: a casualty lying in a field was being called critical."""
    assessment = triage.assess(observe())

    assert assessment.priority == "P3"
    assert "in the open" in assessment.rationale


def test_a_casualty_beside_a_structure_outranks_one_in_the_open():
    beside = triage.assess(observe(structure_distance_m=3.0, exposure=0.7))
    open_ground = triage.assess(observe())

    assert beside.score > open_ground.score
    assert beside.priority == "P2"


def test_a_casualty_under_rubble_is_first():
    assessment = triage.assess(observe(exposure=0.25, structure_distance_m=2.0))

    assert assessment.priority == "P1"
    assert "25% of the body visible" in assessment.rationale


def test_immersion_alone_makes_a_casualty_first():
    """Cold water around a warm body: the casualty is in the flood and cooling into it."""
    assessment = triage.assess(observe(surround_k=286.0))

    assert assessment.priority == "P1"
    assert "in water" in assessment.rationale


def test_a_body_barely_above_ambient_claims_no_urgency():
    assessment = triage.assess(observe(peak_k=295.0))

    assert assessment.priority == "P3"
    assert assessment.score == 0.0
    assert "no live thermal signature" in assessment.rationale


def test_cooling_skin_raises_urgency_over_a_healthy_signature():
    cooling = triage.assess(observe(peak_k=299.0, structure_distance_m=12.0))
    healthy = triage.assess(observe(peak_k=310.0, structure_distance_m=12.0))

    assert cooling.score > healthy.score
    assert cooling.priority == "P2" and healthy.priority == "P3"


def test_structure_risk_fades_with_distance():
    against = triage.assess(observe(structure_distance_m=2.0))
    away = triage.assess(observe(structure_distance_m=12.0))
    far = triage.assess(observe(structure_distance_m=40.0))

    assert against.structure > away.structure > far.structure == 0.0


def test_exposure_needs_the_altitude_to_mean_anything():
    """The same blob is a covered body low down and a whole one seen from height."""
    focal_px = 200.0
    assert triage.exposure_of(area_px=100.0, focal_px=focal_px, height_m=10.0) < 0.4
    assert triage.exposure_of(area_px=100.0, focal_px=focal_px, height_m=30.0) == 1.0


def test_exposure_is_one_when_the_geometry_is_unknown():
    assert triage.exposure_of(area_px=50.0, focal_px=0.0, height_m=30.0) == 1.0
    assert triage.exposure_of(area_px=50.0, focal_px=200.0, height_m=0.0) == 1.0


def test_an_oblique_look_expects_a_smaller_body():
    """The camera is tilted 55 degrees down, so a body is seen about 35 degrees off vertical."""
    straight_down = triage.expected_area_px(focal_px=200.0, height_m=30.0)
    oblique = triage.expected_area_px(focal_px=200.0, height_m=30.0,
                                      cos_incidence=math.cos(math.radians(35.0)))

    assert oblique == pytest.approx(straight_down * math.cos(math.radians(35.0)) ** 3)


def test_an_uncovered_body_seen_obliquely_is_fully_exposed():
    """The live bug: open-ground casualties read ~55% covered because the tilt was ignored."""
    cos_35 = math.cos(math.radians(35.0))
    seen_px = triage.expected_area_px(200.0, 30.0, cos_35)       # exactly a whole body, obliquely

    assert triage.exposure_of(seen_px, 200.0, 30.0) < 0.6         # the old, tilt-blind answer
    assert triage.exposure_of(seen_px, 200.0, 30.0, cos_35) == pytest.approx(1.0)


def test_incidence_is_measured_from_the_line_of_sight():
    assert triage.cos_incidence((0.0, 0.0, 30.0), (0.0, 0.0, 0.0)) == pytest.approx(1.0)
    assert triage.cos_incidence((0.0, 0.0, 30.0), (30.0, 0.0, 0.0)) == pytest.approx(math.cos(math.radians(45.0)))
