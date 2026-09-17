"""Visual-inertial odometry in the map frame: fitting OpenVINS's own frame onto the map, and drift.

OpenVINS starts its world frame where it initialises, with gravity along -z but an arbitrary
heading and origin. With both tracks recorded while GPS is good, a heading and an offset fit one
onto the other (gravity is already shared, so 4 degrees of freedom, not 6). Pure numpy, no ROS.
"""
import math

import numpy as np


def rotate(xy, yaw: float) -> np.ndarray:
    """Points (N x 2) turned anticlockwise by `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(xy, float) @ np.array([[c, s], [-s, c]])


def fit_yaw_translation(source_xy, target_xy) -> tuple:
    """Least-squares (yaw, tx, ty) taking `source_xy` points onto `target_xy` (N x 2 each)."""
    source, target = np.asarray(source_xy, float), np.asarray(target_xy, float)
    if len(source) < 2 or source.shape != target.shape:
        raise ValueError("need at least two matching points")
    s_mean, t_mean = source.mean(axis=0), target.mean(axis=0)
    s, t = source - s_mean, target - t_mean
    yaw = math.atan2(float(np.sum(s[:, 0] * t[:, 1] - s[:, 1] * t[:, 0])), float(np.sum(s * t)))
    tx, ty = t_mean - rotate(s_mean, yaw)
    return yaw, float(tx), float(ty)


def apply(xy, fit: tuple) -> np.ndarray:
    yaw, tx, ty = fit
    return rotate(xy, yaw) + (tx, ty)


def drift_report(estimate_xy, truth_xy) -> dict:
    """Horizontal error of an already-aligned track against the truth, over the distance flown."""
    estimate, truth = np.asarray(estimate_xy, float), np.asarray(truth_xy, float)
    errors = np.linalg.norm(estimate - truth, axis=1)
    flown = float(np.sum(np.linalg.norm(np.diff(truth, axis=0), axis=1)))
    return {"samples": len(errors), "flown_m": flown, "rmse_m": float(np.sqrt(np.mean(errors ** 2))),
            "max_m": float(errors.max()), "final_m": float(errors[-1]),
            "final_percent_of_flown": 100.0 * float(errors[-1]) / flown if flown > 0 else math.nan}
