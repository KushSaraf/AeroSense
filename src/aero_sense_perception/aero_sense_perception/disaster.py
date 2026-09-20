"""Disaster segmentation's input and classes, shared by training (ml/segformer/) and the onboard
hazard mapper, so the network sees in flight exactly what it was trained on. No ROS.

The input is multimodal: the thermal image and the part of the RGB image the thermal camera sees.
The thermal camera's 57 deg is the centre of the RGB camera's 127 deg and the two sit centimetres
apart (nothing at 20-35 m), so the RGB crop is a scale about the image centre; both become one
4-channel image at the thermal camera's resolution, in its geometry.
"""
import math

import cv2
import numpy as np

#: What each pixel is. The order is the network's output channels.
CLASSES = ("background", "road", "intact", "damaged", "collapsed", "water", "vehicle")
CLASS_INDEX = {name: i for i, name in enumerate(CLASSES)}
#: Thermal normalisation: kelvin across this range maps onto 0..1 (ground 298 K, bodies ~310 K).
THERMAL_RANGE_K = (285.0, 325.0)
#: ImageNet statistics, which SegFormer's pretrained encoder expects for RGB.
RGB_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
RGB_STD = np.array([0.229, 0.224, 0.225], np.float32)


def focal_px(width: int, hfov_rad: float) -> float:
    return (width / 2) / math.tan(hfov_rad / 2)


def rgb_in_thermal_view(rgb: np.ndarray, rgb_hfov: float, thermal_size: tuple, thermal_hfov: float) -> np.ndarray:
    """The RGB pixels the thermal camera sees, resampled onto the thermal image's grid.

    `thermal_size` is (width, height). Both cameras are pinhole with square pixels, looking the
    same way from (nearly) the same point.
    """
    height, width = rgb.shape[:2]
    scale = focal_px(width, rgb_hfov) / focal_px(thermal_size[0], thermal_hfov)
    half_w, half_h = thermal_size[0] / 2 * scale, thermal_size[1] / 2 * scale
    cx, cy = width / 2, height / 2
    # an affine map from thermal pixels to RGB pixels, sampled with bilinear interpolation
    matrix = np.array([[scale, 0.0, cx - half_w], [0.0, scale, cy - half_h]], np.float32)
    return cv2.warpAffine(rgb, matrix, thermal_size, flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                          borderMode=cv2.BORDER_REPLICATE)


def network_input(rgb_view: np.ndarray, kelvin: np.ndarray) -> np.ndarray:
    """(4, H, W) float32: RGB (uint8, already in the thermal view) normalised as ImageNet, then
    thermal (kelvin) scaled over THERMAL_RANGE_K."""
    rgb = (rgb_view.astype(np.float32) / 255.0 - RGB_MEAN) / RGB_STD
    low, high = THERMAL_RANGE_K
    thermal = np.clip((kelvin.astype(np.float32) - low) / (high - low), 0.0, 1.0)
    return np.concatenate([rgb.transpose(2, 0, 1), thermal[None]], axis=0)
