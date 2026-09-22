"""The learned thermal person detector (YOLOv8n: HIT-UAV, then this world's renders; ml/thermal/).

Pure numpy, no ROS. Training (ml/thermal/make_dataset.py) and the drone (victim_detector) both
turn kelvin into the network's image through `to_image`; change it in one place.
"""
import numpy as np

#: Kelvin mapped onto 0-255, white-hot, as HIT-UAV's camera shows people: water (291 K) near
#: black, ground (298 K) mid-grey, a live body (306 K and up) near white. A fixed window, not
#: per-frame gain, so an empty frame is not stretched into contrast that is not there.
WINDOW_K = (290.0, 307.0)


def to_image(kelvin: np.ndarray, window_k=WINDOW_K) -> np.ndarray:
    """(H, W, 3) uint8: the frame through `window_k`, grey in all three channels."""
    low, high = window_k
    grey = np.clip((kelvin - low) * (255.0 / (high - low)), 0.0, 255.0).round().astype(np.uint8)
    return np.repeat(grey[..., None], 3, axis=2)
