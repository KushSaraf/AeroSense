"""WebRTC: a browser-side peer offers, the bridge answers, and the camera's frames arrive."""
import asyncio

import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription

from aero_sense_bridge.webrtc import WebRtcCameras

FRAME = np.zeros((96, 128, 3), np.uint8)
FRAME[:, :64] = (0, 0, 255)                 # left half red (BGR)


async def _first_frame(image):
    cameras = WebRtcCameras(image)
    viewer = RTCPeerConnection()
    viewer.addTransceiver("video", direction="recvonly")
    received = asyncio.get_running_loop().create_future()

    @viewer.on("track")
    def on_track(track):
        async def read():
            received.set_result(await track.recv())
        asyncio.ensure_future(read())

    await viewer.setLocalDescription(await viewer.createOffer())
    answer = await cameras.answer("rgb", viewer.localDescription.sdp, viewer.localDescription.type)
    await viewer.setRemoteDescription(RTCSessionDescription(**answer))
    try:
        return await asyncio.wait_for(received, 20)
    finally:
        await viewer.close()
        await cameras.close()


def test_the_camera_frame_reaches_the_viewer():
    frame = asyncio.run(_first_frame(lambda camera: FRAME if camera == "rgb" else None)).to_ndarray(format="bgr24")
    assert frame.shape == FRAME.shape
    assert frame[48, 20, 2] > 200 and frame[48, 100, 2] < 50      # red left, black right, through VP8


def test_a_malformed_offer_is_refused():
    async def offer():
        try:
            await WebRtcCameras(lambda camera: FRAME).answer("rgb", "not sdp", "offer")
        except ValueError:
            return True
        return False
    assert asyncio.run(offer())
