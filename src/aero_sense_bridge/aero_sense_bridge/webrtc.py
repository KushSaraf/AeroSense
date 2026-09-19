"""The drone's cameras over WebRTC (aiortc): the browser offers, the bridge answers with a video track.

The track sends the same downlink frames the MJPEG stream does, so a network outage blanks both
alike. Only host candidates are gathered (no STUN): the dashboard reaches the bridge directly on
the ground station's network. MJPEG (/api/camera/{camera}) stays for replays and old browsers.
"""
import asyncio
import fractions
import time

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

#: Frames sent per second. The downlink delivers a few a second; more would re-encode stale frames.
FPS = 10
CLOCK_RATE = 90000                   # RTP's video clock
#: While a camera has produced nothing yet, check again this often.
WAIT_S = 0.2


class CameraTrack(VideoStreamTrack):
    """One camera's latest frame, re-read FPS times a second."""

    def __init__(self, image):
        super().__init__()
        self._image = image              # () -> BGR ndarray, or None before the first frame
        self._start = None
        self._count = 0

    async def recv(self) -> VideoFrame:
        if self._start is None:
            self._start = time.monotonic()
        self._count += 1
        due = self._start + self._count / FPS
        await asyncio.sleep(max(0.0, due - time.monotonic()))
        image = self._image()
        while image is None:
            await asyncio.sleep(WAIT_S)
            image = self._image()
        frame = VideoFrame.from_ndarray(image, format="bgr24")
        frame.pts = self._count * CLOCK_RATE // FPS
        frame.time_base = fractions.Fraction(1, CLOCK_RATE)
        return frame


class WebRtcCameras:
    """Answers offers and keeps the peer connections until they close."""

    def __init__(self, image):
        self._image = image              # (camera) -> BGR ndarray or None
        self._peers = set()

    async def answer(self, camera: str, sdp: str, kind: str) -> dict:
        peer = RTCPeerConnection()
        self._peers.add(peer)

        @peer.on("connectionstatechange")
        async def _closed():
            if peer.connectionState in ("failed", "closed"):
                await peer.close()
                self._peers.discard(peer)

        try:
            await peer.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=kind))
            peer.addTrack(CameraTrack(lambda: self._image(camera)))
            await peer.setLocalDescription(await peer.createAnswer())
        except Exception:
            await peer.close()
            self._peers.discard(peer)
            raise
        return {"sdp": peer.localDescription.sdp, "type": peer.localDescription.type}

    async def close(self) -> None:
        await asyncio.gather(*(peer.close() for peer in self._peers))
        self._peers.clear()
