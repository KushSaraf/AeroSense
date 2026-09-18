"""Per-frame detections into casualties with stable identity.

One warm frame is not a casualty: a track has to be seen repeatedly, from poses that differ, and
its position is averaged over those looks. Identity is what lets the rest of the system talk
about "V-003" instead of a fresh blob every 100 ms.

Association is ByteTrack's BYTE, in the map frame rather than the image: confident detections are
matched to tracks first (optimal one-to-one, Hungarian), weak ones only to the tracks still
unmatched, and only a confident detection starts a track. A weak look (a hand at the edge of the
frame, a body seen through a gap) keeps a casualty going; it never invents one. ByteTrack's
Kalman filter is left out: a casualty does not move, and the map frame already removes the
drone's own motion.
"""
import math
from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import linear_sum_assignment

#: Each corroborating look removes this share of the remaining doubt. Repeated looks from
#: different poses raise confidence, and no number of them reaches certainty.
DOUBT_DECAY = 0.6
MAX_CONFIDENCE = 0.99


@dataclass(frozen=True)
class Track:
    track_id: str
    position: tuple          # map frame (x, y, z)
    hits: int
    confidence: float
    peak_k: float
    first_seen_s: float
    last_seen_s: float
    #: The clearest look at this casualty, which is the one triage should reason from: a body
    #: glimpsed edge-on through rubble tells you less than the same body seen from overhead.
    exposure: float = 1.0
    surround_k: float = 0.0


class Tracker:
    """BYTE association in the map frame. Immutable tracks: every update produces new ones, so a
    caller can hold a snapshot without it changing underneath."""

    def __init__(self, associate_radius_m: float, confirm_hits: int, forget_after_s: float,
                 new_track_confidence: float):
        self._radius = associate_radius_m
        self._confirm_hits = confirm_hits
        self._forget_after = forget_after_s
        self._new_track_confidence = new_track_confidence
        self._tracks = {}
        self._next_id = 1

    @property
    def tracks(self) -> tuple:
        return tuple(self._tracks.values())

    def confirmed(self, now_s: float = 0.0) -> tuple:
        """Every track seen often enough, however long ago. A casualty does not move and does not
        stop existing when the drone flies on: dropping stale tracks once erased victims from
        /aero_sense/victims mid-search."""
        return tuple(t for t in self._tracks.values() if t.hits >= self._confirm_hits)

    def current(self, now_s: float) -> tuple:
        """Confirmed tracks seen recently, for callers that care about what is in view now."""
        return tuple(t for t in self.confirmed() if now_s - t.last_seen_s <= self._forget_after)

    def update(self, detections, now_s: float) -> tuple:
        """`detections`: (position, confidence, peak_k, exposure, surround_k) tuples.

        Returns the confirmed tracks.
        """
        strong = [d for d in detections if d[1] >= self._new_track_confidence]
        weak = [d for d in detections if d[1] < self._new_track_confidence]
        unmatched = set(self._tracks)
        leftover = self._associate(strong, unmatched, now_s)
        self._associate(weak, unmatched, now_s)          # weak leftovers are dropped
        for detection in leftover:
            # a second blob of a casualty already matched this frame (a body split by rubble, spread
            # up to 4.3 m apart by an oblique look) joins it; anything else is new
            match = self._nearest(detection[0])
            if match is None:
                self._start(detection, now_s)
            else:
                self._tracks[match.track_id] = self._merge(match, *detection, now_s)
        return self.confirmed(now_s)

    def _associate(self, detections, unmatched: set, now_s: float) -> list:
        """Match `detections` one-to-one to the `unmatched` tracks within the association radius,
        minimising total distance; merge the pairs, drop their tracks from `unmatched`, and return
        the detections left over."""
        track_ids = sorted(unmatched)
        if not detections or not track_ids:
            return list(detections)
        cost = np.array([[math.dist(self._tracks[t].position, d[0]) for t in track_ids]
                         for d in detections])
        leftover = set(range(len(detections)))
        for row, col in zip(*linear_sum_assignment(np.minimum(cost, 2 * self._radius))):
            if cost[row, col] <= self._radius:
                track = self._tracks[track_ids[col]]
                self._tracks[track.track_id] = self._merge(track, *detections[row], now_s)
                unmatched.discard(track.track_id)
                leftover.discard(row)
        return [detections[i] for i in sorted(leftover)]

    def _start(self, detection, now_s: float) -> None:
        position, confidence, peak_k, exposure, surround_k = detection
        track_id = f"V-{self._next_id:03d}"
        self._next_id += 1
        self._tracks[track_id] = Track(track_id, tuple(position), 1, confidence, peak_k,
                                       now_s, now_s, exposure, surround_k)

    def _nearest(self, position):
        candidates = [(math.dist(t.position, position), t) for t in self._tracks.values()]
        candidates = [(d, t) for d, t in candidates if d <= self._radius]
        return min(candidates, key=lambda c: c[0])[1] if candidates else None

    def _merge(self, track: Track, position, confidence: float, peak_k: float, exposure: float,
               surround_k: float, now_s: float) -> Track:
        hits = track.hits + 1
        averaged = tuple((old * track.hits + new) / hits for old, new in zip(track.position, position))
        strongest = max(confidence, self._strength(track))
        clearer = exposure > track.exposure
        return replace(track, position=averaged, hits=hits, peak_k=max(track.peak_k, peak_k),
                       confidence=self._corroborated(strongest, hits), last_seen_s=now_s,
                       exposure=max(track.exposure, exposure),
                       surround_k=surround_k if clearer else track.surround_k)

    @staticmethod
    def _corroborated(strength: float, hits: int) -> float:
        return min(MAX_CONFIDENCE, 1.0 - (1.0 - strength) * DOUBT_DECAY ** (hits - 1))

    @staticmethod
    def _strength(track: Track) -> float:
        """The single-look confidence behind a track's current value, so corroboration compounds
        from the evidence rather than from itself."""
        return 1.0 - (1.0 - track.confidence) / DOUBT_DECAY ** (track.hits - 1)
