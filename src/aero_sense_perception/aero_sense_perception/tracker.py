"""Per-frame detections into casualties with stable identity.

One warm frame is not a casualty: a track has to be seen repeatedly, from poses that differ, and
its position is averaged over those looks. Identity is what lets the rest of the system talk
about "V-003" instead of a fresh blob every 100 ms.
"""
import math
from dataclasses import dataclass, replace

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


class Tracker:
    """Nearest-neighbour association in the map frame. Immutable tracks: every update produces
    new ones, so a caller can hold a snapshot without it changing underneath."""

    def __init__(self, associate_radius_m: float, confirm_hits: int, forget_after_s: float):
        self._radius = associate_radius_m
        self._confirm_hits = confirm_hits
        self._forget_after = forget_after_s
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
        """`detections`: (position, confidence, peak_k) triples. Returns the confirmed tracks."""
        for position, confidence, peak_k in detections:
            match = self._nearest(position)
            if match is None:
                track_id = f"V-{self._next_id:03d}"
                self._next_id += 1
                self._tracks[track_id] = Track(track_id, tuple(position), 1, confidence, peak_k,
                                               now_s, now_s)
            else:
                self._tracks[match.track_id] = self._merge(match, position, confidence, peak_k, now_s)
        return self.confirmed(now_s)

    def _nearest(self, position):
        candidates = [(math.dist(t.position, position), t) for t in self._tracks.values()]
        candidates = [(d, t) for d, t in candidates if d <= self._radius]
        return min(candidates, key=lambda c: c[0])[1] if candidates else None

    def _merge(self, track: Track, position, confidence: float, peak_k: float, now_s: float) -> Track:
        hits = track.hits + 1
        averaged = tuple((old * track.hits + new) / hits for old, new in zip(track.position, position))
        strongest = max(confidence, self._strength(track))
        return replace(track, position=averaged, hits=hits, peak_k=max(track.peak_k, peak_k),
                       confidence=self._corroborated(strongest, hits), last_seen_s=now_s)

    @staticmethod
    def _corroborated(strength: float, hits: int) -> float:
        return min(MAX_CONFIDENCE, 1.0 - (1.0 - strength) * DOUBT_DECAY ** (hits - 1))

    @staticmethod
    def _strength(track: Track) -> float:
        """The single-look confidence behind a track's current value, so corroboration compounds
        from the evidence rather than from itself."""
        return 1.0 - (1.0 - track.confidence) / DOUBT_DECAY ** (track.hits - 1)
