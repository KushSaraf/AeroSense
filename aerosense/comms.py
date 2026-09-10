"""Store-and-forward downlink: while the link is down the drone keeps flying and
queues its reports; on reconnect the queue is flushed to the ground in order."""
import threading

#: Message types where only the newest one is worth sending after an outage.
COALESCED = frozenset({"telemetry", "map", "frame"})


def _enqueue(buffer: tuple, msg: dict) -> tuple:
    if msg["type"] in COALESCED:
        buffer = tuple(m for m in buffer if m["type"] != msg["type"])
    return buffer + (msg,)


class StoreAndForwardLink:
    def __init__(self, deliver, outages=()):
        """deliver(msg) hands a message to the ground station; outages are (start_s, end_s)."""
        self._deliver = deliver
        self._outages = tuple(outages)
        self._forced_down = False
        # ponytail: unbounded queue; cap it (drop oldest events) if outages get long.
        self._buffer = ()
        self._lock = threading.Lock()

    def set_forced_down(self, is_down: bool) -> None:
        self._forced_down = is_down

    def is_up(self, t: float) -> bool:
        return not self._forced_down and not any(a <= t < b for a, b in self._outages)

    @property
    def buffered(self) -> int:
        return len(self._buffer)

    def send(self, msg: dict, t: float) -> bool:
        with self._lock:
            if not self.is_up(t):
                self._buffer = _enqueue(self._buffer, msg)
                return False
            pending, self._buffer = self._buffer, ()
            for m in pending + (msg,):
                self._deliver(m)
            return True
