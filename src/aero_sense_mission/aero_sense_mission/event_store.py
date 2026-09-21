"""The drone's outbox on disk: what it has found and not yet managed to tell anyone.

`comms_link` holds reports while the link is down. Held in memory they last exactly as long as
the process does, and the flight where that matters is the one where the drone is deep in a dead
zone with four casualties nobody on the ground has heard of. This is the same queue, in SQLite,
so it survives a restarted node, a killed process and a power cycle, and so the sortie leaves a
record that can be read after it.

Pure sqlite3 and bytes, no ROS: the caller serialises. Nothing here can stop the drone relaying
- every call that touches the database is allowed to fail, says so once, and the link carries on
without it (`EventStore.unreported_fault`).

    store = EventStore(Path("~/.ros/aero_sense/downlink.sqlite").expanduser())
    row = store.hold("victims", serialise(msg), "aero_sense_interfaces/msg/VictimArray",
                     priority=0, replace=True)
    ...
    store.delivered(row)
"""
import sqlite3
import time
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS report (
  id        INTEGER PRIMARY KEY,           -- also the order they were held in
  key       TEXT    NOT NULL,              -- victims | events | mission_state | ...
  priority  INTEGER NOT NULL,              -- lower goes first (comms.FLUSH_ORDER)
  stamp     REAL    NOT NULL,              -- seconds since the epoch, at the drone
  type      TEXT    NOT NULL,              -- ROS message type, or "event" for the log lines
  payload   BLOB    NOT NULL,
  delivered INTEGER NOT NULL DEFAULT 0
);
CREATE INDEX IF NOT EXISTS report_pending ON report (delivered, priority, id);
"""


class EventStore:
    """The outbox. One row per report the ground has not heard, plus what it has."""

    def __init__(self, path: Path, keep_delivered: bool = True):
        self.path = Path(path)
        self._keep = keep_delivered
        self.faults = 0                     # how many database calls have failed
        self.last_fault = ""
        self._db = None
        self._said = False                  # the first fault is logged, the rest are counted
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._db = sqlite3.connect(str(self.path), check_same_thread=False)
            self._db.execute("PRAGMA journal_mode=WAL")       # a kill -9 leaves the rows behind
            self._db.execute("PRAGMA synchronous=NORMAL")
            self._db.executescript(SCHEMA)
            self._db.commit()
        except Exception as exc:                              # no disk, no permission, no store
            self._fault(exc)

    # -- writing ------------------------------------------------------------------

    def hold(self, key: str, payload: bytes, type_name: str, priority: int,
             stamp: float = None, replace: bool = False) -> int:
        """Record a report as not yet delivered, and return its row id (0 if the store is down).

        `replace` is for the reports where only the newest matters (comms.LATEST): the undelivered
        row for that key goes first, so a drone sitting in a dead zone does not fill the disk with
        telemetry nobody will ever read.
        """
        def write(db):
            if replace:
                db.execute("DELETE FROM report WHERE key = ? AND delivered = 0", (key,))
            cursor = db.execute(
                "INSERT INTO report (key, priority, stamp, type, payload) VALUES (?, ?, ?, ?, ?)",
                (key, priority, time.time() if stamp is None else stamp, type_name, payload))
            return cursor.lastrowid
        return self._run(write, 0)

    def delivered(self, row_id: int) -> None:
        """Mark one report as having reached the ground."""
        if not row_id:
            return

        def write(db):
            if self._keep:
                db.execute("UPDATE report SET delivered = 1 WHERE id = ?", (row_id,))
            else:
                db.execute("DELETE FROM report WHERE id = ?", (row_id,))
        self._run(write, None)

    # -- reading ------------------------------------------------------------------

    def pending(self) -> tuple:
        """Everything still undelivered as (id, key, priority, stamp, type, payload), in the
        order it should be sent: priority first, then oldest.

        This is what a restarted `comms_link` picks up: the queue from before it died.
        """
        return self._run(lambda db: tuple(db.execute(
            "SELECT id, key, priority, stamp, type, payload FROM report "
            "WHERE delivered = 0 ORDER BY priority, id").fetchall()), ())

    def counts(self) -> dict:
        """{key: how many are still undelivered}, for the log line and the tests."""
        return dict(self._run(lambda db: db.execute(
            "SELECT key, COUNT(*) FROM report WHERE delivered = 0 GROUP BY key").fetchall(), ()))

    def close(self) -> None:
        if self._db is not None:
            try:
                self._db.close()
            except Exception as exc:
                self._fault(exc)
            self._db = None

    # -- plumbing -----------------------------------------------------------------

    def _run(self, work, default):
        """Run one statement batch. A store that cannot be written to must not stop the relay."""
        if self._db is None:
            return default
        try:
            result = work(self._db)
            self._db.commit()
            return result
        except Exception as exc:
            self._fault(exc)
            return default

    def _fault(self, exc: Exception) -> None:
        self.faults += 1
        self.last_fault = f"{type(exc).__name__}: {exc}"

    @property
    def unreported_fault(self) -> str:
        """The first fault's message, once: the caller logs it, later ones only count."""
        if self.faults and not self._said:
            self._said = True
            return self.last_fault
        return ""
