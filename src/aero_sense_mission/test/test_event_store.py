"""The drone's outbox on disk: what survives a link, a process and a power cut."""
import time

from aero_sense_mission import comms
from aero_sense_mission.event_store import EventStore


def test_what_the_ground_has_not_heard_comes_back_after_a_restart(tmp_path):
    """The reason this exists: a node killed in a dead zone must not lose the casualties."""
    path = tmp_path / "downlink.sqlite"
    store = EventStore(path)
    victims = store.hold("victims", b"two casualties", "VictimArray", comms.priority("victims"))
    store.hold("events", b"found V-003", "event", comms.priority("events"))
    store.close()

    resumed = EventStore(path)                      # the new process
    pending = resumed.pending()
    assert [row[1] for row in pending] == ["victims", "events"]     # casualties first
    assert pending[0][0] == victims and pending[0][5] == b"two casualties"

    resumed.delivered(victims)
    assert [row[1] for row in resumed.pending()] == ["events"]
    assert resumed.counts() == {"events": 1}


def test_only_the_newest_of_a_latest_report_is_kept(tmp_path):
    """Telemetry while the drone sits in a dead zone would otherwise fill the disk with samples
    nobody will ever read."""
    store = EventStore(tmp_path / "downlink.sqlite")
    for sample in range(5):
        store.hold("pose", f"at {sample}".encode(), "PoseStamped", comms.priority("pose"), replace=True)
    store.hold("events", b"first", "event", comms.priority("events"))
    store.hold("events", b"second", "event", comms.priority("events"))

    assert store.counts() == {"pose": 1, "events": 2}               # every event, one pose
    assert [row[5] for row in store.pending() if row[1] == "pose"] == [b"at 4"]


def test_a_delivered_report_stays_as_the_flights_record(tmp_path):
    store = EventStore(tmp_path / "downlink.sqlite")
    row = store.hold("victims", b"V-001", "VictimArray", 0, stamp=1789939117.39)
    store.delivered(row)

    assert store.pending() == ()
    kept = store._db.execute("SELECT key, stamp, delivered FROM report").fetchall()
    assert kept == [("victims", 1789939117.39, 1)]


def test_a_store_that_cannot_be_written_does_not_stop_the_link(tmp_path):
    """Relaying matters more than recording: a broken outbox says so once and is ignored."""
    unwritable = tmp_path / "file"
    unwritable.write_text("not a database")
    store = EventStore(unwritable / "downlink.sqlite")              # a file, not a directory

    assert store.hold("victims", b"V-001", "VictimArray", 0) == 0
    assert store.pending() == () and store.counts() == {}
    assert store.faults and store.unreported_fault                  # said once
    assert not store.unreported_fault                               # and only once


def test_reports_come_back_in_the_order_they_should_be_sent(tmp_path):
    store = EventStore(tmp_path / "downlink.sqlite")
    now = time.time()
    store.hold("events", b"early", "event", comms.priority("events"), stamp=now)
    store.hold("mission_state", b"searching", "MissionStatus", comms.priority("mission_state"), stamp=now)
    store.hold("victims", b"V-002", "VictimArray", comms.priority("victims"), stamp=now)
    store.hold("events", b"late", "event", comms.priority("events"), stamp=now)

    assert [(row[1], row[5]) for row in store.pending()] == [
        ("victims", b"V-002"), ("mission_state", b"searching"), ("events", b"early"), ("events", b"late")]
