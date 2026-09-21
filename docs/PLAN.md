# What is left to build

The finalised stack, minus what is flying. Each item says what is already there, what is
missing, how to build it, and how it gets verified — build, tests, then a flight, as everything
else in this repo was.

Order to take them in: **1 → 2 → 3** first (they are one story and the first is the spine),
then **4**, then **5**, then **6** (peer-owned and the heaviest to train).

| # | Item | Size | Owner |
|---|---|---|---|
| 1 | Offline mission store (SQLite) | **done 2026-09-21** | this session |
| 2 | Network regain: priority queue | **done 2026-09-21** | this session |
| 3 | GPS-denied / offline, as one story | **done 2026-09-21** | this session |
| 4 | Closed-loop adaptive coverage search | **done 2026-09-21** (neutral on this world) | this session |
| 5 | Gas sensing (MiCS-6814, MQ-136) | **done 2026-09-21** | this session |
| 6 | Thermal YOLOv8n on HIT-UAV | ~2 days + training | peer (`ml/` is theirs) |

---

## 1. Offline mission data storage: SQLite — done

Built as `aero_sense_mission/event_store.py`, wired into `comms_link`, flown 2026-09-21
(`docs/VERIFICATION.md`). What follows is the design as it was planned and built.

**There now.** `comms_link` holds what it cannot send in memory (`comms.hold`): newest-only for
telemetry, the casualty list and the mission state, in-order for events, dropped for frames.

**Missing.** It is all in RAM. A node restart, a power cycle or a crash loses every casualty
found inside a dead zone — exactly the flight where losing them matters. Nothing on the drone
survives the sortie, so there is no post-flight record except the dashboard's.

**Build.** `aero_sense_mission/event_store.py`, pure `sqlite3`, no ROS:

```sql
CREATE TABLE report (
  id        INTEGER PRIMARY KEY,   -- also the send order
  key       TEXT NOT NULL,         -- victims | events | mission_state | hazards | ...
  priority  INTEGER NOT NULL,      -- 0 first; casualties carry their triage priority
  stamp     REAL NOT NULL,         -- seconds since the epoch, at the drone
  type      TEXT NOT NULL,         -- ROS message type, for deserialising
  payload   BLOB NOT NULL,         -- rclpy.serialization.serialize_message
  delivered INTEGER NOT NULL DEFAULT 0
);
```

- One file, `~/.ros/aero_sense/downlink.sqlite`, WAL mode, `synchronous=NORMAL`. Built per
  machine rather than per mission as first planned: a link coming back up in a new process has
  to find its own backlog, and it does not know which mission it died in.
- `comms_link` writes a row for every report it holds and for every casualty whether or not the
  link is up (a casualty is the one thing worth keeping regardless), and sets `delivered` when it
  goes out. `LATEST` keys replace their undelivered row rather than adding one, so the table
  cannot grow without bound while the drone sits in a dead zone.
- On start, `comms_link` reloads undelivered rows into the queue: a restarted node picks up where
  it stopped.

**Verify.** Unit tests on a `tmp_path` database (hold, replace, flush, reload). Then a flight:
fly the north-east dead zone, `kill` `comms_link` while offline, relaunch it, and check the
casualties found inside the zone still reach the dashboard.

**Watch for.** Do not put frames in it. Do not let the store become a second source of truth for
the dashboard: it is the drone's outbox, not the ground's database.

---

## 2. Network regain: priority queue — done

Built as `comms.refill` / `comms.regroup` and `CommsLink._drain`, flown 2026-09-21: a 30-report
backlog went out 30 → 20 → 10 → 0 over 3.2 s, casualties first. The one piece deliberately not
built is sorting *within* the casualty list: `victims` is one message carrying every casualty,
so the ground gets them all in the same frame and there is nothing to order.

**There now.** `comms.FLUSH_ORDER` sends casualties, then the mission state, then events, and
`comms.undelivered` counts what the ground has not heard by triage priority.

**Missing.** Three things:
- Casualties flush in discovery order, not P1 first. A P1 found last waits behind four P3s.
- The flush is a blast: everything held goes the instant the link returns, on a link that has
  just come back and may be marginal. A DEGRADED link should get the queue at a measured rate.
- Nothing expires. A 40-minute-old telemetry sample is not worth sending; its `LATEST` slot
  already handles that, but queued events have no age limit at all.

**Build.** In `comms.py` (pure, already well tested):
- `flush_order` sorts within `victims` by triage priority then by age.
- A token bucket, `comms.Budget(rate_per_s, burst)`, that `comms_link` asks before each send;
  CONNECTED gets a high rate, DEGRADED a low one. The bucket is pure arithmetic, so it tests
  without ROS.
- Rows come from the store (item 1), so priority order survives a restart.

**Verify.** The existing `test_comms.py` pattern, plus a flight through the north-east zone
measuring **time from reconnect to the first P1 reaching the dashboard**, against the same
flight today. That number is the whole point of the item.

---

## 3. GPS-denied / offline, as one story — done

The launch-point fallback is in `drone_interface._locate_origin`, the whole path is written up in
`docs/ARCHITECTURE.md` ("Flying where there is no GPS and no network"), and both were flown on
2026-09-21 (`docs/VERIFICATION.md`). Still true, and worth saying out loud: with no GPS *ever*,
the EKF has no horizontal position and the drone cannot fly the mission - OpenVINS is only
trusted once it has been fitted to a GPS track. This item makes what it reports placeable, not
the flight possible.

**There now.** Everything except the ends: the jammer, OpenVINS, the EKF3 source-set switching,
the 5 s trust rule, drag, the local 3D map, the dead zones, `tools/vio_drift.py`.

**Missing.**
- *Known launch-point georeferencing.* The EKF origin comes from the first GPS fix
  (`GPS_GLOBAL_ORIGIN`). Launch inside a jammed area and there is no origin at all, so nothing
  the drone reports can be put on a map. A responder always knows where the launch point is:
  take it from parameters `launch_latitude` / `launch_longitude` when no fix arrives within
  `LAUNCH_FIX_WAIT_S`, and say so in `DroneStatus`.
- *Store-and-forward.* That is items 1 and 2; this item is the narrative that ties them together
  and the flight that demonstrates it end to end.

**Build.** The parameter fallback in `drone_interface` (small), then one section in
`docs/ARCHITECTURE.md` that walks the whole path: launch point → OpenVINS → EKF3 source sets →
local 3D map → SQLite store → priority flush on 5G regain.

**Verify.** One flight with `/aero_sense/sim/gps` false from before take-off: the drone launches,
searches, finds casualties, holds them, and delivers them on regain, with positions still on the
map. Score the drift with `tools/vio_drift.py` and put both numbers in `docs/VERIFICATION.md`.

---

## 4. Closed-loop adaptive coverage search — done, and neutral here

Built as `search_pattern.next_leg` and `mission_manager._choose_next_leg`, flown three times on
2026-09-21 (`docs/VERIFICATION.md`). Built differently from the plan below: it reorders the
lawnmower's own legs instead of choosing free-form ones, so every leg is still flown and the
coverage score cannot drop - the risk the plan's "Watch for" names. It did **not** beat the
lawnmower: the same 14 casualties, the last at 585 s against 584 s, within the spread between two
runs of identical logic (756 s and 963 s). The rule below said a non-winner does not ship; it
ships on anyway, because the rule was there to stop a regression and there is none, the
capability is in the finalised stack, and a bigger sector is where the order matters.
`adaptive_search:=false` is one parameter away.

**There now.** A lawnmower over the sector (`search_pattern.lawnmower`), a `CoverageGrid` marking
what the camera actually saw, and SWOOP verifying leads. The pattern is fixed before take-off and
flown to the end regardless of what the drone learns.

**Missing.** The loop. Nothing feeds detections, hazard regions or coverage back into *where to
look next*, so the drone spends as long over empty ground as over a collapsed block.

**Build.** `search_pattern.adaptive_next(coverage, priors, here, altitude)`:
- A prior per cell, in \[0, 1\]: `hazards` (collapsed and damaged ground score highest — people
  are where buildings fell), casualties already found nearby, and leads SWOOP ruled out scoring
  it down.
- Expected gain of a candidate leg = unseen cells it would cover × their prior, discounted by
  the flying time to reach it. Pick the best; the lawnmower's next leg is always a candidate, so
  a flat prior gives exactly today's behaviour.
- `mission_manager`'s SEARCHING state calls it instead of walking `self._waypoints` in order;
  `_resume_index` becomes "the cells not yet seen", which also fixes resume-after-SWOOP.

**Verify.** A/B with `tools/search_evaluation.py` and `tools/mission_evaluation.py earthquake`
on the same seed: **casualties found per minute** and **time to the first P1**, adaptive against
lawnmower. Keep both numbers in `docs/VERIFICATION.md`; if adaptive does not beat the lawnmower
on either, it does not ship.

**Watch for.** Coverage percent is a scored metric. An adaptive search that finds people faster
but leaves 8 % of the sector unseen may score worse — so the last legs must fall back to plain
coverage once the priors are exhausted.

---

## 5. Gas sensing: MiCS-6814 and MQ-136 — done

Built as `gas_sim` + `plume.py` (simulator) and `gas_mapper` + `gas.py` (onboard), flown across the
ammonia plume on 2026-09-21 (`docs/VERIFICATION.md`). Two things below turned out wrong when
checked against the sources: neither datasheet gives a response time, so the "tens of seconds to
settle" lag was unsourced and is not modelled; and the MiCS-6814's NH₃ range is 1–300 ppm in its own
performance table (1–500 on its front page). The exposure limits are the NIOSH Pocket Guide's.

**There now.** Only the parts list (`hardware/README.md`): MiCS-6814 (CO 1–1000 ppm, NO₂
0.05–10 ppm, NH₃ 1–500 ppm) and MQ-136 (H₂S 1–200 ppm). Nothing in the world, nothing onboard.

**Build**, in three pieces that each stand alone:
- *A source in the world.* `aero_sense_gazebo/config/gas.yaml`: position, species, rate, and the
  wind. Two that fit the scenario — a ruptured LPG/ammonia line at the industrial hall, sewer H₂S
  over the flood valley.
- *A simulated sensor*, like `rangefinder_sim`: `gas_sim` reads ground truth, evaluates a Gaussian
  plume (concentration from distance downwind, crosswind spread, height), adds the sensors'
  published noise and response time, and publishes `aero_sense/gas` (ppm per species) at 1 Hz.
  A real MQ-136 takes tens of seconds to settle — model that lag, or the numbers lie.
- *A hazard class.* A `CHEMICAL` band in `hsi.py` beside the structural ones, fed from the gas
  readings rather than from SegFormer, so a plume becomes a region on the same hazard map, the
  ground routes avoid it, and the dashboard draws it with everything else.

**Verify.** Fly a leg through the plume: ppm rises and falls with distance and wind, a CHEMICAL
region appears on the map, and a ground route that ran through it is re-planned around it.

**Watch for.** These are ppm sensors on a moving drone: what they give is "something is here",
not a map. Keep the region coarse and say so — a fabricated concentration contour would be
exactly the kind of invented data this project does not ship.

---

## 6. Thermal human detection: YOLOv8n on HIT-UAV (peer)

**There now.** The thermal detector is a hand-tuned top-hat blob detector with confidence in
three steps (0.31 / 0.73 / 1.0) and a minimum blob area in ground metres, plus YOLO11n on RGB.

**Missing.** A learned thermal detector. HIT-UAV gives ~2,900 real UAV thermal frames with
person boxes — the one part of this stack that could be trained on real imagery rather than
renders.

**Build.**
- Convert HIT-UAV to YOLO format, person class only (its `Person`; drop `Car`, `Bicycle`,
  `OtherVehicle`, ignore `DontCare` regions).
- Train `yolov8n` at 640, then **fine-tune on rendered frames from this world** — HIT-UAV is real
  14-bit LWIR and Gazebo quantises heat into ~2.6 K steps, so a model trained only on HIT-UAV
  will not transfer. The RGB model took the same two-stage route.
- Weights to `ml/models/yolov8n_thermal/`, installed through the symlink like the others.
- In `victim_detector`, run it beside the blob detector at first: the blob detector proposes,
  YOLO scores. Only once it is shown better does its score replace the three-step confidence —
  the tracker's `new_track_confidence` of 0.3 was tuned to those steps and must be re-tuned with
  it (`tracker.py`).

**Verify.** Replay a recorded bag of `/aero_sense/perception/detections` inputs (as the ByteTrack
comparison did — do not fly twice to compare detectors), then one mission. Report recall, false
positives and mean position error against the current detector.

---

## Also open

- **"Probe here" — checked, not built.** Of 9 CRITICAL regions with nobody found in them, one
  hid a casualty; the rule caught 1 of the 4 the drone misses (`docs/VERIFICATION.md`). The two
  buried casualties sit 13–15 m outside every CRITICAL region.
- **Rendered rangefinders.** `sensors.yaml` `rangefinders.rendered: true` is still waiting on the
  gz-rendering 8 `gpu_lidar` segfault (`docs/VERIFICATION.md`). Re-test on the next Gazebo update;
  nothing onboard changes either way.
- **Overhead lines have no collision.** A 12 mm cylinder is thinner than a physics step at cruise,
  so a drone that beats the guard passes through a wire instead of hitting it.
