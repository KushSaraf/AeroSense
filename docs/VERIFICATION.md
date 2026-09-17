# Verification

What was tested, how, and what came out. Every number here was produced by the code in this
repo; re-run the commands to reproduce them.

## Automated tests

```bash
source /opt/ros/humble/setup.bash && source ~/uav_ws/install/setup.bash
colcon build --base-paths src && source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest legacy/prototype/tests src -q    # 182 passed
cd frontend && npx tsc -b && npm run build                           # type-check and build
```

| Test folder | Tests | Covers |
|---|---|---|
| `legacy/prototype/tests` | 32 | the single-process prototype: projection, risk, A*, store-and-forward |
| `src/aero_sense_bridge/test` | 29 | ROS → dashboard JSON, alerts and reports derived only from real data and scoped to their mission, NaN handling, link freshness, spawned-process environment |
| `src/aero_sense_bringup/test` | 20 | world files, model paths, stopping every simulation process |
| `src/aero_sense_description/test` | 17 | the drone model and its sensor payload |
| `src/aero_sense_mission/test` | 22 | search pattern, measured coverage, obstacle detours round tall structures |
| `src/aero_sense_navigation/test` | 8 | LiDAR corridor obstacle detection (library, for a future LiDAR; no simulated one) |
| `src/aero_sense_perception/test` | 28 | thermal detection, geolocation, tracking, triage, structure map (read from the generated buildings) |
| `src/aero_sense_scenario_manager/test` | 22 | the casualty table and its ground truth |
| `src/aero_sense_visualization/test` | 4 | RViz markers |
| **Total** | **182** | all passing |

## Flight tests

### Real people as casualties (2026-09-17)

The casualties became posed people (`tools/make_people.py`), rubble piles of slabs and brick, and
flood casualties on a car roof, two roof terraces and at two windows. Each sector was flown by the
mission manager (`/aero_sense/mission/start`, SWOOP on), 30 m search altitude, fresh simulation each.

| Sector | Found | Position error | False positives | SWOOP | Missed |
|---|---|---|---|---|---|
| Earthquake | 8 of its 18 (was 14 with the manikin) | 0.95 m | 0 | 3 descents, 1 proven | V06, V15, V18 seated with legs under rubble; V13, V14 in collapse cracks; V21 hand; V22 feet; V16, V17 buried; V09 dead |
| Flood | 3 of its 5 | 0.31 m | 0 | 1 descent, ruled out | V20, V23 leaning out of first-floor windows, under the sunshade |

Why recall fell: the old DARPA manikin was bulky. A real person seen from above is small. Warm area
visible from overhead, from the meshes, at 30 m (0.20 m a pixel; the detector needs 4 pixels):

| Casualty | Visible | Pixels at 30 m | At 22 m |
|---|---|---|---|
| lying in the open (V05, V09, V19) | 0.4-0.5 m2 | 10-12 | 19-23 |
| legs under rubble, lying (V01, V04, V07) | 0.33 m2 | 8 | 15 |
| seated, legs under rubble (V06, V15, V18) | 0.07 m2 | 2 | 3 |
| standing on a car or roof (V10, V12) | 0.1 m2 | 2.5 | 4.6 |
| feet only (V22) / hand only (V21) | 0.05 / 0.01 m2 | 1.2 / 0.3 | 2.3 / 0.5 |

The two standing flood casualties were still found (higher and closer to the camera than the
ground). Both window casualties are hidden from above by the chajja over the window.

### Network outage (2026-09-17)

Mission M-20260917-154227 started from the dashboard bridge, earthquake sector, cruise 8 m/s, with the
bridge reading only the drone's downlink (`comms_link`).

| What | Result |
|---|---|
| Cut by hand on the pad | the ground went silent and showed OFFLINE after 3 s; `pause` was refused ("command not sent"); restore sent 1 held event |
| First outage (SWOOP inside the north-east blocks) | 77 s offline. On reconnect: "sending 3 P2 and 14 held events", each with its original time and `held on board` 14–77 s |
| Legs 3 and 4 through the zone | 16 s offline along leg 4; DEGRADED (video paused) at the edges |
| Mission | carried on unchanged: 14 casualties, 98% searched, MISSION_COMPLETE |

Found on this flight: hovering on the zone boundary flapped the link (outages of 0–5 s), so the
link now returns only 3 m outside a zone (`RECONNECT_MARGIN_M`, tested).

### The Indian society world (2026-09-17)

`tools/search_evaluation.py` flown over each sector of the generated society (seed 23: 96
buildings, winding roads and galis), 30 m altitude, 25 m leg spacing, fresh simulation for each.

| Sector | Found | Position error | False positives | Missed, and why |
|---|---|---|---|---|
| Earthquake (`--x-range -180 -20 --y-range 15 90`) | 13 of its 18 | 0.2 m | 0 | V09 dead at ambient; V16, V17 buried (the warm patch is below the detector's threshold); V21 a hand out of rubble and V22 only feet, too few pixels from 30 m |
| Flood (`--x-range 20 180 --y-range 20 80`) | 3 of its 4 | 0.1 m | 0 | V20, only a waving hand above the water |

The three casualties trapped inside collapses (V13-V15) are found through the crack in the pancaked
slabs, where the old wooden house meshes hid them completely.

### Earlier flights (Iris quadrotor, grid world)

Each flown end to end in Gazebo with ArduPilot SITL, started from the dashboard's API.

| Mission | What it showed | Result |
|---|---|---|
| M-20260911-102001 | the failure that led to obstacle avoidance: search leg 3 ran through the 44 m radio mast | drone hung disarmed on the mast; mission waited 74 min — now caught as EMERGENCY within 3 s |
| M-20260911-115500 | obstacle avoidance: routed round the mast, the industrial hall, the water tower and the fire station | 8 casualties, 92% measured coverage, landed on the pad |
| M-20260911-120231 | the dashboard's restart: simulation torn down and brought back through `POST /api/simulation/restart` | 8 casualties, 90% coverage, landed on the pad |
| M-20260911-151948 | the flight on the website, recorded with `tools/record_replay.py` | 8 casualties found, 91% of the sector searched, 406 s |

## Triage against the scenario

The scenario table (`aero_sense_scenario_manager/config/victims.yaml`) records the priority a
correct triage should reach. Perception never reads it; this compares after the flight.
Agreement on the recorded flight: **3/8**.

| Found as | Scenario | Position error | Triage | Expected | Why the engine decided |
|---|---|---|---|---|---|
| V-001 | V01 | 0.1 m | P2 | P2 | 6 m from a structure |
| V-002 | V02 | 0.2 m | P2 | P1 | 6 m from a structure |
| V-003 | V04 | 0.3 m | P2 | P1 | 6 m from a structure |
| V-004 | V05 | 0.4 m | P3 | P2 | skin +13 K over ambient and falling; 9 m from a structure |
| V-005 | V08 | 0.2 m | P2 | P1 | 2 m from a structure |
| V-006 | V07 | 0.2 m | P2 | P1 | skin +13 K over ambient and falling; 5 m from a structure |
| V-007 | V06 | 0.2 m | P2 | P2 | 0 m from a structure |
| V-008 | V03 | 0.3 m | P2 | P2 | 2 m from a structure |

The ordering is right: the casualty in open ground ranks below those against buildings. Where it
disagrees, it under-ranks: casualties the scenario marks P1 because they are pinned or waving come
out P2, because from 30 m the drone cannot see that someone is trapped or moving. Every casualty
is still found and located to within half a metre; the ranking is decision support for the
ground team, not a diagnosis.

Not found in the earthquake sector: V09 (no useful thermal contrast; only RGB shape betrays this one); V13 (inside the second terrace, under the collapsed roof); V14 (inside the market-block house, visible only where the roof has gone); V15 (sheltering inside the north terrace, conscious). Casualties fully inside intact
buildings are invisible to a downward thermal camera, and a deceased casualty at ambient
temperature has no thermal contrast.

## Screenshots

The website, playing back the recorded flight:

| | |
|---|---|
| ![Live dashboard](images/site-dashboard.png) | ![Live map](images/site-map.png) |
| Live dashboard 3:20 into the flight: six casualties, 50% measured coverage | Live map: both sectors on satellite imagery, the drone and the triaged casualties |
| ![Alerts](images/site-alerts.png) | ![Reports](images/site-reports.png) |
| Alert center, raised only by real detections | Printable A4 mission report |
| ![Mission command](images/site-mission-command.png) | |
| Mission command: the two sectors | |

The drone's own cameras during the recorded flight:

| RGB | Thermal (LWIR) |
|---|---|
| ![RGB at 39 s](images/drone-rgb_0039.jpg) | ![Thermal at 39 s](images/drone-thermal_0039.jpg) |
| 39 s: first casualty in the lane between collapsed terraces | the same moment in thermal |
| ![RGB at 250 s](images/drone-rgb_0250.jpg) | ![Thermal at 158 s](images/drone-thermal_0158.jpg) |
| 250 s: over the north-east block | 158 s: a warm body beside the radio mast the route goes round |
