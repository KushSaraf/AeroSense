# Verification

What was tested, how, and what came out. Every number here was produced by the code in this
repo; re-run the commands to reproduce them.

## Automated tests

```bash
source /opt/ros/humble/setup.bash && source ~/uav_ws/install/setup.bash
colcon build --base-paths src && source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest legacy/prototype/tests src -q    # 160 passed
cd frontend && npx tsc -b && npm run build                           # type-check and build
```

| Test folder | Tests | Covers |
|---|---|---|
| `legacy/prototype/tests` | 32 | the single-process prototype: projection, risk, A*, store-and-forward |
| `src/aero_sense_bridge/test` | 29 | ROS → dashboard JSON, alerts and reports derived only from real data and scoped to their mission, NaN handling, link freshness, spawned-process environment |
| `src/aero_sense_bringup/test` | 21 | world files, model paths, stopping every simulation process |
| `src/aero_sense_description/test` | 10 | the drone model and its sensor payload |
| `src/aero_sense_mission/test` | 22 | search pattern, measured coverage, obstacle detours round tall structures |
| `src/aero_sense_navigation/test` | 8 | LiDAR corridor obstacle detection |
| `src/aero_sense_perception/test` | 27 | thermal detection, geolocation, tracking, triage, structure map |
| `src/aero_sense_scenario_manager/test` | 10 | the casualty table and its ground truth |
| `src/aero_sense_visualization/test` | 4 | RViz markers |
| **Total** | **163** | all passing |

## Flight tests

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
