# Aero Sense — search-and-rescue drone simulation

Autonomous SAR drone in a Gazebo disaster zone, flown by ArduPilot SITL, with the onboard
AI stack (detection, thermal fusion, mapping, risk engine, safe-route planning,
store-and-forward comms) and a live command-centre dashboard.

```
Gazebo Harmonic ──(rgbd + thermal)──> ros_gz_bridge ──ROS 2──> aerosense (companion computer)
      ▲                                                            │  YOLO11n + ByteTrack, LWIR,
      │ FDM (JSON)                                                 │  SegFormer-B0, 2.5-D map,
ArduPilot SITL (EKF3) <──────────── MAVLink (GUIDED) ──────────────┤  risk engine, A* routes
                                                                   ▼
                                           store-and-forward downlink ──> dashboard :8080
```

## ROS 2 workspace

The system is being rebuilt as ROS 2 packages under `src/` (see `docs/ARCHITECTURE.md`). Landed
so far: interfaces and `system_check`; Gazebo + ArduPilot SITL flown from ROS; the sensor payload
(RGB, depth, LWIR, LiDAR, IMU, barometer) with quality profiles; the disaster world — dense
earthquake blocks, a flooded village and a landslide ridge; casualties with real body heat;
thermal perception that finds them; and the RViz view.

The single-process prototype below still works (`./run_sim.sh`) until its pieces are migrated
into the packages.

## Running the full system

One command, in tmux. Each first destroys everything a previous run left behind (the tmux
session, Gazebo, SITL, MAVProxy, the ROS nodes, RViz, the bridge on :8000, the frontend on
:5173), so nothing stale holds a port or publishes old topics:

```bash
tools/dashboard.sh        # web dashboard: bridge + frontend + simulation, browser opens on it
tools/demo.sh             # no dashboard: simulation + RViz, then a scored search
tools/dashboard.sh --stop # stop everything (either script)
tmux attach -t aerosense  # see the logs
```

From the dashboard, Mission Command starts the earthquake or flood sector, and the live
dashboard's GAZEBO, RVIZ and RESTART SIM buttons act on the running simulation.

By hand:

```bash
source /opt/ros/humble/setup.bash && source ~/uav_ws/install/setup.bash
cd ~/sih_2026 && colcon build --base-paths src && source install/setup.bash

ros2 launch aero_sense_bringup full_system.launch.py     # world + drone + perception + RViz
```

Arguments: `gui:=false` (no Gazebo window), `rviz:=false`, `quality:=low|medium|high`,
`victims:=false`, `world:=…`, `namespace:=drone_01`.

**Re-source after pulling or rebuilding.** `ros2 launch` resolves packages from the environment
as it was sourced: a terminal sourced before a package existed will start everything except that
package's nodes, which looks like a partly-working simulation.

The drone spawns on the command-base pad at **(0, −110)**, 110 m south of the world origin where
the Gazebo camera starts — it is off-screen until you move the view, and it does not move until
commanded:

```bash
ros2 service call /aero_sense/drone/takeoff std_srvs/srv/Trigger
python3 tools/search_evaluation.py       # fly a search and score it against ground truth
python3 tools/camera_snapshot.py --at 55 8 35 --look-at 60 40 --tag flood
ros2 run aero_sense_bringup system_check
ros2 run aero_sense_bringup stop_sim     # stop everything, including what outlives Ctrl-C
```

Only one simulation can run at a time: SITL needs port 5760, and a second launch now fails with
that message instead of quietly starting a world whose drone no autopilot flies.

### What RViz shows

`full_system.launch.py` opens RViz on `aero_sense_visualization/config/aero_sense.rviz`:
the drone's axes and TF tree, **red spheres for casualties the drone found** (labelled with id,
priority and confidence), **green translucent spheres for ground truth** (evaluation only —
switch the layer off to watch the search honestly), the LiDAR cloud, and the RGB and thermal
camera streams. The view starts over the earthquake sector; the base is to the south.

## Reference assets

The disaster world is built from open GitHub model repos, used in place (not vendored, so
`reference/` is gitignored). Clone or unpack them into `reference/`, or point
`AERO_SENSE_REFERENCE` at another directory:

| Directory in `reference/` | Used for |
|---|---|
| `tdf_gazebo-main` | collapsed houses / industrial / fire & police stations, vehicles, trees, radio mast, water tower |
| `gazebo_models_worlds_collection-master` | debris meshes, broken brick walls (wrapped in `aero_sense_gazebo/models` with PBR materials) |
| `Autonomous-robot-for-fire-detection-main` | `suv` textures used by the tdf bus; fire model (Phase 14) |
| `darpa_subt_worlds-main` | jersey barriers; survivor and tunnel models for later phases |

`ros2 run aero_sense_bringup system_check` reports them missing. Their meshes name textures by
bare filename, so each model's `materials/textures` goes on `GZ_SIM_RESOURCE_PATH`
(`aero_sense_bringup/worlds.py`).

## Quick start (prototype)

Needs the existing `~/uav_ws` (ArduPilot SITL, `ardupilot_gazebo`, `ros_gz`) and ROS 2 Humble.

```bash
pip install --user -r requirements.txt       # first run also downloads YOLO11n + SegFormer-B0
./run_sim.sh                                 # Gazebo GUI + SITL + bridge + mission
# open http://127.0.0.1:8080
```

Options: `./run_sim.sh --headless` (no Gazebo GUI), `--no-mission` (sim only), and mission args
after `--`, e.g. `./run_sim.sh -- --outage 30:60 --outage 100:120` or `-- --no-outage`.
Logs go to `logs/`. Ctrl-C stops everything.

## The mission

1. Pre-flight: wait for camera streams and EKF/pre-arm, arm, take off to 15 m.
2. Lawnmower search of a 50 × 48 m box (5 lanes, 12 m apart) at 4 m/s.
3. Every frame: detect people (YOLO11n + ByteTrack) and fuse with LWIR body heat; find fire
   (hot) and flood (SegFormer water ∪ LWIR-cold); geo-locate via depth + pose; update the map.
4. Every 3 s: rank each confirmed survivor LOW→CRITICAL and plan a safe ground route from base.
5. A scripted comm blackout (default T+45–85 s) — the drone keeps searching, queues its
   reports on board and flushes them in order when the link returns. The dashboard's
   **Simulate comm loss** button does the same on demand.
6. Depth-based avoidance: a 25 m mast stands on lane 3, above search altitude; the drone
   stops, climbs over it and descends again.
7. RTL and land.

## Spec → simulation

| Spec component | Here | Notes |
|---|---|---|
| RGB + stereo depth | Gazebo `rgbd_camera` 640×480 | 55° forward-down |
| LWIR thermal | Gazebo `thermal` camera, radiometric L16 | people ~310 K, fire 520–550 K, water 285 K |
| IMU/GPS + EKF fusion | ArduPilot SITL EKF3 | real autopilot firmware |
| PX4 flight controller | **ArduPilot** (what `~/uav_ws` has) | same MAVLink interface |
| YOLO11-N | ultralytics `yolo11n.pt`, class person | real model |
| ByteTrack | ultralytics built-in tracker | real |
| SegFormer-B0 | `nvidia/segformer-b0-finetuned-ade-512-512` water classes | runs, but labels flat untextured sim ground as "sky"; LWIR-cold cue carries flood in sim |
| Depth Anything V2-S | not run | sim gives metric depth; drop-in for RGB-only hardware |
| ORB-SLAM3 / VIO | not run | pose comes from EKF3 (GPS); see gaps |
| RTAB-Map 3-D map | 2.5-D height grid (1 m cells) from depth | enough for routing + avoidance |
| A* / RRT* | A* (8-connected, fire clearance, flood cost) | RRT* not needed on a grid |
| Risk engine | `aerosense/risk.py` | P(survivor) × (base + hazard + inaccessibility) |
| ROS 2 + MAVLink | ROS 2 for sensors, pymavlink for flight | |
| Store-and-forward | `aerosense/comms.py` | in-process link model |
| Dashboard | Flask + canvas, `/api/state`, MJPEG feed | |

## Scenario ground truth (local NED metres, origin = launch)

| | N | E | Context |
|---|---|---|---|
| S1 Rescue Randy | 47 | -10 | standing in flood water |
| S2 Rescue Randy (sitting) | 33 | 8 | 3.5 m from fire |
| S3 Standing person | 15 | -18 | open ground, no heat-signature model |
| S4 Walking person | 42 | 14 | beside collapsed building |
| S5 Rescue Randy (sitting) | 24 | 20 | open ground |

## Code map

| File | Role |
|---|---|
| `aerosense/mission.py` | entry point + main loop (sense → control → report) |
| `aerosense/perception.py` | YOLO11n/ByteTrack, thermal fusion, SegFormer, annotated feed |
| `aerosense/world_model.py` | height map, hazard evidence grids, survivor registry |
| `aerosense/risk.py` | risk scoring and levels |
| `aerosense/planner.py` | cost grid + A* safe routes |
| `aerosense/comms.py` | store-and-forward link |
| `aerosense/flight.py` | ArduCopter GUIDED client (pymavlink) |
| `aerosense/sensors.py` | ROS 2 camera subscriber (no cv_bridge — NumPy 2) |
| `aerosense/geo.py` | pixel + depth → NED |
| `aerosense/dashboard.py`, `static/index.html` | command centre |
| `src/aero_sense_gazebo/worlds/prototype_disaster.sdf`, `src/aero_sense_description/models/aerosense_drone_prototype` | Gazebo world + drone (run_sim.sh spawns it) |

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
```

Covers projection, risk, A*, store-and-forward, world model, mission logic (lawnmower,
avoidance) and the dashboard API. The perception/flight path is verified against the live sim.

## Firmware notes

`~/uav_ws` builds **ArduCopter 4.8.0-dev**, which renamed the waypoint parameters to SI
units: `WP_SPD` (m/s), `WP_ACC` (m/s²), `RTL_ALT_M` (m) — the old `WPNAV_SPEED`,
`WPNAV_ACCEL`, `RTL_ALT` no longer exist. `Flight.set_param` waits for the autopilot's
echo and raises if a name is unknown, because ArduPilot otherwise drops it silently.

## Known gaps

- **GPS-denied flight** is not simulated yet. Path: ArduPilot optical-flow/visual-odometry
  sources (`EK3_SRC*`) or ORB-SLAM3 feeding `VISION_POSITION_ESTIMATE`, switched on GPS loss.
- Frames are projected with the pose interpolated at their capture time: frame stamps are
  Gazebo sim time, MAVLink stamps are SITL boot time, and the (constant, lockstep) offset
  is estimated online from `/clock`. On real hardware this is camera–IMU hardware sync.
  Hovering level, mapped ground heights are within ±0.2 m out to the 30 m map range.
- SegFormer needs realistic textures (or a flood-trained checkpoint) to contribute in sim.
