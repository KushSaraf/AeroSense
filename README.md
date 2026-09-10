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

## Quick start

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
| `sim/worlds/disaster.sdf`, `sim/models/aerosense_drone` | Gazebo world + drone |

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
```

Covers projection, risk, A*, store-and-forward, world model, mission logic (lawnmower,
avoidance) and the dashboard API. The perception/flight path is verified against the live sim.

## Known gaps

- **GPS-denied flight** is not simulated yet. Path: ArduPilot optical-flow/visual-odometry
  sources (`EK3_SRC*`) or ORB-SLAM3 feeding `VISION_POSITION_ESTIMATE`, switched on GPS loss.
- Frames are projected with the pose interpolated at their capture time: frame stamps are
  Gazebo sim time, MAVLink stamps are SITL boot time, and the (constant, lockstep) offset
  is estimated online from `/clock`. On real hardware this is camera–IMU hardware sync.
  Hovering level, mapped ground heights are within ±0.2 m out to the 30 m map range.
- SegFormer needs realistic textures (or a flood-trained checkpoint) to contribute in sim.
