# Aero Sense — Architecture & Implementation Plan

A real end-to-end SAR simulation: Gazebo physics and sensors → ROS 2 → onboard autonomy →
MAVLink → a real autopilot flying the simulated airframe → FastAPI/WebSocket → dashboard.
Nothing on the dashboard is produced anywhere except by the ROS nodes below.

## Platform decisions (and why they differ from the brief)

| Brief | Built on | Reason |
|---|---|---|
| ROS 2 Jazzy | **ROS 2 Humble** | Jazzy needs Ubuntu 24.04; this machine is 22.04 with Humble installed |
| PX4 SITL | **ArduPilot SITL (ArduCopter 4.8-dev)** | Installed in `~/uav_ws` and already flown; owner's choice. Autopilot access is isolated in `aero_sense_mission/autopilot.py` (MAVLink only), so PX4 is a contained swap |
| Gazebo Harmonic | Gazebo Harmonic 8 | as briefed |
| RTAB-Map / OctoMap / PCL / robot_localization | built-in mappers first, apt packages when installed | `sudo` needs a password; see *Dependencies*. Topic contracts are the same either way |
| ORB-SLAM3 | **VIO abstraction**: RGB-D visual odometry (OpenCV features + depth, PnP) fused with IMU | ORB-SLAM3 has no Humble binary and needs Pangolin; the `aero_sense_localization` interface is the swap point |
| YOLO11-N / SegFormer-B0 / Depth Anything V2-S | ground-truth-assisted perception **with camera projection, occlusion, noise and false positives**, behind the exact detector interfaces; real YOLO11-N + thermal path kept as a selectable backend | brief §19 |

GPS-denied flight is **real**: GPS-loss zones degrade ArduPilot's simulated receiver
(`SIM_GPS1_ACC/NOISE/NUMSATS/JAM`, then `SIM_GPS1_TYPE=0`), and the VIO pose is fed to the
EKF as `VISION_POSITION_ESTIMATE` (`VISO_TYPE=1`), with the EKF source set switched over
MAVLink. The chemical sensor is Gazebo's `environmental_sensor`, sampling a concentration
field loaded from CSV (generated from `hazards.yaml`).

## Architecture tree

```
Gazebo Harmonic (disaster world, 5 sectors)
 ├─ physics, ArduPilotPlugin ◄──JSON FDM──► ArduPilot SITL (EKF3) ◄─MAVLink─┐
 ├─ sensors: RGB, depth, thermal, LiDAR, IMU, NavSat, baro, chemical        │
 └─ ground-truth poses (perception aid + evaluation only)                   │
        │ ros_gz_bridge                                                     │
        ▼                                                                   │
 ROS 2 bus  (/aero_sense/…, namespaced per drone)                           │
  ├─ aero_sense_perception   detectors → tracker (ByteTrack-style) → fusion │
  │                          → geolocation → victims; masks → polygons      │
  ├─ aero_sense_localization GPS monitor, VIO, source arbitration ──────────┤
  ├─ aero_sense_mapping      point cloud → occupancy / 3-D / semantic / cov │
  ├─ aero_sense_navigation   search planner, local planner, safe routes     │
  ├─ aero_sense_triage       P1/P2/P3 with rationale                        │
  ├─ aero_sense_mission      state machine + autopilot adapter ─────────────┘
  ├─ aero_sense_alerts       alert engine
  ├─ aero_sense_comms        link model + store-and-forward (P1 first)
  ├─ aero_sense_reports      logger + JSON/CSV/HTML/PDF reports, rosbag2
  ├─ aero_sense_scenario_manager  sector loading, dynamic hazards, failures
  ├─ aero_sense_visualization     RViz markers/overlays
  └─ aero_sense_bridge       FastAPI + WebSocket → dashboard
```

## Packages

| Package | Kind | Phase | Contents |
|---|---|---|---|
| `aero_sense_interfaces` | ament_cmake | 1 | all msgs/srvs |
| `aero_sense_bringup` | ament_python | 1 → | launch files, `config/*.yaml`, `system_check` |
| `aero_sense_description` | ament_python | 2, 3 | `config/sensors.yaml` + `render.py` → drone SDF, bridge config, sensor TFs; prototype drone |
| `aero_sense_gazebo` | ament_python | 2, 4, 13–16 | world, sector models, bridge config |
| `aero_sense_perception` | ament_python | 6 | detectors, tracker, fusion, geolocation, polygons |
| `aero_sense_localization` | ament_python | 7, 17 | GPS monitor, VIO, source arbitration |
| `aero_sense_mapping` | ament_python | 8 | occupancy, point cloud, semantic, coverage |
| `aero_sense_navigation` | ament_python | 9, 12, 26 | local planner, search planner, A*/D* Lite routes |
| `aero_sense_triage` | ament_python | 10 | triage engine |
| `aero_sense_mission` | ament_python | 2, 36 | state machine, autopilot adapter, battery |
| `aero_sense_alerts` | ament_python | 19 | alert engine |
| `aero_sense_comms` | ament_python | 18 | link states, store-and-forward |
| `aero_sense_reports` | ament_python | 20 | logger, reports, replay |
| `aero_sense_scenario_manager` | ament_python | 13–16, 40, 41 | scenarios, failure injection, dynamic hazards |
| `aero_sense_visualization` | ament_python | 21 | RViz config, markers, status overlay |
| `aero_sense_bridge` | ament_python | 22 | FastAPI/WebSocket, JSON contracts |

A package is created in the phase that first needs it — no empty skeletons. The earlier
single-process prototype in `aerosense/` is migrated into these packages phase by phase
(its geo, risk, A*, world-model, store-and-forward and flight code is reused) and removed
once superseded.

## Frames

`map` (ENU, origin = command base, the world origin) → `odom` → `base_link` →
`camera_link` → `camera_optical` (RGB, depth and thermal share one mount, tilt and HFOV);
`base_link` → `lidar_link`, `imu_link`, `baro_link`. Sensor frames are static TFs rendered from
the same table as the model (below); a namespaced drone prefixes them (`drone_01/base_link`).
Lat/lon come from the world's `spherical_coordinates` (WGS84) through one conversion module.

## Topic architecture

All names are relative (`aero_sense/…`) so a drone launched under namespace `drone_01`
publishes `/drone_01/aero_sense/…`; the single-drone default namespace is empty.

| Topic | Type | Producer |
|---|---|---|
| `aero_sense/camera/{rgb,depth,thermal}/image_raw`, `…/camera_info` | Image (rgb8 / 32FC1 / mono16 0.01 K), CameraInfo | Gazebo |
| `aero_sense/lidar/points` | sensor_msgs/PointCloud2 | Gazebo |
| `aero_sense/imu`, `aero_sense/baro` | Imu, FluidPressure | Gazebo (companion IMU, barometer) |
| `aero_sense/gps/fix` | NavSatFix | autopilot GPS (degradable, Phase 17) |
| `aero_sense/drone/pose`, `/velocity`, `/battery`, `/status` | PoseStamped, TwistStamped, BatteryState, DroneStatus | mission (from MAVLink) |
| `aero_sense/localization/pose`, `/status` | PoseStamped, LocalizationStatus | localization |
| `aero_sense/perception/detections` | vision_msgs-style raw detections | perception |
| `aero_sense/victims` | VictimArray | perception + triage |
| `aero_sense/hazards` | HazardArray | perception |
| `aero_sense/map/occupancy`, `/pointcloud`, `/semantic`, `/coverage` | OccupancyGrid, PointCloud2, OccupancyGrid, OccupancyGrid | mapping |
| `aero_sense/navigation/path`, `/obstacles` | Path, MarkerArray | navigation |
| `aero_sense/routes/safe` | SafeRoute (one per victim) | navigation |
| `aero_sense/alerts` | Alert | alerts |
| `aero_sense/mission/state`, `/events` | MissionStatus, String | mission |
| `aero_sense/communication/status` | CommunicationStatus | comms |

### Sensor payload (Phase 3)

`aero_sense_description/config/sensors.yaml` is the single source for the payload:
`render.py` turns it into the drone SDF, the ros_gz_bridge config and the static TFs, and
`simulation.launch.py quality:=low|medium|high` picks the resolution/rate profile. Measured
on the medium profile (RTF 1.0, headless, RTX 2050):

| Sensor | Medium profile | Measured rate | Noise (configured → measured) |
|---|---|---|---|
| RGB | 960×540 | 9.1 Hz / 10 | σ 0.007 of full scale |
| Depth | 640×480 | 4.8 Hz / 5 | σ 0.02 m → 0.018 m (18.50 m vs 18.52 m geometric at 15 m AGL) |
| Thermal (LWIR) | 320×256 mono16 | 8.7 Hz / 9 | none — gz-sensors 8 segfaults on thermal `<noise>`; 0.01 K quantisation only |
| LiDAR | 16 × 900, ±15°, 0.5–100 m | 9.7 Hz / 10 | σ 0.01 m; no self-hits in flight |
| IMU (companion) | — | 97 Hz / 100 | gyro σ 0.0009 → 0.00091 rad/s, accel σ 0.017 → 0.0168 m/s² |
| Barometer | — | 9.7 Hz / 10 | σ 5 Pa → 5.2 Pa |

The flight IMU inside the iris model stays noise-free: ArduPilot SITL consumes it and adds
its own sensor model. GPS is the autopilot's (`SIM_GPS1_*`), so GPS denial acts on what the
EKF actually uses.

Services: `aero_sense/mission/start` (StartMission), `…/pause`, `…/resume`, `…/abort`,
`…/return_to_base` (std_srvs/Trigger), `…/set_search_area` (SetSearchArea),
`…/set_priority` (SetPriority), `aero_sense/sim/inject_failure` (InjectFailure).

## Launch architecture

```
full_system.launch.py
 ├─ simulation.launch.py      gz sim (world + quality profile), SITL, ros_gz_bridge,
 │                            all onboard nodes, recorder
 ├─ visualization.launch.py   RViz2 (grouped displays), rqt_image_view (optional)
 └─ dashboard_bridge.launch.py FastAPI/WebSocket bridge
replay.launch.py              rosbag2 play + RViz
```

## World structure (one world, five sectors, ~400 × 300 m)

```
            N
 ┌──────────────────────────────┐   y +200
 │ S4 LANDSLIDE (heightmap)      │
 ├───────────────┬──────────────┤   y +100
 │ S1 EARTHQUAKE │ S2 FLOOD      │
 ├───────────────┼──────────────┤   y 0     ◄ command base at (0, -110)
 │ S3 FIRE       │ S5 CHEMICAL   │
 └───────────────┴──────────────┘   y -100
   x -200          0            x +200
```

World file: `aero_sense_gazebo/worlds/aero_sense_disaster.sdf`. Roads connect every sector
through the central intersection. Ground and roads are tiled-UV OBJ quads
(`aero_sense_ground`, `aero_sense_roads`) with reference textures (tdf dirt; DARPA SubT
Asphalt01 albedo/normal/roughness) and LWIR temperatures for daytime contrast: asphalt 305 K,
pad 303 K, soil 298 K, everything else ambient 293 K. Drones spawn on the command-base pad (`<frame
name="drone_spawn">`, read by `aero_sense_bringup/worlds.py`). SITL's home is the world's
`spherical_coordinates`, so ArduPilot's local NED origin is the world origin and `map` =
Gazebo world frame (verified: ROS pose (0.00, −109.99) vs Gazebo (0.00, −110.00) at the pad).

**S1 earthquake (Phase 4):** four blocks around streets x = −100 and y = 50 — collapsed
houses ×4, collapsed industrial (24 m), fire and police stations, rubble spreads, broken brick
walls, crashed bus/pickups/hatchbacks, trees, a police cordon of jersey barriers, and two tall
obstacles above the search altitude: a 44 m radio mast at (−40, 60) and a 10 m water tower.

Assets come from `reference/` (gitignored, used in place; see README "Reference assets"):
tdf_gazebo, the Gazebo model collection, the fire-detection repo and DARPA SubT. Their meshes
name textures by bare filename, so each model's `materials/textures` is on
`GZ_SIM_RESOURCE_PATH`. Harmonic ignores Classic OGRE-script materials; the collection's
debris meshes are wrapped in `aero_sense_gazebo/models` with plain colours, because an SDF
albedo map on those meshes aborts the gz 8 thermal camera (see the regression test in
`aero_sense_description/test/test_render.py`) and their malformed submeshes crash DART's mesh
collider (so the wrappers are visual-only, all under 0.4 m).

## Dependencies

Installed: ROS 2 Humble (rclpy, sensor_msgs, nav2_msgs/map_server, rosbag2, rviz2,
rqt_image_view), Gazebo Harmonic 8 + ros_gz (built in `~/uav_ws`), ArduPilot SITL,
pymavlink, OpenCV, numpy/scipy, shapely, torch + ultralytics + transformers, FastAPI,
uvicorn, websockets, reportlab, matplotlib, jinja2.

Needs `sudo` (run once when ready; the system works without them, using built-in mappers):

```bash
sudo apt install ros-humble-rtabmap-ros ros-humble-octomap-server ros-humble-pcl-ros \
                 ros-humble-robot-localization ros-humble-vision-msgs
```

## Phased plan

Each phase ends with: build → launch → test → inspect topics → fix → document → commit.

| # | Phase | Done when |
|---|---|---|
| 1 | Repository + ROS workspace | interfaces build; message round-trip tests pass; `system_check` runs |
| 2 | ArduPilot + Gazebo drone | `simulation.launch.py` spawns the drone; takeoff + hover by MAVLink from a ROS node |
| 3 | Sensors | RGB/thermal/depth/LiDAR/IMU/GPS on `/aero_sense/*` at configured rates, with noise |
| 4 | Earthquake sector | sector renders, drone flies it, GT victims placed with line of sight |
| 5 | Victim simulation | `victims.yaml`-driven actors: lying/waving/walking, thermal, occlusion |
| 6 | Perception | detections → persistent IDs → fused confidence → lat/lon on `/aero_sense/victims` |
| 7 | Localization | EKF pose + VIO pose published; VIO drift measured against GT |
| 8 | Mapping | occupancy, point cloud, semantic, coverage live in RViz |
| 9 | Obstacle avoidance | local planner clears rubble/poles/mast in closed loop |
| 10 | Triage | P1/P2/P3 with rationale; unit-tested |
| 11 | Hazards | masks → polygons with severity on `/aero_sense/hazards` |
| 12 | Safe routing | A*/D* Lite routes with cost classes; never cross CRITICAL |
| 13–16 | Flood, fire, landslide, chemical sectors | each with its hazards, victims, sensor effects |
| 17 | GPS-denied | GPS degrades in zones, EKF switches to VIO, flight continues |
| 18 | Communication loss | link states, offline autonomy, P1→P2→P3→map→media sync |
| 19 | Alerts | every event type on `/aero_sense/alerts` with rationale |
| 20 | Reports | JSON/CSV/HTML/PDF from recorded data; rosbag2 recording |
| 21 | RViz | grouped config, status overlay |
| 22 | Dashboard bridge | FastAPI/WebSocket, JSON contracts, dashboard consumes it |
| 23 | Full demo | `ros2 launch aero_sense_bringup full_system.launch.py` runs the scripted mission |
