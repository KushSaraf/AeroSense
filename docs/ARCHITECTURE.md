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
 ├─ sensors: RGB, depth, thermal, IMU, NavSat, baro, chemical               │
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
| `aero_sense_description` | ament_python | 2, 3 | `config/sensors.yaml` + `render.py` → hexacopter SDF, bridge config, sensor TFs; `config/hexa.parm` SITL frame |
| `aero_sense_gazebo` | ament_python | 2, 4, 13–16 | world, sector models, bridge config |
| `aero_sense_perception` | ament_python | 6 | thermal detector, geolocation, tracker; RGB/YOLO and hazard polygons later |
| `aero_sense_localization` | ament_python | 7, 17 | GPS monitor, VIO, source arbitration |
| `aero_sense_mapping` | ament_python | 8 | occupancy, point cloud, semantic, coverage |
| `aero_sense_navigation` | ament_python | 9, 12, 26 | local planner, search planner, A*/D* Lite routes |
| `aero_sense_triage` | ament_python | 10 | triage engine |
| `aero_sense_mission` | ament_python | 2, 36 | state machine, autopilot adapter, battery |
| `aero_sense_alerts` | ament_python | 19 | alert engine |
| `aero_sense_comms` | ament_python | 18 | link states, store-and-forward |
| `aero_sense_reports` | ament_python | 20 | logger, reports, replay |
| `aero_sense_scenario_manager` | ament_python | 5, 13–16, 40, 41 | `config/victims.yaml` + victim spawning and ground truth; scenarios, failure injection, dynamic hazards |
| `aero_sense_visualization` | ament_python | 21 | RViz config, markers, status overlay |
| `aero_sense_bridge` | ament_python | 22 | FastAPI/WebSocket, JSON contracts |

A package is created in the phase that first needs it — no empty skeletons. The earlier
single-process prototype (now in `legacy/prototype/`) is migrated into these packages phase by phase
(its geo, risk, A*, world-model, store-and-forward and flight code is reused) and removed
once superseded.

## Frames

`map` (ENU, origin = command base, the world origin) → `odom` → `base_link` →
`camera_link` → `camera_optical` (RGB, depth and thermal share one mount, pointing straight down, and one optical centre);
`base_link` → `imu_link`, `baro_link`. Sensor frames are static TFs rendered from
the same table as the model (below); a namespaced drone prefixes them (`drone_01/base_link`).
Lat/lon come from the world's `spherical_coordinates` (WGS84) through one conversion module.

## Topic architecture

All names are relative (`aero_sense/…`) so a drone launched under namespace `drone_01`
publishes `/drone_01/aero_sense/…`; the single-drone default namespace is empty.

| Topic | Type | Producer |
|---|---|---|
| `aero_sense/camera/{rgb,depth,thermal}/image_raw`, `…/camera_info` | Image (rgb8 / 32FC1 / mono16 0.01 K), CameraInfo | Gazebo |
| `aero_sense/imu`, `aero_sense/baro` | Imu, FluidPressure | Gazebo (OAK-D IMU, flight-controller barometer) |
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
| `aero_sense/ground_truth/victims` | VictimArray (latched) | scenario manager — **evaluation only**, never an input to perception or the dashboard |

### Sensor payload (Phase 3)

`aero_sense_description/config/sensors.yaml` is the single source for the payload:
`render.py` turns it into the drone SDF, the ros_gz_bridge config and the static TFs, and
`simulation.launch.py quality:=low|medium|high` picks the OAK-D's resolution/rate profile.
Every camera has the real part's field of view (hardware/README.md); the Lepton 3.5 renders at its
native 160×120 in every profile. There is no LiDAR: none is part of the build, and no node read
the simulated one (`aero_sense_navigation/obstacle_field.py` is ready for one if it is added).
Medium profile (RTF 1.0, headless, RTX 2050):

| Sensor | Real part | Simulated (medium) | Measured rate | Noise (configured → measured) |
|---|---|---|---|---|
| RGB | OAK-D Pro W OV9782, 127° | 960×600, 127° | 9.0 Hz / 10 | σ 0.007 of full scale |
| Depth | OAK-D Pro W OV9282 stereo, 127°, 0.7–12 m | 1280×800, 127°, 0.7–12 m | 4.5 Hz / 5 | σ 0.02 m (measured 0.018 m at the earlier 68.8° FOV) |
| Thermal (LWIR) | FLIR Lepton 3.5, 160×120, 57°, 8.6 Hz | 160×120 mono16, 57° | 8.4 Hz / 8.6 | none — gz-sensors 8 segfaults on thermal `<noise>`; real quantisation is ~2.6 K, not the 0.01 K count scale |
| IMU | OAK-D Pro W BNO086 | — | 97 Hz / 100 | gyro σ 0.0009 → 0.00091 rad/s, accel σ 0.017 → 0.0168 m/s² |
| Barometer | flight controller | — | 9.7 Hz / 10 | σ 5 Pa → 5.2 Pa |

The flight IMU inside the hexacopter model stays noise-free: ArduPilot SITL consumes it and adds
its own sensor model. GPS is the autopilot's (`SIM_GPS1_*`), so GPS denial acts on what the
EKF actually uses.

### Victims (Phase 5)

`aero_sense_scenario_manager/config/victims.yaml` is the scenario's casualty list: position,
pose, LWIR skin temperature, occlusion and the priority a correct triage engine should reach.
The launch spawns one static manikin per entry (DARPA SubT `survivor` mesh) carrying a Thermal
plugin at that temperature — body heat is what makes LWIR search meaningful — and
`victim_ground_truth` republishes the table on its own latched topic for evaluation.
`victims:=false` runs the world empty.

Nine victims sit in S1: lying, seated, prone and trapped, from clear ground to heavy occlusion,
plus one deceased at 295 K that thermal *cannot* find (RGB shape only) so triage has a P3 case.
Verified from the air at 25 m: every live casualty shows 306-311 K against 298 K ground
(11-47 pixels at the medium thermal profile), and the deceased shows none.

Two placement rules the first pass got wrong, both silent: a victim inside a building's mesh is
invisible to every sensor, and a surface within one thermal quantisation step (~2.6 K) of body
heat hides casualties in the same grey level.

### Perception (Phase 6)

`victim_detector` reads the LWIR frame and nothing else: warm connected regions of a plausible
size (`config/perception.yaml`), each pixel turned into a map position by intersecting its
camera ray with the ground, then associated across frames into tracks with stable ids. A track
is published only after several looks agree, and confidence compounds as
`1 - (1 - strength) * 0.6^(looks - 1)`: corroboration raises it, nothing makes it certain.

Ground truth is never an input here. Scoring a lawnmower search of S1 at 30 m against it:
**8 of 8 live casualties found, mean position error 0.3 m, no false positives.** The deceased
casualty (295 K) is invisible to thermal by construction — finding it needs the RGB shape cue,
which is the honest reason to add a second detector rather than a reason to lower the threshold.

Two defects the first flight exposed: tracks were being forgotten while the search continued
(a found casualty must stay found — `confirmed()` keeps them, `current()` is for what is in
view), and a camera that looks 21 m ahead leaves gaps beside the flight line, which is a
search-pattern problem for Phase 12, not a detector one.

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
Asphalt01 albedo/normal/roughness) and LWIR temperatures: asphalt 301 K, pad 300 K,
soil 298 K, everything else ambient 293 K. gz quantises heat sources to ~2.6 K steps
(256 x the thermal `resolution`, and the range must still cover 550 K fire), so surfaces are
kept more than one step below the 306-310 K of a body. Drones spawn on the command-base pad (`<frame
name="drone_spawn">`, read by `aero_sense_bringup/worlds.py`). SITL's home is the world's
`spherical_coordinates`, so ArduPilot's local NED origin is the world origin and `map` =
Gazebo world frame (verified: ROS pose (0.00, −109.99) vs Gazebo (0.00, −110.00) at the pad).

**S4 landslide:** a 200 x 200 m ridge north of the city rising 25 m, with the failure scar
gouged down its face and a debris fan at the toe, generated by `tools/make_terrain.py` and
committed as meshes. Its south edge is at zero height, so it meets the city plain exactly and
nothing in the other sectors moves; the ground quad and road spine stop at the toe rather than
running under it.

It is a **mesh, not a `<heightmap>`**: Gazebo's heightmap renders for physics and the GUI but
stays invisible to the camera sensors, which would have left the drone flying at a ridge it
could not see. Measured with the depth camera: ground 5.8 m above the plain where the model
predicts it. Grass below, bare rubble above, with the changeover dithered across a 5-9 m band
so the boundary breaks up instead of stepping along the mesh grid.

**S2 flood:** a village standing in 0.6 m of water. The flood surface is a transparent quad
laid over the sector, so the buildings and vehicles beneath it are submerged by construction
rather than by separate "flooded" models. It reads 291 K in LWIR — *colder* than the 298 K
ground — so a casualty in the water stands out by contrast the opposite way round from one on
dry land, and the same detector finds them without a special case. Verified: all three flood
casualties found, 0.5-1.1 m error. That error is larger than S1's 0.2 m because they float
0.6 m above the ground plane the projection assumes, which is the cue to use depth for range.

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
| 3 | Sensors | RGB/thermal/depth/IMU/GPS on `/aero_sense/*` at configured rates, with noise |
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
