# Aero Sense — Architecture & Implementation Plan

A real end-to-end SAR simulation: Gazebo physics and sensors → ROS 2 → onboard autonomy →
MAVLink → a real autopilot flying the simulated airframe → FastAPI/WebSocket → dashboard.
Nothing on the dashboard is produced anywhere except by the ROS nodes below.

## Platform decisions (and why they differ from the brief)

| Brief | Built on | Reason |
|---|---|---|
| ROS 2 Jazzy | **ROS 2 Humble** | Jazzy needs Ubuntu 24.04; this machine is 22.04 with Humble installed |
| PX4 SITL | **ArduPilot SITL (ArduCopter 4.8-dev)** | Installed in `~/uav_ws` and already flown; owner's choice. Autopilot access is isolated in `aero_sense_mission/autopilot.py` (MAVLink only), so PX4 is a contained swap |
| Sensor fusion: PX4 EKF2 | **ArduPilot EKF3** | Same job (IMU, GPS, baro, compass, external vision into one state), and it switches GPS to OpenVINS in flight through its source sets, which GPS-denied flight uses. Owner's choice, 2026-09-18 |
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
Every camera has the real part's field of view (hardware/README.md); the thermal camera renders at
256×192 in every profile (the Lepton 3.5's field of view at a 256×192 core's resolution; the part is not chosen yet). There is no LiDAR: none is part of the build, and no node read
the simulated one (`aero_sense_navigation/obstacle_field.py` is ready for one if it is added).
Medium profile (RTF 1.0, headless, RTX 2050):

| Sensor | Real part | Simulated (medium) | Measured rate | Noise (configured → measured) |
|---|---|---|---|---|
| RGB | OAK-D Pro W OV9782, 127° | 960×600, 127° | 9.0 Hz / 10 | σ 0.007 of full scale |
| Depth | OAK-D Pro W OV9282 stereo, 127°, 0.7–12 m | 1280×800, 127°, 0.7–12 m | 4.5 Hz / 5 | σ 0.02 m (measured 0.018 m at the earlier 68.8° FOV) |
| Thermal (LWIR) | 256×192 core, not yet chosen (was FLIR Lepton 3.5, 160×120, 57°, 8.6 Hz) | 256×192 mono16, 57° | 8.4 Hz / 8.6 (measured at 160×120) | none — gz-sensors 8 segfaults on thermal `<noise>`; real quantisation is ~2.6 K, not the 0.01 K count scale |
| IMU | OAK-D Pro W BNO086 | — | 97 Hz / 100 | gyro σ 0.0009 → 0.00091 rad/s, accel σ 0.017 → 0.0168 m/s² |
| Barometer | flight controller | — | 9.7 Hz / 10 | σ 5 Pa → 5.2 Pa |

The flight IMU inside the hexacopter model stays noise-free: ArduPilot SITL consumes it and adds
its own sensor model. GPS is the autopilot's (`SIM_GPS1_*`), so GPS denial acts on what the
EKF actually uses.

### Victims (Phase 5)

`aero_sense_scenario_manager/config/victims.yaml` is the scenario's casualty list: position,
pose, LWIR skin temperature, occlusion and the priority a correct triage engine should reach.
The launch spawns one person per entry: a Fuel character (man, woman, nurse, child) that
`tools/make_people.py` poses by skinning its rig (the thermal camera cannot see animated actors, so
every pose is baked into a static textured mesh). The body carries a Thermal plugin at that
temperature — body heat is what makes LWIR search meaningful. Rubble over a partly visible or
buried casualty is a seeded pile of overlapping lumps, broken slabs and brick at ambient
temperature, sized from the pose's joints so exactly the named part shows. A waving arm is split off
the same mesh and swings from the shoulder on a joint. `victim_ground_truth` republishes the table,
and
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

## Flying where there is no GPS and no network

The two failures arrive together - the blocks that jam GPS are the blocks with no coverage - so
the drone is built to lose both and still be useful. One path, end to end:

```
  where am I?                                what have I found?
  ───────────                                ──────────────────
  GPS fix        -> EKF3 source set 1        found       -> comms_link
   (none?)          drone_interface              |            |
  surveyed          _locate_origin               |        link up? -> downlink -> dashboard
  launch point      places the map origin        |            |
       |                                         |        link down? -> held in memory
  OpenVINS       -> VISION_POSITION_ESTIMATE     |                      AND a row in SQLite
   fitted to        EKF3 source set 2            |                      (event_store.py)
   the GPS track                                 |            |
       |                                         |        coverage back? -> drained at a rate,
  neither        -> source set 3, no horizontal  |                          casualties first
                    position: the EKF lands      |
                    where it is                  v
                                            local 3D map (vio_map) + obstacle map, both relayed
```

1. **The origin.** EKF3 sets it at its first GPS fix. Launch into a jammed area and it never gets
   one, and without an origin nothing the drone reports can be placed at all. `drone_interface`
   then georeferences on the surveyed launch point (`launch_point`, what a responder always
   knows), and hands over to the autopilot's own origin if a fix ever turns up, saying how far
   apart they were.
2. **Position.** OpenVINS is fitted to the EKF's track while GPS is good and streamed as
   VISION_POSITION_ESTIMATE; when GPS goes the EKF switches to source set 2 and flies on it, and
   with neither it goes to set 3 and lands where it is rather than chase a fake fix.
3. **What it sees.** The stereo feature cloud and the rangefinder returns are both kept as voxel
   maps in the map frame, so the ground gets a local 3D view and a map of obstacles that no
   survey had.
4. **What it found.** Every report the ground has not acknowledged is both held in memory and
   written to the drone's SQLite outbox, so a crashed or restarted link still knows what is owed.
5. **Getting it out.** On coverage returning, the backlog drains in priority order under a token
   bucket: casualties first, then the mission state, then the event log, at a rate the link can
   take rather than as one burst.

## Launch architecture

```
full_system.launch.py
 ├─ simulation.launch.py      gz sim (world + quality profile), SITL, ros_gz_bridge,
 │                            all onboard nodes, recorder
 ├─ visualization.launch.py   RViz2 (grouped displays), rqt_image_view (optional)
 └─ dashboard_bridge.launch.py FastAPI/WebSocket bridge
replay.launch.py              rosbag2 play + RViz
```

## World structure (one world, ~440 × 250 m)

```
            N
 ┌───────────────┬──────────────┐   y +100
 │ S1 EARTHQUAKE │ S2 FLOOD      │   built
 ├───────────────┼──────────────┤   y 0
 │ (S3 fire)     │ (S5 chemical) │   planned, not built
 └───────────────┴──────────────┘   y -100  ◄ command base at (0, -110)
   x -200          0            x +200
```

World file: `aero_sense_gazebo/worlds/aero_sense_disaster.sdf`. So that someone new can read it at a glance,
`aero_sense_zone_signs` adds signboards beside the spine road pointing to each sector, the zone
names painted large on the ground (legible from the Gazebo overview), a COMMAND BASE board and
lettering at the pad, and each mission's search area outlined in yellow (tested against
`mission_manager.SCENARIO_AREAS`); `aero_sense_surroundings` runs fields to the horizon. All of it
is outside the search areas or at ambient temperature, below the thermal detector's threshold.
The society is generated, not hand placed. `tools/make_buildings.py` writes the building models
(`aero_sense_building_*`: RCC row houses, houses, G+3 apartment blocks, pancaked and slumped
collapses, each with a collision box covering its whole footprint), and `tools/layout_world.py`
writes everything between the GENERATED markers in the world plus the road, ground and flood
meshes. Roads and galis are Catmull-Rom curves whose control points include the casualties lying
in them. Buildings go up along both sides of each street, facing it, then fill in behind. No
building covers a casualty except the three trapped inside a collapse on purpose, and the layout
leaves the hand-placed landmarks alone. `structure_map` reads the generated footprints and heights
from the models, and the 16.4 m G+3 blocks set the 22 m inspection altitude. Ground, roads, galis,
dust and mud are OBJ meshes with LWIR temperatures: asphalt 301 K, concrete galis 300 K, pad
300 K, earth and dust 298 K, dirt tracks 297 K, mud 294 K, flood water 291 K, everything else
ambient 293 K. gz quantises heat sources to ~2.6 K steps
(256 x the thermal `resolution`, and the range must still cover 550 K fire), so surfaces are
kept more than one step below the 306-310 K of a body. Drones spawn on the command-base pad (`<frame
name="drone_spawn">`, read by `aero_sense_bringup/worlds.py`). SITL's home is the world's
`spherical_coordinates`, so ArduPilot's local NED origin is the world origin and `map` =
Gazebo world frame (verified: ROS pose (0.00, −109.99) vs Gazebo (0.00, −110.00) at the pad).

**S4 landslide:** removed. The world is the earthquake (S1) and flood (S2) sectors on a flat plain.

**S2 flood:** a village in a valley (`tools/flood_valley.py`). The ground inside a ragged
shoreline falls away to 1.8 m below the plain, and the water stands level at 0.6 m above it, so it
is up to 2.4 m deep in the middle and shallow at the muddy edge. Buildings, vehicles, tracks and
the ground meshes all sit on the sunken floor. The flood surface is a transparent mesh, so what is
under it is submerged by construction rather than by separate "flooded" models. It reads 291 K in
LWIR, *colder* than the 298 K ground. Nobody alive is in water up to 2.4 m deep: each flood casualty
has a `perch`, and `layout_world.py` builds it round them before anything else. A hatchback stranded
on the track takes a man on its roof, single- and two-storey houses take people on their roof
terraces, and houses whose first-floor window faces the track take a nurse and a child leaning out.
Their spawn z must equal the terrain under the perch plus its roof or floor height, or the layout
refuses (so nobody floats or stands in the water).

**S1 earthquake:** the society's older half, where nothing is left intact (tested). The damage
follows what Bhuj (2001), Nepal (2015) and Turkey (2023) left behind:
- pancaked floors, including row houses
- a block toppled like dominoes
- a soft storey crushed under the floors above
- brick infill blown out of the RC frame
- a fallen corner or top floor, and a slumped half

Rubble (allowed to spill onto roads and into neighbouring rubble, never onto a casualty) and dust
fill the crooked galis, and poles lean. The hand-placed tdf landmarks add a collapsed industrial
hall (24 m), fire and police stations, a police cordon of jersey barriers, and two tall obstacles
above the search altitude: a 44 m radio mast at (−40, 60) and a 10 m water tower.

**Network:** `comms_link` is the only path off the drone: it relays onboard topics to
`aero_sense/downlink/...`, and the dashboard bridge subscribes to nothing else.
- **Link state:** OFFLINE inside a dead zone (`comms.NO_NETWORK_ZONES`, drawn red in Gazebo) or
  when cut through `/aero_sense/sim/network`; DEGRADED within 10 m of a zone (video dropped);
  CONNECTED elsewhere.
- **Offline:** the mission carries on unchanged. The newest telemetry, casualty list and mission
  state are kept, every event is queued with the time it happened, and camera frames are dropped.
- **Reconnect:** a "network restored" event, then casualties, mission state and the held events in order.
- **Ground side:** an outage is inferred from 3 s of silence, and what the drone holds stays
  unknown until it reconnects; drone commands are refused meanwhile.

**SWOOP (Suspect, Weigh, Observe Overhead, Prove):** `victim_detector` also publishes faint leads
on `/aero_sense/perception/suspects`:
- **Finding them:** a white top-hat keeps warm features smaller than 2.5 m and drops roads and roofs.
- **Rating them:** each lead's probability comes from its contrast above one gz quantisation step and its size against a person at that height.
- **Keeping them:** the best single look is kept, and frames never compound.

`mission_manager` flies down to the nearest lead that is at least 5 % likely, inside the search area
and not already explained:
- **Height:** as low as `swoop.verify_altitude` allows, which is 5 m above the tallest structure whose no-fly circle covers the spot, never below 10 m.
- **Proving it:** the drone looks for 5 s, then proves the lead (a casualty is confirmed within 8 m) or rules it out.
- **Resuming:** it climbs back and resumes the lawnmower.

Leads beside the mast, where nothing below cruise height is clear, are left for the ground team.

Assets come from `reference/` (gitignored, used in place; see README "Reference assets"):
tdf_gazebo, the Gazebo model collection, the fire-detection repo and DARPA SubT. Their meshes
name textures by bare filename, so each model's `materials/textures` is on
`GZ_SIM_RESOURCE_PATH`. Harmonic ignores Classic OGRE-script materials, and an SDF albedo map on a malformed reference
mesh aborts the gz 8 thermal camera (see the regression test in
`aero_sense_description/test/test_render.py`), so the generated meshes use plain colours or
clean OBJs.

## Dependencies

Installed: ROS 2 Humble (rclpy, sensor_msgs, nav2_msgs/map_server, rosbag2, rviz2,
rqt_image_view), Gazebo Harmonic 8 + ros_gz (built in `~/uav_ws`), ArduPilot SITL,
pymavlink, OpenCV, numpy/scipy, shapely, torch + ultralytics + transformers, FastAPI,
uvicorn, websockets, aiortc (WebRTC video; installed with `pip install aiortc`), reportlab, matplotlib, jinja2.

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
