# Hardware

The real parts of the Aero Sense hexacopter: their manufacturer CAD, and what is still to be chosen.
The simulated drone (`src/aero_sense_description`) shows each part from its CAD once it exists here,
and a labelled placeholder until then.

```
hardware/
├── concept/                                  the first hexacopter look, saved as GLB + STL
├── images/                                   the frame as designed (CAD screenshots)
└── cad/                                      STEP files (not in git: see below)
    ├── hexacopter_frame/                     the team's frame: final assembly + parts/ (plates, arms,
    │                                         motor mounts, landing gear)
    ├── oak_d_pro_w/OAK-D-PRO-W.step          Luxonis enclosure assembly
    └── flir_lepton_3_5/Lepton-3.5_socket-10502821001_57deg.step
```

The STEP files are large (and the camera ones under their vendors' terms), so git ignores `hardware/cad/`; the
meshes made from them (`src/aero_sense_description/meshes/`) are committed. To regenerate a mesh,
download the CAD into the path above: OAK-D Pro W from the
[Luxonis product page](https://docs.luxonis.com/hardware/products/OAK-D%20Pro%20W) (CAD / STEP),
Lepton 3.5 "IDD CAD data" from [FLIR](https://oem.flir.com/products/lepton/?model=500-0771-01).

## Parts list

CAD is kept for the major parts only: the frame (with its landing gear) and the two cameras. The
battery is a labelled box; motors and propellers stay generic until they are bought.

| Part | Chosen | CAD | In the sim |
|---|---|---|---|
| Flight controller | **Holybro Pixhawk 6C Mini** (ArduCopter, Hexa-X) | not needed | labelled box 54.3 × 39 × 17.5 mm on the bottom plate, under the top plate's opening |
| Companion computer | **Qualcomm RB5** | not needed | labelled box 85 × 54 × 20 mm behind the FC; cables to FC and both cameras |
| Stereo depth + RGB camera | **Luxonis OAK-D Pro W** with IR (dot projector + flood illuminator), OV9782 colour | ✅ `cad/oak_d_pro_w` | mesh `oak_d_pro_w.glb`, pointing straight down under the battery |
| Thermal camera | **FLIR Lepton 3.5** (500-0771-01, 160 × 120, radiometric) in Molex socket 105028-2001 | ✅ `cad/flir_lepton_3_5` | mesh `flir_lepton_3_5.glb`, beside the OAK-D |
| ESCs (×6) | **Readytosky BLHeli 45A, 2–6S** | not needed | labelled box 28.5 × 13 × 6 mm under each arm, power and motor leads along the arm |
| Motors (×6) | generic (not bought yet; T-Motor 400–450 KV class planned) | not needed | generic: ArduPilot iris motor model |
| Battery | **6S 10000 mAh LiPo** | not needed (box) | box labelled "6S 10000mAh", 200 × 77 × 63 mm, strapped under the bottom plate |
| Propellers | generic (not bought yet) | not needed | generic: iris 10" props |
| Frame (plates, arms, motor mounts) | **team design** (`cad/hexacopter_frame`) | ✅ | mesh `hexacopter_frame.glb`: motors 567 mm from the centre |
| Landing gear | part of the frame (skids, 583 mm below the plates' mid-plane) | ✅ | in the frame mesh; lands on the skids |
| GNSS module | Holybro M10-class GPS puck on a folding mast (to confirm) | not needed | white φ50 × 14.4 mm puck with LED ring on a 140 mm mast at the back of the top plate |
| Lepton carrier board | — | not needed | green board between the camera tray and the Lepton |
| Camera mount | — | not needed | tray below the battery, hung from the bottom plate by two side plates |
| Power distribution board | — | not needed | red 36 mm board "PDB" under the FC (stacked on standoffs): battery lead in, six ESC leads out |
| Power module, telemetry radio, RC receiver | — | not needed | not drawn |

You wrote the OAK-D's sensor as "OV9728"; the OAK-D Pro W's colour options are the IMX378 and
the OV9782, and OV9782 is the one that pairs with a W model's wide stereo, so the sim uses it
(127° HFOV, 1280 × 800). Say if the unit is the IMX378 version (95°).

From the Lepton package only the socket-mounted module is used: FLIR's file also carries an
"Optical Area 57deg HFoV" solid, the keep-out cone from their interface drawing, which is not a
physical part and is dropped when the mesh is made. The no-socket and older-socket variants were
not needed (they remain in FLIR's download).

## Still needed, and why

1. **Motors and propellers: generic for now.** The sim flies on ArduPilot's iris motor and 10"
   prop model. When real ones are bought, their KV, prop size and thrust replace it (400–450 KV
   on 6S usually swings 15–17" props; the frame's 567 mm arms leave room for up to ~22").
2. **Frame mass and inertia.** The frame's geometry is in the sim; its mass is not. Weigh it (or
   give the materials) to set `body_mass_kg` and `body_inertia`.
3. **Mass budget.** Battery, RB5, OAK-D, motors and frame together set `airframe.body_mass_kg`
   and inertia; the sim still uses 2.2 kg until the parts are weighed or their datasheet masses
   are added up.
4. **GNSS module:** the sim shows a Holybro M10-size puck ([φ50 × 14.4 mm](https://holybro.com/products/m10-gps))
   on a 140 mm folding mast; confirm the model you buy (the Pixhawk 6C Mini has no GNSS of its own).
5. **Lepton carrier board.** The socket has to sit on a PCB that breaks out SPI/I²C to the RB5,
   or a USB board such as PureThermal.
6. **Power:** a power module rated for 6S to feed the Pixhawk and measure the pack, and a 5 V
   supply for the RB5 and cameras.
7. **Telemetry radio** and **RC receiver**.

## Real sensor specs, as simulated

| | Real part | Simulated (`quality:=medium`) |
|---|---|---|
| Thermal | Lepton 3.5: 160 × 120, 57° HFOV, 8.6 Hz, 8–14 µm, < 50 mK NETD, radiometric, −10 to 400 °C (low gain) | 160 × 120, 57°, 8.6 Hz; 253–600 K range, ~2.6 K rendering step |
| RGB | OAK-D Pro W: OV9782 global shutter 1280 × 800, 127° HFOV | 960 × 600, 127° (1280 × 800 on `quality:=high`) |
| Depth | OAK-D Pro W: OV9282 stereo pair 1280 × 800, 127° HFOV, 75 mm baseline, ~0.7–12 m | 1280 × 800, 127°, 0.7–12 m |
| IMU | BNO086 inside the OAK-D Pro W | 100 Hz, noisy |
| LiDAR | none | none (removed; `aero_sense_navigation/obstacle_field.py` is ready if one is added) |

Sources: [Luxonis OAK-D Pro W](https://docs.luxonis.com/hardware/products/OAK-D%20Pro%20W),
[FLIR Lepton 3.5](https://oem.flir.com/products/lepton/?model=500-0771-01).

## Regenerating a mesh

Parts without CAD are labelled boxes at their published sizes (Pixhawk 6C Mini:
[Holybro](https://docs.holybro.com/autopilot/pixhawk-6c-mini/technical-specification); RB5 dev kit
85 × 54 mm: [Thundercomm](https://www.thundercomm.com/product/qualcomm-robotics-rb5-development-kit/),
height estimated; Readytosky 45A ESC 28.5 × 13 mm:
[Rotorama](https://www.rotorama.com/product/readytosky-45a-2-6s-blheli-s)). Where they sit, and the
straps, camera tray and wires, come from `airframe:` in `sensors.yaml` (`render.layout`).

```bash
M=src/aero_sense_description/meshes
python3 tools/make_label_box.py $M/battery.glb --size 0.200 0.077 0.063 --label "6S 10000mAh"
python3 tools/make_label_box.py $M/flight_controller.glb --size 0.0543 0.039 0.0175 \
    --label "Pixhawk 6C Mini" --body 0.86 0.86 0.88 --text 0.05 0.05 0.06
python3 tools/make_label_box.py $M/companion_computer.glb --size 0.085 0.054 0.020 \
    --label "Qualcomm RB5" --body 0.05 0.25 0.12
python3 tools/make_label_box.py $M/esc.glb --size 0.0285 0.013 0.006 --label "ESC 45A" --body 0.1 0.2 0.6
python3 tools/make_label_box.py $M/pdb.glb --size 0.036 0.036 0.004 --label "PDB" --body 0.55 0.1 0.1
python3 tools/make_label_box.py $M/gps_label.glb --size 0.030 0.030 0.0015 --label "GPS" \
    --body 0.95 0.95 0.95 --text 0.1 0.1 0.12
```

A test checks each mesh is the size the table says.

For parts with CAD: Gazebo loads triangle meshes, not STEP. After replacing a STEP file:

```bash
pip install cadquery-ocp trimesh          # only this tool needs them
python3 tools/step_to_mesh.py "hardware/cad/hexacopter_frame/hexacopter Final assembly.STEP" \
    src/aero_sense_description/meshes/hexacopter_frame.glb --y-up --origin-mm 0 50 0 \
    --drop "socket head" --drop "hex nut" \
    --colour "bottom plate=0.07,0.07,0.08" --colour "topplate2=0.07,0.07,0.08" \
    --colour "Arm main=0.09,0.09,0.1" --colour "arm connector=0.72,0.72,0.74" \
    --colour "motor mount=0.72,0.72,0.74" --colour "landing vertical=0.8,0.8,0.82" \
    --colour "landing horizontal=0.8,0.8,0.82" --colour "landing handle=0.06,0.06,0.06" \
    --colour "landing split=0.06,0.06,0.06" --colour "landing damper=0.03,0.03,0.03"
python3 tools/step_to_mesh.py hardware/cad/oak_d_pro_w/OAK-D-PRO-W.step \
    src/aero_sense_description/meshes/oak_d_pro_w.glb \
    --colour 606=0.16,0.16,0.18 --colour D-PRO-W-GLASS=0.03,0.03,0.05 \
    --colour D-PRO-W-LENS-ADHESIVE=0.02,0.02,0.02 --colour 611=0.01,0.01,0.01 \
    --colour 612=0.1,0.1,0.1 --colour 602=0.6,0.6,0.62
python3 tools/step_to_mesh.py hardware/cad/flir_lepton_3_5/Lepton-3.5_socket-10502821001_57deg.step \
    src/aero_sense_description/meshes/flir_lepton_3_5.glb \
    --drop NAUO5 --colour NAUO2=0.7,0.7,0.72 --colour NAUO3=0.08,0.08,0.08
```

The frame is drawn Y up with Z forward, so `--y-up` turns it to the drone's axes and
`--origin-mm 0 50 0` (midway between the plates) becomes base_link; its 144 screws and nuts are
dropped. If the frame changes, re-measure `airframe.arm_m`, `motor_mount_top_m`, `plate_stack_m`
and `skids` from it (and `worlds.GEAR_HEIGHT_M`).

`--drop` leaves out a part by name and `--colour` colours parts the CAD left uncoloured. The mesh's
origin is the centre of the part's front face, looking along +X, so it lines up with its sensor.
