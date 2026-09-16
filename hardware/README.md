# Hardware

The real parts of the Aero Sense hexacopter: their manufacturer CAD, and what is still to be chosen.
The simulated drone (`src/aero_sense_description`) shows each part from its CAD once it exists here,
and a labelled placeholder until then.

```
hardware/
└── cad/                                      manufacturer STEP files, as downloaded (not in git: see below)
    ├── oak_d_pro_w/OAK-D-PRO-W.step          Luxonis enclosure assembly
    └── flir_lepton_3_5/Lepton-3.5_socket-10502821001_57deg.step
```

The STEP files are large and under their vendors' terms, so git ignores `hardware/cad/`; the
meshes made from them (`src/aero_sense_description/meshes/`) are committed. To regenerate a mesh,
download the CAD into the path above: OAK-D Pro W from the
[Luxonis product page](https://docs.luxonis.com/hardware/products/OAK-D%20Pro%20W) (CAD / STEP),
Lepton 3.5 "IDD CAD data" from [FLIR](https://oem.flir.com/products/lepton/?model=500-0771-01).

## Parts list

| Part | Chosen | CAD | In the sim |
|---|---|---|---|
| Stereo depth + RGB camera | **Luxonis OAK-D Pro W** | ✅ `cad/oak_d_pro_w` | mesh `oak_d_pro_w.glb` |
| Thermal camera | **FLIR Lepton 3.5** (500-0771-01) in Molex socket 105028-2001 | ✅ `cad/flir_lepton_3_5` | mesh `flir_lepton_3_5.glb` |
| Battery | — | ❌ | black box, 155 × 48 × 52 mm placeholder |
| Frame (centre plates, arms) | — | ❌ | white puck + arms, 0.30 m arm length |
| Motors, propellers, ESCs | — | ❌ | ArduPilot iris rotor ×6 (its thrust model) |
| Flight controller | — | ❌ | not shown (ArduPilot SITL) |
| GNSS module | — | ❌ | not shown (SITL GPS) |
| Companion computer | — | ❌ | not shown |
| Lepton carrier board | — | ❌ | not shown |
| Landing gear | — | ❌ | none: the drone lands on its body |
| Camera bracket (55° down) | — | ❌ | not shown |
| Power module, telemetry radio, RC receiver | — | ❌ | not shown |

From the Lepton package only the socket-mounted module is used: FLIR's file also carries an
"Optical Area 57deg HFoV" solid, the keep-out cone from their interface drawing, which is not a
physical part and is dropped when the mesh is made. The no-socket and older-socket variants were
not needed (they remain in FLIR's download).

## Still needed, and why

1. **Motors, propellers and ESCs** — the thrust the sim uses is the iris's. Motor KV, prop size and
   a thrust curve set `airframe.motor_max_rad_s` and the rotor model, and so how much the drone
   can lift.
2. **Battery** — capacity and cell count set flight time; its dimensions and mass replace the
   placeholder (`airframe.battery_size_m`, `body_mass_kg`).
3. **Frame** — centre plates and arms (bespoke, or tubes + printed parts). Sets `arm_m`, mass and
   inertia.
4. **Flight controller** (runs ArduCopter, e.g. a Pixhawk-class board) and a **GNSS module** with
   compass on a mast.
5. **Companion computer** — the OAK-D Pro W needs USB 3 and the Lepton talks video over SPI, so
   the board needs both (Raspberry Pi 5 or a Jetson Orin Nano class board).
6. **Lepton carrier board** — the socket has to sit on a PCB that breaks out SPI/I²C (a Lepton
   breakout board, or a USB board such as PureThermal).
7. **Landing gear** — without it the drone lands on its belly with the OAK-D 1.5 mm above the
   ground. Fine in simulation, not on rubble.
8. **Camera bracket** holding both cameras 55° down (3-D printed), plus power module, telemetry
   radio and RC receiver.

## Real sensor specs vs. what the sim renders

| | Real part | Simulated now (`quality:=medium`) |
|---|---|---|
| Thermal | Lepton 3.5: 160 × 120, 57° HFOV, 8.6 Hz, 8–14 µm, < 50 mK NETD, radiometric, −10 to 400 °C (low gain) | 320 × 256, 68.8° HFOV, 9 Hz |
| RGB | OAK-D Pro W: IMX378 4056 × 3040, 95° HFOV | 960 × 540, 68.8° HFOV |
| Depth | OAK-D Pro W: OV9282 stereo pair 1280 × 800, 127° HFOV, 75 mm baseline, ~0.7–12 m | 640 × 480, 68.8° HFOV, 0.3–100 m |
| IMU | BNO086 inside the OAK-D Pro W | separate companion IMU |
| LiDAR | none chosen | 16-channel, not used by any node |

Sources: [Luxonis OAK-D Pro W](https://docs.luxonis.com/hardware/products/OAK-D%20Pro%20W),
[FLIR Lepton 3.5](https://oem.flir.com/products/lepton/?model=500-0771-01).

## Regenerating a mesh

Gazebo loads triangle meshes, not STEP. After replacing a STEP file:

```bash
pip install cadquery-ocp trimesh          # only this tool needs them
python3 tools/step_to_mesh.py hardware/cad/oak_d_pro_w/OAK-D-PRO-W.step \
    src/aero_sense_description/meshes/oak_d_pro_w.glb \
    --colour 606=0.16,0.16,0.18 --colour D-PRO-W-GLASS=0.03,0.03,0.05 \
    --colour D-PRO-W-LENS-ADHESIVE=0.02,0.02,0.02 --colour 611=0.01,0.01,0.01 \
    --colour 612=0.1,0.1,0.1 --colour 602=0.6,0.6,0.62
python3 tools/step_to_mesh.py hardware/cad/flir_lepton_3_5/Lepton-3.5_socket-10502821001_57deg.step \
    src/aero_sense_description/meshes/flir_lepton_3_5.glb \
    --drop NAUO5 --colour NAUO2=0.7,0.7,0.72 --colour NAUO3=0.08,0.08,0.08
```

`--drop` leaves out a part by name and `--colour` colours parts the CAD left uncoloured. The mesh's
origin is the centre of the part's front face, looking along +X, so it lines up with its sensor.
