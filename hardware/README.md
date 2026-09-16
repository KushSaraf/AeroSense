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
| Flight controller | **Holybro Pixhawk 6C Mini** (ArduCopter, Hexa-X) | ❌ | ArduPilot SITL, not drawn |
| Companion computer | **Qualcomm RB5** | ❌ | not drawn |
| Stereo depth + RGB camera | **Luxonis OAK-D Pro W** with IR (dot projector + flood illuminator), OV9782 colour | ✅ `cad/oak_d_pro_w` | mesh `oak_d_pro_w.glb`, pointing straight down under the battery |
| Thermal camera | **FLIR Lepton 3.5** (500-0771-01, 160 × 120, radiometric) in Molex socket 105028-2001 | ✅ `cad/flir_lepton_3_5` | mesh `flir_lepton_3_5.glb`, beside the OAK-D |
| ESCs (×6) | **Readytosky BLHeli 45A, 2–6S** | ❌ | not drawn |
| Motors (×6) | **T-Motor, 400–450 KV** (exact model to confirm) | ❌ | ArduPilot iris rotor thrust model |
| Battery | **6S 10000 mAh LiPo** | ❌ | black box labelled "6S 10000mAh", 200 × 77 × 63 mm, slung under the body |
| Propellers | — | ❌ | iris 10" props |
| Frame (centre plates, arms) | — | ❌ | white puck + arms, 0.30 m arm length |
| Landing gear | — | ❌ | four 200 mm legs hanging from the diagonal arms |
| GNSS module | — | ❌ | SITL GPS, not drawn |
| Lepton carrier board | — | ❌ | not drawn |
| Camera bracket | — | ❌ | dark plate between battery and cameras |
| Power module, telemetry radio, RC receiver | — | ❌ | not drawn |

You wrote the OAK-D's sensor as "OV9728"; the OAK-D Pro W's colour options are the IMX378 and
the OV9782, and OV9782 is the one that pairs with a W model's wide stereo, so the sim uses it
(127° HFOV, 1280 × 800). Say if the unit is the IMX378 version (95°).

From the Lepton package only the socket-mounted module is used: FLIR's file also carries an
"Optical Area 57deg HFoV" solid, the keep-out cone from their interface drawing, which is not a
physical part and is dropped when the mesh is made. The no-socket and older-socket variants were
not needed (they remain in FLIR's download).

## Still needed, and why

1. **Exact motor model and propeller size.** 400–450 KV on 6S usually swings 15–17" props. That
   sets the frame size (props that large need arms of roughly 0.45 m or more, not the sim's
   0.30 m) and the thrust the simulation should use: today it still flies on the iris's 10" rotor
   model, so hover throttle, top speed and payload margin in the sim are not yet this drone's.
2. **Frame** sized for those props: centre plates and arms. Sets `arm_m`, mass and inertia.
3. **Mass budget.** Battery, RB5, OAK-D, motors and frame together set `airframe.body_mass_kg`
   and inertia; the sim still uses 2.2 kg until the parts are weighed or their datasheet masses
   are added up.
4. **GNSS module** with compass, on a mast (the Pixhawk 6C Mini has no GNSS of its own).
5. **Lepton carrier board.** The socket has to sit on a PCB that breaks out SPI/I²C to the RB5,
   or a USB board such as PureThermal.
6. **Power:** a power module rated for 6S to feed the Pixhawk and measure the pack, and a 5 V
   supply for the RB5 and cameras.
7. **Landing gear, camera bracket** (CAD for both; the sim's are simple shapes), **telemetry radio**
   and **RC receiver**.
8. **CAD** for the Pixhawk 6C Mini, RB5, ESCs, motors and battery, to replace "not drawn" and the
   battery box with the real parts (`tools/step_to_mesh.py` converts them).

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

The battery has no CAD; its labelled box is generated:

```bash
python3 tools/make_battery_mesh.py --size 0.200 0.077 0.063 --label "6S 10000mAh" \
    src/aero_sense_description/meshes/battery.glb
```

For parts with CAD: Gazebo loads triangle meshes, not STEP. After replacing a STEP file:

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
