# Mechanical Design (CAD)

Mechanical design files of the five-finger robotic hand. The palm, fingers, covers and transmission supports are designed for 3D printing, and each finger is an independently assembled and tested module.

## Folder contents

```text
cad/
├── native/        # Autodesk Inventor project: parts (.ipt), assemblies (.iam), drawings (.idw/.dwg), presentations (.ipn), project file (mano_VF.ipj)
├── stl/           # STL files for 3D printing (50 files)
├── step/          # STEP files of the hand parts (33 files); copies used by the Simscape models are in ../matlab/geometry
└── README.md
```

## Design summary

| Item | Description |
|---|---|
| Fingers | 5 (thumb and four long fingers) |
| Represented joint angles | 15 |
| Actuated axes | 10 (MCP and PIP of each long finger; thumb opposition/abduction and MCP flexion) |
| Distal coupling | Four-bar linkage: DIP follows PIP in the long fingers; IP follows MCP flexion in the thumb. Distal angles are constrained by the linkage geometry and are not independent degrees of freedom |
| Actuation | N20 DC gearmotors with encoders, bevel-gear transmission |
| Nominal actuated-joint range | Approximately 0–90°, with operating limits that avoid near-singular configurations |
| Fabrication | 3D printing |

The design prioritizes low-cost fabrication, modular assembly and measurable joint motion. It is not designed or validated as a clinical prosthetic device.

## Using the files

- **Printing:** use the files in `stl/`. Part names follow the pattern `mano_final_V3_<part>_<n>.stl` (palm, metacarpal, proximal, distal and intermediate phalanges, covers, ground/base pieces, motor transmission links). Print orientation, material and slicer settings are not prescribed; the paper reports filament purchase quantities only, not printed-part mass.
- **Editing:** open `native/mano_VF.ipj` in Autodesk Inventor. Keep the folder structure unchanged so that assembly references resolve.
- **Simulation:** the Simscape Multibody models in [`../matlab`](../matlab) use the STEP files stored in `../matlab/geometry`. Keep that folder on the MATLAB path.

## Bill of materials

Materials and equipment are listed in Table VI of the paper (see [`../docs`](../docs)). The reported budget is approximately USD 450 (CLP 434,712 at CLP 965/USD), including purchased spares and laboratory power supplies. It excludes engineering development time and charges for the use of university manufacturing equipment, and is not an installed-parts-only replication cost.
