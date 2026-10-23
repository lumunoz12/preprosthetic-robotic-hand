# Mechanical Design (CAD)

Mechanical design files of the five-finger robotic hand: the native Autodesk Inventor project and STL files for 3D printing. The palm, fingers, covers and transmission supports are 3D-printed, and each finger is an independently assembled and tested module.

## Folder contents

The `native/` folder is an Autodesk Inventor **Pack and Go** package of the complete hand, so it can be opened on another computer with all references resolved.

```text
cad/
├── native/
│   ├── mano_VF.ipj                   Inventor project file (open this first)
│   ├── Workgroups/
│   │   └── NotInProject/             Design files of the hand
│   │       ├── mano_VF.iam           Main assembly of the complete hand
│   │       ├── engranes.iam          Gear subassembly
│   │       ├── Palma.ipt, Cubre_palma.ipt, 12T.ipt, 24T.ipt, ...
│   │       ├── anular_VF/            Finger parts (see "Finger composition")
│   │       ├── indice_VF/
│   │       ├── medio_VF/
│   │       ├── menique_VF/
│   │       ├── pulgar_VF/
│   │       └── sabado/
│   ├── Design Data/                  Inventor support data (materials, styles, libraries)
│   ├── Templates/                    Inventor templates
│   ├── OldVersions/                  Backup copies created by Inventor
│   ├── mano_VF.ipj.old
│   └── packngo.log                   Pack and Go log
├── stl/                              STL files for 3D printing (50 files)
└── README.md
```

`Design Data/` and `Templates/` are standard Autodesk Inventor content added by Pack and Go so that the project is self-contained. They are not part of the authors' design and remain subject to Autodesk's terms.

## Opening the native files

1. Install **Autodesk Inventor Professional 2026** (Build 175, Release 2026.0.1) or later.
2. Download or clone the whole `cad/native/` folder and keep its structure unchanged.
3. In Inventor, open the **Projects** dialog, browse to `native/mano_VF.ipj` and set it as the active project.
4. Open `Workgroups/NotInProject/mano_VF.iam`.

The model was originally created in Inventor 2021 and is currently saved in Inventor 2026. Inventor files cannot be opened in a release older than the one that last saved them, so these files **cannot be opened in Inventor 2021**.

Inventor 2021 was used to export the model to Simscape Multibody through the Simscape Multibody Link plug-in, which was only available up to that release. The geometry used by the simulations is stored in [`../matlab/simulink/cad`](../matlab/simulink/cad); see [`../matlab/README.md`](../matlab/README.md).

## Finger composition

The part files are not stored by finger, and some names reflect the design history rather than the finger they belong to. For example, the index finger uses parts named `*_anular`, and the ring finger uses parts named `*_Indice`. The table below lists the parts that make up each finger.

| Finger | Parts |
|---|---|
| Index | `Meta_indice`, `Prox_anularVF2`, `segunda_anular2`, `Diag_Anular`, `Tierra_Indice`, `Tapa_meta_indice`, `Tapa_prox_medio`, `transmision_motor` |
| Middle | `Meta_medio`, `Prox_medio`, `distal_medio`, `Diag_medio`, `Tierra_medio`, `Tapa_meta_indice`, `Tapa_prox_medio`, `transmision_motor`, `transmision_motor_medio` |
| Ring | `Meta_anular`, `Prox_Indice`, `distal_indice`, `Diag_Indice`, `Tierra_Indice`, `Tapa_meta_indice`, `Tapa_prox_medio`, `transmision_motor`, `transmision_motor_anular` |
| Little | `Meta_menique`, `Prox_menique`, `distal_menique`, `Diag_menique`, `Tierra_menique`, `Tapa_meta_indice`, `Tapa_prox_medio`, `transmision_motor` |
| Thumb | `Prox_pulgar`, `distal_pulgar`, `Diag_pulgar`, `Tierra_pulgar`, `tapa_pulgar`, `tapa_pulgar_PIP`, `transmision_motor` |

`Tapa_meta_indice`, `Tapa_prox_medio` and `transmision_motor` are common parts used in several fingers. All remaining files belong to the palm and general assembly, including `Palma`, `Cubre_palma`, the gear parts and the N20 gearmotor model.

## STL files for printing

The `stl/` folder contains one STL file per printed part instance. File names follow the pattern `mano_final_V3_<part>_<n>.stl`, where `<part>` is the native part name and `<n>` is the instance number in the assembly. The number of files per part is therefore the number of copies to print.

| Part | STL files |
|---|---:|
| `Palma` | 1 |
| `Cubre_palma` | 1 |
| `Diag_*` (one part per finger, two copies each) | 10 |
| `Meta_*` (index, middle, ring, little) | 4 |
| `Prox_*` (one per finger, including thumb) | 5 |
| `distal_*` (indice, medio, menique, pulgar) | 4 |
| `segunda_anular2` | 1 |
| `Tierra_*` (`Tierra_Indice` ×2, medio, menique, pulgar) | 5 |
| `Tapa_meta_indice` | 4 |
| `Tapa_prox_medio` | 4 |
| `tapa_pulgar`, `tapa_pulgar_PIP` | 2 |
| `transmision_motor` ×7, `transmision_motor_medio`, `transmision_motor_anular` | 9 |
| **Total** | **50** |

Print orientation, material and slicer settings are not prescribed. The paper reports filament purchase quantities only, not printed-part mass.

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

## Modeling notes

- **Bevel gears.** The CAD model represents the bevel gears in a simplified form, as 12T/24T gears (`12T.ipt`, `24T.ipt`). The physical prototype uses M0.5 bevel gears with 13 and 25 teeth (paper, Table VI). The firmware uses a nominal motor-output-to-joint ratio of nb = 2 as an approximation of this transmission.
- **N20 gearmotor model.** `N20 Mini Micro Metall Gear Motor DC 3-6-12V v1.ipt` is the model [N20 Mini Micro Metal Gear Motor with Encoder](https://grabcad.com/library/n20-mini-micro-metal-gear-motor-with-encoder-1) by [Steve Hentschke](https://grabcad.com/steve.hentschke-1), published in the GrabCAD Library on October 7, 2023. It is used for fit and assembly only, is not part of the authors' original design, and remains subject to the GrabCAD terms of use.

## Bill of materials

Materials and equipment are listed in Table VI of the paper and in [`../data/Budget.xlsx`](../data/Budget.xlsx), with the purchase link of each item. The reported budget is approximately USD 450 (CLP 434,712 at CLP 965/USD), including purchased spares and laboratory power supplies. It excludes engineering development time and charges for the use of university manufacturing equipment, and is not an installed-parts-only replication cost.

## Limitations

The design is a laboratory research prototype. It is not designed or validated as a clinical prosthetic device.
