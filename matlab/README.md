# MATLAB, Simulink and Simscape Multibody

This folder contains the three Simscape Multibody models used in the paper (pinch, lateral pinch and cylindrical grasp), the data file and CAD geometry they depend on, and the MATLAB scripts used for kinematics, grasp trajectories and motor identification/LQR design.

Simulation outputs are **modeled** quantities. Contact parameters were tuned by trial and error without calibration against physical contact measurements, so simulated forces are not experimentally validated and do not predict physical grasp success.

## Folder contents

```text
matlab/
├── models/
│   ├── mano_buena_pinza_paper.slx        # pinch
│   ├── mano_buena_lateral_paper.slx      # lateral pinch
│   ├── mano_buena_cilindrico_paper.slx   # cylindrical grasp
│   └── LQR_simu.slx                      # LQR simulation of a single motor
├── data/
│   └── mano_vf_DataFile1.m               # geometry/inertia data required by the models
├── geometry/                             # STEP files of the hand parts used by the models
├── scripts/
│   ├── Cinematica_inversa.m
│   ├── Cdirecta.m
│   ├── media_distal_indice.m
│   ├── LQR_motorV2.m
│   ├── TipPinchGrasp.m
│   ├── LateralPinchGrasp.m
│   ├── CylindricalGrasp.m
│   ├── CILINDRICO.m
│   ├── cyl_targets.c
│   ├── cyl_targets.h
│   └── escalon_motor2.mat
└── README.md
```

## Requirements

| Item | Requirement |
|---|---|
| MATLAB release | **MATLAB R20XXx** (replace with the release used to verify the models) |
| Required products | MATLAB, Simulink, Simscape, Simscape Multibody |
| Optional products | Control System Toolbox and System Identification Toolbox, only for the motor identification and LQR design scripts |

The three models use Spatial Contact Force blocks (Simscape Multibody). They were run with a variable-step `ode23` solver, maximum step size 1e-3 s, relative tolerance 1e-2 and absolute tolerance 1e-4.

## Initialization

Start from a **new** MATLAB session with no variables loaded. All paths below are relative to the repository root.

```matlab
% 1. Make the matlab/ folder the current folder and add its subfolders to the path
cd matlab
addpath(genpath(pwd))

% 2. Load the model data into the base workspace
run('data/mano_vf_DataFile1.m')

% 3. Set the contact-force threshold of the simulated grasp logic [N]
F_threshold = 2;

% 4. Open and simulate one of the models
open_system('mano_buena_pinza_paper')        % pinch
% open_system('mano_buena_lateral_paper')    % lateral pinch
% open_system('mano_buena_cilindrico_paper') % cylindrical grasp
sim('mano_buena_pinza_paper')
```

Notes:

- `mano_vf_DataFile1.m` and `F_threshold` must exist in the **base workspace** before a model is simulated. Repeat steps 2–3 in every new session.
- `F_threshold = 2` is the contact-force threshold (in newtons) that triggers the motion-holding condition in the simulated grasp logic. It is distinct from the firmware threshold of 1500 ADC counts, and equivalence between the two was not established.
- No absolute paths are required. If MATLAB cannot find a geometry file, check that `geometry/` is on the path (step 1).

## Reported simulation settings and results

Simulation end times were set to the approximate grasp-completion times identified during simulation. They are simulated elapsed time, not computation time or measured physical grasp duration.

| Model | Grasp | End time [s] | Peak normal contact force, index [N] | Peak normal contact force, thumb [N] |
|---|---|---:|---:|---:|
| `mano_buena_pinza_paper` | Pinch | 10.5 | 1.86 | 1.62 |
| `mano_buena_lateral_paper` | Lateral pinch | 8.3 | – | 1.77 |
| `mano_buena_cilindrico_paper` | Cylindrical | 15.3 | 0.99 | 1.88 |

Forces are the maximum normal contact-force magnitudes over the full simulation, obtained from Spatial Contact Force blocks associated with the distal phalanges. The lateral-pinch index value is not reported because contact occurred at the proximal phalanx rather than the distal surface monitored in the model; the dash does not denote zero force.

Physical bench trials used different joint targets and, in some cases, different objects. The two stages are not a matched validation dataset.

## Scripts

| File | Purpose |
|---|---|
| `Cinematica_inversa.m`, `Cdirecta.m` | Kinematics of the four-bar linkage that couples each distal joint to its adjacent actuated joint |
| `media_distal_indice.m` | Auxiliary script related to the index-finger distal joint |
| `TipPinchGrasp.m`, `LateralPinchGrasp.m`, `CylindricalGrasp.m` | Joint-trajectory definitions for the three grasp patterns |
| `CILINDRICO.m`, `cyl_targets.c`, `cyl_targets.h` | Cylindrical-grasp joint targets; the C files are also used by the firmware |
| `LQR_motorV2.m` | First-order motor model identification, discretization (Ts = 0.02 s) and discrete LQR design |
| `escalon_motor2.mat` | Step-test data of an isolated N20 gearmotor (same test as `data/escalon_motor2.csv`) |

The identified model was estimated with `ssest` from 699 input–velocity pairs. The reported 99.81% fit concerns the identification data; no independent validation dataset was used.

## Models

`LQR_simu.slx` is not one of the three Simscape grasp models; it is provided as supplementary material for the LQR design.

Generated files (`slprj/`, `*.slxc`) are not tracked and are recreated by MATLAB.
