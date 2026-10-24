# MATLAB, Simulink and Simscape Multibody

This folder contains the MATLAB scripts used for kinematics, grasp setpoints, motor identification and LQR design, and the encoder comparison, together with the MATLAB project that holds the three Simscape Multibody models used in the paper (pinch, lateral pinch and cylindrical grasp).

Simulation outputs are **modeled** quantities. Contact parameters were tuned by trial and error without calibration against physical contact measurements, so simulated forces are not experimentally validated and do not predict physical grasp success.

## Folder contents

```text
matlab/
├── script/
│   ├── Cdirecta.m
│   ├── Cinematica_inversa.m
│   ├── media_distal_indice.m
│   ├── TipPinchGrasp.m
│   ├── LateralPinchGrasp.m
│   ├── CylindricalGrasp.m
│   ├── LQR_motorV2.m
│   ├── escalon_motor2.mat
│   ├── graficar_motorN20.m
│   └── datos_motor.mat
├── simulink/
│   ├── simulink.prj                              MATLAB project file
│   ├── resources/project/                        MATLAB project metadata (do not edit)
│   ├── modelos/
│   │   ├── mano_buena_pinza_paper.slx            Pinch
│   │   ├── mano_buena_lateral_paper.slx          Lateral pinch
│   │   ├── mano_buena_cilindrico_paper.slx       Cylindrical grasp
│   │   └── LQR_simu.slx                          LQR simulation of a single motor
│   ├── datos/
│   │   └── mano_vf_DataFile1.m                   Geometry and inertia data required by the models
│   ├── cad/                                      STEP geometry used by the models (33 files)
│   ├── script/
│   │   ├── CILINDRICO.m
│   │   ├── cyl_targets.c
│   │   └── cyl_targets.h
└── README.md
```

The STEP files in `simulink/cad/` were exported from the Inventor 2021 version of the hand model through Simscape Multibody Link (see [`../cad/README.md`](../cad/README.md)). Some of their part names differ from the current native CAD files.

## Requirements

All files were developed and run with **MATLAB R2025a** on Windows 11.

| Product | Required for |
|---|---|
| MATLAB, Simulink, Simscape, Simscape Multibody | Models in `simulink/modelos/` |
| System Identification Toolbox, Control System Toolbox | `LQR_motorV2.m` (`iddata`, `ssest`, `c2d`, `dlqr`) |
| Statistics and Machine Learning Toolbox | `graficar_motorN20.m` (`normpdf`) |

The kinematics and grasp-setpoint scripts use only base MATLAB.

## Running the simulations

1. Open `simulink/simulink.prj` in MATLAB.
2. Define the contact-force threshold in the Command Window:

   ```matlab
   F_threshold = 2;
   ```

3. Open and run the models in `simulink/modelos/`.

`F_threshold` is the contact-force threshold (in newtons) of the simulated grasp logic and must be defined in every new MATLAB session. It is distinct from the firmware threshold of 1500 ADC counts, and equivalence between the two was not established.

The three grasp models use Spatial Contact Force blocks and were run with a variable-step `ode23` solver, maximum step size 1e-3 s, relative tolerance 1e-2 and absolute tolerance 1e-4.

## Reported simulation results

Simulation end times were set to the approximate grasp-completion times identified during simulation. They are simulated elapsed time, not computation time or measured physical grasp duration.

| Model | Grasp | End time [s] | Peak normal contact force, index [N] | Peak normal contact force, thumb [N] |
|---|---|---:|---:|---:|
| `mano_buena_pinza_paper` | Pinch | 10.5 | 1.86 | 1.62 |
| `mano_buena_lateral_paper` | Lateral pinch | 8.3 | – | 1.77 |
| `mano_buena_cilindrico_paper` | Cylindrical | 15.3 | 0.99 | 1.88 |

Forces are the maximum normal contact-force magnitudes over the full simulation, obtained from Spatial Contact Force blocks associated with the distal phalanges. The lateral-pinch index value is not reported because contact occurred at the proximal phalanx rather than the distal surface monitored in the model; the dash does not denote zero force.

Physical bench trials used different joint targets and, in some cases, different objects. The two stages are not a matched validation dataset.

## Scripts

Scripts in `script/` load their data files by name, so run them with `script/` as the current folder.

| File | Purpose |
|---|---|
| `media_distal_indice.m` | Newton–Raphson solution of the index-finger four-bar linkage over the input angle; tabulates the PIP–DIP relation and interpolates the DIP angle for a PIP angle entered by the user |
| `Cdirecta.m` | Forward kinematics of the complete hand (four long fingers and thumb), using polynomial fits of the distal coupling; plots the hand and prints fingertip positions |
| `Cinematica_inversa.m` | Inverse kinematics of the five fingers with an analytic Jacobian and damped least squares; plots the resulting hand posture |
| `TipPinchGrasp.m` | Setpoint function for the simulated pinch grasp: joint targets by inverse kinematics, staged closure and hold on contact |
| `LateralPinchGrasp.m` | Setpoint function for the simulated lateral pinch grasp |
| `CylindricalGrasp.m` | Setpoint function for the simulated cylindrical grasp |
| `LQR_motorV2.m` | First-order motor model identification from `escalon_motor2.mat`, augmentation with angular position, zero-order-hold discretization (Ts = 0.02 s), discrete LQR design and step simulation |
| `escalon_motor2.mat` | Step-test data of an isolated N20 gearmotor (same data as `data/escalon_motor2.csv`) |
| `graficar_motorN20.m` | Incremental encoder vs. AS5600 comparison: error statistics and four figures (scatter, error per target, error histogram, time response), exported as PDF and SVG to the current folder |
| `datos_motor.mat` | Encoder comparison data: commanded targets, logged encoder and AS5600 readings per target, and steady-state estimates |
| `simulink/script/CILINDRICO.m`, `cyl_targets.c`, `cyl_targets.h` | Cylindrical-grasp joint targets; the C files are also used by the firmware in `firmware/Core/` (keep both copies identical) |

The setpoint functions are defined as `TipPinchGrasp_setpoint`, `LateralPinchGrasp_setpoint` and `CylindricalGrasp_setpoint`. `TipPinchGrasp` corresponds to the pinch grasp of the paper. `LateralPinchGrasp.m` and `TipPinchGrasp.m` use finger lengths for the middle, ring and little fingers that differ from those in the other kinematics scripts.

**Motor identification.** The model was estimated with `ssest` from 699 input–velocity pairs. The reported 99.81% fit concerns the identification data; no independent validation dataset was used. The resulting gain is K = [2192.4  757.4].

**Encoder comparison.** `graficar_motorN20.m` computes the error as `enc_est − as_est` over the seven targets (10°–90°). These steady-state estimates give the values reported in the paper: mean bias −0.31°, MAE 1.77°, RMSE 2.81° and maximum absolute error 6.43°. The file also contains `enc_final` and `as_final`, which are not used in the reported statistics.

`LQR_simu.slx` is not one of the three Simscape grasp models; it is provided as supplementary material for the LQR design.
