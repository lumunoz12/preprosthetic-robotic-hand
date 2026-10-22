# Design and Bench Evaluation of a Low-Cost Robotic Hand with Mechanically Coupled Distal Joints

This repository accompanies the research paper **“Design and Bench Evaluation of a Low-Cost Robotic Hand with Mechanically Coupled Distal Joints.”** It contains the source files, simulation models, mechanical designs, embedded firmware, and experimental datasets associated with the development and bench evaluation of a low-cost five-finger robotic hand.

The platform was developed for laboratory research on robotic grasping and pre-prosthetic systems. It is intended as a research prototype, not as a clinically validated prosthetic device.

## Overview

The robotic hand integrates mechanical design, embedded control, sensing, and simulation in a modular, 3D-printed platform.

Key features include:

- **Five-finger anthropomorphic design** with 15 represented joint angles and 10 actuated axes.
- **Mechanically coupled distal joints** using four-bar linkages to coordinate PIP–DIP motion in the long fingers and MCP–IP motion in the thumb.
- **Ten DC gearmotors** with bevel-gear transmissions.
- **STM32F767ZI microcontroller** for embedded motion control.
- **Encoder-based position feedback** and discrete Linear Quadratic Regulator (LQR) control.
- **Two fingertip force-sensitive resistors (FSRs)** for threshold-based contact supervision.
- **MATLAB/Simulink and Simscape Multibody** models for grasp simulation.

## Grasp Evaluation

Three grasp patterns were evaluated in simulation and physical bench trials:

| Grasp pattern | Successful physical trials | Success rate |
|---|---:|---:|
| Pinch | 9/15 | 60% |
| Lateral pinch | 15/15 | 100% |
| Cylindrical grasp | 3/15 | 20% |
| **Total** | **27/45** | **60%** |

Each grasp pattern was tested with three selected objects and five attempts per object. An attempt was considered successful when the hand reached the intended configuration and retained the unsupported object for at least 10 seconds without visible slip.

These results apply only to the selected objects and tested bench conditions. They should not be interpreted as standardized performance benchmarks or evidence of clinical suitability.

## Repository Structure

The repository is organized into the following directories:

```text
preprosthetic-robotic-hand/
├── README.md
├── matlab/
│   └── README.md
├── firmware/
│   └── README.md
├── cad/
│   └── README.md
├── data/
│   └── README.md
└── docs/
    └── README.md
```

The directories are being populated and documented as the project files are organized.

- **`matlab/`** — MATLAB scripts, Simulink models, and Simscape Multibody files for the three grasp patterns.
- **`firmware/`** — STM32CubeIDE project and embedded-control source code.
- **`cad/`** — Native mechanical CAD files and exchange formats for the hand and its components.
- **`data/`** — Original experimental datasets associated with motor characterization, sensor calibration, and bench testing.
- **`docs/`** — Research paper and supplementary technical documentation.

## Embedded Control

The embedded controller runs on an STM32F767ZI development board. It uses incremental encoder measurements to estimate actuated-joint positions and applies discrete LQR state feedback for position control.

High-level commands and telemetry are communicated through a UART interface at 115200 baud. The nominal control update period is 20 ms (50 Hz), while telemetry is transmitted every 100 ms.

The two FSR sensors, located at the thumb and index fingertips, provide threshold-based motion inhibition. They do not implement closed-loop force control or measure the contact-force distribution across all fingers.

## Simulation

The Simscape Multibody models represent the hand's mechanical geometry, joint constraints, coupled distal joints, and virtual grasp objects. Pinch, lateral pinch, and cylindrical grasp configurations were simulated.

Simulated contact forces depend on the selected contact and friction parameters. They were not experimentally validated against physical contact-force measurements, and simulation outcomes should not be treated as predictions of physical grasp success.

## Experimental Data

The repository is intended to include the original datasets used in the reported experimental characterization and evaluation. Each dataset will be accompanied by a description of its variables, units, and experimental context where this information is available.

Reported experiments include:

- Incremental-encoder comparison against an AS5600 absolute encoder.
- FSR sensor characterization.
- DC gearmotor characterization and model identification.
- Physical grasp trials, including encoder measurements and raw FSR readings.

Raw FSR readings below the calibrated range are not interpreted as force measurements. The reported grasp outcomes are determined independently from the FSR threshold.

## Requirements

The following software and hardware are associated with the project:

- MATLAB and Simulink.
- Simscape Multibody.
- STM32CubeIDE and the appropriate STM32 software packages.
- Compatible CAD software for native mechanical design files.
- An STM32F767ZI development board and the corresponding hardware for physical testing.

Exact software versions, dependencies, and instructions for running the models or building the firmware will be documented as the repository is validated. The availability of source files alone does not guarantee that every simulation or build will run without additional configuration.

## Limitations

This project is a laboratory research prototype. The reported study does not establish:

- Contact-driven passive adaptation of the fingers.
- Superior performance relative to alternative controllers.
- Experimentally validated simulated contact forces.
- Closed-loop force regulation.
- Clinical effectiveness, safety, or suitability as a prosthetic device.

The physical results are specific to the tested objects, placements, and bench conditions. Further work is needed to improve quantitative contact and slip measurements, match simulation and experimental conditions, and compare the controller against alternative approaches.

## Reference

**Paper title:** *Design and Bench Evaluation of a Low-Cost Robotic Hand with Mechanically Coupled Distal Joints*

**Authors:** Luis Muñoz and Mario Fernández-Fernández

**Affiliation:** Universidad de Talca, Chile

**Conference:** IEEE ICA-ACCA 2026 — International Conference on Automation / XXVII Congreso de la Asociación Chilena de Control Automático.

The conference is scheduled for December 9–11, 2026, at Universidad del Bío-Bío, Concepción, Chile.

A permanent publication link and the final bibliographic details will be added when available.

## Acknowledgments

The authors acknowledge the support of the Laboratorio de Mecatrónica Agroindustrial (LaMAI) at Universidad de Talca during the development of this work.

## License

No open-source license has been assigned to this repository yet. Unless a license is added, all rights remain with the respective copyright holders. Please contact the authors regarding reuse or redistribution.
