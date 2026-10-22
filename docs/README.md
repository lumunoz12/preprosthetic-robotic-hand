# Documentation

Research paper and supplementary technical documentation.

## Contents

| File | Description |
|---|---|
| `paper.pdf` | *Design and Bench Evaluation of a Low-Cost Robotic Hand with Mechanically Coupled Distal Joints* |

## Paper summary

The paper presents a low-cost, five-finger, 3D-printed robotic hand for pre-prosthetic laboratory research: ten DC gearmotors, bevel transmissions, mechanically coupled distal joints, encoder-based discrete LQR position control on an STM32F767ZI, and two fingertip FSRs for contact supervision. Evaluation combined Simscape Multibody simulations with bench trials of pinch, lateral pinch and cylindrical grasps (three objects per pattern, five attempts per object).

| Grasp | Successful trials |
|---|---:|
| Lateral pinch | 15/15 |
| Pinch | 9/15 |
| Cylindrical | 3/15 |

The results are specific to the tested objects and conditions. Simulated forces were not experimentally validated, and passive adaptation, controller superiority and prosthetic suitability were not established.

## Where to find each part of the paper

| Paper section | Repository location |
|---|---|
| II. Robotic hand design (Figs. 1–2, Table I) | [`../cad`](../cad) |
| III. Embedded control architecture (Fig. 3, Eqs. 2–9) | [`../firmware`](../firmware) |
| III-A. Sensor characterization, motor identification | [`../data`](../data), [`../matlab`](../matlab) |
| IV. Simulation (Table III) | [`../matlab`](../matlab) |
| IV–V. Bench trials (Tables II, IV, V) | [`../data`](../data) |
| V. Budget (Table VI) | Paper, Table VI |

## Reference

L. Muñoz and M. Fernández-Fernández, Universidad de Talca, Chile. IEEE ICA-ACCA 2026, Concepción, Chile, December 9–11, 2026.

A permanent publication link and final bibliographic details will be added when available.
