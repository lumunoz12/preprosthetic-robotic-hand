# Experimental Data

Original datasets behind the experimental characterization reported in the paper.

## Files

| File | Content | Used for |
|---|---|---|
| `escalon_motor2.csv` | Step-response test of an isolated N20 gearmotor: PWM input and encoder-derived angular velocity | Motor model identification and LQR design (paper Eqs. 4–5) |
| `FSR.xlsx` | Force-sensitive resistor calibration with known loads | FSR calibration fit (paper Eq. 3) |
| `Test_Hand.xlsx` | Physical bench grasp trials: outcomes, raw FSR readings and final encoder-derived joint angles | Paper Tables IV and V |

The same step-test data is also provided in MATLAB format as `matlab/scripts/escalon_motor2.mat`.

## Experimental context

**Motor characterization.** PWM levels of 0, 1000, 2000, 3000 and 4000 counts were applied for 2, 3, 3, 3 and 3 s, respectively. The dataset has 699 input–velocity pairs, with angular velocity from 0 to 5.0598 rad/s. Timestamps are MATLAB reception times over serial communication, not microcontroller acquisition times; uniform spacing of Ts = 0.02 s was assumed and actual acquisition intervals were not verified.

**FSR calibration.** Six known loads (masses of 130, 140, 150, 170, 180 and 230 g, approximately 1.27–2.26 N) were applied over the active sensing area. Readings were acquired with the 12-bit ADC of the microcontroller using a 1 kΩ voltage-divider resistor. The fit is F = a·N_ADC + b with a = 1.767 × 10⁻³ N/count, b = −1.087 N and R² = 0.982. It is a single ascending loading sequence; repeatability and hysteresis were not quantified, and the fit is valid only within the tested load range.

**Bench grasp trials.** Three grasp patterns (pinch, lateral pinch, cylindrical), three objects per pattern, five attempts per object (45 trials). An attempt was successful if the hand reached the intended configuration and retained the unsupported object for at least 10 s without visible slip, assessed visually.

| ID prefix | Grasp | Objects |
|---|---|---|
| PIN | Pinch | Phone box, PCB, USB drive |
| LAT | Lateral pinch | Phone charger, PCB, phone box |
| CIL | Cylindrical | Deodorant container, plastic bottle, aluminum bottle |

Object masses and dimensions are in Table II of the paper.

## Conventions and caveats

- FSR values are the maximum raw ADC reading (0–4095 counts) recorded over each grasp attempt. Zeros are recorded measurements, not missing data.
- Readings below the calibrated ADC range are **not** converted to force and are not force measurements. The highest recorded value was 623 counts, below the programmed 1500-count threshold.
- Joint angles are in degrees and derived from encoder counts relative to the extension reference posture. Distal joint angles are inferred from the four-bar constraint, not measured.
- Statistics in the paper are mean ± sample standard deviation over n = 5 attempts per object, including failed attempts. Index FSR values were not reported for lateral pinch.
- Objects were positioned by hand, so millimeter-scale placement variation is possible.
- Results apply only to the tested objects and conditions.
