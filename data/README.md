# Experimental Data

Original datasets behind the experimental characterization reported in the paper.

## Files

| File | Content | Used for |
|---|---|---|
| `escalon_motor2.csv` | Step-response test of an isolated N20 gearmotor: time, PWM input and encoder-derived angular velocity | Motor model identification and LQR design (paper Eqs. 4, 5, 7, 8) |
| `FSR.xlsx` | Force-sensitive resistor calibration with known loads, and the linear fit | FSR calibration fit (paper Eq. 3) |
| `Test_Hand.xlsx` | Physical bench grasp trials: test objects, outcomes, peak raw FSR readings, final encoder-derived joint angles and per-object statistics | Paper Tables II, IV and V |

Related data stored in [`../matlab/script`](../matlab/script):

- `escalon_motor2.mat`: the same step-test data in MATLAB format.
- `datos_motor.mat`: incremental encoder vs. AS5600 comparison (paper Sec. III-A).

## File formats

**`escalon_motor2.csv`.** Semicolon-separated, with a comma as decimal separator. Columns: `time (s)`, `pwm` (counts) and `w (rad/s)`.

**`FSR.xlsx`.** Applied mass (g), equivalent force (N) and ADC reading for two voltage-divider resistors (10 kΩ and 1 kΩ), in sheet `Data`. Sheet `Fit` computes the slope, intercept and R² of the reported fit from the 1 kΩ column.

**`Test_Hand.xlsx`.** One sheet per item:

| Sheet | Content |
|---|---|
| `Objects` | Object ID, grasp, object, mass (g), dimensions (mm) and stiffness (`D` = ductile, `R` = rigid) |
| `Test_Pinch` | Pinch trials: attempt, object, success (1) or failure (0), peak FSR readings, final index MCP/PIP and thumb ABD/MCP angles, observations |
| `Test_Lateral` | Lateral pinch trials, same columns as `Test_Pinch` (without observations) |
| `Test_Cylindrical` | Cylindrical grasp trials, same columns plus the final MCP/PIP angles of the middle, ring and little fingers |
| `Stats` | Per-object successes, and mean and sample standard deviation of the FSR readings and of the four final index/thumb angles, computed with formulas from the trial sheets (paper Tables IV and V) |

Column headers are in Spanish (`Intento` = attempt, `Objeto` = object, `Éxito/Fracaso` = success/failure). The middle, ring and little finger angles in `Test_Cylindrical` are not reported in the paper.

## Experimental context

**Motor characterization.** PWM levels of 0, 1000, 2000, 3000 and 4000 counts were applied for 2, 3, 3, 3 and 3 s, respectively. The dataset has 699 input–velocity pairs, with angular velocity from 0 to 5.0598 rad/s. Timestamps are MATLAB reception times over serial communication, not microcontroller acquisition times; uniform spacing of Ts = 0.02 s was assumed and actual acquisition intervals were not verified.

**FSR calibration.** Six known loads (masses of 130, 140, 150, 170, 180 and 230 g, approximately 1.27–2.26 N) were applied over the active sensing area. Readings were acquired with the 12-bit ADC of the microcontroller. The 1 kΩ divider resistor was selected, giving F = a·N_ADC + b with a = 1.767 × 10⁻³ N/count, b = −1.087 N and R² = 0.982. It is a single ascending loading sequence; repeatability and hysteresis were not quantified, and the fit is valid only within the tested load range.

**Bench grasp trials.** Three grasp patterns (pinch, lateral pinch, cylindrical), three objects per pattern, five attempts per object (45 trials). An attempt was successful if the hand reached the intended configuration and retained the unsupported object for at least 10 s without visible slip, assessed visually.

| ID prefix | Grasp | Objects |
|---|---|---|
| PIN | Pinch | Phone box, PCB, USB drive |
| LAT | Lateral pinch | Phone charger, PCB, phone box |
| CIL | Cylindrical | Deodorant container, plastic bottle, aluminum bottle |

## Conventions and caveats

- FSR values are the maximum raw ADC reading (0–4095 counts) recorded over each grasp attempt. Zeros are recorded measurements, not missing data.
- Readings below the calibrated ADC range are **not** converted to force and are not force measurements. The highest recorded value was 623 counts, below the programmed 1500-count threshold.
- Index FSR values were not recorded for lateral pinch (`-` in `Test_Lateral`).
- Joint angles are in degrees and derived from encoder counts relative to the extension reference posture. Distal joint angles are inferred from the four-bar constraint, not measured.
- Statistics in the paper are mean ± sample standard deviation over n = 5 attempts per object, including failed attempts.
- Objects were positioned by hand, so millimeter-scale placement variation is possible.
- Results apply only to the tested objects and conditions.
