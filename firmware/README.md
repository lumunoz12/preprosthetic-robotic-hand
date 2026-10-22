# Embedded Firmware

STM32CubeIDE project for the embedded controller of the robotic hand, running on an **STM32F767ZI** development board. The firmware coordinates the ten actuated joints, runs discrete LQR position control from encoder feedback, executes the grasp sequences and applies threshold-based FSR supervision. Commands and telemetry are exchanged over UART.

## Folder contents

```text
firmware/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── cyl_targets.h            Cylindrical-grasp target computation (header)
│   │   ├── stm32f7xx_hal_conf.h
│   │   └── stm32f7xx_it.h
│   ├── Src/
│   │   ├── main.c                   Application: UART commands, homing, grasp sequences, control loop, telemetry
│   │   ├── cyl_targets.c            Cylindrical-grasp target computation
│   │   ├── stm32f7xx_hal_msp.c      Peripheral low-level initialization
│   │   ├── stm32f7xx_it.c           Interrupt handlers
│   │   ├── syscalls.c, sysmem.c
│   │   └── system_stm32f7xx.c
│   └── Startup/
│       └── startup_stm32f767zitx.s
├── Drivers/
│   ├── CMSIS/                       Arm CMSIS and ST device headers
│   └── STM32F7xx_HAL_Driver/        ST HAL and LL drivers
├── Debug/                           Build output of the Debug configuration (regenerated when building)
├── final.ioc                        STM32CubeMX configuration (pins, clocks, peripherals)
├── final Debug.launch               Debug launch configuration
├── STM32F767ZITX_FLASH.ld           Linker script (flash)
├── STM32F767ZITX_RAM.ld             Linker script (RAM)
├── .project, .cproject, .mxproject  STM32CubeIDE project files
├── .settings/                       STM32CubeIDE project settings
└── README.md
```

The files in `Drivers/` are distributed by ST and Arm under their own licenses; see the `LICENSE.txt` files in `Drivers/CMSIS/`, `Drivers/CMSIS/Device/ST/STM32F7xx/` and `Drivers/STM32F7xx_HAL_Driver/`.

`cyl_targets.c` and `cyl_targets.h` are also stored in `matlab/simulink/script/`. Keep both copies identical if either is modified.

## Requirements

- STM32CubeIDE **1.19.0**.
- STM32F767ZI development board.
- H-bridge motor drivers, ten N20 gearmotors with incremental encoders and two FSRs, as described in the paper.

## Build and flash

1. Open STM32CubeIDE and choose **File → Import → General → Existing Projects into Workspace**.
2. Select this `firmware/` folder as the root directory. The project is imported with the name `final`.
3. Build with the **Debug** configuration (**Project → Build Project**).
4. Connect the board and flash it with **Run → Debug** (uses `final Debug.launch`) or **Run**.

Peripheral and pin assignments are defined in `final.ioc`.

## Serial interface

USART3, **115200 baud, 8 data bits, no parity, 1 stop bit**, no flow control. Each command is a line terminated by `\n` (`\r` is ignored). Joint angles are in degrees and are limited to 0–90°.

| Command | Action |
|---|---|
| `home` | Homing of all ten axes (see below) |
| `reset` | Stops all motors, sets the current position of every axis as zero and cancels any active sequence |
| `i_mcp,<a>` / `i_pip,<a>` | Index MCP / PIP to angle `<a>` |
| `p_abd,<a>` / `p_mcp,<a>` | Thumb opposition-abduction / MCP to angle `<a>` |
| `m_mcp,<a>` / `m_pip,<a>` | Middle finger MCP / PIP |
| `a_mcp,<a>` / `a_pip,<a>` | Ring finger MCP / PIP |
| `q_mcp,<a>` / `q_pip,<a>` | Little finger MCP / PIP |
| `ik_i,<x>,<y>,<z>` | Index fingertip target [mm]; MCP and PIP solved on board (damped least squares) |
| `ik_p,<x>,<y>,<z>` | Thumb fingertip target [mm]; abduction solved on board, then MCP |
| `agarre_pinza` | Pinch grasp sequence |
| `agarre_lateral` | Lateral pinch grasp sequence |
| `agarre_cilindrico[,<R>]` | Cylindrical grasp sequence; `<R>` is the object radius passed to `cyl_compute_targets` (default 25) |

Unknown commands return `ERROR: comando invalido`.

### Telemetry

Every 100 ms the firmware sends one line with the target (`T`) and measured (`C`) angle of each axis, in degrees, and the raw ADC readings of both FSRs:

```text
I_MCP T:<deg> C:<deg> | I_PIP T:... C:... | P_ABD ... | P_MCP ... | M_MCP ... | M_PIP ... | A_MCP ... | A_PIP ... | Q_MCP ... | Q_PIP ... | FSR_I:<adc> FSR_P:<adc>
```

During homing, progress messages (`[HOME] ...`) are sent instead. `DONE` is sent when all moving axes have stopped and no grasp sequence is active.

Axis prefixes: `I` index, `P` thumb, `M` middle, `A` ring (*anular*), `Q` little finger (*meñique*).

## Homing

`home` drives each axis backward toward its extension stop at a fixed PWM of 1400 counts. An axis is considered at its stop when its encoder count changes by no more than 2 counts for 400 ms (800 ms for the thumb), and its count is then reset to zero. Axes are homed one at a time in this order: index MCP, index PIP, thumb MCP, thumb abduction, middle MCP and PIP, ring MCP and PIP, little MCP and PIP. The resulting zero is the extension reference posture used for all joint angles.

## Control implementation

| Item | Implementation in `main.c` |
|---|---|
| Control cycle | 20 ms elapsed-time check on `HAL_GetTick()` (nominal 50 Hz), with fixed Ts = 0.02 s in the calculations; actual cycle intervals were not characterized |
| Joint angle from encoder | θ = 360°·C / (4·Np·nr·nb), with Np = 7, nr = 298, nb = 2 (16,688 counts per nominal joint revolution) |
| Encoder decoding | 4× quadrature. Index MCP/PIP, thumb abduction/MCP and middle PIP use timers in encoder mode (TIM1, TIM2, TIM3, TIM4, TIM8); middle MCP, ring MCP/PIP and little MCP/PIP use external interrupts on both edges |
| Velocity estimate | ω̂ₖ = 0.7·ω̂ₖ₋₁ + 0.3·(θₖ − θₖ₋₁)/Ts |
| Reference trajectory | Trapezoidal profile per axis, maximum velocity 40°/s and acceleration 200°/s², generating position and velocity references |
| Controller | Discrete LQR state feedback, K = [2192.4  757.4] (from Q = diag(5000, 50), R = 0.001), same gains for all ten axes; no state observer |
| Control law | u* = 2192.4(θref − θ) + 757.4(ωref − ω̂), with angles in rad and velocities in rad/s |
| Output | u* limited to ±4095 counts; a constant compensation is added in the direction of the trajectory, and the final signed PWM is saturated to ±4095 counts (12-bit PWM, timer period 4095). Compensation: 1400 counts for all axes |
| Position-based stop | When the trajectory has finished and the absolute error to the target is below 1.5° |

## FSR supervision

| Sensor | ADC input | Effect when the reading exceeds 1500 counts |
|---|---|---|
| Index fingertip FSR | ADC2, channel 13 | Latched stop of index MCP and PIP |
| Thumb fingertip FSR | ADC3, channel 10 | Latched stop of thumb abduction and MCP |

Both FSRs are read once per control cycle (12-bit ADC). The stop flags are cleared by `home`, `reset` or a new grasp command. In the cylindrical grasp sequence, the index stop flag also freezes the middle, ring and little fingers at their current position; these fingers have no FSR of their own.

## Grasp sequences

Each grasp runs as a state machine; every stage starts when the previous axes have reached their targets or have been stopped by their FSR.

| Command | Sequence | Joint targets in this firmware [°] |
|---|---|---|
| `agarre_pinza` | Thumb abduction → thumb MCP → index MCP + PIP | Thumb ABD 70, thumb MCP 55, index MCP 85, index PIP 70 |
| `agarre_lateral` | Index MCP + PIP → thumb abduction → thumb MCP | Index MCP 75, index PIP 45, thumb ABD computed on board from the index posture, thumb MCP 80 |
| `agarre_cilindrico` | Thumb abduction → thumb MCP → MCP of all long fingers → PIP of all long fingers | Thumb ABD 80, thumb MCP 50, long-finger MCP from `cyl_compute_targets(R)`, all PIP 60 |

These are the targets stored in this version of `main.c`. Joint targets were adjusted during the bench tests, so they do not necessarily match the final joint angles reported in Table V of the paper.

## Limitations

- The FSRs inhibit motion; they do not regulate force or determine grasp success.
- Neither FSR reached the programmed threshold during the reported trials, so these experiments do not validate FSR-triggered stopping or closed-loop force regulation.
- Distal joint angles are inferred from the four-bar kinematic constraint and were not measured independently.
- No matched comparison with another position controller was performed.
