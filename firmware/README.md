# Embedded Firmware

STM32CubeIDE project for the embedded controller of the robotic hand, running on an **STM32F767ZI** development board. The firmware coordinates the ten actuated joints, runs discrete LQR position control from encoder feedback, and applies threshold-based FSR supervision.

## Folder contents

```text
firmware/
├── Core/
│   ├── Inc/                     # main.h, HAL configuration, interrupt headers, cyl_targets.h
│   ├── Src/                     # main.c, cyl_targets.c, HAL MSP, interrupts, system files
│   └── Startup/                 # startup_stm32f767zitx.s
├── Drivers/                     # STM32 HAL driver and CMSIS (ST, unmodified)
├── final.ioc                    # STM32CubeMX configuration
├── STM32F767ZITX_FLASH.ld       # linker script (flash)
├── STM32F767ZITX_RAM.ld         # linker script (RAM)
├── .project  .cproject  .mxproject  .settings/
└── README.md
```

Build output (`Debug/`, `.elf`, `.map`, `.list`) is not tracked.

## Requirements

- STM32CubeIDE (project created with the STM32CubeMX configuration in `final.ioc`).
- STM32F767ZI development board.
- H-bridge motor drivers, ten N20 gearmotors with incremental encoders and two FSRs, as described in the paper.

## Build and flash

1. Open STM32CubeIDE and choose **File → Import → General → Existing Projects into Workspace**.
2. Select this `firmware/` folder as the root directory and import the project.
3. Build with the **Debug** configuration (**Project → Build Project**).
4. Connect the board and flash it with **Run → Debug** or **Run**.

Peripheral and pin assignments are defined in `final.ioc`.

## Implemented behavior

| Item | Value |
|---|---|
| Communication | UART, 115200 baud: homing, individual joint positions, predefined grasp patterns; telemetry (reference angles, measured joint positions, FSR readings) |
| Control cycle | 20 ms elapsed-time check (nominal 50 Hz); actual cycle intervals were not characterized |
| Telemetry period | 100 ms |
| Joint angle from encoder | θ = 2πC / (4·Np·nr·nb), with Np = 7, nr = 298, nb = 2 (16,688 counts per nominal joint revolution) |
| Velocity estimate | Filtered finite difference: ω̂ₖ = 0.7·ω̂ₖ₋₁ + 0.3·(θₖ − θₖ₋₁)/Ts |
| Controller | Discrete LQR state feedback; Q = diag(5000, 50), R = 0.001; K = [2192.4  757.4]; same gains for all ten axes; no state observer |
| Control law | u* = 2192.4(θref − θ) + 757.4(ωref − ω̂) |
| Output limits | u* limited to ±4095 counts, plus a 1400-count compensation selected by the trajectory direction; final signed PWM saturated to ±4095 counts |
| Position-based stop | After trajectory completion, when absolute endpoint error is below 1.5° |
| FSR supervision | ADC reading above 1500 counts sets a latched stop flag (thumb and index only) |

The index flag stops its MCP and PIP actuators; the thumb flag stops its opposition and MCP actuators. The middle, ring and little fingers have no dedicated FSR-based stopping.

## Limitations

- The FSRs inhibit motion; they do not regulate force or determine grasp success.
- Neither FSR reached the programmed threshold during the reported trials, so these experiments do not validate FSR-triggered stopping or closed-loop force regulation.
- Distal joint angles are inferred from the four-bar kinematic constraint and were not measured independently.
- No matched comparison with another position controller was performed.
