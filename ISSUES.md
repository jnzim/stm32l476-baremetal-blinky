# Known Issues

## Control Loop

- **Position loop no feedforward** — will lag on fast profiles. Add vel_cmd feedforward from trajectory sample before hardware bring-up.
- **PLANT_B artificially high** — set to 0.001 to prevent instability during sim tuning. Not realistic for AKM11E. Tune to real motor specs once hardware available.
- **Velocity loop units inconsistent** — vel_cmd is in rad/s internally but vel_fbk is logged in counts/sec. Confusing for analysis. Standardize to one or the other.

## Telemetry / Pi

- **telem cmd x-axis offset** — Pi starts logging slightly late, t0 alignment is approximate. First ~100 samples consumed before logging begins.
- **vel_fbk resolution** — int16_t limits velocity feedback range. May need scaling factor for high speed moves.

## Hardware (not started)

- **No fault handling** — DRV8353RS nFAULT pin not monitored. Must be wired and handled before any real PWM output.
- **Encoder cable** — need to find or make cable before encoder bring-up.
- **No alignment pulse** — AKM11E has no Hall sensors (N2 option). Rotor alignment pulse needed at startup before FOC can run.
- **FOC not implemented** — Park/Clarke transforms, d/q axis current control not written yet.
- **PWM output not wired** — TIM1 complementary outputs not connected to DRV8353RS yet.

## Build / Toolchain

- **VS Code CMake Tools stomps toolchain** — must build from terminal, not F5 or CMake Tools UI. Documented in README.

## Resolved

- **sim_active startup transient** — fixed with state machine, plant_init() on STATE_ENABLED entry
- **No state machine** — drive.c implemented, IDLE → ENABLED on BLOCK_HDR