# Plant Model — AKM11E Motor Only

Simulation model for controller design and gain tuning. Motor shaft only — no leadscrew or load.

**Not used on hardware.** Real ADC current feedback replaces this model when the current loop closes on the physical motor.

---

## Parameters

| Symbol | Value | Description |
|---|---|---|
| `kt` | 0.1125 Nm/A | Torque constant |
| `R` | 3.9 Ω | Winding resistance |
| `L` | 2.68 mH | Winding inductance |
| `J` | 1.7×10⁻⁶ kg·m² | Rotor inertia |
| `B` | 1×10⁻⁴ N·m·s/rad | Viscous damping |

---

## Derivation

### Current

Current loop assumed ideal — electrical dynamics handled by the current loop, not the plant. Instantaneous current:

```
i = v_q / R
```

### Torque

```
T = kt · i
```

### Mechanical (Newton-Euler)

```
J · dω/dt = T - B·ω
J · dω/dt = kt·i - B·ω
```

Solving for acceleration:

```
dω/dt = (kt·i - B·ω) / J
```

### Position

```
dθ/dt = ω
```

---

## Transfer Function

From voltage to angular velocity (current loop ideal, no electrical lag):

```
Ω(s)     kt / (R · J)
──── = ─────────────────
V(s)    s + B/J
```

From voltage to angular position:

```
Θ(s)       kt / (R · J)
──── = ─────────────────────
V(s)    s · (s + B/J)
```

Mechanical pole:

```
s = -B/J = -0.0001 / 1.7e-6 = -58.8 rad/s
```

---

## Discrete Implementation (Euler, dt = 50µs)

```c
i     = v_q / R
accel = (kt·i - B·ω) / J
ω    += accel · dt
θ    += ω · dt

pos_counts = θ · (8192 / 2π)
vel_counts = ω · (8192 / 2π)
```

---

## Notes

- L/R electrical lag not modeled — handled by the current loop on hardware
- Leadscrew and load inertia not included — to be added after system identification
- Mechanical pole at -58.8 rad/s (~9.4 Hz) — slow, dominated by low damping
