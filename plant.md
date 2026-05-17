# Plant Model — AKM11E Servo Axis

Derives the simplified plant model for the STM32-based servo axis.

**Signal chain:**
```
Voltage / PWM → motor current → torque → angular velocity → angular position → linear position
```

Intended for controller design, simulation, tuning intuition, and future system identification.

---

## 1. Physical System

- Kollmorgen AKM11E-ANCN2-00 servo motor
- Ball screw / leadscrew linear actuator
- Linear bearing guided carriage
- Incremental quadrature encoder (N2 option, 2048 CPR → 8192 counts/rev after 4x decode)
- STM32F446RE real-time control loop (bare metal, no HAL)
- PWM power stage (DRV8353RS-EVM)

Mechanical load modeled as reflected rotary inertia at the motor shaft.

---

## 2. Motor Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Torque constant | `kt` | 0.1125 Nm/A |
| Back EMF constant | `ke` | 7.24 V/krpm → **0.0691 V·s/rad** |
| Winding resistance | `R` | 3.9 Ω |
| Winding inductance | `L` | 2.68 mH → **0.00268 H** |
| Rotor inertia | `Jm` | 0.017 kg·cm² → **1.7×10⁻⁶ kg·m²** |

**Back EMF conversion:**
```
1 krpm = 1000 rev/min = 104.72 rad/s
ke = 7.24 / 104.72 = 0.0691 V·s/rad
```

---

## 3. Electrical Model

Motor electrical equation:
```
V = L·(di/dt) + R·i + ke·Ω
```

Laplace transform:
```
V(s) = (L·s + R)·I(s) + ke·Ω(s)
```

**Electrical pole:**
```
s = -R/L = -3.9 / 0.00268 = -1455 rad/s
```

**Electrical time constant:**
```
τe = L/R = 0.687 ms
```

---

## 4. Mechanical Model

Motor torque:
```
T = kt·i
```

Mechanical equation (Newton-Euler):
```
kt·i - B·Ω = Jeq·(dΩ/dt)
```

Laplace transform:
```
kt·I(s) - B·Ω(s) = Jeq·s·Ω(s)
```

---

## 5. Equivalent Inertia

```
Jeq = Jm + Jscrew + Jload
```

For a leadscrew with pitch `p` (m/rev) and carriage mass `M`:
```
Jload = M · (p / 2π)²
Jeq   = Jm + Jscrew + M·(p / 2π)²
```

> **Note:** Jscrew and load mass not yet characterized. `Jeq = Jm = 1.7×10⁻⁶ kg·m²` used for simulation — will be updated after system identification.

---

## 6. Voltage-to-Velocity Transfer Function

Substituting electrical and mechanical equations:

```
Ω(s) / V(s) = kt / [ (L·s + R)(Jeq·s + B) + ke·kt ]
```

---

## 7. Voltage-to-Linear-Position Transfer Function

Linear position from angular:
```
x = (p / 2π) · θ
Ω(s) = s·Θ(s)
X(s) = (p / 2π) · Ω(s) / s
```

**Full plant transfer function:**
```
Gp(s) = X(s) / V(s)

       (p / 2π) · kt
     = ─────────────────────────────────────────────
       s · [ (L·s + R)(Jeq·s + B) + ke·kt ]
```

---

## 8. Numerical Substitution

Known values:
```
kt  = 0.1125
ke  = 0.0691
R   = 3.9
L   = 0.00268
Jm  = 1.7e-6
ke·kt = 0.00777
```

```
Gp(s) = (p / 2π) · 0.1125
        / s · [ (0.00268·s + 3.9)(Jeq·s + B) + 0.00777 ]
```

---

## 9. Expanded Denominator

```
(L·s + R)(Jeq·s + B) = L·Jeq·s² + (L·B + R·Jeq)·s + R·B
```

Full denominator (with outer integrator):
```
D(s) = L·Jeq·s³ + (L·B + R·Jeq)·s² + (R·B + ke·kt)·s
```

**This is a third-order plant:**
1. Electrical pole — fastest
2. Mechanical pole — dominant
3. Position integrator

---

## 10. Pole Separation

```
Electrical pole:   s = -R/L   = -1455 rad/s   (fast)
Mechanical pole:   s = -B/Jeq              (slow, load dependent)
```

`R/L >> B/Jeq` in most cases — electrical dynamics are fast, mechanical dynamics dominate.

This pole separation is why nested loop architecture works:
- **Current loop** — closes around electrical pole, fastest (~20 kHz)
- **Velocity loop** — closes around mechanical pole, medium (~5 kHz)
- **Position loop** — outermost, slowest (~1 kHz)

---

## 11. Simplified Current-Controlled Plant

When the current loop bandwidth >> velocity loop bandwidth, the electrical dynamics are hidden inside the current loop. The plant seen by the velocity loop simplifies to:

```
Ω(s) / I(s) = kt / (Jeq·s + B)
```

For linear position:
```
X(s) / I(s) = (p / 2π) · kt / [ s·(Jeq·s + B) ]
```

With low damping (`B ≈ 0`):
```
X(s) / I(s) ≈ (p / 2π) · kt / (Jeq·s²)
```

**Double integrator** — standard result for a current-controlled servo with negligible friction.

---

## 12. Simulation Implementation

Two plant models implemented in firmware:

| File | Model | Use |
|---|---|---|
| `plant.c` | Mechanical only — no L/R lag, instantaneous current | Fast simulation, initial tuning |
| `plant_em.c` | Full electromechanical — L/R lag + back EMF | Accurate 2nd order behavior, current loop validation |

**`plant_em.c` state equations (discrete Euler, dt = 50µs):**
```
di/dt  = (v_q - R·i - ke·ω) / L       — electrical
dω/dt  = (kt·i - B·ω) / Jeq           — mechanical
dθ/dt  = ω                             — position integrator
```

---

## 13. Future Model Improvements

- Coulomb friction and stiction
- Screw and coupler compliance
- Backlash
- Structural resonance
- Encoder quantization noise
- PWM and ADC delay modeling
- Discrete-time ZOH effects
- System identification (sysid) to validate Jeq and B