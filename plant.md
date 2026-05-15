# Plant Model

This document derives the simplified plant model for the STM32-based servo axis.

The goal is to model the physical path:

Voltage / PWM command → motor current → motor torque → angular velocity → angular position → linear position

The model is intended for controller design, simulation, tuning intuition, and future system identification.

---

## 1. Physical System

The axis consists of:

- Kollmorgen AKM11E-ANCN2-00 servo motor
- Ball screw / leadscrew linear actuator
- Linear bearing guided carriage
- Encoder feedback
- STM32 real-time control loop
- PWM power stage

The mechanical load is modeled as a reflected rotary inertia at the motor shaft.

---

## 2. Motor Parameters

Known AKM11E motor parameters:

| Parameter | Symbol | Value |
|---|---:|---:|
| Torque constant | `kt` | 0.1125 Nm/A |
| Back EMF constant | `ke` | 7.24 V/krpm |
| Resistance | `R` | 3.9 ohm |
| Inductance | `L` | 2.68 mH |
| Rotor inertia | `Jm` | 0.017 kg-cm^2 |

Convert units:

```text
L  = 0.00268 H
Jm = 0.017 kg-cm^2 = 1.7e-6 kg-m^2
```

Back EMF conversion:

```text
1 krpm = 1000 rev/min = 104.72 rad/sec

ke = 7.24 / 104.72
ke = 0.0691 V-sec/rad
```

---

## 3. Electrical Model

The motor electrical equation is:

```text
V = L * di/dt + R*i + ke*Omega
```

Taking the Laplace transform:

```text
V(s) = (L*s + R)I(s) + ke*Omega(s)
```

Electrical pole:

```text
s = -R/L
```

Using motor values:

```text
s = -3.9 / 0.00268
s = -1455 rad/sec
```

Electrical time constant:

```text
tau_e = L/R
tau_e = 0.687 ms
```

---

## 4. Mechanical Model

Motor torque:

```text
T = kt*i
```

Mechanical equation:

```text
kt*i - B*Omega = Jeq * dOmega/dt
```

Laplace transform:

```text
kt*I(s) - B*Omega(s) = Jeq*s*Omega(s)
```

Rearrange:

```text
I(s) = ((Jeq*s + B) / kt) * Omega(s)
```

---

## 5. Equivalent Inertia

```text
Jeq = Jm + Jscrew + Jload
```

Leadscrew relationship:

```text
x = (p / 2*pi) * theta
```

Reflected load inertia:

```text
Jload = M * (p / 2*pi)^2
```

So:

```text
Jeq = Jm + Jscrew + M*(p / 2*pi)^2
```

Known:

```text
Jm = 1.7e-6 kg-m^2
```

---

## 6. Voltage-to-Velocity Transfer Function

Start with:

```text
V(s) = (L*s + R)I(s) + ke*Omega(s)
```

Substitute:

```text
I(s) = ((Jeq*s + B) / kt) * Omega(s)
```

Then:

```text
Omega(s) / V(s)
=
kt / [ (L*s + R)(Jeq*s + B) + ke*kt ]
```

---

## 7. Voltage-to-Linear-Position Transfer Function

```text
x = (p / 2*pi) * theta
```

```text
Omega(s) = s*Theta(s)
```

Therefore:

```text
X(s) = (p / 2*pi) * Omega(s) / s
```

Final plant:

```text
Gp(s) = X(s) / V(s)

Gp(s) =
(p / 2*pi) * kt
/
s[ (L*s + R)(Jeq*s + B) + ke*kt ]
```

---

## 8. Substitute Known Motor Values

Known:

```text
kt = 0.1125
ke = 0.0691
R  = 3.9
L  = 0.00268
Jm = 1.7e-6
```

So:

```text
Gp(s) =
(p / 2*pi) * 0.1125
/
s[ (0.00268*s + 3.9)(Jeq*s + B) + (0.0691)(0.1125) ]
```

Calculate:

```text
ke*kt = 0.00777
```

So:

```text
Gp(s) =
(p / 2*pi) * 0.1125
/
s[ (0.00268*s + 3.9)(Jeq*s + B) + 0.00777 ]
```

---

## 9. Expanded Denominator

Expand:

```text
(L*s + R)(Jeq*s + B)
=
L*Jeq*s^2 + (L*B + R*Jeq)s + R*B
```

Multiply by the outer integrator:

```text
D(s) =
L*Jeq*s^3
+
(L*B + R*Jeq)s^2
+
(R*B + ke*kt)s
```

So:

```text
Gp(s) =
(p / 2*pi) * kt
/
[
L*Jeq*s^3
+
(L*B + R*Jeq)s^2
+
(R*B + ke*kt)s
]
```

This is a third-order plant:
1. electrical pole
2. mechanical pole
3. position integrator

---

## 10. Pole Intuition

Electrical pole:

```text
s = -R/L
s = -1455 rad/sec
```

Mechanical pole:

```text
s = -B/Jeq
```

Usually:

```text
R/L >> B/Jeq
```

So:
- electrical dynamics are fast
- mechanical dynamics are slower and dominant

This is why nested servo loops work well:
- current loop -> fastest
- velocity loop -> medium
- position loop -> slowest

---

## 11. Simplified Current-Controlled Plant

If the current loop is closed and much faster than the velocity loop:

```text
Omega(s) / I(s)
=
kt / (Jeq*s + B)
```

For linear position:

```text
X(s) / I(s)
=
(p / 2*pi) * kt
/
[ s(Jeq*s + B) ]
```

If damping is very small:

```text
B ≈ 0
```

then:

```text
X(s) / I(s)
≈
(p / 2*pi) * kt
/
(Jeq*s^2)
```

This approximates a double integrator.

---

## 12. Future Model Improvements

Potential future additions:

- Coulomb friction
- static friction / stiction
- screw compliance
- coupler compliance
- backlash
- resonance
- encoder quantization
- PWM delay
- ADC delay
- discrete-time effects
- current-loop dynamics
- system identification