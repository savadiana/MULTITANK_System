# MULTITANK — Fuzzy Level Control of Three Interconnected Tanks

Automatic water-level control of the **INTECO MULTITANK** system using a **Mamdani fuzzy logic controller**, designed and validated in MATLAB/Simulink and tested on the physical rig.

The three tanks have **different geometries** and are coupled **in cascade**, so the plant is strongly **nonlinear**. The only actuator available is the **pump** (the valves are fixed), which makes the control problem the central challenge of the project.

---

## Overview

- **Plant:** three interconnected tanks (rectangular, trapezoidal, and round-bottomed), each with a maximum level of 35 cm.
- **Goal:** reach and hold user-defined reference levels in all three tanks despite their different shapes and the gravity-driven cascade coupling.
- **Manipulated variable:** pump flow rate (only the first tank is fed directly by the pump).
- **Controlled variables:** the three water levels, `H1`, `H2`, `H3`.
- **Controller:** Mamdani fuzzy inference system — chosen over classic approaches (relay, LQR) because it handles the nonlinearities directly, without a model linearized around an operating point.

---

## The Physical System

| Tank | Shape | Dimensions |
|------|-------|------------|
| 1 | rectangular (parallelepiped) | `H1max = 35 cm`, `w = 3.5 cm`, `a = 25 cm` |
| 2 | trapezoidal | `H2max = 35 cm`, short base `c = 10 cm`, long base `b = 34.5 cm`, `w = 3.5 cm` |
| 3 | round bottom | `H3max = 35 cm`, radius `R = 36.4 cm`, `w = 3.5 cm` |

The pump feeds **only tank 1**; water then flows from one tank to the next by gravity, through fixed valves. Because the cross-sections vary with height, each tank fills differently — this is the source of the nonlinearity. Tank 1 is effectively the "heart" of the system: if its reference is set too low, there is not enough flow for the downstream tanks to reach higher levels.

---

## Signal Acquisition

Everything runs in **MATLAB/Simulink** through the **RT-DAC/PCI** acquisition board and the rig driver (the `Tank3` block).

- Each tank has a **piezoresistive pressure transducer** that outputs a frequency signal proportional to the hydrostatic pressure, i.e. to the water level.
- The driver converts frequency to level (in meters) with a linear characteristic: `Level = Gain × (Freq − bias)`.
- The pump command is sent as a **PWM** signal.
- Simulation uses a **fixed-step** solver (`ode5`) with a sample time of **0.01 s**; real-time execution on the rig is generated via **Real-Time Windows Target**.
- Identification data was collected in the **Open-Loop** configuration (step input on the pump, recording the three level responses).

---

## Modeling & Identification

### Analytical model (starting point)
From Bernoulli's equation, the outflow of a tank is:

```
Q = A · sqrt(2 · g · h)
```

where `A` is the flow cross-section, `g = 9.81 m/s²`, and `h` is the water column height. This is only a reference point — the real behavior is nonlinear because of the variable cross-sections.

### Experimental model (used in practice)
A **step** of amplitude `q = 3·10⁻⁵` was applied to the pump flow. The responses are exponential and non-oscillatory, so each tank is approximated by a **first-order** model `H(s) = K / (T·s + 1)`:

| Tank | Transfer function | Note |
|------|-------------------|------|
| 1 | `Hf1(s) = 2420 / (37.8·s + 1)` | fills fastest (small `T`) |
| 2 | `Hf2(s) = 4220 / (75·s + 1)` | medium time constant |
| 3 | `Hf3(s) = 4450 / (142.5·s + 1)` | slowest (large `T`) |

`K` is the gain (how high the level rises for a given command) and `T` is the time constant (how fast the tank reacts).

---

## Fuzzy Controller

A **Mamdani** fuzzy inference system maps the three level errors to a single pump command. The pipeline is **fuzzification → rule inference → defuzzification**.

### Inputs and output
The controller inputs are the **errors**, not the levels:

```
error = reference − measured level
```

- positive error → level **below** target → **increase** the pump command
- negative error → level **above** target → **decrease** the command
- error ≈ 0 → level **at** target → **hold** the command

### Membership functions (Gaussian)
Gaussian membership functions are used for **smooth transitions** between fuzzy sets, which gives a smooth pump command without abrupt jumps.

| Variable | # of MFs | Labels |
|----------|----------|--------|
| Error 1 (tank 1) | 5 | Negative Large, Negative Small, Zero, Positive Small, Positive Large |
| Error 2 (tank 2) | 3 | Negative, Zero, Positive |
| Error 3 (tank 3) | 3 | Negative, Zero, Positive |
| Command (output) | 5 | Decrease Much, Decrease Little, Hold, Increase Little, Increase Much |

Error 1 has 5 MFs (fine tuning) because tank 1 has the largest impact on the whole system.

### Rule base — 45 rules
Rules have the form **IF … AND … AND … THEN …**, for example:

```
IF Error1 is Positive Large AND Error2 is Zero AND Error3 is Zero
THEN Command is Increase Much
```

The total of **45** rules comes from covering all relevant input combinations: `5 × 3 × 3 = 45`.

### Defuzzification — bisector method
The fired rules produce a single result area; the pump needs a single number. The **bisector** is the vertical line that splits that area into **two equal halves** (50% / 50%); the value where it falls becomes the command. It produces a **stable, median** output.

---

## Implementation

The control system is implemented in Simulink, in closed loop, in **two variants**:

1. **With the identified transfer functions** — validates the controller on the simplified process model.
2. **With the real rig model** (the `Tank3` block plus command saturation) — verifies the controller on a faithful model of the real process.

Key blocks: `Constant` (references), `Sum` (error computation), `Mux` (input vector), `Fuzzy Logic Controller`, `readfis` (loads the FIS matrix into the workspace), and `Scope` (visualization).

---

## Results

| Test | References | Result |
|------|-----------|--------|
| 1 — equal references | `0.1 m` on all tanks | steady-state error: tank 1 ≈ 0.06 m, tank 2 ≈ 0.1 m, tank 3 ≈ 0.14 m |
| 2 — coherent references | `0.096 / 0.166 / 0.172 m` | the position error disappears in simulation; all tanks reach their references |
| 3 — physical rig | (on the stand) | good but not perfect; small deviations |

**Takeaway:** the system performs well when the references are chosen **coherently** with the cascade dynamics. Equal references cause a position error because of the cascade coupling (tank 1 feeds everything); coherent references remove it. On the physical rig, small deviations appear, explained by real nonlinearities, friction, and the actual pump/valve characteristics.

---

## Requirements

- MATLAB / Simulink
- Fuzzy Logic Toolbox
- (For real-time operation on the rig) Real-Time Windows Target and the INTECO MULTITANK driver with an RT-DAC/PCI board

---

## How to Run (simulation)

1. Open MATLAB and set this repository as the working folder.
2. Load the fuzzy controller into the workspace, e.g.:
   ```matlab
   fis = readfis('fuzzy/controller.fis');
   ```
3. Open the Simulink model (identified-model variant or `Tank3` variant) and set the desired reference levels.
4. Run the simulation (fixed-step solver `ode5`, sample time `0.01 s`) and inspect the levels and command in the `Scope`.

> Adjust the file names above to match your actual model and FIS file names.

---



## License

Add a license of your choice (e.g. MIT) if you intend to share this project publicly.
