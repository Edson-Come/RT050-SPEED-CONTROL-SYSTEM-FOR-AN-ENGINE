# RT050 Motor Speed Control — MATLAB / PID Controller

A MATLAB-based speed control system for the RT050 DC motor rig, covering open-loop characterisation, closed-loop discrete PID control, and frequency response analysis. The system interfaces with the RT050 hardware via LabJack DAQ using a set of provided precompiled driver functions.

Developed as Practical Work 2 for the Control Systems course — University of Aveiro (DETI).

---

## Table of Contents
- [About the Project](#about-the-project)
- [System Model](#system-model)
- [Part 1 — Open-Loop Analysis](#part-1--open-loop-analysis)
- [Part 2.1 — Closed-Loop PID (Fixed Reference)](#part-21--closed-loop-pid-fixed-reference)
- [Part 2.2 — Closed-Loop PID (Variable Reference)](#part-22--closed-loop-pid-variable-reference)
- [Part 3 — Frequency Response](#part-3--frequency-response)
- [PID Parameters](#pid-parameters)
- [Hardware Interface](#hardware-interface)
- [File Structure](#file-structure)
- [Getting Started](#getting-started)
- [Author](#author)

---

## About the Project

This project analyses and controls the rotational speed of the RT050 DC motor across three tasks. First, the open-loop step response is measured to identify the plant time constant. Then, a discrete-time PID controller is implemented in closed loop to track both fixed and time-varying speed references. Finally, sinusoidal and square wave inputs are applied to study the frequency response of the motor.

---

## System Model

The RT050 motor is modelled as a first-order transfer function:

```
        K
G(s) = -----
       tau*s + 1
```

Parameters identified from the open-loop step response:

| Parameter | Value | Description |
|---|---|---|
| Steady-state speed at 4V | 3073.24 RPM | Speed before voltage drop |
| Step input | -3 V (4V to 1V) | Applied at t = 0 |
| Time constant (tau) | 8.1 s | Speed decays 63.2% of total variation |
| Speed at t = 30 s | 612.3 RPM | Observed steady-state at 1V |

Step response equation used:

```
y(t) = -3K + 3K * exp(-t / tau)    ->    y(t) = -1.896K at steady state
```

---

## Part 1 — Open-Loop Analysis

**File**: `Trabalho_Pratico2.m` (first section)

Stabilises the motor at 4V for 15 s, then drops to 1V and records the speed decay over 30 s at T = 0.1 s sampling period. The time constant tau is estimated as the time at which speed has fallen by 63.2% of the total variation.

```matlab
tensao = 4;
T = 0.1;               % sampling period (s)
tempoTotal = 30;

RT050_SetMotorVoltage(4);
pause(15);             % allow motor to reach steady state

RT050_SetMotorVoltage(1);
for i = 1:length(tempo)
    tic;
    velocidade_rotacao(i) = RT050_GetMotorSpeed;
    while toc < T, end
end

% Estimate tau from 63.2% decay point
saida_final = velocidade_rotacao(end);
percentual_632 = 1.896 * saida_final;
[~, idx_tau] = min(abs(velocidade_rotacao - percentual_632));
tau = tempo(idx_tau);
```

**Output**: Figure 1 — speed decay curve with tau marker at 8.1 s and 63.2% reference line.

---

## Part 2.1 — Closed-Loop PID (Fixed Reference)

**File**: `Trabalho_Pratico2.m` (second section)

Implements a discrete-time PID controller to drive the motor to a fixed reference of 4000 RPM and hold it there for 30 s.

The discrete PID control law:

```
v(k) = Kp * e(k)  +  Ki * T * sum(e(k))  +  (Kd / T) * (e(k) - e(k-1))
```

Error is computed in normalised units using K = 1/5000:

```matlab
Es(i+1) = Xs*K - Ys*K;
Es_i    = Es_i + Es(i+1)*T;                          % integral accumulator
Vs      = Kp*Es(i+1) + Ki*Es_i + Kd*(Es(i+1)-Es(i))/T;
RT050_SetMotorVoltage(Vs);
```

**Results**:
- Reference: 4000 RPM
- Settling time: ~10 s
- Steady-state at t = 30 s: 4001.95 RPM
- Overshoot: minimal

**Output**: Figure 2 — closed-loop speed response to 4000 RPM step.

---

## Part 2.2 — Closed-Loop PID (Variable Reference)

**File**: `Trabalho_Pratico2.m` (third section)

Same PID gains applied over 80 s with a time-varying reference:

| Time (s) | Reference (RPM) |
|---|---|
| 0 -- 20 | 2000 |
| 20 -- 40 | 3000 |
| 40 -- 60 | 2500 |
| 60 -- 80 | 0 |

**Output**: Figure 3 — closed-loop speed tracking across multiple setpoints.

---

## Part 3 — Frequency Response

**File**: `RT050TrabalhoPratico.m`

The reference voltage is computed from the student number digits:

```matlab
Xmed = (1 + 1 + 5 + 6 + 4 + 0) / 6;
refVoltage = (Xmed / 2) + 2;    % = 4.83 V
```

The motor is stabilised at `refVoltage`, then two input signals are applied in sequence over 30 s at T = 0.1 s:

**Sinusoidal input**:
```matlab
tensao = refVoltage + 0.8 * sin(2 * pi * 1 * tempo(i));
```
Amplitude and phase of the speed response are computed. Phase is extracted using the Hilbert transform.

**Square wave input**:
```matlab
tensao = refVoltage + 0.6 * square(2 * pi * 0.5 * tempo(i));
```

**Outputs**: Amplitude response plot, phase response plot, and square wave comparison plot.

---

## PID Parameters

Gains were tuned by trial and error, prioritising minimum overshoot and stable settling over a faster rise time.

| Parameter | Value | Role |
|---|---|---|
| `Kp` | 5 | Proportional — increases initial response, reduces settling time |
| `Ki` | 2 | Integral — kept low to avoid windup |
| `Kd` | 6 | Derivative — reduces oscillations |
| `T` | 0.15 s | Sampling period |
| `K` | 1/5000 | Motor normalisation gain |
| `Xs` | 4000 RPM | Fixed reference speed (Part 2.1) |

---

## Hardware Interface

The RT050 rig interfaces with MATLAB through a LabJack DAQ. Three precompiled `.p` functions and associated `.dll` / `.mexw64` binaries are required at runtime.

| Function | Description |
|---|---|
| `RT050_SetMotorVoltage(V)` | Applies voltage V (volts) to the motor via the DAQ analog output |
| `RT050_GetMotorSpeed` | Returns current motor speed in RPM from the encoder via analog input |
| `RT050_SetLoadState(s)` | Sets the mechanical load state on the motor shaft |

Supporting binaries needed in the MATLAB path:

| File | Purpose |
|---|---|
| `EAnalogIn.dll` / `.mexw64` | LabJack analog input driver |
| `EAnalogOut.dll` / `.mexw64` | LabJack analog output driver |
| `EDigitalIn.dll` / `.mexw64` | LabJack digital input driver |
| `EDigitalOut.dll` / `.mexw64` | LabJack digital output driver |
| `ljackuw.dll` | LabJack core driver library |

---

## File Structure

```
rt050-motor-control/
├── Trabalho_Pratico2.m             -- Open-loop analysis + closed-loop PID (Parts 1, 2.1, 2.2)
├── RT050TrabalhoPratico.m          -- Frequency response analysis (Part 3)
├── RT050_GetMotorSpeed.p           -- Precompiled: reads motor speed (RPM)
├── RT050_SetMotorVoltage.p         -- Precompiled: sets motor voltage
├── RT050_SetLoadState.p            -- Precompiled: sets load state
├── EAnalogIn.dll / .mexw64         -- LabJack analog input driver
├── EAnalogOut.dll / .mexw64        -- LabJack analog output driver
├── EDigitalIn.dll / .mexw64        -- LabJack digital input driver
├── EDigitalOut.dll / .mexw64       -- LabJack digital output driver
├── ljackuw.dll                     -- LabJack core driver
├── fig1.fig / fig2.fig / fig3.fig  -- Saved MATLAB figures (open-loop results)
├── figura1.fig -- figura4.fig      -- Saved MATLAB figures (closed-loop results)
├── 115640_RT050.pdf                -- Project presentation slides
└── README.md
```

---

## Getting Started

### Requirements
- MATLAB R2018b or later
- Signal Processing Toolbox (for `hilbert` and `square` functions)
- LabJack U3 or U6 hardware connected via USB
- RT050 motor rig

### Setup
1. Place all `.p`, `.dll`, and `.mexw64` files in the same directory as the `.m` scripts, or add the directory to the MATLAB path:
```matlab
addpath('path/to/project/folder');
```
2. Connect the RT050 rig and LabJack DAQ via USB
3. Open `Trabalho_Pratico2.m` in MATLAB

### Running

**Open-loop characterisation (Part 1)**:
```matlab
% Run the first section of Trabalho_Pratico2.m
% Motor stabilises at 4V, then drops to 1V
% tau is printed to the console and marked on Figure 1
```

**Closed-loop fixed reference (Part 2.1)**:
```matlab
% Run the second section of Trabalho_Pratico2.m
% Motor ramps to 4000 RPM and holds
```

**Closed-loop variable reference (Part 2.2)**:
```matlab
% Run the third section of Trabalho_Pratico2.m
% Motor tracks 2000 -> 3000 -> 2500 -> 0 RPM
```

**Frequency response (Part 3)**:
```matlab
% Run RT050TrabalhoPratico.m
% Sinusoidal then square wave inputs are applied
% Amplitude and phase plots are generated
```

> Allow the motor to fully stop (RT050_SetMotorVoltage(0) + pause) between each experiment to avoid residual speed affecting the next measurement.

---

## Author

- Edson Come — 115640

**Course**: Sistemas de Controlo — LEEComp, University of Aveiro
