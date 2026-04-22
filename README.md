# Predictive-Bilateral-Control-for-a-Powered-Ankle-Prosthesis
**Powered ankle prosthesis with real-time gait phase estimation and predictive torque control. Fuses IMU + FSR data on an ESP32-S2 at 100Hz, classifies gait into 4 biomechanical states via phase-portrait segmentation, and applies a 75ms predictive model to compensate system latency. Delivers feedforward torque via Moteus C1 over CAN-FD. 

# 🦾 Powered Ankle Prosthesis — Real-Time Adaptive Control

> A smart robotic ankle that predicts where you are in your walk, and pushes back at exactly the right moment.

## What Is This?

This project is a powered ankle prosthesis for below-knee amputees that:
- **Detects your gait phase** in real time using an IMU + heel pressure sensor
- **Predicts 75ms into the future** to compensate for system latency
- **Fires an assistive torque pulse** at the biomechanically correct push-off moment
- **Classifies terrain** (stairs, ramps, flat ground) with 85.9% accuracy using ML

No fixed parameters. No activity-specific tuning. Just continuous, adaptive control.

## Hardware Stack

| Component | Role |
|---|---|
| ESP32-S2 Mini | Main microcontroller (100 Hz control loop) |
| MPU6050 IMU | Ankle angle + angular velocity |
| FSR (heel-mounted) | Ground contact detection + phase reset |
| Moteus C1 | Brushless motor controller (CAN-FD) |
| MJ Bots BLDC Motor | 1.72 Nm direct torque |
| 36:1 Planetary Gearbox | ~44.6 Nm realistic ankle output |

The gearbox is a two-stage compound planetary inspired by the **Bilateral Drive Gear** (Matsuki et al.) — high reduction, compact, and backdrivable. It was 3D printed in PLA. The planet teeth fractured under heel-strike impact loading (PLA is too brittle for shock loads — lesson learned).

## How the Control Works

### 1. Sensing
- IMU data fused via **complementary filter** (α = 0.98) at 200 Hz
- Angular velocity computed from **central-difference derivative of filtered angle** — NOT raw gyro output
  - Why? The MPU6050 clips at ±250°/s. Raw gyro caused a broken 54%/45% Mid-Stance/Push-Off split. The fix restored the expected ~25% distribution across all 4 states.

### 2. Phase Portrait
- Ankle angle + velocity plotted together form a loop — one full loop = one stride
- Loop is mean-centered, normalized, and aligned so **φ = 0 always = push-off**
- Continuous phase: `φ = atan2(ω̂, θ̂)` → maps stride to `[-π, π]`

### 3. Gait State Machine

| State | Phase Range | What's Happening | Motor Command |
|---|---|---|---|
| Heel Strike | `[-π, -π/2)` | Foot hits ground | Transparent (kd=0.1) |
| Mid-Stance | `[-π/2, 0)` | Body over foot | Transparent (kd=0.1) |
| **Push-Off** | `[0, π/2)` | **Ankle pushes you forward** | **2.0 Nm feedforward** |
| Swing | `[π/2, π]` | Foot in the air | Return to neutral |

FSR triggers a hard phase reset to `φ = -π` at every heel strike, keeping the system anchored to real biomechanics.

### 4. Latency Compensation
Total system latency: **~75 ms** (≈ 8.4% of a stride at 67 steps/min).

The fix: predict 75ms ahead using phase velocity —
```
φ_predicted = atan2(sin(φ + p_vel × 0.075), cos(φ + p_vel × 0.075))
```
The FSM runs on the predicted phase, so torque arrives on time.

---

## Simulation & ML Validation

A **60.7-meter synthetic terrain course** was built in OpenSim with 7 activity segments:
flat ground → stairs up → platform → stairs down → flat → ramp up → ramp down

- Ground reaction force data from the **Camargo et al. (2021) EPIC Lab** dataset (22 subjects, force plates)
- Scaled to 75 kg model (735.75 N peak GRF)
- Cleaned with zero-phase Butterworth filter at 100 Hz

### Results

| Metric | Value |
|---|---|
| Terrain classification accuracy | **85.9%** (7 classes) |
| Ankle angle prediction RMSE | **3.91°** |
| Contralateral ankle mapping RMSE | 2.4° (R² = 0.87) |
| Best classified activities | Running (11/11), Normal walking (9/9) |

At every terrain transition, ML confidence drops to ~15% and recovers above 60% within **2–3 gait cycles (~1.5 s)**. The ankle stumbles briefly (±15°) then self-corrects — visible in simulation, manageable in practice.

**OpenSim MOCO** physics validation confirmed prosthesis peak dorsiflexion of **12.5°** during stair ascent, matching the synthetic prediction of **11.7°**.

---

## Known Issues & Fixes

| Issue | Cause | Fix |
|---|---|---|
| Gyro clipping → broken gait distribution | MPU6050 saturates at ±250°/s | Use central-difference derivative of filtered angle instead |
| FSR couldn't detect heel contact | Wrong pull-down resistor (10kΩ instead of 470Ω) | Replace with 470Ω — fix identified, not yet implemented |
| Planet gear teeth fractured | PLA too brittle for heel-strike shock loads | Reprint in PETG/Nylon or machine in aluminium |

---

## Repo Structure

```
/firmware        # ESP32-S2 gait estimation + CAN-FD control loop
/hardware        # Fusion 360 gearbox + prosthetic assembly files
/data            # Logged walking trials (timestamp, angle, velocity, phase)
```

## Built With

- **ESP32-S2 Mini** + Arduino/C++
- **Moteus C1** motor controller (Python `moteus` library)
- **OpenSim 4.x** + MOCO optimal control
- **scikit-learn** for terrain classification
- **Python** for data logging and ML pipeline


## References

- Matsuki, Nagano & Fujimoto — *Bilateral Drive Gear* (high-reduction backdrivable planetary)
- Camargo et al. (2021) — [EPIC Lab Open-Source Biomechanics Dataset](https://github.com/epiCENTER-Georgia-Tech)
- OpenSim Gait2392 musculoskeletal model (23 DOF, 92 muscles)