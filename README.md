# BSRT Kalman Filters  
**Real-time state estimation for amateur rocket avionics**

## Overview

This repository implements a **Kalman Filter–based state estimation pipeline** for amateur rocketry avionics, developed as part of the Bishop Strachan Rocketry Team (BSRT).

The goal of this project is twofold:

1. **Educational:** provide a clear, beginner-friendly introduction to Kalman filtering for team members with little to no background in control systems.
2. **Practical:** implement a **real-time Extended Kalman Filter (EKF)** suitable for flight use, operating under noisy sensors, vibration, and tight timing constraints.

To support this, the repository is intentionally structured in two stages:
- a **1D Kalman Filter** for learning and intuition
- a **full EKF** used for flight-relevant state estimation

---

## Why Kalman Filters?

In high-power rocketry, raw sensor readings are:
- noisy
- biased
- sampled at different rates
- occasionally unreliable (especially during boost)

A Kalman Filter is a control algorithm which provides a principled way to:
- fuse multiple sensors
- track hidden states (e.g., velocity)
- quantify uncertainty
- remain computationally lightweight enough for embedded hardware

---

## Repository Structure

### 1. 1D Kalman Filter (Educational)

The 1D Kalman Filter serves as an **introductory stepping stone** for new team members.

It models a simplified vertical system with:
- **state:** altitude
- **measurement:** barometric altitude
- **assumption:** constant vertical velocity over short intervals

This implementation is intentionally minimal and heavily commented, focusing on:
- prediction vs update steps
- covariance intuition
- tuning process noise vs measurement noise
- understanding filter behavior through plots

This section exists so that future team members can learn *what a Kalman Filter is doing* before encountering Jacobians or nonlinear dynamics.

---

### 2. Extended Kalman Filter (Flight-Oriented)

The EKF implementation extends the 1D filter to a **nonlinear, multi-state system** appropriate for real flight data.

#### State Vector
Typical states include:
- altitude
- vertical velocity

#### Sensors
- barometric pressure sensor (altitude)
- IMU accelerometer (vertical acceleration)

#### Key Characteristics
- nonlinear state transition model
- sensor fusion with different noise profiles
- covariance propagation and update
- designed with embedded constraints in mind (runtime, memory)

This EKF is intended to run on a microcontroller-based flight computer and serve as the foundation for:
- apogee detection
- event timing (e.g., deployment logic)
- post-flight analysis

---

## System Design Considerations

This implementation was designed with real-world constraints in mind:

- **Sensor noise:** accelerometer vibration during boost, barometer lag
- **Timing:** predictable execution time per update step
- **Robustness:** reasonable behavior during brief sensor dropouts
- **Simplicity:** no matrix libraries that would be infeasible on embedded hardware

The filter prioritizes **reliable state trends** over aggressive responsiveness.

---

## Results & Validation*

The filter has been tested on:
- simulated flight profiles
- logged sensor data
- injected noise scenarios to observe stability and convergence

Plots and logs demonstrate:
- reduced noise compared to raw measurements
- smooth velocity estimates
- stable covariance evolution

*All results produced in simulation, not through genuine test-flights. The complete, SRAD-based avionics system is currently being engineered for payload-testing for Launch Canada 2026. 
---

## Educational Intent

This repository is designed to be **read**, not just run.

It is actively used as a learning reference for:
- high school rocketry students
- first-time exposure to estimation theory
- understanding how theory translates into embedded systems

Code clarity and progression were prioritized over abstraction.

---

## Future Work

Planned extensions include:
- tighter integration with full flight software
- sensor bias estimation
- adaptive noise tuning
- validation on flight hardware
- complete integration of srad systems (removal of all COTS pyrotechnics, and otherwise)

---

## Acknowledgements

Developed as part of the Bishop Strachan Rocketry Team (BSRT) avionics effort for the Tripoli Competition '25, Launch Canada 2025 and thereafter.
