# Way Back Home

**Way Back Home** is a project developed for the Mobile Robotics course (FIUBA, 2025). It simulates a mobile robot placed at a random start location, tasked with estimating its pose and autonomously planning a path to the LAR (Laboratory of Automation & Robotics), using a **particle filter with likelihood fields** for localization and **A\*** for path planning. The full implementation is in **MATLAB**, integrated within the course’s provided simulator framework.

---

## 🧭 Overview

The objective is to demonstrate how a robot can:
- **Localize** itself on the map starting from an unknown position using a particle filter enhanced with likelihood fields.
- **Plan** an optimal route to a fixed destination (the LAR) using the A\* algorithm.
- **Navigate** using a hybrid motion system that follows the planned path while avoiding dynamic or unmapped obstacles through **reactive behavior**.

---

## 📦 Hardware Platform

The algorithm was successfully tested on a **real robot**, using the hardware setup provided by the course:

- **Base**: Kobuki differential drive platform (Roomba-type)
- **Computer**: Embedded Raspberry Pi
- **Sensors**:
  - **LIDAR**: Hokuyo URG-04LX-UG01 (180° field of view, 171 points per scan at ~10Hz)
  - **Encoders**: Differential wheel encoders for odometry (dead-reckoning)
- **Motion**:
  - Linear velocity range: `-0.5 to 0.5 m/s` (recommended: ±0.15 m/s)
  - Angular velocity range: `-4.25 to 4.25 rad/s` (recommended: ±0.5 rad/s)
  - Sample time: 0.1 s
- **Sensor placement**:
  - LIDAR mounted at 20 cm height, located at (x = 0.07 m, y = 0 m) in robot frame
  - Oriented forward (0 rad), with readings from -π/2 to π/2

> The simulation and control code maintain compatibility with this hardware configuration, ensuring a smooth transition from simulator to real-world validation.

---

## 🧠 Methodology

### 1. Localization: Particle Filter with Likelihood Fields

- A set of particles is initialized randomly across the map.
- Each particle simulates a LiDAR scan, which is compared to real sensor data using a precomputed likelihood field.
- Particles are weighted and resampled with noise based on similarity to actual readings.

## 2. Localization Confirmation

- Once the particle filter has converged — indicated by a low variance in particle distribution — the estimated pose is considered a candidate for localization.
- To validate this estimate, a raycasting operation simulates LiDAR beams from the estimated pose. These simulated measurements are then compared to the actual LiDAR readings.
- If the similarity between real and simulated scans exceeds a predefined threshold, the localization is confirmed and path planning can proceed. If not, the particle filter is reinitialized from scratch, discarding the current particle set, and the robot continues exploring reactively.

### 3. Path Planning: A* Search

- Once localization is considered reliable, the A* algorithm computes the shortest path to the LAR.
- The map is treated as a grid; it uses euclidean distance heuristic to optimize the path.

### 4. Navigation: Path Following + Reactive Avoidance

- The robot follows the planned path using a differential drive controller.
- In addition, it employs **reactive behaviors** to avoid **unmapped or dynamic obstacles**.
- This hybrid control ensures safe navigation even in partially known environments.

---

## 📁 Repository Structure

```text
Way‑Back‑Home/
|
├── main.m # Main simulation script
|
├── localization/ # Particle filter + likelihood fields implementation
|
├── planning/ # A* path planning module
|
├── movement/ # Differential-drive + reactive movement functions
|
└── maps/ # Map files (e.g., .mat, .tif) used in the simulator
```

---

## ⚙️ Requirements & Setup

### Requirements
- **MATLAB R2016b** or later (no additional toolboxes required).
- Compatible with the course-provided simulator; hardware integration is optional via `use_roomba`.

### Usage Instructions

```bash
git clone https://github.com/felipeperassi/Way-Back-Home.git
```

Then, in MATLAB:

```bash
cd 'Way-Back-Home'
main  #runs the simulation
```

By default, the main script runs in simulation mode. To prepare for hardware (if desired):

```bash
use_roomba = true;
```

> Note: hardware mode has been tested with the Kobuki + Hokuyo + Raspberry Pi robot setup.

---

## 👨‍💻 Authors

- **Felipe Perassi**
- **Maximiliano Rodríguez**

---

## 📄 License

This project is licensed under the MIT License — see the `LICENSE` file.
