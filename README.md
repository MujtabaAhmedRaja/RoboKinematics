<div align="center">
  <h1> RoboKinematics Engine</h1>
  <p><strong>A high-performance, containerized 3D Kinematics mathematical engine and physical Hardware Abstraction Layer for N-Degree-of-Freedom robotic arms.</strong></p>
</div>

---

## 📜 Table of Contents

- [Overview](#-overview)
- [Core Architecture](#-core-architecture)
- [Key Features](#-key-features)
  - [Mathematical Solvers](#mathematical-solvers)
  - [3D WebGL Visualization](#3d-webgl-visualization)
  - [Advanced Engineering Toolkit](#advanced-engineering-toolkit)
- [Hardware Integration SDK](#-hardware-integration-sdk)
- [Installation & Deployment](#-installation--deployment)
  - [Docker (Production)](#docker-production)
  - [Local Development](#local-development)
- [API Documentation](#-api-documentation)

---

## 🔬 Overview

**RoboKinematics Engine** bridges the gap between robotic mathematical simulation and physical hardware deployment. 

Unlike standard static IK solvers, this project pairs a fully dynamic Python backend (powered by Scipy spatial optimizations) with a sub-millisecond Vanilla JS / **Three.js** frontend. This allows robotics engineers to dynamically build, simulate, visualize, and physically export structural routines for robotic arms with complete control over Denavit-Hartenberg (DH) parameters.

---

## 🏗 Core Architecture

The repository is modularized into two distinct runtimes communicating over an asynchronous `WebSocket` pipeline:

1. **The Kinematics Engine (Backend)**: Built on Python 3.10 and FastAPI. Contains the mathematical core manipulating transformation matrices, bounding box spatial physics, and computing `L-BFGS-B` Non-Linear Inverse Kinematic tracking.
2. **The Visualizer (Frontend)**: A heavily optimized, dependency-free (vanilla JS) dashboard utilizing WebGL `Three.js` arrays to map backend physical arrays into a glowing, 60-FPS simulated laboratory environment.

---

## 🚀 Key Features

### Mathematical Solvers
- **Universal N-DOF Generation**: Instantly spawn 2-DOF planar arms or 6-DOF spatial manipulators by specifying real-world DH structural limits (`a`, `d`, `alpha`, `theta_offset`).
- **Forward Kinematics (FK)**: Processes instantaneous $T_0^N$ transformation matrices.
- **Inverse Kinematics (IK)**: Fallback hybrid solver. Uses pure analytical geometry for standardized 2-DOF/3-DOF manipulators, then transitions automatically to high-speed Scipy Optimization for complex spatial coordinates, completely respecting physical joint bounds to prevent self-intersection.

### 3D WebGL Visualization
- **Collision Physics Constraints**: Integrated object intersections! Using sphere-to-line mathematical algorithms, the UI will proactively block the arm from entering solid 3D space, reverting limits instantly and flashing emissive red on mechanical impact.
- **Interactive UI**: Orbit controls, dynamic lighting, lockable joints, dynamic link modification sliders, and End-Effector XYZ coordinate inputs.

### Advanced Engineering Toolkit
- **Monte Carlo Workspace Envelopes**: Run a 3,000-point stochastic kinematic simulation directly in the browser to plot a translucent point-cloud of the absolute mechanical reaching limits of your specific arm.
- **Parametric Trajectories (Path Math)**: Inject native `Math.sin(t)` equations mapping $X/Y/Z(t)$ functions. The engine will evaluate and route continuous multi-frame IK trajectories (like an automated factory Pick-and-Place line).
- **Ghost Targeting**: A translucent secondary mechanical arm indicates the instantaneous numerical target ahead of the visualization interpolation.
- **Matrix Extraction**: View exact Roll/Pitch/Yaw metrics parsed live from the spatial Quaternions.
- **CSV Data Exports**: 1-click download of the complete standard Denavit-Hartenberg geometric configuration array.

---

## 🧰 Hardware Integration SDK

One of the most powerful features of *RoboKinematics* is its ability to be stripped natively from the browser and imported directly into physical Python environments (like a Raspberry Pi wired to Servos).

We have structured the `backend/` directory as a standard Python module constraint.

### Installation

```bash
cd backend
pip install -e .
```

### SDK Implementation

By inheriting the included `AbstractHardwareController` base-class, your mathematical IK coordinates can instantly trigger serial protocols:

```python
from kinematics import RobotArm, AbstractHardwareController
import numpy as np

class PhysicalArmController(AbstractHardwareController):
    def write_angles_to_motors(self, angles):
        # `angles` is an array of radians: e.g., [1.57, -0.4, 0.1]
        degrees = np.degrees(angles)
        
        # Example: Transmit to Arduino via pyserial
        # serial_port.write(f"{degrees[0]},{degrees[1]},{degrees[2]}\n".encode())
        print(f"Sent {degrees} to hardware servos!")

# 1. Provide your DH constraints exported from the Web Dashboard
my_arm = RobotArm([
    {'theta_offset': 0, 'd': 2.5, 'a': 0, 'alpha': np.pi/2},
    {'theta_offset': 0, 'd': 0, 'a': 3.5, 'alpha': 0},
    {'theta_offset': 0, 'd': 0, 'a': 3.0, 'alpha': 0}
])

# 2. Bind the relay
relay = PhysicalArmController(my_arm)

# 3. Instruct the robot
# Instantly calculates IK and transmits pulses to physical motors.
relay.move_to_ik_target(x=3.0, y=4.0, z=1.0)
```

---

## 📦 Installation & Deployment

### Docker (Production)

To instantly provision the asynchronous ASGI backend alongside the static UI layers without fighting Python environment variables, boot the container:

```bash
# Clone the repository
git clone https://github.com/yourusername/robokinematics.git
cd robokinematics

# Boot detached production containers
docker-compose up -d --build
```
Navigate to `http://localhost:8000`.

### Local Development

If you intend to modify the kinematics or the UI directly:

```bash
# Setup Python Environment
cd backend
python -m venv venv
source venv/bin/activate  # or `venv\Scripts\activate` on Windows

# Install numerical dependencies & web routers
pip install -r requirements.txt

# Run Hot-Reloading Dev Server
uvicorn main:app --reload --port 8000
```
---

## 📡 API Documentation (WebSocket)

The UI communicates completely symmetrically with the FastAPI container via WebSockets for sub-millisecond kinematic tracking.

**Connection**: `ws://localhost:8000/ws`

**Payload Format Structure**:
```json
{
  "mode": "ik", // "fk", "ik", or "workspace"
  "dof": 3,
  "dh_params": [
      {"theta_offset": 0, "d": 2.5, "a": 0, "alpha": 1.5708}
  ],
  "joint_angles": [0, 0, 0], // Only used/updated heavily during 'fk'
  "locked_joints": [false, true, false], // Overrides optimization bounds 
  "target": {"x": 3.0, "y": 4.0, "z": 1.0} // Ignored during 'fk'
}
```
**Response Format**:
Returns `"frames"` (an ordered array of `4x4` transformation matrices for every joint link and the end-effector) alongside the solved radian `"angles"`.
