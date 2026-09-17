# 6-DOF Flight Dynamics & Control Engine

A modular, 6-Degree-of-Freedom (6-DOF) rigid-body flight dynamics simulation suite built in Python, featuring coupled lateral-directional dynamics, atmospheric modeling, and active PID closed-loop flight control.

## Overview

This simulator models a fixed-wing aircraft in mid-air cruise conditions. It solves non-linear 12-state translational and rotational equations of motion using standard aerodynamic derivatives and dynamic atmospheric properties, allowing analysis of longitudinal (Phugoid, Short Period) and lateral-directional flight modes (Dutch Roll, Roll Subsidence, Spiral Mode).

## Key Features

* **Atmospheric Physics Engine:** Implements the 1976 US Standard Atmosphere model to compute altitude-dependent air density, pressure, and speed of sound.
* **Full 12-State Kinematics:** Tracks spatial position $(x, y, z)$, body velocities $(u, v, w)$, Euler attitude angles $(\phi, \theta, \psi)$, and angular body rates $(p, q, r)$.
* **Coupled 6-DOF Flight Dynamics:** Accurately models cross-coupling between roll, pitch, and yaw, including dihedral effects, weathercocking, and adverse yaw.
* **Discrete PID Flight Control:** Features an active closed-loop Yaw Damper to eliminate high-frequency Dutch Roll oscillations and restore lateral stability.
* **Multi-Axis Visualization Suite:** Renders multi-panel dashboards for Euler angles, body angular rates, aerodynamic angles $(\alpha, \beta)$, and 3D flight trajectories.

## Repository Architecture

```text
6dof-flight-dynamics/
├── core/
│   ├── atmosphere.py       # 1976 US Standard Atmosphere model
│   ├── aircraft.py         # Geometry, mass, and 6-DOF stability derivatives
│   ├── integrator.py       # 4th-Order Runge-Kutta (RK4) solver
│   └── control.py          # Discrete PID controller with anti-windup
├── models/
│   └── flight_dynamics.py  # 12-state non-linear equations of motion
├── visualization/
│   └── plot_results.py     # Multi-panel time series & 3D trajectory plotting
├── sim.py                  # Closed-loop simulation runner
└── README.md

