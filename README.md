\# 6-DOF Flight Dynamics Simulator



A 6-Degree-of-Freedom (6-DOF) rigid-body flight dynamics engine built in Python for numerical physics modeling and aerodynamic stability analysis.



\## Project Overview



This simulator models a fixed-wing aircraft flying at mid-air cruise conditions. It solves non-linear translational and rotational equations of motion using standard aerodynamic derivatives and dynamic atmospheric properties, allowing observation of coupled longitudinal flight modes.



\## Key Features



\* \*\*Atmospheric Physics Engine:\*\* Models the 1976 US Standard Atmosphere to compute altitude-dependent air density, pressure, and speed of sound.

\* \*\*12-State Kinematics:\*\* Tracks position $(x, y, z)$, body velocities $(u, v, w)$, Euler angles $(\\phi, \\theta, \\psi)$, and angular body rates $(p, q, r)$.

\* \*\*4th-Order RK4 Integrator:\*\* Employs Runge-Kutta numerical integration (`RK4`) to evaluate differential equations with high accuracy and zero drift.

\* \*\*Flight Stability Analysis:\*\* Simulates elevator doublet control inputs to excite and plot high-frequency Short Period and long-period Phugoid oscillations.



\## Project Architecture



\* `flight\_dynamics.py` – Core physics engine containing atmospheric models, baseline aircraft geometry, force/moment calculations, and the RK4 step solver.

\* `sim.py` – Simulation runner managing initial state vectors, elevator control inputs, and 60-second trajectory state logging.

\* `plot\_results.py` – Multi-axis plotting engine utilizing Matplotlib to visualize attitude, altitude, airspeed, and angular pitch rates.



\## Installation \& Requirements



Ensure you have Python installed, then install the required dependencies:



```bash

pip install numpy matplotlib

