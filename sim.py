import numpy as np
from core.integrator import RK4Integrator
from models.flight_dynamics import FlightDynamics6DOF

def run_simulation():
    model = FlightDynamics6DOF()
    
    # Initial State: Cruise @ 1000m, 50m/s airspeed
    # State: [x, y, z, u, v, w, phi, theta, psi, p, q, r]
    state = np.array([0.0, 0.0, -1000.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    dt = 0.01
    t_end = 40.0
    time = np.arange(0, t_end, dt)
    history = []

    for t in time:
        # Control sequence: Rudder Doublet between t=5s and t=7s to excite Dutch Roll
        dr = 0.0
        if 5.0 <= t < 6.0:
            dr = np.radians(5.0)   # 5 deg rudder right
        elif 6.0 <= t < 7.0:
            dr = np.radians(-5.0)  # 5 deg rudder left
            
        controls = {'thrust': 400.0, 'delta_e': np.radians(-1.2), 'delta_a': 0.0, 'delta_r': dr}
        
        # Step solver using RK4
        state = RK4Integrator.step(lambda _t, _y: model.state_derivatives(_t, _y, controls), t, state, dt)
        history.append(state)

    print("6-DOF Dutch Roll Simulation Completed Successfully!")

if __name__ == "__main__":
    run_simulation()
