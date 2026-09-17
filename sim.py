import numpy as np
from flight_dynamics import AircraftConfig, rk4_step

def run_simulation(duration_sec=60.0, dt=0.01):
    """
    Simulates aircraft flight dynamics over a fixed time horizon.
    Injects a elevator doublet pulse to excite pitching modes (Phugoid / Short Period).
    """
    ac = AircraftConfig()

    # Initial State Vector X = [x, y, z, u, v, w, phi, theta, psi, p, q, r]
    # Altitude = 1000m (z = -1000m in NED), Initial Speed u = 50 m/s (~100 knots)
    state = np.array([0.0, 0.0, -1000.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Control Input Vector: [elevator (rad), aileron (rad), rudder (rad), throttle (0-1)]
    # Trim elevator slightly negative (-0.03 rad) to balance baseline Cm0 pitch
    trim_elevator = -0.03
    trim_throttle = 0.45

    num_steps = int(duration_sec / dt)
    time_history = np.zeros(num_steps)
    state_history = np.zeros((num_steps, 12))

    print(f"Starting 6-DOF Simulation: {duration_sec}s horizon at dt={dt}s...")

    for i in range(num_steps):
        t = i * dt
        time_history[i] = t
        state_history[i, :] = state

        # Elevator doublet pulse at t = 5.0s to induce pitch oscillation
        elevator_input = trim_elevator
        if 5.0 <= t < 6.0:
            elevator_input += np.radians(3.0)   # Pitch down command
        elif 6.0 <= t < 7.0:
            elevator_input -= np.radians(3.0)   # Pitch up command

        control = np.array([elevator_input, 0.0, 0.0, trim_throttle])

        # RK4 Integration Step
        state = rk4_step(state, control, ac, dt)

    print("Simulation complete! State matrix populated.")
    return time_history, state_history

if __name__ == "__main__":
    t, states = run_simulation(duration_sec=60.0, dt=0.01)
    
    # Quick sanity check on initial vs final altitude and velocity
    init_alt = -states[0, 2]
    final_alt = -states[-1, 2]
    init_speed = states[0, 3]
    final_speed = states[-1, 3]

    print(f"Initial Altitude: {init_alt:.2f} m  | Final Altitude: {final_alt:.2f} m")
    print(f"Initial Forward Airspeed (u): {init_speed:.2f} m/s | Final Forward Airspeed (u): {final_speed:.2f} m/s")
