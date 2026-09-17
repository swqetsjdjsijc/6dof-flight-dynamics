import numpy as np
from core.integrator import RK4Integrator
from core.control import PIDController
from models.flight_dynamics import FlightDynamics6DOF
from visualization.plot_results import plot_flight_data

def run_simulation():
    model = FlightDynamics6DOF()
    
    # Pure Proportional Yaw Damper (negative feedback loop)
    yaw_damper = PIDController(Kp=0.5, Ki=0.0, Kd=0.0, output_limits=(np.radians(-15), np.radians(15)))
    
    state = np.array([0.0, 0.0, -1000.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    dt = 0.01
    t_end = 40.0
    time = np.arange(0, t_end, dt)
    history = []

    for t in time:
        # Rudder doublet excitation pulse between 5s and 7s
        dr_command = 0.0
        if 5.0 <= t < 6.0:
            dr_command = np.radians(5.0)
        elif 6.0 <= t < 7.0:
            dr_command = np.radians(-5.0)
            
        yaw_rate = state[11] # r
        
        # Negative feedback: rudder opposes positive yaw rate
        damper_correction = -yaw_damper.compute(setpoint=0.0, measurement=yaw_rate, dt=dt)
        
        controls = {
            'thrust': 400.0,
            'delta_e': np.radians(-1.2),
            'delta_a': 0.0,
            'delta_r': dr_command + damper_correction
        }
        
        state = RK4Integrator.step(lambda _t, _y: model.state_derivatives(_t, _y, controls), t, state, dt)
        history.append(state)

    print("Rendering Stabilized Yaw Damper Dashboard...")
    plot_flight_data(time, history, title="Stabilized Closed-Loop Dutch Roll & Yaw Damper Simulation")

if __name__ == "__main__":
    run_simulation()
