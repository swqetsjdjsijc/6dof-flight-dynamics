import numpy as np
from core.integrator import RK4Integrator
from core.control import PIDController
from models.flight_dynamics import FlightDynamics6DOF

def run_simulation():
    model = FlightDynamics6DOF()
    
    # Initialize Yaw Damper PID (Feeds yaw rate 'r' to rudder 'delta_r')
    yaw_damper = PIDController(Kp=0.0, Ki=0.0, Kd=1.5, output_limits=(np.radians(-15), np.radians(15)))
    
    # Initial State: Cruise @ 1000m altitude, 50m/s velocity
    state = np.array([0.0, 0.0, -1000.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    dt = 0.01
    t_end = 40.0
    time = np.arange(0, t_end, dt)

    for t in time:
        # Side gust / disturbance simulated via brief rudder input between t=5s and 6s
        dr_command = 0.0
        if 5.0 <= t < 6.0:
            dr_command = np.radians(5.0)
            
        # Closed-loop Feedback: Yaw rate feedback dampens oscillation
        yaw_rate = state[11] # r state
        damper_correction = yaw_damper.compute(setpoint=0.0, measurement=yaw_rate, dt=dt)
        
        total_dr = dr_command + damper_correction

        controls = {
            'thrust': 400.0,
            'delta_e': np.radians(-1.2),
            'delta_a': 0.0,
            'delta_r': total_dr
        }
        
        state = RK4Integrator.step(lambda _t, _y: model.state_derivatives(_t, _y, controls), t, state, dt)

    print("PID Yaw Damper Closed-Loop Simulation Completed Successfully!")

if __name__ == "__main__":
    run_simulation()
