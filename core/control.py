import numpy as np

class PIDController:
    """Discrete PID Controller with anti-windup clamping."""
    
    def __init__(self, Kp: float, Ki: float, Kd: float, output_limits: tuple = (-0.5, 0.5)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_out, self.max_out = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup clamping
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        D = self.Kd * derivative
        
        self.prev_error = error
        
        # Raw output and saturation clamping
        raw_output = P + I + D
        clamped_output = float(np.clip(raw_output, self.min_out, self.max_out))
        
        # Anti-windup: clamp integral accumulation if output saturated
        if raw_output != clamped_output and np.sign(error) == np.sign(raw_output):
            self.integral -= error * dt
            
        return clamped_output
