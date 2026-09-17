import numpy as np
from typing import Callable

class RK4Integrator:
    """Generic 4th-order Runge-Kutta numerical solver."""
    
    @staticmethod
    def step(derivatives_fn: Callable[[float, np.ndarray], np.ndarray], 
             t: float, y: np.ndarray, dt: float) -> np.ndarray:
        """Marches state vector y forward by dt using RK4 integration."""
        k1 = derivatives_fn(t, y)
        k2 = derivatives_fn(t + 0.5 * dt, y + 0.5 * dt * k1)
        k3 = derivatives_fn(t + 0.5 * dt, y + 0.5 * dt * k2)
        k4 = derivatives_fn(t + dt, y + dt * k3)
        
        return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
