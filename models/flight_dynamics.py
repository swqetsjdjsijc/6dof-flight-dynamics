import numpy as np
from core.atmosphere import Atmosphere
from core.aircraft import AircraftConfig

class FlightDynamics6DOF:
    """Coupled 6-DOF Rigid Body Aircraft Dynamics Model."""
    
    def __init__(self, config: AircraftConfig = None):
        self.ac = config if config else AircraftConfig()

    def state_derivatives(self, t: float, state: np.ndarray, controls: dict) -> np.ndarray:
        """
        Computes 12-state vector derivatives [x, y, z, u, v, w, phi, theta, psi, p, q, r].
        Controls dict keys: 'thrust', 'delta_e', 'delta_a', 'delta_r' (in radians).
        """
        # Unpack State Vector
        x, y, z, u, v, w, phi, theta, psi, p, q, r = state
        
        # Atmospheric & Kinematic Calculations
        rho, _, _, _ = Atmosphere.get_properties(-z) # z is negative altitude NED
        V = np.sqrt(u**2 + v**2 + w**2)
        if V < 1e-3:
            V = 1e-3  # Avoid division by zero
            
        alpha = np.arctan2(w, u)
        beta = np.arcsin(np.clip(v / V, -1.0, 1.0))
        q_dyn = 0.5 * rho * V**2

        # Unpack Control Surface Deflections
        T = controls.get('thrust', 0.0)
        de = controls.get('delta_e', 0.0)
        da = controls.get('delta_a', 0.0)
        dr = controls.get('delta_r', 0.0)

        # Non-dimensional rates
        p_hat = (p * self.ac.b) / (2.0 * V)
        q_hat = (q * self.ac.c_bar) / (2.0 * V)
        r_hat = (r * self.ac.b) / (2.0 * V)

        # Longitudinal Aerodynamic Coefficients
        CL = self.ac.CL0 + self.ac.CLa * alpha + self.ac.CL_de * de
        CD = self.ac.CD0 + self.ac.k * CL**2
        Cm = self.ac.Cm0 + self.ac.Cma * alpha + self.ac.Cmq * q_hat + self.ac.Cm_de * de

        # Lateral-Directional Aerodynamic Coefficients
        CY = self.ac.CYb * beta + self.ac.CYr * r_hat + self.ac.CY_dr * dr
        Cl = self.ac.Clb * beta + self.ac.Clp * p_hat + self.ac.Clr * r_hat + self.ac.Cl_da * da + self.ac.Cl_dr * dr
        Cn = self.ac.Cnb * beta + self.ac.Cnp * p_hat + self.ac.Cnr * r_hat + self.ac.Cn_da * da + self.ac.Cn_dr * dr

        # Forces in Body Axes (X, Y, Z)
        # Transform Lift & Drag (Wind Frame) to Body Frame
        fx_aero = q_dyn * self.ac.S * (-CD * np.cos(alpha) + CL * np.sin(alpha))
        fz_aero = q_dyn * self.ac.S * (-CD * np.sin(alpha) - CL * np.cos(alpha))
        fy_aero = q_dyn * self.ac.S * CY

        X = fx_aero + T
        Y = fy_aero
        Z = fz_aero

        # Moments in Body Axes (L, M, N)
        L = q_dyn * self.ac.S * self.ac.b * Cl
        M = q_dyn * self.ac.S * self.ac.c_bar * Cm
        N = q_dyn * self.ac.S * self.ac.b * Cn

        # Translational Accelerations (Body Frame)
        g = Atmosphere.G0
        udot = r * v - q * w - g * np.sin(theta) + X / self.ac.mass
        vdot = p * w - r * u + g * np.cos(theta) * np.sin(phi) + Y / self.ac.mass
        wdot = q * u - p * v + g * np.cos(theta) * np.cos(phi) + Z / self.ac.mass

        # Rotational Accelerations (Body Frame Equations of Motion)
        pdot = (L + (self.ac.Iyy - self.ac.Izz) * q * r) / self.ac.Ixx
        qdot = (M + (self.ac.Izz - self.ac.Ixx) * p * r) / self.ac.Iyy
        rdot = (N + (self.ac.Ixx - self.ac.Iyy) * p * q) / self.ac.Izz

        # Euler Rate Kinematics
        phidot = p + (q * np.sin(phi) + r * np.cos(phi)) * np.tan(theta)
        thetadot = q * np.cos(phi) - r * np.sin(phi)
        psidot = (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta)

        # Earth-Frame Trajectory Velocities (NED Position Derivatives)
        xdot = u * np.cos(theta) * np.cos(psi) + v * (np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi)) + w * (np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi))
        ydot = u * np.cos(theta) * np.sin(psi) + v * (np.sin(phi) * np.sin(theta) * np.sin(psi) + np.cos(phi) * np.cos(psi)) + w * (np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi))
        zdot = -u * np.sin(theta) + v * np.sin(phi) * np.cos(theta) + w * np.cos(phi) * np.cos(theta)

        return np.array([xdot, ydot, zdot, udot, vdot, wdot, phidot, thetadot, psidot, pdot, qdot, rdot])
