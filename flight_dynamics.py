import numpy as np

# Physical Constants
G0 = 9.80665  # Sea-level gravitational acceleration (m/s^2)
R_AIR = 287.058  # Specific gas constant for air (J/(kg*K))
GAMMA = 1.4  # Specific heat ratio for air


def get_atmosphere(altitude_m: float):
    """
    Computes troposphere parameters (< 11,000m) based on US Standard Atmosphere 1976.
    Returns: temperature (K), pressure (Pa), density (kg/m^3), speed_of_sound (m/s)
    """
    T0 = 288.15  # Sea-level temperature (K)
    P0 = 101325.0  # Sea-level pressure (Pa)
    L = 0.0065  # Temperature lapse rate (K/m)

    h = max(0.0, altitude_m)

    if h <= 11000.0:
        T = T0 - L * h
        P = P0 * (1.0 - (L * h) / T0) ** (G0 / (R_AIR * L))
    else:
        T_11 = 216.65
        P_11 = 22632.1
        T = T_11
        P = P_11 * np.exp(-G0 * (h - 11000.0) / (R_AIR * T_11))

    rho = P / (R_AIR * T)
    a = np.sqrt(GAMMA * R_AIR * T)

    return T, P, rho, a


class AircraftConfig:
    def __init__(self):
        # Mass and Inertia Tensor
        self.mass = 1100.0  # Total mass (kg)
        self.Ixx = 1285.3  # Roll moment of inertia (kg*m^2)
        self.Iyy = 1824.9  # Pitch moment of inertia (kg*m^2)
        self.Izz = 2667.0  # Yaw moment of inertia (kg*m^2)
        self.Ixz = 0.0  # Cross-product inertia

        # Reference Geometry
        self.S = 16.2  # Wing reference area (m^2)
        self.b = 11.0  # Wingspan (m)
        self.c = 1.5  # Mean aerodynamic chord (m)

        # Baseline Stability Derivatives
        self.CL0 = 0.25  # Zero-AoA lift coefficient
        self.CLa = 4.47  # Lift curve slope (per rad)
        self.CD0 = 0.03  # Zero-lift drag coefficient
        self.e = 0.8  # Oswald efficiency factor

        # Pitch Stability
        self.Cm0 = 0.05
        self.Cma = -0.65  # Pitch stiffness (negative for static pitch stability)
        self.Cmq = -12.0  # Pitch damping

        # Control Surface Effectiveness
        self.CL_de = 0.4  # Elevator effectiveness on Lift
        self.Cm_de = -1.2  # Elevator effectiveness on Pitch


def euler_rates(phi, theta, psi, p, q, r):
    """
    Transforms body angular rates (p, q, r) into Euler angle rates (phi_dot, theta_dot, psi_dot).
    """
    cos_theta = np.cos(theta)
    if abs(cos_theta) < 1e-4:
        cos_theta = 1e-4 * np.sign(cos_theta)

    tan_theta = np.tan(theta)

    phi_dot = p + (q * np.sin(phi) + r * np.cos(phi)) * tan_theta
    theta_dot = q * np.cos(phi) - r * np.sin(phi)
    psi_dot = (q * np.sin(phi) + r * np.cos(phi)) / cos_theta

    return np.array([phi_dot, theta_dot, psi_dot])


def compute_forces_and_moments(state, control, ac: AircraftConfig):
    """
    Computes total forces (body frame) and moments acting on the aircraft.
    state: [x, y, z, u, v, w, phi, theta, psi, p, q, r]
    control: [elevator, aileron, rudder, throttle]
    """
    x, y, z, u, v, w, phi, theta, psi, p, q, r = state
    de, da, dr, throttle = control

    # Airspeed & Angles
    V_total = np.sqrt(u**2 + v**2 + w**2)
    if V_total < 1.0:
        V_total = 1.0

    alpha = np.arctan2(w, u)
    beta = np.arcsin(np.clip(v / V_total, -1.0, 1.0))

    # Dynamic Pressure
    _, _, rho, _ = get_atmosphere(-z)
    q_dyn = 0.5 * rho * V_total**2

    # Aerodynamic Coefficients
    CL = ac.CL0 + ac.CLa * alpha + ac.CL_de * de
    AR = (ac.b**2) / ac.S
    CD = ac.CD0 + (CL**2) / (np.pi * AR * ac.e)
    Cm = ac.Cm0 + ac.Cma * alpha + ac.Cm_de * de + ac.Cmq * (q * ac.c / (2 * V_total))

    # Forces in Stability Frame -> Convert to Body Frame
    lift = q_dyn * ac.S * CL
    drag = q_dyn * ac.S * CD

    Fx_aero = -drag * np.cos(alpha) + lift * np.sin(alpha)
    Fz_aero = -drag * np.sin(alpha) - lift * np.cos(alpha)
    Fy_aero = 0.0

    # Thrust
    max_thrust = 1600.0
    Fx_thrust = throttle * max_thrust

    # Gravity in Body Frame
    Fx_g = -ac.mass * G0 * np.sin(theta)
    Fy_g = ac.mass * G0 * np.sin(phi) * np.cos(theta)
    Fz_g = ac.mass * G0 * np.cos(phi) * np.cos(theta)

    # Total Forces & Moments
    Fx = Fx_aero + Fx_thrust + Fx_g
    Fy = Fy_aero + Fy_g
    Fz = Fz_aero + Fz_g

    My = q_dyn * ac.S * ac.c * Cm
    Mx = 0.0
    Mz = 0.0

    return np.array([Fx, Fy, Fz]), np.array([Mx, My, Mz])


def derivatives(state, control, ac: AircraftConfig):
    """
    Evaluates dX/dt for the 12-state vector.
    """
    x, y, z, u, v, w, phi, theta, psi, p, q, r = state
    F, M = compute_forces_and_moments(state, control, ac)

    # Linear Accelerations
    du = (F[0] / ac.mass) - (q * w - r * v)
    dv = (F[1] / ac.mass) - (r * u - p * w)
    dw = (F[2] / ac.mass) - (p * v - q * u)

    # Angular Accelerations
    dp = M[0] / ac.Ixx
    dq = M[1] / ac.Iyy
    dr = M[2] / ac.Izz

    # Kinematics
    c_th, s_th = np.cos(theta), np.sin(theta)
    c_ph, s_ph = np.cos(phi), np.sin(phi)
    c_ps, s_ps = np.cos(psi), np.sin(psi)

    dx = u * (c_th * c_ps) + v * (s_ph * s_th * c_ps - c_ph * s_ps) + w * (c_ph * s_th * c_ps + s_ph * s_ps)
    dy = u * (c_th * s_ps) + v * (s_ph * s_th * s_ps + c_ph * c_ps) + w * (c_ph * s_th * s_ps - s_ph * c_ps)
    dz = -u * s_th + v * (s_ph * c_th) + w * (c_ph * c_th)

    d_euler = euler_rates(phi, theta, psi, p, q, r)

    return np.array([dx, dy, dz, du, dv, dw, d_euler[0], d_euler[1], d_euler[2], dp, dq, dr])


def rk4_step(state, control, ac: AircraftConfig, dt: float):
    """
    4th Order Runge-Kutta numerical integration step.
    """
    k1 = derivatives(state, control, ac)
    k2 = derivatives(state + 0.5 * dt * k1, control, ac)
    k3 = derivatives(state + 0.5 * dt * k2, control, ac)
    k4 = derivatives(state + dt * k3, control, ac)

    return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


if __name__ == "__main__":
    ac = AircraftConfig()
    T_sl, P_sl, rho_sl, a_sl = get_atmosphere(0)
    T_5k, P_5k, rho_5k, a_5k = get_atmosphere(5000)

    print(f"Sea-Level Density: {rho_sl:.4f} kg/m^3 | Speed of Sound: {a_sl:.2f} m/s")
    print(f"5,000m Density:    {rho_5k:.4f} kg/m^3 | Speed of Sound: {a_5k:.2f} m/s")
