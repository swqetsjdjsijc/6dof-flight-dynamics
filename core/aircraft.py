from dataclasses import dataclass

@dataclass
class AircraftConfig:
    """Rigid-body aircraft mass, geometry, and aerodynamic derivatives."""
    # Mass & Inertia
    mass: float = 1100.0          # Mass [kg]
    Ixx: float = 1285.3           # Roll inertia [kg·m^2]
    Iyy: float = 1825.2           # Pitch inertia [kg·m^2]
    Izz: float = 2667.0           # Yaw inertia [kg·m^2]
    Ixz: float = 0.0              # Cross product inertia [kg·m^2]

    # Reference Geometry
    S: float = 16.2               # Wing area [m^2]
    b: float = 11.0               # Wingspan [m]
    c_bar: float = 1.5            # Mean aerodynamic chord [m]

    # Longitudinal Derivatives
    CL0: float = 0.25             # Zero-alpha lift coefficient
    CLa: float = 4.47             # Lift curve slope [1/rad]
    CD0: float = 0.03             # Parasitic drag coefficient
    k: float = 0.045              # Induced drag factor (CD = CD0 + k*CL^2)
    Cm0: float = 0.0              # Zero-alpha moment coefficient
    Cma: float = -0.65            # Pitch stiffness [1/rad]
    Cmq: float = -12.5            # Pitch damping coefficient [1/rad]
    Cm_de: float = -1.15          # Elevator control authority [1/rad]
    CL_de: float = 0.43           # Elevator lift contribution [1/rad]
