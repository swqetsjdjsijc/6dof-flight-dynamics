from dataclasses import dataclass

@dataclass
class AircraftConfig:
    """Rigid-body aircraft mass, geometry, and 6-DOF aerodynamic derivatives."""
    # Mass & Inertia Properties
    mass: float = 1100.0          # Mass [kg]
    Ixx: float = 1285.3           # Roll inertia [kg·m^2]
    Iyy: float = 1825.2           # Pitch inertia [kg·m^2]
    Izz: float = 2667.0           # Yaw inertia [kg·m^2]
    Ixz: float = 0.0              # Cross-product inertia [kg·m^2]

    # Reference Geometry
    S: float = 16.2               # Wing area [m^2]
    b: float = 11.0               # Wingspan [m]
    c_bar: float = 1.5            # Mean aerodynamic chord [m]

    # Longitudinal Derivatives
    CL0: float = 0.25
    CLa: float = 4.47             # Lift curve slope [1/rad]
    CD0: float = 0.03             # Parasitic drag
    k: float = 0.045              # Induced drag factor
    Cm0: float = 0.0
    Cma: float = -0.65            # Pitch stiffness [1/rad]
    Cmq: float = -12.5            # Pitch damping [1/rad]
    Cm_de: float = -1.15          # Elevator control authority [1/rad]
    CL_de: float = 0.43

    # Lateral-Directional Derivatives
    CYb: float = -0.56            # Sideforce due to sideslip [1/rad]
    CYr: float = 0.24             # Sideforce due to yaw rate [1/rad]
    CY_dr: float = 0.15           # Rudder sideforce authority [1/rad]
    
    Clb: float = -0.074           # Dihedral effect / Roll stiffness [1/rad]
    Clp: float = -0.41            # Roll damping [1/rad]
    Clr: float = 0.107            # Roll due to yaw rate [1/rad]
    Cl_da: float = 0.134          # Aileron roll authority [1/rad]
    Cl_dr: float = 0.0117         # Rudder roll authority [1/rad]
    
    Cnb: float = 0.117            # Directional stiffness / Weathercocking [1/rad]
    Cnp: float = -0.0237          # Yaw due to roll rate [1/rad]
    Cnr: float = -0.125           # Yaw damping [1/rad]
    Cn_da: float = -0.0035        # Adverse yaw due to aileron [1/rad]
    Cn_dr: float = -0.062         # Rudder yaw authority [1/rad]
