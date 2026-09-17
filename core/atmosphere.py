import numpy as np

class Atmosphere:
    """1976 US Standard Atmosphere Model (Troposphere up to 11,000m)."""
    
    T0 = 288.15      # Sea level temperature [K]
    P0 = 101325.0    # Sea level pressure [Pa]
    RHO0 = 1.225     # Sea level density [kg/m^3]
    L = 0.0065       # Temperature lapse rate [K/m]
    R = 287.058      # Specific gas constant [J/(kg·K)]
    GAMMA = 1.4      # Ratio of specific heats
    G0 = 9.80665     # Acceleration due to gravity [m/s^2]

    @classmethod
    def get_properties(cls, altitude_m: float):
        """Returns (density [kg/m^3], pressure [Pa], temperature [K], speed_of_sound [m/s])."""
        h = max(0.0, float(altitude_m))
        
        if h <= 11000.0:
            T = cls.T0 - cls.L * h
            P = cls.P0 * (T / cls.T0) ** (cls.G0 / (cls.R * cls.L))
        else:
            # Simple isothermal extension for stratosphere boundary
            T = 216.65
            P = 22632.1 * np.exp(-cls.G0 * (h - 11000.0) / (cls.R * T))
            
        rho = P / (cls.R * T)
        a = np.sqrt(cls.GAMMA * cls.R * T)
        return rho, P, T, a
