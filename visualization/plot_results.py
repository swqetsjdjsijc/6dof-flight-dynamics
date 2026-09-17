import numpy as np
import matplotlib.pyplot as plt

def plot_flight_data(time: np.ndarray, history: np.ndarray, title: str = "6-DOF Flight Dynamics Analysis"):
    """
    Renders a 4-panel dashboard showing Euler angles, angular rates, 
    sideslip/alpha, and 3D flight trajectory.
    """
    history = np.array(history)
    
    # Extract States
    x, y, z = history[:, 0], history[:, 1], -history[:, 2]  # Convert NED z to altitude
    u, v, w = history[:, 3], history[:, 4], history[:, 5]
    phi = np.degrees(history[:, 6])
    theta = np.degrees(history[:, 7])
    psi = np.degrees(history[:, 8])
    p = np.degrees(history[:, 9])
    q = np.degrees(history[:, 10])
    r = np.degrees(history[:, 11])

    V = np.sqrt(u**2 + v**2 + w**2)
    alpha = np.degrees(np.arctan2(w, u))
    beta = np.degrees(np.arcsin(np.clip(v / V, -1.0, 1.0)))

    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(title, fontsize=16, fontweight='bold')

    # 1. Euler Angles
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(time, phi, label=r'Roll ($\phi$)', color='tab:red')
    ax1.plot(time, theta, label=r'Pitch ($\theta$)', color='tab:blue')
    ax1.plot(time, psi, label=r'Yaw ($\psi$)', color='tab:green')
    ax1.set_ylabel('Attitude [deg]')
    ax1.set_title('Euler Angles')
    ax1.legend()
    ax1.grid(True)

    # 2. Body Angular Rates
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(time, p, label=r'Roll rate ($p$)', color='tab:red', linestyle='--')
    ax2.plot(time, q, label=r'Pitch rate ($q$)', color='tab:blue', linestyle='--')
    ax2.plot(time, r, label=r'Yaw rate ($r$)', color='tab:green')
    ax2.set_ylabel('Angular Rates [deg/s]')
    ax2.set_title('Body Angular Rates (Yaw Damper Response)')
    ax2.legend()
    ax2.grid(True)

    # 3. Aerodynamic Angles
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(time, alpha, label=r'Alpha ($\alpha$)', color='tab:purple')
    ax3.plot(time, beta, label=r'Beta ($\beta$)', color='tab:orange')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Angle [deg]')
    ax3.set_title('Angle of Attack & Sideslip')
    ax3.legend()
    ax3.grid(True)

    # 4. 3D Spatial Trajectory
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    ax4.plot(x, y, z, color='tab:blue', linewidth=2)
    ax4.set_xlabel('North [m]')
    ax4.set_ylabel('East [m]')
    ax4.set_zlabel('Altitude [m]')
    ax4.set_title('3D Spatial Flight Path')

    plt.tight_layout()
    plt.show()
