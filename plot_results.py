import matplotlib.pyplot as plt
import numpy as np
from sim import run_simulation


def plot_flight_data(t, states):
    # Extract state vectors
    altitude = -states[:, 2]  # Convert NED z to positive altitude
    u = states[:, 3]  # Axial airspeed (m/s)
    w = states[:, 5]  # Vertical velocity (m/s)
    theta_deg = np.degrees(states[:, 7])  # Pitch angle (degrees)
    q_deg = np.degrees(states[:, 10])  # Pitch rate (deg/s)

    # Compute Angle of Attack (alpha)
    alpha_deg = np.degrees(np.arctan2(w, u))

    plt.style.use("seaborn-v0_8-darkgrid" if "seaborn-v0_8-darkgrid" in plt.style.available else "default")
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("6-DOF Flight Dynamics: Elevator Doublet Response (Phugoid Excitations)", fontsize=14, fontweight="bold")

    # Subplot 1: Altitude vs Airspeed
    ax1_twin = axs[0].twinx()
    p1 = axs[0].plot(t, altitude, "b-", label="Altitude (m)", linewidth=1.8)
    p2 = ax1_twin.plot(t, u, "r--", label="Airspeed u (m/s)", linewidth=1.8)
    axs[0].set_ylabel("Altitude [m]", color="b")
    ax1_twin.set_ylabel("Airspeed [m/s]", color="r")
    axs[0].set_title("Trajectory & Speed Mode Interaction")

    # Combine legends for twin axis
    lines = p1 + p2
    labels = [l.get_label() for l in lines]
    axs[0].legend(lines, labels, loc="upper right")

    # Subplot 2: Pitch Angle vs Angle of Attack
    axs[1].plot(t, theta_deg, "g-", label="Pitch Angle (theta)", linewidth=1.8)
    axs[1].plot(t, alpha_deg, "m--", label="Angle of Attack (alpha)", linewidth=1.8)
    axs[1].set_ylabel("Angle [deg]")
    axs[1].set_title("Longitudinal Dynamics (Attitude)")
    axs[1].legend(loc="upper right")

    # Subplot 3: Pitch Rate (q)
    axs[2].plot(t, q_deg, "k-", label="Pitch Rate (q)", linewidth=1.5)
    axs[2].axvspan(5.0, 7.0, color="orange", alpha=0.2, label="Elevator Doublet Pulse")
    axs[2].set_xlabel("Time [seconds]")
    axs[2].set_ylabel("Rate [deg/s]")
    axs[2].set_title("Body Angular Pitch Rate")
    axs[2].legend(loc="upper right")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    t, states = run_simulation(duration_sec=60.0, dt=0.01)
    plot_flight_data(t, states)
