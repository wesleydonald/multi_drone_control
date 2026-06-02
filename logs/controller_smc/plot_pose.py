#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

def main():
    # Load CSV
    df = pd.read_csv("log.csv")

    # Extract time and position
    t = df["sim_time"].to_numpy()
    x = df["pose_x"].to_numpy()
    y = df["pose_y"].to_numpy()
    z = df["pose_z"].to_numpy()

    # ---- Plot position vs time ----
    plt.figure()
    plt.plot(t, x, label="x")
    plt.plot(t, y, label="y")
    plt.plot(t, z, label="z")
    plt.xlabel("Time [s]")
    plt.ylabel("Position [m]")
    plt.title("Position vs Time")
    plt.legend()
    plt.grid()

    # ---- Plot 3D trajectory ----
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("3D Trajectory")

    plt.show()


if __name__ == "__main__":
    main()