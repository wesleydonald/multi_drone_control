"""
plot_trajectories.py
────────────────────
Plot 4-drone formation trajectories in 3D from controller_mpc_multi log files.

Usage:
    python3 plot_trajectories.py --logs_dir /path/to/logs/controller_mpc_multi
    python3 plot_trajectories.py --files drone0.csv drone1.csv drone2.csv drone3.csv
"""

import argparse
import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection


# ── Colours per drone ─────────────────────────────────────────────────────────
DRONE_COLOURS = ['#2196F3', '#F44336', '#4CAF50', '#FF9800']
DRONE_LABELS  = ['Drone 0', 'Drone 1', 'Drone 2', 'Drone 3']


def load_log(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Normalise column names (strip whitespace)
    df.columns = df.columns.str.strip()
    return df


def find_logs(logs_dir: str) -> list[str]:
    """Find the most recent log.csv for each drone under logs_dir."""
    # Expected structure:
    #   logs_dir/
    #     circle_drone0_YYYYMMDD_HHMMSS/log.csv
    #     circle_drone1_YYYYMMDD_HHMMSS/log.csv
    #     ...
    csvs = sorted(glob.glob(os.path.join(logs_dir, '**', 'log.csv'), recursive=True))
    
    # Group by drone id and take most recent
    drone_files = {}
    for path in csvs:
        for i in range(4):
            if f'drone{i}' in path:
                drone_files[i] = path  # sorted so last = most recent

    if not drone_files:
        # Fall back: just take up to 4 csvs in order
        for idx, path in enumerate(csvs[:4]):
            drone_files[idx] = path

    return [drone_files[k] for k in sorted(drone_files.keys())]


def colour_by_time(ax, x, y, z, colour, alpha=0.9):
    """Draw a 3D line with fading alpha to show time progression."""
    points = np.array([x, y, z]).T.reshape(-1, 1, 3)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    alphas = np.linspace(0.2, alpha, len(segments))
    
    lc = Line3DCollection(segments, linewidths=1.5)
    lc.set_color([(*plt.matplotlib.colors.to_rgb(colour), a) for a in alphas])
    ax.add_collection3d(lc)


def plot_3d(dataframes: list, labels: list, colours: list, title: str = "Formation Trajectories"):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    for df, label, colour in zip(dataframes, labels, colours):
        x = df['pose_x'].values
        y = df['pose_y'].values
        z = df['pose_z'].values

        # Full trajectory with time-fade
        colour_by_time(ax, x, y, z, colour)

        # Start marker
        ax.scatter(x[0], y[0], z[0], color=colour, marker='o', s=60, zorder=5)
        # End marker
        ax.scatter(x[-1], y[-1], z[-1], color=colour, marker='x', s=80, zorder=5,
                   linewidths=2)
        # Legend proxy
        ax.plot([], [], [], color=colour, label=label, linewidth=2)

    ax.set_xlabel('X (m)', labelpad=8)
    ax.set_ylabel('Y (m)', labelpad=8)
    ax.set_zlabel('Z (m)', labelpad=8)
    ax.set_title(title, pad=15)
    ax.legend(loc='upper left')

    # Equal aspect ratio
    all_x = np.concatenate([df['pose_x'].values for df in dataframes])
    all_y = np.concatenate([df['pose_y'].values for df in dataframes])
    all_z = np.concatenate([df['pose_z'].values for df in dataframes])
    max_range = max(all_x.ptp(), all_y.ptp(), all_z.ptp()) / 2
    mid_x, mid_y, mid_z = all_x.mean(), all_y.mean(), all_z.mean()
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(max(0, mid_z - max_range), mid_z + max_range)

    plt.tight_layout()
    return fig


def plot_top_down(dataframes: list, labels: list, colours: list):
    """Bird's eye view — clearest way to see formation geometry."""
    fig, ax = plt.subplots(figsize=(8, 8))

    for df, label, colour in zip(dataframes, labels, colours):
        x = df['pose_x'].values
        y = df['pose_y'].values
        ax.plot(x, y, color=colour, linewidth=1.5, alpha=0.8, label=label)
        ax.scatter(x[0],  y[0],  color=colour, marker='o', s=80, zorder=5)
        ax.scatter(x[-1], y[-1], color=colour, marker='x', s=100, zorder=5,
                   linewidths=2)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Formation Trajectories — Top Down View')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig


def plot_altitude(dataframes: list, labels: list, colours: list):
    """Z vs time — useful for checking takeoff/land sync."""
    fig, ax = plt.subplots(figsize=(12, 4))

    for df, label, colour in zip(dataframes, labels, colours):
        # Normalise time to start at 0
        t = df['sim_time'].values
        t = t - t[0]
        ax.plot(t, df['pose_z'].values, color=colour, linewidth=1.5,
                alpha=0.85, label=label)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude Z (m)')
    ax.set_title('Altitude over Time (Takeoff / Hover / Land)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig


def plot_formation_error(dataframes: list, labels: list, colours: list):
    """
    Formation error: deviation of each drone from its expected offset
    relative to drone 0's actual position at each timestep.
    Requires all logs to have the same number of rows (resampled to min length).
    """
    OFFSETS = {0: (0.0, 0.0), 1: (1.0, 0.0), 2: (0.0, 1.0), 3: (1.0, 1.0)}

    min_len = min(len(df) for df in dataframes)
    dfs = [df.iloc[:min_len].reset_index(drop=True) for df in dataframes]

    t = dfs[0]['sim_time'].values
    t = t - t[0]

    fig, ax = plt.subplots(figsize=(12, 4))

    x0 = dfs[0]['pose_x'].values
    y0 = dfs[0]['pose_y'].values

    for i, (df, label, colour) in enumerate(zip(dfs, labels, colours)):
        if i == 0:
            continue  # drone 0 is the reference
        ox, oy = OFFSETS[i]
        expected_x = x0 + ox
        expected_y = y0 + oy
        error = np.sqrt((df['pose_x'].values - expected_x)**2 +
                        (df['pose_y'].values - expected_y)**2)
        ax.plot(t, error, color=colour, linewidth=1.5, label=label)
        print(f"{label} — mean error: {error.mean():.4f}m, "
              f"max error: {error.max():.4f}m, RMS: {np.sqrt((error**2).mean()):.4f}m")

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Formation Error (m)')
    ax.set_title('Formation Error Relative to Drone 0')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Plot multi-drone trajectories')
    parser.add_argument('--logs_dir', type=str, default=None,
                        help='Path to logs/controller_mpc_multi directory')
    parser.add_argument('--files', nargs='+', default=None,
                        help='Explicit CSV file paths for drone 0,1,2,3')
    parser.add_argument('--save', action='store_true',
                        help='Save figures as PNG instead of displaying')
    args = parser.parse_args()

    # ── Load data ─────────────────────────────────────────────────────────
    if args.files:
        file_paths = args.files
    elif args.logs_dir:
        file_paths = find_logs(args.logs_dir)
        if not file_paths:
            print(f"No log.csv files found under {args.logs_dir}")
            return
    else:
        # Try default location
        default = os.path.expanduser(
            '~/multi_drone_control/logs/controller_mpc_multi')
        file_paths = find_logs(default)
        if not file_paths:
            print("No logs found. Use --logs_dir or --files to specify paths.")
            return

    print(f"Loading {len(file_paths)} log file(s):")
    for p in file_paths:
        print(f"  {p}")

    dataframes = [load_log(p) for p in file_paths]
    n = len(dataframes)
    labels  = DRONE_LABELS[:n]
    colours = DRONE_COLOURS[:n]

    # ── Plots ─────────────────────────────────────────────────────────────
    fig3d   = plot_3d(dataframes, labels, colours)
    fig_top = plot_top_down(dataframes, labels, colours)
    fig_alt = plot_altitude(dataframes, labels, colours)

    if n == 4:
        fig_err = plot_formation_error(dataframes, labels, colours)

    if args.save:
        fig3d.savefig('trajectories_3d.png',   dpi=150, bbox_inches='tight')
        fig_top.savefig('trajectories_top.png', dpi=150, bbox_inches='tight')
        fig_alt.savefig('trajectories_alt.png', dpi=150, bbox_inches='tight')
        if n == 4:
            fig_err.savefig('formation_error.png', dpi=150, bbox_inches='tight')
        print("Figures saved.")
    else:
        plt.show()


if __name__ == '__main__':
    main()