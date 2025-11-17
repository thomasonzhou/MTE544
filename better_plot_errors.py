import matplotlib.pyplot as plt
from utilities import FileReader
import os
import glob

def plot_pf_vs_odom(robotpose_csv, out_dir):
    headers, values = FileReader(robotpose_csv).read_file()

    time_list = []
    # Identify indices by header names for robustness
    try:
        idx_odom_x = headers.index("odom_x")
        idx_odom_y = headers.index("odom_y")
        idx_odom_th = headers.index("odom_th")
        idx_pf_x = headers.index("pf_x")
        idx_pf_y = headers.index("pf_y")
        idx_pf_th = headers.index("pf_th")
        idx_stamp = headers.index("stamp")
    except ValueError as e:
        raise SystemExit(f"Expected headers not found in {robotpose_csv}: {e}")

    if not values:
        raise SystemExit(f"No data rows found in {robotpose_csv}")

    first_stamp = values[0][idx_stamp]
    for val in values:
        time_list.append((val[idx_stamp] - first_stamp) / 1e9)  # seconds

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"PF vs Odom from {os.path.basename(robotpose_csv)}")

    # XY trajectory: odometry vs particle filter
    axes[0].plot([row[idx_odom_x] for row in values], [row[idx_odom_y] for row in values], label="odom")
    axes[0].plot([row[idx_pf_x] for row in values], [row[idx_pf_y] for row in values], label="pf")
    axes[0].set_title("XY Trajectory (odom vs pf)")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].legend()
    axes[0].grid()
    axes[0].axis('equal')

    # Theta over time: odometry vs particle filter
    axes[1].set_title("Theta over Time (odom vs pf) - laser_sig = 0.2")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("theta (rad)")
    axes[1].plot(time_list, [row[idx_odom_th] for row in values], label="odom_th")
    axes[1].plot(time_list, [row[idx_pf_th] for row in values], label="pf_th")
    axes[1].legend()
    axes[1].grid()

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    # Save the figure to output directory instead of showing it
    os.makedirs(out_dir, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(robotpose_csv))[0]
    fig_filename = os.path.join(out_dir, f"{base_name}_pf_vs_odom.png")
    plt.savefig(fig_filename)
    print(f"Saved plot to {fig_filename}")
    plt.close(fig)

import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot PF vs Odom from robotPose.csv in a folder.')
    parser.add_argument('--folder', required=True, help='Folder containing CSV log files')
    parser.add_argument('--out', default='plots', help='Output folder for plots (default: plots)')
    args = parser.parse_args()

    logs_dir = args.folder
    out_dir = args.out

    if not os.path.isdir(logs_dir):
        raise SystemExit(f"Input folder does not exist: {logs_dir}")

    robotpose_path = os.path.join(logs_dir, 'robotPose.csv')
    if not os.path.isfile(robotpose_path):
        # Fallback: try to find any file that looks like robotPose.csv
        candidates = [p for p in glob.glob(os.path.join(logs_dir, '*.csv')) if os.path.basename(p).lower().startswith('robotpose')]
        if candidates:
            robotpose_path = candidates[0]
        else:
            raise SystemExit(f"robotPose.csv not found in {logs_dir}")

    print(f"Plotting PF vs Odom from {robotpose_path} -> {out_dir}")
    plot_pf_vs_odom(robotpose_path, out_dir)