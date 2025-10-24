import matplotlib.pyplot as plt
from utilities import FileReader
import os
import glob

def plot_errors(filename, out_dir):
    headers, values = FileReader(filename).read_file()

    time_list = []
    first_stamp = values[0][-1]
    for val in values:
        time_list.append(val[-1] - first_stamp)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"Error Data from {filename}")

    # State space plot
    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axes[0].set_title(f"{headers[0]} vs {headers[1]}")
    axes[0].set_xlabel(headers[0])
    axes[0].set_ylabel(headers[1])
    axes[0].grid()

    # Time plot for each variable except the last one (timestamp)
    axes[1].set_title("Evolution of States Over Time")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Value")

    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label=headers[i])
    axes[1].legend()
    axes[1].grid()

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    # Save the figure to output directory instead of showing it
    os.makedirs(out_dir, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(filename))[0]
    fig_filename = os.path.join(out_dir, f"{base_name}_error_plot.png")
    plt.savefig(fig_filename)
    print(f"Saved plot to {fig_filename}")
    plt.close(fig)

import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot all CSV logs in a folder.')
    parser.add_argument('--folder', required=True, help='Folder containing CSV log files')
    parser.add_argument('--out', default='plots', help='Output folder for plots (default: plots)')
    args = parser.parse_args()

    logs_dir = args.folder
    out_dir = args.out

    if not os.path.isdir(logs_dir):
        raise SystemExit(f"Input folder does not exist: {logs_dir}")

    csv_files = sorted(glob.glob(os.path.join(logs_dir, '*.csv')))
    if not csv_files:
        print(f"No CSV files found in {logs_dir}")
    else:
        print(f"Plotting {len(csv_files)} files from {logs_dir} -> {out_dir}")
        for filename in csv_files:
            plot_errors(filename, out_dir)
