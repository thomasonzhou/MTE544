# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader
import argparse
import os
import glob

def _parse_file_context(filename):
    base=os.path.basename(filename).lower()
    if 'imu' in base:
        data_type='imu'
    elif 'odom' in base:
        data_type='odom'
    elif 'laser' in base:
        data_type='laser'
    else:
        data_type='sensor'

    scenario=None
    for tag in ('line','circle','spiral'):
        if f"_{tag}" in base:
            scenario=tag
            break
    return data_type, scenario

_UNITS_BY_HEADER={
    'acc_x':'[m/s^2]',
    'acc_y':'[m/s^2]',
    'angular_z':'[rad/s]',
    'x':'[m]',
    'y':'[m]',
    'th':'[rad]',
}

def plot_errors(filename, save_path=None):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    data_type, scenario=_parse_file_context(filename)

    plt.figure()
    for i in range(0, len(headers) - 1):
        h=headers[i].strip()
        unit=_UNITS_BY_HEADER.get(h.lower(), '')
        series_label=f"{h} {unit}".strip()
        plt.plot(time_list, [lin[i] for lin in values], label=series_label)

    # Axis labels and plot title for the time plots based on file name
    plt.xlabel('Time [s]')

    if data_type=='imu':
        plt.ylabel('Acceleration [m/s^2] / Angular rate [rad/s]')
        base_title='IMU over time'
    elif data_type=='odom':
        plt.ylabel('Position [m] / Heading [rad]')
        base_title='Position/Heading over time'
    elif data_type=='laser':
        plt.ylabel('Sensor value')
        base_title='Laser data over time'
    else:
        plt.ylabel('Value')
        base_title='Sensor data over time'

    title=f"{base_title}"
    if scenario is not None:
        title+=f" - {scenario}"
    plt.title(title)
    plt.legend()
    plt.grid()
    if save_path is not None:
        plt.tight_layout()
        plt.savefig(save_path)
        plt.close()
    else:
        plt.show()

    header_index = {header.strip().lower(): idx for idx, header in enumerate(headers)}
    if 'x' in header_index and 'y' in header_index:
        x_vals = [row[header_index['x']] for row in values]
        y_vals = [row[header_index['y']] for row in values]

        plt.figure()
        plt.plot(x_vals, y_vals)
        plt.xlabel('x position [m]')
        plt.ylabel('y position [m]')
        xy_title='Trajectory (x vs y)'
        if scenario is not None:
            xy_title+=f" - {scenario}"
        plt.title(xy_title)
        plt.axis('equal')
        plt.grid(True)

        if save_path is not None:
            base, ext = os.path.splitext(save_path)
            xy_save_path = f"{base}_xy{ext}"
            plt.tight_layout()
            plt.savefig(xy_save_path)
            plt.close()
        else:
            plt.show()
    
if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Plot all CSV files in a directory.')
    parser.add_argument('--dir', required=True, help='Directory containing CSV files to process')
    
    args = parser.parse_args()
    
    input_dir=os.path.abspath(args.dir)
    if not os.path.isdir(input_dir):
        raise NotADirectoryError(f"Provided path is not a directory: {input_dir}")

    # Discover CSV files in the directory
    filenames=sorted([p for p in glob.glob(os.path.join(input_dir, '*.csv')) if os.path.isfile(p)])
    print("plotting the files", filenames)

    # Prepare output directory next to the input directory
    dirname=os.path.basename(os.path.normpath(input_dir))
    parent_dir=os.path.dirname(os.path.normpath(input_dir))
    output_dir=os.path.join(parent_dir, f"plots_{dirname}")
    os.makedirs(output_dir, exist_ok=True)

    for filename in filenames:
        base=os.path.splitext(os.path.basename(filename))[0]
        save_path=os.path.join(output_dir, f"{base}.png")
        plot_errors(filename, save_path=save_path)
