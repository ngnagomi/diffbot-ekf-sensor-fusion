"""
Evaluate a recorded run: wheel odometry and EKF vs. ground truth.
Usage (from ~/ekf_ws):  ros2 run diffbot_ekf evaluate runs/square_1
Writes plots and metrics to results/.
"""
import argparse
import json
import math
import os

import matplotlib
matplotlib.use('Agg')           
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ODOM_TOPICS = {
    '/ground_truth/odom': 'truth',
    '/wheel/odometry': 'wheel',
    '/odometry/filtered': 'ekf',
}
IMU_TOPIC = '/imu/data'

STYLE = {   # name: (legend label, colour)
    'truth': ('Ground truth', 'tab:green'),
    'wheel': ('Wheel odometry', 'tab:red'),
    'ekf':   ('EKF (wheels + IMU)', 'tab:blue'),
    'imu':   ('Gyro only (integrated)', 'tab:gray'),
}


# Helpers

def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    # Heading (rotation about z) from a quaternion, in radians.
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    # Wrap angles in radians to [-pi, pi), so 179 vs -179 deg is a 2 deg error.
    return (angle + np.pi) % (2 * np.pi) - np.pi


# Reading the bag

def read_bag(path):
    # Return {'truth'|'wheel'|'ekf': rows of (t, x, y, yaw), 'imu': rows of (t, wz)}.
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    msg_types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

    rows = {'truth': [], 'wheel': [], 'ekf': [], 'imu': []}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic in ODOM_TOPICS:
            msg = deserialize_message(raw, msg_types[topic])
            p = msg.pose.pose
            rows[ODOM_TOPICS[topic]].append(
                (stamp_to_sec(msg.header.stamp), p.position.x, p.position.y,
                 yaw_from_quaternion(p.orientation)))
        elif topic == IMU_TOPIC:
            msg = deserialize_message(raw, msg_types[topic])
            rows['imu'].append((stamp_to_sec(msg.header.stamp), msg.angular_velocity.z))

    data = {}
    for name, r in rows.items():
        if r:
            arr = np.array(r)
            data[name] = arr[np.argsort(arr[:, 0])]   # sort by timestamp
    return data


# Computing errors

def evaluate(data):
    truth = data['truth']
    t, tx, ty = truth[:, 0], truth[:, 1], truth[:, 2]
    tyaw = np.unwrap(truth[:, 3]) # remove the jumps at +-180 deg so we can interpolate
    distance = float(np.sum(np.hypot(np.diff(tx), np.diff(ty))))

    metrics, errors = {}, {}

    for name in ('wheel', 'ekf'):
        if name not in data:
            continue
        est = data[name]
        keep = (t >= est[0, 0]) & (t <= est[-1, 0]) # only times both sources cover
        tt = t[keep]
        # Estimate's value at each ground-truth timestamp
        x = np.interp(tt, est[:, 0], est[:, 1])
        y = np.interp(tt, est[:, 0], est[:, 2])
        yaw = np.interp(tt, est[:, 0], np.unwrap(est[:, 3]))

        pos_err = np.hypot(x - tx[keep], y - ty[keep])
        yaw_err = np.degrees(wrap(yaw - tyaw[keep]))
        errors[name] = (tt, pos_err, yaw_err)
        metrics[name] = {
            'final_pos_err_m': float(pos_err[-1]),
            'rms_pos_err_m': float(np.sqrt(np.mean(pos_err ** 2))),
            'max_pos_err_m': float(np.max(pos_err)),
            'drift_pct': float(100 * pos_err[-1] / distance) if distance > 0 else 0.0,
            'final_heading_err_deg': float(yaw_err[-1]),
            'rms_heading_err_deg': float(np.sqrt(np.mean(yaw_err ** 2))),
        }

    if 'imu' in data:
        ti, wz = data['imu'][:, 0], data['imu'][:, 1]
        # Add up turning rate x time step (trapezoid rule), starting from the true heading
        steps = 0.5 * (wz[1:] + wz[:-1]) * np.diff(ti)
        yaw_gyro = np.interp(ti[0], t, tyaw) + np.concatenate(([0.0], np.cumsum(steps)))
        keep = (t >= ti[0]) & (t <= ti[-1])
        tt = t[keep]
        yaw_err = np.degrees(wrap(np.interp(tt, ti, yaw_gyro) - tyaw[keep]))
        errors['imu'] = (tt, None, yaw_err)
        metrics['imu'] = {
            'final_heading_err_deg': float(yaw_err[-1]),
            'rms_heading_err_deg': float(np.sqrt(np.mean(yaw_err ** 2))),
        }

    summary = {'distance_m': distance, 'duration_s': float(t[-1] - t[0])}
    return summary, metrics, errors


# Plots

def plot_paths(data, metrics, title, out_png):
    fig, ax = plt.subplots(figsize=(7, 7))
    for name, width in (('truth', 4.0), ('wheel', 1.8), ('ekf', 1.8)):
        if name not in data:
            continue
        label, colour = STYLE[name]
        if name in metrics:
            label += '  (final error ' + str(round(metrics[name]['final_pos_err_m'], 2)) + ' m)'
        ax.plot(data[name][:, 1], data[name][:, 2], color=colour, linewidth=width, label=label)
    ax.plot(0, 0, 'ko', markersize=7, label='Start')
    ax.set_aspect('equal') # 1 m looks the same in x and y
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(title)
    ax.legend(loc='best', fontsize=9)
    fig.savefig(out_png, dpi=150, bbox_inches='tight')
    plt.close(fig)


def plot_errors(errors, title, out_png):
    fig, (ax_pos, ax_yaw) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    t0 = min(e[0][0] for e in errors.values())
    for name, (tt, pos_err, yaw_err) in errors.items():
        label, colour = STYLE[name]
        style = '--' if name == 'imu' else '-'
        if pos_err is not None:
            ax_pos.plot(tt - t0, pos_err, color=colour, label=label)
        ax_yaw.plot(tt - t0, yaw_err, color=colour, linestyle=style, label=label)
    ax_pos.set_ylabel('Position error (m)')
    ax_yaw.set_ylabel('Heading error (deg)')
    ax_yaw.set_xlabel('Time (s, simulation)')
    for ax in (ax_pos, ax_yaw):
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
    ax_pos.set_title(title)
    fig.savefig(out_png, dpi=150, bbox_inches='tight')
    plt.close(fig)


# Main

def main():
    parser = argparse.ArgumentParser(description='Evaluate a recorded run against ground truth.')
    parser.add_argument('bag', help='path to the recorded run, e.g. runs/square_1')
    parser.add_argument('--out', default='results', help='folder for plots and metrics')
    parser.add_argument('--title', default=None, help='plot title (default: run name)')
    args = parser.parse_args()

    run_name = os.path.basename(os.path.normpath(args.bag))
    title = args.title or run_name
    os.makedirs(args.out, exist_ok=True)

    data = read_bag(args.bag)
    if 'truth' not in data:
        raise SystemExit('No /ground_truth/odom messages in this bag.')

    summary, metrics, errors = evaluate(data)

    print()
    print('Run:', run_name)
    print('Distance driven:', round(summary['distance_m'], 2), 'm')
    print('Duration:', round(summary['duration_s'], 1), 's (sim time)')

    for name, m in metrics.items():
        print()
        print(STYLE[name][0])
        if 'final_pos_err_m' in m:
            print('  final position error: ', round(m['final_pos_err_m'], 3), 'm')
            print('  RMS position error:   ', round(m['rms_pos_err_m'], 3), 'm')
            print('  max position error:   ', round(m['max_pos_err_m'], 3), 'm')
            print('  drift:                ', round(m['drift_pct'], 1), '% of distance')
        print('  final heading error:  ', round(m['final_heading_err_deg'], 1), 'deg')
        print('  RMS heading error:    ', round(m['rms_heading_err_deg'], 1), 'deg')

    paths_png = os.path.join(args.out, run_name + '_paths.png')
    errors_png = os.path.join(args.out, run_name + '_errors.png')
    metrics_json = os.path.join(args.out, run_name + '_metrics.json')

    plot_paths(data, metrics, title, paths_png)
    plot_errors(errors, title, errors_png)

    results = {'run': run_name}
    results.update(summary)
    results['metrics'] = metrics
    with open(metrics_json, 'w') as f:
        json.dump(results, f, indent=2)

    print()
    print('Saved:')
    print(' ', paths_png)
    print(' ', errors_png)
    print(' ', metrics_json)

if __name__ == '__main__':
    main()