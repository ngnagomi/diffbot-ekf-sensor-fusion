"""
Print comparison table from all *_metrics.json files in a folder.
"""
import glob
import json
import os
import sys


def cell(value, unit, width):
    # A number rounded and padded to a fixed width.
    if value is None:
        return '-'.rjust(width)
    return (str(round(value, 3)) + unit).rjust(width)


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else 'results'
    paths = sorted(glob.glob(os.path.join(folder, '*_metrics.json')))
    if not paths:
        print('No *_metrics.json files in', folder)
        return

    print('Run'.ljust(26) + 'Wheels final'.rjust(14) + 'EKF final'.rjust(12)
          + 'EKF RMS'.rjust(12) + 'EKF heading'.rjust(14))
    for path in paths:
        with open(path) as f:
            result = json.load(f)
        m = result['metrics']
        wheels = m.get('wheel_in', m.get('wheel', {})) 
        ekf = m.get('ekf', {})
        print(result['run'].ljust(26)
              + cell(wheels.get('final_pos_err_m'), ' m', 14)
              + cell(ekf.get('final_pos_err_m'), ' m', 12)
              + cell(ekf.get('rms_pos_err_m'), ' m', 12)
              + cell(ekf.get('final_heading_err_deg'), ' deg', 14))


if __name__ == '__main__':
    main()