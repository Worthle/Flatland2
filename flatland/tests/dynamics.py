#!/usr/bin/env python3
"""Check the NARX replay tool against analytic first-order responses."""
import csv
import io
from pathlib import Path
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


def main():
    share = Path(get_package_share_directory('flatland'))
    commands = [(0., 0.)]*3 + [(.6, .2)]*25 + [(-.2, -.1)]*12 + [(0., 0.)]*20
    with tempfile.TemporaryDirectory(prefix='flatland_dynamics_') as folder:
        dataset = Path(folder)/'commands.csv'
        with dataset.open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['cmd_speed', 'cmd_steering_angle'])
            writer.writerows(commands)
        result = subprocess.run(['ros2', 'run', 'flatland_plugins', 'narx_sim_tool',
            str(share/'config/forklift_dynamics.yaml'), str(dataset), '0'],
            text=True, capture_output=True)
        assert result.returncode == 0, result.stderr
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        assert len(rows) == len(commands)
        linear = angular = 0.0
        previous_speed = previous_steering = 0.0
        for row, (speed, steering) in zip(rows, commands):
            linear = .7*linear+.3*previous_speed
            angular = .7*angular+(.3/1.1)*previous_speed*previous_steering
            assert abs(float(row['meas_linear_velocity'])-linear) < 1e-10
            assert abs(float(row['meas_angular_velocity'])-angular) < 1e-10
            previous_speed, previous_steering = speed, steering
        dataset.write_text('cmd_speed,cmd_steering_angle\r\n0.6\r\n')
        invalid = subprocess.run(['ros2', 'run', 'flatland_plugins', 'narx_sim_tool',
            str(share/'config/forklift_dynamics.yaml'), str(dataset), '0'],
            text=True, capture_output=True)
        assert invalid.returncode != 0 and 'column count' in invalid.stderr, invalid.stderr
    print('PASS: NARX forward, reverse and stop responses match the analytic recurrence; CRLF input and malformed-row handling')


if __name__ == '__main__':
    main()
