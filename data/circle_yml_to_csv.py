#
# See circle_plot.R for details.
#

import csv
import yaml

ANGLES = ['0.125', '0.25', '0.50', '0.75']

for angle_string in ANGLES:
    angle = float(angle_string)
    input_pathname = 'circle_{}.yml'.format(angle_string)
    output_pathname = 'circle_{}.csv'.format(angle_string)
    with open(input_pathname, 'rb') as input_file:
        data = yaml.load_all(input_file)
        with open(output_pathname, 'wb') as output_file:
            csv_writer = csv.writer(output_file)
            csv_writer.writerow(['steer_angle', 't', 'x', 'y'])
            for datum in data:
                if datum is None:
                    continue
                stamp = datum['header']['stamp']
                time = stamp['secs'] + stamp['nsecs'] * 1e-9
                position = datum['pose']['position']
                x, y = position['x'], position['y']
                csv_writer.writerow([angle_string, time, x, y])
