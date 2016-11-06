#!/usr/bin/env python

import time
from collections import OrderedDict

import numpy

from getch import getch
from robot import Robot


def main():

    robot = Robot()

    pos_dict = OrderedDict()

    pos_dict['front'] = (1, 2, 3, 4)
    pos_dict['left_corner'] = (0, 1, 2)
    pos_dict['left_side'] = (0, 1, 2)
    pos_dict['left_side_sideways'] = (0, 1, 2)
    pos_dict['right_corner'] = (3, 4, 5)
    pos_dict['right_side'] = (3, 4, 5)
    pos_dict['right_side_sideways'] = (3, 4, 5)
    pos_dict['rear'] = (6, 7)

    color = "circle"
    distance = raw_input('cm: ') + 'cm'

    def store_sensor_readings():
        for pos, sensors in pos_dict.items():
            print pos

            readings = []
            t_end = time.time() + 5

            while time.time() < t_end:
                ir = robot.read_ir()
                readings.append([ir[idx] for idx, val in enumerate(ir) if idx in sensors])
                time.sleep(.02)

            mean_arr = []
            std_arr = []

            for i in range(len(sensors)):
                readings_for_sensor = [el[i] for el in readings]
                mean_arr.append(numpy.mean(readings_for_sensor))
                std_arr.append(numpy.std(readings_for_sensor))

            file_name = "../sensor-stats/{}-{}.out".format(color, distance)

            with open(file_name, "a+") as f:
                f.write("META: {} - {} - {}\n".format(color, distance, pos))
                for i in range(len(sensors)):
                    f.write("SENSOR [{}]\tMEAN [{}]\tSD [{}]\tSIZE [{}]\n".format(sensors[i], mean_arr[i], std_arr[i], len(readings)))

                f.write('\n\n')

            print "Sensor Readings paused. Press any key to continue"

            inp = getch()

            if inp == "q":
                print "QUITTING"
                robot.stop()
                return readings

    store_sensor_readings()
    robot.stop()

if __name__ == '__main__':
    main()
