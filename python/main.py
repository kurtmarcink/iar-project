#!usr/bin/env/python

from robot import Robot

import time


def main():
    robot = Robot()
    robot.go(4)

    try:
        while True:
            ir_result = robot.read_ir()

            if robot.continue_turning(ir_result) or robot.avoid_obstacle(ir_result):
                time.sleep(.2)
            else:
                robot.check_wall_parallel(ir_result)
                if not robot.adjust_for_wall(ir_result):
                    robot.go(4)
                time.sleep(.025)

    except KeyboardInterrupt:
        robot.stop()

if __name__ == "__main__":
    main()
