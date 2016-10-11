#!usr/bin/env/python

from robot import Robot

import time


def main():
    start_time = time.time()
    robot = Robot()
    robot.go(12)

    try:
        while time.time() - start_time <= 20:
            ir_result = robot.read_ir()

            if robot.continue_turning(ir_result) or robot.avoid_obstacle(ir_result):
                time.sleep(.02)

            else:
                robot.set_following_wall(ir_result)

                if not robot.adjust_for_wall(ir_result):
                    robot.go(12)

                time.sleep(.02)

        robot.stop()
        time.sleep(.5)
        robot.set_wheel_positions(1045, -1045)
        time.sleep(2)

        print len(robot.wheel_count_list)
        for i in reversed(robot.wheel_count_list[1:-1]):
            robot._set_speeds(i[1], i[0], count_wheels=False)

            while True:
                wheel_count = robot.read_counts()

                if abs(wheel_count[0]) >= abs(i[3]) and abs(wheel_count[1]) >= abs(i[2]):
                    robot._set_speeds(0, 0, count_wheels=False)
                    break

                time.sleep(.02)

    except KeyboardInterrupt:
        robot.stop()

    except:
        robot.stop()

if __name__ == "__main__":
    main()
