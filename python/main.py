#!/usr/bin/env python

import logging
import pprint
import time
import traceback
import turtle

import numpy as np

from robot import Robot


def main():
    start_time = time.time()
    robot = Robot()
#    robot.set_wheel_positions(1030,-1030)
    robot.go(10)
    time.sleep(.02)

    try:
        while time.time() < start_time + 30:
            ir_result = robot.read_ir()

            if robot.continue_turning(ir_result) or robot.avoid_obstacle(ir_result):
                time.sleep(.02)

            else:
                robot.set_following_wall(ir_result)

                if not robot.adjust_for_wall(ir_result):
                    robot.go(10)

                time.sleep(.02)

        robot.stop()
        time.sleep(.5)
        robot.set_wheel_positions(1036, -1036)
        time.sleep(2.5)

        forward_move_list = list(robot.move_list)
        backtracking_instructions = reversed(robot.move_list[1:-1])
        for move in backtracking_instructions:
            robot._set_speeds(move['right_speed'], move['left_speed'])

            while True:
                wheel_counts = robot.read_wheel_counts()

                # # robot._set_speeds(0, 0, count_wheels=False)
                # # robot.set_counts(0, 0)
                # time.sleep(.02)

                # time.sleep(.02)

                if abs(move['right_speed']) != abs(move['left_speed']):

                    if abs(wheel_counts['left']) > 3000 or abs(wheel_counts['right']) > 3000:

                        if abs(wheel_counts['left']) / abs(move['right_wheel_count'] > .70) or \
                            abs(wheel_counts['right']) / abs(move['left_wheel_count'] > .70):
                            break
                    elif (abs(wheel_counts['left']) >= (abs(move['right_wheel_count']) - 150) or
                                abs(wheel_counts['right']) >= (abs(move['left_wheel_count']) - 150)):
                        break

                if (abs(wheel_counts['left']) >= (abs(move['right_wheel_count'])) or
                            abs(wheel_counts['right']) >= (abs(move['left_wheel_count']))):
                    break


        robot.stop()

        backtracking_move_list = list(robot.move_list[len(forward_move_list):])

        t = turtle.Turtle()
        t.screen.setup(width=.9, height=.9)
        t.speed(0)
        t.dot(20, 'red')

        pp = pprint.PrettyPrinter(indent=4)

        print "FORWARD"
        pp.pprint(forward_move_list)
        trace_robot(forward_move_list[1:-1], t, 'red')

        t.left(180)
        t.dot(20, 'purple')

        print "BACK"
        pp.pprint(list(reversed(backtracking_move_list)))

        trace_robot(backtracking_move_list[:-1], t, 'blue')

        t.dot(20, 'blue')

        print "turtle pos: " + str(t.pos())
        print "turtle angle to origin: " + str(t.towards(0,0))

        correct_posn(robot, t.xcor(), t.ycor())

        try:
            while True:
                time.sleep(.02)

        except KeyboardInterrupt:
            pass

    except KeyboardInterrupt:
        robot.stop()

    except Exception as e:
        logging.error(traceback.format_exc())

    finally:
        robot.stop()

def correct_posn(robot, x, y):
    conv = 30.
    tol = 40
    if abs(y) > tol:
        if y < 0:
            robot.set_angle(90)
        else:
            robot.set_angle(270)
        robot.go(4)
        time.sleep(abs(y) / conv)
        robot.stop()
    if abs(x) > tol:
        if x < 0:
            robot.set_angle(0)
        else:
            robot.set_angle(180)
        robot.go(4)
        time.sleep(abs(x) / conv)
        robot.stop()


def trace_robot(move_list, t, dot_color):
    for move in move_list:
        # robot moves straight
        if move['left_speed'] == move['right_speed']:
            distance = np.mean([move['left_wheel_count'], move['right_wheel_count']])

            t.forward(distance / 10)

        # robot turns in place
        elif abs(move['left_speed']) == abs(move['right_speed']):
            # wheel counts aren't perfect, so take the mean to get the turn
            # distance = np.mean()
            distance = np.mean([abs(move['left_wheel_count']), abs(move['right_wheel_count'])])

            # 1045 wheel turns is 180 degrees, so use this ratio to determine the angle
            # 180 / 1045 = angle / distance
            angle = (180 * distance) / 920
            # robot turns right
            if move['left_speed'] > move['right_speed']:
                t.right(angle)

            # robot turns left
            else:
                t.left(angle)

        # robot turns and moves at the same time
        else:
            # robot turns right
            if move['left_speed'] > move['right_speed']:
                t.circle(-440, .01 * move['right_wheel_count'])

            # robot turns left
            else:
                t.circle(440, .01 * move['left_wheel_count'])

        t.dot(10, dot_color)


if __name__ == '__main__':
    main()
