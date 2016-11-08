#!/usr/bin/env python

import logging
import time
import traceback

from arena import build_arena
from particle_filter import ParticleFilter
from robot import Robot


def main():
    arena = build_arena('arena_16_small.bmp')

    pf = ParticleFilter(500, arena)

    robot = Robot(pf, arena)
    robot.set_counts(0, 0)

    arena.show()

    def check_hit_something(turning=0):
        # print 'turning = ' + str(turning)
        turn = robot.going_to_hit_obstacle()
        if turn:  # Blocked
            arena.show()
            current_count = robot.read_wheel_counts()

            if turning == 1:  # Continue turning away from left
                robot.turn_at_angle(-20)
                check_hit_something(turning=turning)

            elif turning == 2:  # Continue turning away from right
                robot.turn_at_angle(20)
                check_hit_something(turning=turning)

            else:
                robot.stop()
                time.sleep(.3)
                if turn == 1: # Obstacle on left
                    robot.turn_at_angle(-20)
                else: # Obstacle on right
                    robot.turn_at_angle(20)
                check_hit_something(turning=turn)

    def search_for_food():
        try:
            start_time = time.time()

            while True: # time.time() < start_time + 6:
                robot.go(4)

                time.sleep(.02)
                check_hit_something()

                # time.sleep(.02)
                arena.show()

        except KeyboardInterrupt:
            go_home()

    def go_home():
        robot.stop()
        robot.arena.mark_food()
        try:
            while robot.distance_home() > 5:
                print 'Distance to home: ' + str(robot.distance_home()) + ' cm'
                robot.face_home()
                check_hit_something()
                robot.go(10)
                # robot.arena.show(wait_time=20)
                # time.sleep(.02)
                arena.show()

            robot.stop()
            # robot.pinpoint_home()
            print('HOME')
            robot.blink_leds()


        except KeyboardInterrupt:
            robot.stop()

    def go_to_food():
        robot.stop()
        try:
            while robot.distance(robot.food[0]) > 5:
                robot.face_food()
                check_hit_something()
                robot.go(10)
                # robot.arena.show(wait_time=20)
                # time.sleep(.02)
                arena.show()
            search_for_food()
        except KeyboardInterrupt:
            go_home()


    try:
        search_for_food()
        # robot.pinpoint_home()
        while True:
            go_to_food()
        corner_to_home(robot)

    except Exception:
        logging.error(traceback.format_exc())

    finally:
        robot.stop(emergency=True)



if __name__ == '__main__':
    main()
