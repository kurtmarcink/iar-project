#!/usr/bin/env python

import logging
import time
import traceback

from arena import build_arena
# from particle_filter import ParticleFilter
from robot import Robot


def main():
    arena = build_arena('arena_16_small.bmp')
    arena.robot_x = 67
    arena.robot_y = 15

    robot = Robot(arena)

    landmarks = arena.get_landmarks_in_grid()
    initial_pos = arena.get_robot_in_grid() + (0,)

    # pf = ParticleFilter(initial_pos, landmarks, 5000)

    def check_hit_something(turning=0):
        print 'turning = ' + str(turning)
        turn = robot.going_to_hit_obstacle()
        if turn: # Blocked
            if turning == 1: # Continue turning away from left
                robot.turn_at_angle(-20)
                check_hit_something(turning=turning)
            elif turning == 2: # Continue turning away from right
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
            while True:
                robot.go(10)

                # robot.arena.show(wait_time=20)
                time.sleep(.02)
                check_hit_something()

        except KeyboardInterrupt:
            go_home()


    def go_home():
        robot.stop()
        robot.arena.mark_food()
        try:
            while robot.distance_home() > 10:
                print 'Distance to home: ' + str(robot.distance_home()) + ' cm'
                robot.face_home()
                check_hit_something()
                robot.go(10)
                # robot.arena.show(wait_time=20)
                time.sleep(.02)

            robot.stop()
            print('HOME')
            robot.blink_leds()


        except KeyboardInterrupt:
            robot.stop()

    try:
        search_for_food()
        robot.pinpoint_home()
        # while True:
        #     print robot.get_distances(range(6))

    except Exception:
        logging.error(traceback.format_exc())

    finally:
        robot.stop(emergency=True)



if __name__ == '__main__':
    main()
