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

    def check_hit_something():
        if robot.going_to_hit_obstacle():
            robot.stop()
            time.sleep(.3)
            robot.turn_at_angle(20)
            check_hit_something()

    def search_for_food():
        try:
            start_time = time.time()
            while True:
                robot.go(10)

                # robot.arena.show(wait_time=200)
                time.sleep(.02)
                check_hit_something()

        except KeyboardInterrupt:
            go_home()


    def go_home():
        try:
            while robot.distance_home > 10:
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

    except Exception:
        logging.error(traceback.format_exc())

    finally:
        robot.stop()



if __name__ == '__main__':
    main()
