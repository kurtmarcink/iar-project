#!/usr/bin/env python

import logging
import time
import traceback

from arena import build_arena
from particle_filter import ParticleFilter
from robot import Robot


def main():
    arena = build_arena('arena_16_small.bmp')
    arena.robot_x = 67
    arena.robot_y = 15

    robot = Robot()

    landmarks = arena.get_landmarks()
    initial_pos = arena.get_robot_in_grid() + (0,)

    # pf = ParticleFilter(initial_pos, landmarks, 5000)

    def search_for_food():
        try:
            start_time = time.time()
            while True:
                robot.go(10)

                def check_hit_something():
                    if robot.going_to_hit_obstacle():
                        robot.stop()
                        time.sleep(.5)
                        robot.turn_at_angle(20)
                        check_hit_something()

                time.sleep(.2)
                check_hit_something()

        except KeyboardInterrupt:
            go_home()

        except Exception:
            logging.error(traceback.format_exc())

    def go_home():
        try:
            # TODO: going home logic

            robot.blink_leds()
            search_for_food()

        except KeyboardInterrupt:
            robot.stop()

if __name__ == '__main__':
    main()
