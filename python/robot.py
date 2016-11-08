import time

import numpy as np
import serial

from util import normalize_sensor_readings, safe_arctan
from wall import Wall
import arena


class Robot:
    def __init__(self, particle_filter=None, arena=None):
        self.conn = self._open_connection()
        self.following_wall = Wall.NONE
        self.__turning_to_evade = False
        self.move_list = [dict(left_speed=0, right_speed=0)]
        self.current_speed = (0, 0)
        self.pose = [0, 0, 0]
        self.arena = arena
        self.prev_count = (0, 0)

        self.set_counts(0, 0)
        self.particle_filter = particle_filter

    CURVE_LEFT_VAL = (11, 13)
    CURVE_RIGHT_VAL = CURVE_LEFT_VAL[::-1]

    HARD_LEFT_VAL = (-4, 4)
    HARD_RIGHT_VAL = HARD_LEFT_VAL[::-1]

    OBSTACLE_READING_MIN_LEFT = 150
    OBSTACLE_READING_MIN_RIGHT = 150

    WALL_FOLLOWING_MIN = 150

    TICKS_TO_CM = .008
    DEGREES_TO_TICKS = 815.0 / 180
    # DEGREES_TO_TICKS = 1036.0 / 180 # Shelob

    @property
    def turning_to_evade(self):
        return self.__turning_to_evade

    @turning_to_evade.setter
    def turning_to_evade(self, val):
        if val is True:
            self.following_wall = Wall.NONE

        self.__turning_to_evade = val

    def _open_connection(self, port="/dev/ttyS0", baudrate=9600, stopbits=2, timeout=1, **kwargs):
        s = serial.Serial(port=port, baudrate=baudrate, stopbits=stopbits, timeout=timeout, **kwargs)

        if not s.isOpen():
            s.open()

        return s

    def _close_connection(self):
        if self.conn.isOpen():
            self.conn.close()

    def _send_command(self, command, verbose=False):
        if self.conn.inWaiting() > 0:

            if verbose:
                print "WARNING! Messages were waiting to be read!"
                print "This may be indicative of a problem elsewhere in your code."

            while self.conn.inWaiting() > 0:
                message = self.conn.readline()[:-1]

                if verbose:
                    print message

        if command[-1] != "\n":
            command += "\n"
        self.conn.write(command)

        answer = self.conn.readline()

        # now we can check if the response from the Khepera matches our
        # expectations
        if verbose:
            print "SENT     : " + command[:-1]
            print "RECEIVED : " + answer[:-1]

            if len(answer) < 1:
                print "WARNING! No response received!"

            elif answer[0] != command[0].lower():
                print "WARNING! Response does not match issued command!"

        return answer

    def _parse_sensor_string(self, sensor_string):
        try:
            sensor_string = sensor_string[2:].rstrip(' \t\n\r')
            sensor_vals = [int(ss) for ss in sensor_string.split(",")]
            return sensor_vals
        except Exception:
            print 'PARSE ISSUE'
            return -1

    def _set_speeds(self, left, right, homing=False):
        # if not homing and self.current_speed == (left, right):
        #     return

        counts = self.read_wheel_counts()

        left_count = counts['left'] - self.prev_count[0]
        right_count = counts['right'] - self.prev_count[1]

        self.move_list[-1]['left_wheel_count'] = left_count
        self.move_list[-1]['right_wheel_count'] = right_count

        self.update_pose()

        self.move_list.append(dict(left_speed=left, right_speed=right))

        if self.arena:
            mean_count = (left_count + right_count) / 2.0
            cm = mean_count * self.TICKS_TO_CM

            sensor_count = normalize_sensor_readings(self.read_ir())

            self.arena.add_straight(cm)
            self.particle_filter.go(cm, 0, np.mean((sensor_count[3], sensor_count[4])))

        self.prev_count = (counts['left'], counts['right'])

        self.current_speed = (left, right)
        return self._send_command("D," + str(int(left)) + "," + str(int(right)))

    def _toggle_leds(self):
        self._send_command("L,0,2")
        self._send_command("L,1,2")

    def blink_leds(self):
        for i in range(6):
            self._toggle_leds()
            time.sleep(.05)

    def set_angle(self, angle):
        self.stop()
        time.sleep(.5)

        delta_angle = (angle - self.pose[2]) % 360
        counts = delta_angle * self.DEGREES_TO_TICKS

        self.set_wheel_positions(int(-1 * counts), int(counts))
        self.pose[2] = angle % 360

        time.sleep(2)

    def go_home(self):
        tol = 50
        while abs(self.pose[0]) > tol or abs(self.pose[1]) > tol:
            ir_result = self.read_ir()

            if self.continue_turning(ir_result) or self.avoid_obstacle(ir_result):
                print "turning"
                time.sleep(.02)

            else:
                angle = 180 + safe_arctan(float(self.pose[0]), float(self.pose[1])) * 180 / np.pi

                print "POSE: " + str(self.pose)
                print "ANGLE: " + str(angle)

                if abs(self.pose[2] - angle) > 10:
                    self.set_angle(angle)

                self.go(6, homing=True)
                time.sleep(.02)

        self.stop()

    def face_home(self):
        if self.arena:
            dx = self.arena.robot_x - self.arena.home_x
            dy = self.arena.robot_y - self.arena.home_y
            # angle = 180 + safe_arctan(float(dx), float(dy)) * 180 / np.pi
            angle = 180 + np.arctan2(dy, dx) * 180 / np.pi
            dangle = (angle - self.arena.robot_angle) % 360
            if dangle > 180:
                dangle -= 360
            print "\nangle to turn: " + str(dangle)
            if abs(dangle) > 10:
                self.turn_at_angle(dangle)

    def distance_home(self):
        if self.arena:
            dx = self.arena.robot_x - self.arena.home_x
            dy = self.arena.robot_y - self.arena.home_y
            print 'location = (' + str(dx) + ', ' + str(dy) +')'
            return np.sqrt(dx * dx + dy * dy)

    def read_ir(self):
        ir_string = self._send_command("N")

        try:
            return self._parse_sensor_string(ir_string)

        except ValueError:
            return self.read_ir()

    def go(self, speed, homing=False, wonky=False):
        if wonky and speed != 0:
            return self._set_speeds(speed, speed+4, homing)
        return self._set_speeds(speed, speed, homing)

    def stop(self, emergency=False):
        if emergency:
            self._send_command("D,0,0")
        return self.go(0)

    def turn(self, left, right):
        return self._set_speeds(left, right)

    def read_wheel_counts(self):
        count_string = self._send_command("H")
        count_list = self._parse_sensor_string(count_string)
        try:
            return dict(left=float(count_list[0]), right=float(count_list[1]))
        except TypeError:
            return self.read_wheel_counts()

    def set_counts(self, left_count, right_count):
        return self._send_command("G," + str(left_count) + "," + str(right_count))

    def set_wheel_positions(self, left_count, right_count):
        self.set_counts(0, 0)
        counts = self._parse_sensor_string(self._send_command("H"))

        return self._send_command("C," + str(int(counts[0] + left_count)) + "," + str(int(counts[1] + right_count)))

    def turn_at_angle(self, degrees):
        self.stop()
        counts = (1025 * degrees) / 180
        self.set_wheel_positions(-counts, counts)
        time.sleep(1)

        sensor_count = normalize_sensor_readings(self.read_ir())
        self.particle_filter.go(0, degrees, np.mean((sensor_count[3], sensor_count[4])))

        self.pose[2] = (self.pose[2] + degrees) % 360
        if self.arena:
            self.arena.add_angle(degrees)

    def going_to_hit_obstacle(self):
        """ Should (hopefully) supersede #avoid_obstacle """

        distances = normalize_sensor_readings(self.read_ir())

        print "DISTANCES: " + str(distances)

        if (distances[0] <= 1 or distances[1] <= 2.5 or
            distances[2] <= 3 or distances[3] <= 3 or
            distances[4] <= 2.5 or distances[5] <= 1):

            if np.argmin(distances[:5]) <= 2:
                return 1
            return 2

    def get_distances(self, inds):
        distances = normalize_sensor_readings(self.read_ir())
        dists = []
        for i in inds:
            dists.append(distances[i])
        return dists

    def update_pose(self):
        move = self.move_list[-1]

        # robot moves straight
        if move['left_speed'] == move['right_speed']:
            distance = .08 * np.mean([move['left_wheel_count'], move['right_wheel_count']])
            self.pose[0] += distance * np.cos(self.pose[2] * np.pi / 180)
            self.pose[1] += distance * np.sin(self.pose[2] * np.pi / 180)

        # robot turns in place
        elif abs(move['left_speed']) == abs(move['right_speed']):
            print "rotating"
            # wheel counts aren't perfect, so take the mean to get the turn
            distance = np.mean([abs(move['left_wheel_count']), abs(move['right_wheel_count'])])

            # 1045 wheel turns is 180 degrees, so use this ratio to determine the angle
            # 180 / 1045 = angle / distance
            angle = (180 * distance) / 1025
            # robot turns right
            if move['left_speed'] > move['right_speed']:
                angle *= -1
            self.pose[2] = (self.pose[2] + angle) % 360

        # robot turns and moves at the same time
        else:
            print "turning"
            if move['left_speed'] > move['right_speed']:
                # r = 50.0 / (move['left_wheel_count'] / move['right_wheel_count'] - 1)
                r = 410.0 / 2
                theta = move['right_wheel_count'] * .08 / (2 * np.pi * r) * 360
                # self.pose[1] -= r * np.sin(theta * np.pi / 180) * np.sin(self.pose[2] * np.pi / 180)
                # self.pose[1] -= r * (1 - np.cos(theta * np.pi / 180)) * np.cos(self.pose[2] * np.pi / 180)
                dely = 1 - np.cos(theta * np.pi / 180)
                th = np.arcsin(dely)
                thtot = th + theta * np.pi / 180
                self.pose[1] += r * np.sin(thtot)

            else:
                # r = 50.0 / (move['right_wheel_count'] / move['left_wheel_count'] - 1)
                r = 410.0 / 2
                theta = move['left_wheel_count'] * .08 / (2 * np.pi * r) * 360
                # self.pose[1] += r * np.sin(theta * np.pi / 180) * np.sin(self.pose[2] * np.pi / 180)
                # self.pose[1] += r * (1 - np.cos(theta * np.pi / 180)) * np.cos(self.pose[2] * np.pi / 180)
                dely = 1 - np.cos(theta * np.pi / 180)
                th = np.arcsin(dely)
                thtot = th + theta * np.pi / 180
                self.pose[1] -= r * np.sin(thtot)

            self.pose[0] += r * np.sin(theta * np.pi / 180) * np.cos(self.pose[2] * np.pi / 180)
            self.pose[2] += theta
            print "r = " + str(r)
            print "theta = " + str(theta)

    def avoid_obstacle(self, ir_result):
        """ DEPRECATED """

        left_front_sensors = ir_result[1:3]
        right_front_sensors = ir_result[3:5]

        def stop_following_and_evade(turn):
            self.turning_to_evade = True
            self.turn(*turn)

        left_front_sensors_read_obstacle = \
            any([sensor > self.OBSTACLE_READING_MIN_LEFT for sensor in left_front_sensors])
        right_front_sensors_read_obstacle = \
            any([sensor > self.OBSTACLE_READING_MIN_RIGHT for sensor in right_front_sensors])

        if left_front_sensors_read_obstacle or right_front_sensors_read_obstacle:
            if self.following_wall == Wall.LEFT:
                stop_following_and_evade(self.HARD_RIGHT_VAL)

                return True

            elif self.following_wall == Wall.RIGHT:
                stop_following_and_evade(self.HARD_LEFT_VAL)

                return True

            if left_front_sensors_read_obstacle:
                stop_following_and_evade(self.HARD_RIGHT_VAL)

                return True

            if right_front_sensors_read_obstacle:
                stop_following_and_evade(self.HARD_LEFT_VAL)

                return True

        return False

    def adjust_for_wall(self, ir_result):
        """ DEPRECATED """

        left_sensor_reading = ir_result[0]
        right_sensor_reading = ir_result[5]
        min_threshold = 200
        max_threshold = 300

        if self.following_wall == Wall.LEFT:
            if left_sensor_reading > max_threshold:
                self.turn(*self.CURVE_RIGHT_VAL)

                return True

            if left_sensor_reading < min_threshold:
                self.turn(*self.CURVE_LEFT_VAL)

                return True

        if self.following_wall == Wall.RIGHT:
            if right_sensor_reading > max_threshold:
                self.turn(*self.CURVE_LEFT_VAL)

                return True

            if right_sensor_reading < min_threshold:
                self.turn(*self.CURVE_RIGHT_VAL)

                return True

        return False

    def set_following_wall(self, ir_result):
        """ DEPRECATED """

        if ir_result[0] > self.WALL_FOLLOWING_MIN:
            self.following_wall = Wall.LEFT

        elif ir_result[5] > self.WALL_FOLLOWING_MIN:
            self.following_wall = Wall.RIGHT

    def continue_turning(self, ir_result):
        """ DEPRECATED """

        if self.turning_to_evade and (any([sensor > self.OBSTACLE_READING_MIN_LEFT for sensor in ir_result[2:3]]) or
                                          any([sensor > self.OBSTACLE_READING_MIN_RIGHT for sensor in ir_result[3:4]])):
            return True

        self.turning_to_evade = False

        return False

    def pinpoint_home(self):
        dangle = (270 - self.arena.robot_angle) % 360
        if dangle > 180:
            dangle -= 360
        if abs(dangle) > 10:
            self.turn_at_angle(dangle)

        def front_touching():
            dists = self.get_distances([2, 3])
            if dists[0] < 1.5 or dists[1] < 1.5:
                return True

        while not front_touching():
            self.go(6)
            time.sleep(.02)
        while self.get_distances([4])[0] <= 0.1:
            self.turn_at_angle(20)
        while not front_touching():
            self.go(10)
            time.sleep(.02)
        while front_touching():
            self.turn_at_angle(20)
        self.go(10)
        time.sleep(1)
        self.turn_at_angle(90)
        self.go(10)
        time.sleep(1)
        self.turn_at_angle(-90)

