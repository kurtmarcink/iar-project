from wall import Wall

import serial


class Robot:
    def __init__(self):
        self.conn = self._open_connection()
        self.following_wall = Wall.NONE
        self.__turning_to_evade = False
        self.move_list = [{}]
        self.current_speed = (0, 0)

    CURVE_LEFT_VAL = (11, 13)
    CURVE_RIGHT_VAL = CURVE_LEFT_VAL[::-1]

    HARD_LEFT_VAL = (-4, 4)
    HARD_RIGHT_VAL = HARD_LEFT_VAL[::-1]

    OBSTACLE_READING_MIN_LEFT = 110
    OBSTACLE_READING_MIN_RIGHT = 130

    WALL_FOLLOWING_MIN = 150

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
        # we should check if we have built up a backlog of serial messages from the
        # Khepera. If there is a backlog, we should read out of the serial buffer
        # so that future communications aren't messed up.
        if self.conn.inWaiting() > 0:

            if verbose:
                print "WARNING! Messages were waiting to be read!"
                print "This may be indicative of a problem elsewhere in your code."

            while self.conn.inWaiting() > 0:
                message = self.conn.readline()[:-1]

                if verbose:
                    print message

        # we must make sure that the command string is followed by a newline
        # character before sending it to the Khepera
        if command[-1] != "\n":
            command += "\n"
        self.conn.write(command)

        # we check for a response from the Khepera
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
        if len(sensor_string) < 1:
            return -1

        else:
            # we need to remove some superfluous characters in the returned message
            sensor_string = sensor_string[2:].rstrip(' \t\n\r')

            # and cast the comma separated sensor readings to integers
            sensor_vals = [int(ss) for ss in sensor_string.split(",")[:6]]

            return sensor_vals

    def _set_speeds(self, left, right):
        if self.current_speed == (left, right):
            return

        counts = self.read_wheel_counts()
        self.move_list[-1]['left_wheel_count'] = counts['left']
        self.move_list[-1]['right_wheel_count'] = counts['right']
        self.move_list.append(dict(left_speed=left, right_speed=right))

        self.set_counts(0, 0)

        self.current_speed = (left, right)
        return self._send_command("D," + str(int(left)) + "," + str(int(right)))

    def read_ambient(self):
        ambient_string = self._send_command("O")

        return self._parse_sensor_string(ambient_string)

    def read_ir(self):
        ir_string = self._send_command("N")

        try:
            return self._parse_sensor_string(ir_string)

        except ValueError:
            return self.read_ir()

    def go(self, speed):
        return self._set_speeds(speed, speed)

    def stop(self):
        return self.go(0)

    def turn(self, left, right):
        return self._set_speeds(left, right)

    def read_wheel_counts(self):
        count_string = self._send_command("H")

        count_list = self._parse_sensor_string(count_string)

        return dict(left=count_list[0], right=count_list[1])

    def set_counts(self, left_count, right_count):
        return self._send_command("G," + str(left_count) + "," + str(right_count))

    def set_wheel_positions(self, left_count, right_count):
        self.set_counts(0, 0)
        counts = self._parse_sensor_string(self._send_command("H"))

        return self._send_command("C," + str(counts[0] + left_count) + "," + str(counts[1] + right_count))

    def avoid_obstacle(self, ir_result):
        left_front_sensors = ir_result[1:3]
        right_front_sensors = ir_result[3:5]

        def stop_following_and_evade(turn):
            self.turning_to_evade = True
            self.turn(*turn)

        left_front_sensors_read_obstacle = any([sensor > self.OBSTACLE_READING_MIN_LEFT for sensor in left_front_sensors])
        right_front_sensors_read_obstacle = any([sensor > self.OBSTACLE_READING_MIN_RIGHT for sensor in right_front_sensors])

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
        if ir_result[0] > self.WALL_FOLLOWING_MIN:
            self.following_wall = Wall.LEFT

        elif ir_result[5] > self.WALL_FOLLOWING_MIN:
            self.following_wall = Wall.RIGHT

    def continue_turning(self, ir_result):
        if self.turning_to_evade and (any([sensor > self.OBSTACLE_READING_MIN_LEFT for sensor in ir_result[2:3]]) or any([sensor > self.OBSTACLE_READING_MIN_RIGHT for sensor in ir_result[3:4]])):
            return True

        self.turning_to_evade = False
        return False
