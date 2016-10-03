from wall import Wall

import serial


class Robot:
    def __init__(self):
        self.conn = self._open_connection()
        self.following_wall = Wall.NONE
        self.ir_result = None
        self.turning_to_evade = False

    def _open_connection(self, port="/dev/ttyS0", baudrate=9600, stopbits=2, timeout=1, **kwargs):
        s = serial.Serial(port=port, baudrate=baudrate, stopbits=stopbits, timeout=timeout, **kwargs)
        if not s.isOpen():
            s.open()
        return s

    def _close_connection(self):
        if self.conn.isOpen():
            self.conn.close()

    def _send_command(self, command, verbose=True):
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
            print "sensor_string = " + sensor_string
            sensor_string = sensor_string[2:].rstrip(' \t\n\r')

            # and cast the comma separated sensor readings to integers
            sensor_vals = [int(ss) for ss in sensor_string.split(",")]

            return sensor_vals

    def _set_speeds(self, left, right):
        # MAX = 127
        return self._send_command("D," + str(int(left)) + "," + str(int(right)))

    def read_ambient(self):
        ambient_string = self._send_command("O")

        return self._parse_sensor_string(ambient_string)

    def read_ir(self):
        ir_string = self._send_command("N")

        return self._parse_sensor_string(ir_string)

    def go(self, speed):
        return self._set_speeds(speed, speed)

    def stop(self):
        return self.go(0)

    def turn(self, left, right):
        return self._set_speeds(left, right)

    def read_counts(self):
        count_string = self._send_command("H")

        return self._parse_sensor_string(count_string)

    def set_counts(self, left_count, right_count):
        return self._send_command("G," + str(left_count) + "," + str(right_count))

    def set_wheel_positions(self, left_count, right_count):
        self.set_counts(0, 0)
        counts = self._parse_sensor_string(self._send_command("H"))

        return self._send_command("C," + str(counts[0] + left_count) + "," + str(counts[1] + right_count))

    def avoid_obstacle(self, ir_result):
        left_front_sensors = ir_result[1:3]
        right_front_sensors = ir_result[3:5]

        if any([sensor > 120 for sensor in left_front_sensors + right_front_sensors]):
            if self.following_wall == Wall.LEFT:
                self.turn(2, -2)
                self.following_wall = Wall.NONE
                self.turning_to_evade = True

                return True

            elif self.following_wall == Wall.RIGHT:
                self.following_wall = Wall.NONE
                self.turn(-2, 2)
                self.turning_to_evade = True

                return True

        if any([sensor > 120 for sensor in left_front_sensors]):
            self.following_wall = Wall.NONE
            self.turn(2, -2)
            self.turning_to_evade = True

            return True

        if any([sensor > 120 for sensor in right_front_sensors]):
            self.following_wall = Wall.NONE
            self.turn(-2, 2)
            self.turning_to_evade = True

            return True

    def adjust_for_wall(self, ir_result):
        left_sensor = ir_result[0]
        right_sensor = ir_result[5]

        if self.following_wall == Wall.LEFT:
            if left_sensor > 300:
                self.turn(4, 3)

                return True

            if left_sensor < 200:
                self.turn(3, 4)

                return True

        if self.following_wall == Wall.RIGHT:
            if right_sensor > 300:
                self.turn(3, 4)

                return True

            if right_sensor < 200:
                self.turn(4, 3)

                return True

        return False

    def check_wall_parallel(self, ir_result):
        if ir_result[0] > 150:
            self.following_wall = Wall.LEFT

        elif ir_result[5] > 150:
            self.following_wall = Wall.RIGHT

    def continue_turning(self, ir_result):
        if self.turning_to_evade and any([sensor > 120 for sensor in ir_result[1:5]]):
            return True

        self.turning_to_evade = False
        return False
