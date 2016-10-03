#!usr/bin/env/python


import time
import serial
from robot import Robot, Wall

"""
A collection of functions for interfacing with a Khepera-II robot over serial
line, intended for use by students on the Intelligent Autonomous Robotics (IAR)
course at the School of Informatics, University of Edinburgh.

If you find bugs, contact Dylan Ross (s0937976 [at] sms.ed.ac.uk)
"""


def open_connection(port="/dev/ttyS0", baudrate=9600,
                    stopbits=2, timeout=1, **kwargs):
    s = serial.Serial(port=port,
                      baudrate=baudrate,
                      stopbits=stopbits,
                      timeout=timeout,
                      **kwargs)
    if not s.isOpen():
        s.open()
    return s


def close_connection(robot):
    if robot.conn.isOpen():
        robot.conn.close()


def send_command(robot, command, verbose=True):
    # we should check if we have built up a backlog of serial messages from the
    # Khepera. If there is a backlog, we should read out of the serial buffer
    # so that future communications aren't messed up.
    if robot.conn.inWaiting() > 0:
        if verbose:
            print "WARNING! Messages were waiting to be read!"
            print "This may be indicative of a problem elsewhere in your code."
        while robot.conn.inWaiting() > 0:
            message = robot.conn.readline()[:-1]
            if verbose:
                print message

    # we must make sure that the command string is followed by a newline
    # character before sending it to the Khepera
    if command[-1] != "\n":
        command += "\n"
    robot.conn.write(command)

    # we check for a response from the Khepera
    answer = robot.conn.readline()

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


def set_speeds(robot, left, right):
    # MAX = 127
    return send_command(robot, "D," + str(int(left)) + "," + str(int(right)))


def go(robot, speed):
    return set_speeds(robot, speed, speed)


def stop(robot):
    return go(robot, 0)


def turn(robot, left, right):
    return set_speeds(robot, left, right)


def _parse_sensor_string(sensor_string):
    if len(sensor_string) < 1:
        return -1
    else:
        # we need to remove some superfluous characters in the returned message
        print "sensor_string = " + sensor_string
        sensor_string = sensor_string[2:].rstrip(' \t\n\r')

        # and cast the comma separated sensor readings to integers
        sensor_vals = [int(ss) for ss in sensor_string.split(",")]

        return sensor_vals


def read_ir(robot):
    ir_string = send_command(robot, "N")
    return _parse_sensor_string(ir_string)


def read_ambient(robot):
    ambient_string = send_command(robot, "O")
    return _parse_sensor_string(ambient_string)


def set_counts(s, left_count, right_count):
    return send_command(s, "G," + str(left_count) + "," + str(right_count))


def read_counts(robot):
    count_string = send_command(robot, "H")
    return _parse_sensor_string(count_string)


def set_wheel_positions(robot, left_count, right_count):
    set_counts(robot, 0, 0)
    counts = _parse_sensor_string(send_command(robot, "H"))
    return send_command(robot, "C," + str(counts[0] + left_count) + "," + str(counts[1] + right_count))


def avoid_obstacle(robot, ir_result):
    left_front_sensors = ir_result[1:3]
    right_front_sensors = ir_result[3:5]

    if any([sensor > 120 for sensor in left_front_sensors + right_front_sensors]):
        if robot.following_wall == Wall.LEFT:
            turn(robot, 2, -2)
            robot.following_wall = Wall.NONE
            robot.turning_to_evade = True
            return True
        elif robot.following_wall == Wall.RIGHT:
            robot.following_wall = Wall.NONE
            turn(robot, -2, 2)
            robot.turning_to_evade = True
            return True
    if any([sensor > 120 for sensor in left_front_sensors]):
        robot.following_wall = Wall.NONE
        turn(robot, 2, -2)
        robot.turning_to_evade = True
        return True
    if any([sensor > 120 for sensor in right_front_sensors]):
        robot.following_wall = Wall.NONE
        turn(robot, -2, 2)
        robot.turning_to_evade = True
        return True


def adjust_for_wall(robot, ir_result):
    left_sensor = ir_result[0]
    right_sensor = ir_result[5]

    if robot.following_wall == Wall.LEFT:
        if left_sensor > 300:
            turn(robot, 4, 3)
            return True
        if left_sensor < 200:
            turn(robot, 3, 4)
            return True

    if robot.following_wall == Wall.RIGHT:
        if right_sensor > 300:
            turn(robot, 3, 4)
            return True
        if right_sensor < 200:
            turn(robot, 4, 3)
            return True
    return False


def check_wall_parallel(robot, ir_result):
    if ir_result[0] > 150:
        robot.following_wall = Wall.LEFT
    elif ir_result[5] > 150:
        robot.following_wall = Wall.RIGHT


def continue_turning(robot, ir_result):
    if robot.turning_to_evade and any([sensor > 120 for sensor in ir_result[1:5]]):
        return True
    robot.turning_to_evade = False
    return False


def main():
    robot = Robot(open_connection())
    go(robot, 4)

    try:
        while True:
            ir_result = read_ir(robot)

            if continue_turning(robot, ir_result) or avoid_obstacle(robot, ir_result):
                time.sleep(.2)
            else:
                check_wall_parallel(robot, ir_result)
                if not adjust_for_wall(robot, ir_result):
                    go(robot, 4)
                time.sleep(.025)

    except KeyboardInterrupt:
        stop(robot)

if __name__ == "__main__":
    main()
