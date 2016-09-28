#!/usr/bin/env python

import time
import serial

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


def close_connection(s):
    if s.isOpen():
        s.close()


def send_command(s, command, verbose=True):
    # we should check if we have built up a backlog of serial messages from the
    # Khepera. If there is a backlog, we should read out of the serial buffer
    # so that future communications aren't messed up.
    if s.inWaiting() > 0:
        if verbose:
            print "WARNING! Messages were waiting to be read!"
            print "This may be indicative of a problem elsewhere in your code."
        while s.inWaiting() > 0:
            message = s.readline()[:-1]
            if verbose:
                print message

    # we must make sure that the command string is followed by a newline
    # character before sending it to the Khepera
    if command[-1] != "\n":
        command += "\n"
    s.write(command)

    # we check for a response from the Khepera
    answer = s.readline()

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


def set_speeds(s, left, right):
    return send_command(s, "D," + str(int(left)) + "," + str(int(right)))


def go(s, speed):
    return set_speeds(s, speed, speed)


def stop(s):
    return go(s, 0)


def turn(s, left, right):
    return set_speeds(s, left, right)


def _parse_sensor_string(sensor_string):
    if len(sensor_string) < 1:
        return -1
    else:
        # we need to remove some superfluous characters in the returned message
        sensor_string = sensor_string[2:-2]
        # and cast the comma separated sensor readings to integers
        sensor_vals = [int(ss) for ss in sensor_string.split(",")]
        return sensor_vals


def read_ir(s):
    ir_string = send_command(s, "N")
    return _parse_sensor_string(ir_string)


def read_ambient(s):
    ambient_string = send_command(s, "O")
    return _parse_sensor_string(ambient_string)


def set_counts(s, left_count, right_count):
    return send_command(s, "G," + str(left_count) + "," + str(right_count))


def read_counts(s):
    count_string = send_command(s, "H")
    return _parse_sensor_string(count_string)


def set_wheel_positions(s, left_count, right_count):
    counts = _parse_sensor_string(send_command(s, "H"))
    return send_command(s, "C," + str(counts[0] + left_count) + "," + str(counts[1] + right_count))


def turn_left(s):
    return set_wheel_positions(s, -510, 510)


def turn_right(s):
    return set_wheel_positions(s, 510, -510)


def is_blocked_ahead(s):
    ir_sum = sum(read_ir(s)[1:4])
    print ir_sum
    if ir_sum > 900:
        return True
    return False


if __name__ == "__main__":
    serial = open_connection()
    go(serial, 2)
    try:
        while(True):
            if is_blocked_ahead(serial):
                turn_right(serial)
                time.sleep(1)
                go(serial, 2)
    except KeyboardInterrupt:
        stop(serial)
