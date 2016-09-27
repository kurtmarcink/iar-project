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


if __name__ == "__main__":
    # if this file is run as a script, it will run through some function calls
    serial = open_connection()

    print "\nResetting wheel encoders!"
    set_counts(serial, 0, 0)

    print "\nGoing forwards for 1 second!"
    go(serial, 2)
    time.sleep(1)

    print "\nTurning for 1 second!"
    turn(serial, -2, 2)
    time.sleep(1)

    print "\nStopping!"
    stop(serial)

    print "\nReading wheel encoders!"
    print "PARSED   : " + str(read_counts(serial))

    print "\nCollecting reflected IR readings!"
    print "PARSED   : " + str(read_ir(serial))

    print "\nCollecting ambient IR readings!"
    print "PARSED   : " + str(read_ambient(serial))
