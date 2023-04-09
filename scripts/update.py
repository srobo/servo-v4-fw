#!/usr/bin/env python3
import argparse
import subprocess
from time import sleep

import serial
from serial.serialutil import SerialException


def update_sb(firmware, version, log=None):
    res = subprocess.run(['dfu-util', '-E', '1', '-d', '1bda:0011', '-D', firmware])

    sleep(1)

    try:
        s = serial.serial_for_url('hwgrep://1bda:0011', timeout=3)
        s.write(b'*IDN?\n')
        identity_raw = s.readline()

        identity = identity_raw.decode('utf-8').strip().split(':')
        if identity[0] == 'Student Robotics' and identity[1] == 'SBv4B':
            if identity[3] != version:
                print('Incorrect version returned')
            else:
                print(f"Successfully flashed {identity[2]}")
                if log:
                    with open(log, 'a') as f:
                        f.write(f"{identity[2]}\n")
        else:
            print("Failed to flash board")
    except SerialException:
        print("Failed to open serial port")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', help="Firmware file to flash")
    parser.add_argument('version', help="The version number the new firmware reports")
    parser.add_argument('-log', '--serial_log', default=None, help="File to store serials successfully flashed")

    args = parser.parse_args()

    try:
        while True:
            input('Press enter to flash servo board')
            update_sb(args.file, args.version, args.serial_log)
    except (KeyboardInterrupt, EOFError):
        return


if __name__ == '__main__':
    main()
