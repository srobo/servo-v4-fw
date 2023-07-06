#!/usr/bin/env python3
import argparse
import subprocess
import tempfile
from time import sleep
from pathlib import Path

import serial
from serial.serialutil import SerialException

from insert_serial import pad_serial, insert_bin_serial

BOARD_TYPE = 'SBv4B'
BOARD_VID = '1bda'
BOARD_PID = '0011'
SERIAL_NUM_ADDR = 0x1FE0


def dfu_flash_firmware(firmware):
    subprocess.run(['dfu-util', '-E', '1', '-d', f'{BOARD_VID}:{BOARD_PID}', '-D', firmware])

    sleep(1)


def serial_flash_firmware(port, firmware):
    print(
        "To flash the board using the factory bootloader you need to connect "
        "both the USB and serial ports. Once connected press the board's pushbutton")
    input('Press enter once this is done')

    asset_code = input("Enter asset code to bake into the bootloader (will have sr prepended): ")
    asset_code = 'sr' + asset_code.upper()
    print(f"Programming asset code: {asset_code}")

    with tempfile.TemporaryDirectory() as tmpdirname:
        tmpdir = Path(tmpdirname)

        # Apply asset code to bootloader
        stock_data = Path(firmware).read_bytes()
        data = insert_bin_serial(stock_data, pad_serial(asset_code, 16), SERIAL_NUM_ADDR)

        # write fw w/ serial num to temp file
        fw_file = tmpdir / 'main.bin'
        fw_file.write_bytes(data)

        # Flash bootloader+fw bundle w/ stm32flash
        # A slow speed is required with the the servo board serial connection
        subprocess.check_call(['stm32flash', '-b', '9600', '-w', str(fw_file), '-v', '-R', port])
    sleep(1)
    return asset_code


def verify_firmware(version, expected_serial=None):
    try:
        s = serial.serial_for_url(f'hwgrep://{BOARD_VID}:{BOARD_PID}', timeout=3, baudrate=115200)
        s.flush()
        s.write(b'*IDN?\n')
        identity_raw = s.readline()
        if not identity_raw.endswith(b'\n'):
            print(f"Board did not correctly respond to identify, returned: {identity_raw!r}")
            return False

        identity = identity_raw.decode('utf-8').strip().split(':')
        if identity[0] != 'Student Robotics':
            print("Incorrect manufacturer returned")
            return False
        if identity[1] != BOARD_TYPE:
            print("Incorrect board type returned")
            return False
        if identity[3] != version:
            print('Incorrect version returned')
            return False
        if expected_serial is not None and identity[2] != expected_serial:
            print(f"Serial number differs from expected, received {identity[2]!r}")
            return False

        print(f"Successfully flashed {identity[2]}")
        return identity[2]

    except SerialException:
        print("Failed to open serial port")
    return False


def log_success(log_file, asset_code):
    with open(log_file, 'a') as f:
        f.write(f"{asset_code}\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', help="Firmware file to flash")
    parser.add_argument('version', help="The version number the new firmware reports")
    parser.add_argument('-log', '--serial_log', default=None, help="File to store serials successfully flashed")
    parser.add_argument('--bootloader', action='store_true', help="Also Flash the bootloader to the board, this uses stm32flash")
    parser.add_argument('--port', default="", help="The serial port to use for flashing the bootloader")

    args = parser.parse_args()

    try:
        expected_asset = None
        while True:
            input('Press enter to flash servo board')
            if args.bootloader:
                try:
                    expected_asset = serial_flash_firmware(args.port, args.file)
                except subprocess.CalledProcessError:
                    print("Failed to flash firmware, try again")
                    continue
            else:
                dfu_flash_firmware(args.file)
            verified_code = verify_firmware(args.version, expected_asset)
            if verified_code is not False and args.serial_log is not None:
                log_success(args.serial_log, verified_code)
    except (KeyboardInterrupt, EOFError):
        return


if __name__ == '__main__':
    main()
