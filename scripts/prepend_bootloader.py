#!/usr/bin/env python3
import argparse
from pathlib import Path

def prepend_bootloader(bootloader_file, fw_file, output_file):
    bootloader = Path(bootloader_file).read_bytes()
    firmware = Path(fw_file).read_bytes()

    output = bootloader + firmware

    Path(output_file).write_bytes(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bootloader', help="Bootloader file")
    parser.add_argument('firmware', help="Firmware file")
    parser.add_argument('output', help="Output file")

    args = parser.parse_args()

    prepend_bootloader(args.bootloader, args.firmware, args.output)


if __name__ == '__main__':
    main()
