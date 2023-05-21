# Servo Board v4 Firmware

The Servo Board can be used to control up to 12 RC servos. Many devices are available that can be controlled as servos, such as RC motor speed controllers, and these can also be used with the board.

These are split into 2 groups:
- 0-7 have a fixed 5.5V output, supplied from the onboard regulator
- 8-11 have their output voltage supplied from the AUX header and can be provided up to 12 volts

## Instructions

Using a posix system, you require `make`, the `arm-none-eabi` toolchain and `git`.
Before attempting to build anything initialise all the submodules.
```shell
$ git submodule update --init --recursive
```

To build the main binary, run:
```shell
$ make
```
The binary will then be at `src/main.bin`.
This will also build the library libopencm3 the first time you run it.

This can be flashed to an attached servo board that has a bootloader using:
```shell
$ make -C src dfu
```
To use the `dfu` command you need to install dfu-utils. This is a cross-platform utility.

To build the bootloader, run:
```shell
$ make -C bootloader
```
The bootloader binary will then be at `bootloader/usb_dfu.bin`

## LEDs

There are 14 LEDs accessible from the firmware, 12 associated with servo outputs and 2 general status LEDs.
The 12 associated with servo outputs are lit when the corresponding output is enabled.
The blue status LED is lit once the MCU has initialised.
The red status LED is lit when there is an I2C communication error, likely due to missing 12V power.

## USB Interface

The Vendor ID is `1bda` (University of Southampton) and the product ID
is `0011`.

The Servo Board is controlled over USB serial, each command is its own line.

### Serial Commands

Action | Description | Command | Parameter Description | Return | Return Parameters
--- | --- | --- | --- | --- | ---
Identify | Get the board type and version | *IDN? | - | Student Robotics:SBv4B:\<asset tag>:\<software version> | \<asset tag> <br>\<software version>
Status | Get board status | *STATUS? | - | \<Watchdog fail>:\<pgood> | \<Watchdog fail> - watchdog timeout, int, 0-1 <br>\<pgood> - power good, int, 0-1
Reset | Reset board to safe startup state<br>- Disable all servos<br>- Reset the lights | *RESET | - | ACK | -
Disable output | Disable a servo output,set to 0 pulse width | SERVO:\<n>:DISABLE | \<n> servo number, int, 0-11 | ACK | -
Set position | Set the position of a servo | SERVO:\<n>:SET:\<value> | \<n> servo number, int, 0-11 <br>\<value> servo duty time, us, 500-4000 | ACK | -
Get position | Get the current position setting for a servo | SERVO:\<n>:GET? | \<n> servo number, int, 0-11 | \<value> | \<value> servo duty time, us, 500-4000
Read servo current | Get total 5V current draw | SERVO:I? | - | \<current> | \<current> - current, int, measured in mA
Read servo voltage | Get 5V SMPS output voltage | SERVO:V? | - | \<voltage> | \<voltage> - voltage, int, measured in mV

### udev Rule

If you are connecting the Servo Board to a Linux computer with udev, the
following rule can be added in order to access the Servo Board interface
without root privileges:

`SUBSYSTEM=="usb", ATTRS{idVendor}=="1bda", ATTRS{idProduct}=="0011", GROUP="plugdev", MODE="0666"`

It should be noted that `plugdev` can be changed to any Unix group of
your preference.

This should only be necessary when trying to access the bootloader.

## Designs

You can access the schematics and source code of the hardware in the following places.
-   [Full Schematics](https://www.studentrobotics.org/resources/kit/servo-schematic.pdf)
-   [Competitor Facing Docs](https://www.studentrobotics.org/docs/kit/servo_board)
-   [Hardware designs](https://github.com/srobo/servo-v4-hw)
