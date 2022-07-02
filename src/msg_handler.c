#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "msg_handler.h"
#include "servo.h"
#include "global_vars.h"
#include "i2c.h"

static char* itoa(int value, char* string);

int parse_msg(char* buf, char* response, int max_len)
{
    char temp_str[11] = {0};
    response[0] = '\0';  // make a blank string

    char* next_arg = strtok(buf, ":");
    if (strcmp(next_arg, "SERVO") == 0) {
        next_arg = strtok(NULL, ":");
        if (next_arg == NULL) {
            strncat(response, "NACK:Missing servo number", max_len);
            return strlen(response);
        }

        int servo_num;

        if (strcmp(next_arg, "I?") == 0) {
            // Get stored current value
            strncat(response, itoa(board_current_ma, temp_str), max_len);
            return strlen(response);
        } else if (strcmp(next_arg, "V?") == 0) {
            // Get stored voltage value
            strncat(response, itoa(board_voltage_mv, temp_str), max_len);
            return strlen(response);
        } else if (isdigit((int)next_arg[0])) {
            servo_num = atoi(next_arg);
            // bounds check
            if (servo_num < 0 || servo_num >= NUM_SERVOS) {
                strncat(response, "NACK:Invalid servo number", max_len);
                return strlen(response);
            }
        } else {
            strncat(response, "NACK:Missing servo number", max_len);
            return strlen(response);
        }

        next_arg = strtok(NULL, ":");
        if (next_arg == NULL) {
            strncat(response, "NACK:Missing servo command", max_len);
            return strlen(response);
        }

        if (strcmp(next_arg, "SET") == 0) {
            next_arg = strtok(NULL, ":");
            if (next_arg == NULL) {
                strncat(response, "NACK:Missing servo setpoint", max_len);
                return strlen(response);
            } else if (!isdigit((int)next_arg[0])) {
                strncat(response, "NACK:Invalid servo setpoint", max_len);
                return strlen(response);
            }

            int servo_val = atoi(next_arg);

            // bounds check
            if (servo_val < 0 || servo_val > UINT16_MAX) {
                strncat(response, "NACK:Invalid servo setpoint", max_len);
                return strlen(response);
            }
            // Set servo value
            servo_set_pos((uint8_t) servo_num, (uint16_t) servo_val);

            strncat(response, "ACK", max_len);
            return strlen(response);
        } else if (strcmp(next_arg, "GET?") == 0) {
            // Get servo value
            strncat(response, itoa(servo_get_pos((uint8_t)servo_num), temp_str), max_len);
            return strlen(response);
        } else if (strcmp(next_arg, "DISABLE") == 0) {
            // Disable servo_num
            servo_disable((uint8_t)servo_num);

            strncat(response, "ACK", max_len);
            return strlen(response);
        } else {
            strncat(response, "NACK:Unknown servo command", max_len);
            return strlen(response);
        }
    } else if (strcmp(next_arg, "RESET") == 0) {
        servo_reset();

        strncat(response, "ACK", max_len);
        return strlen(response);
    } else if (strcmp(next_arg, "IDN") == 0) {
        // Identifier string: manufacturer, board name, asset tag, version
        strncat(response, "Student Robotics:", max_len);
        strncat(response, BOARD_NAME_SHORT, max_len - strlen(response));
        strncat(response, ":", max_len - strlen(response));
        strncat(response, (const char *)SERIALNUM_BOOTLOADER_LOC, max_len - strlen(response));
        strncat(response, ":", max_len - strlen(response));
        strncat(response, FW_VER, max_len - strlen(response));
        return strlen(response);
    } else if (strcmp(next_arg, "STATUS") == 0) {
        strncat(response, i2c_timed_out ? "1" : "0", max_len);  // I2C is timed out
        strncat(response, ":", max_len - strlen(response));
        strncat(response, detected_power_good ? "1" : "0", max_len - strlen(response));  // power good
        return strlen(response);
    } else if (strcmp(next_arg, "ECHO") == 0) {
        next_arg = strtok(NULL, ":");

        strncat(response, next_arg, max_len);
        return strlen(response);
    } else {
        strncat(response, "NACK:Unknown command: '", max_len);
        strncat(response, next_arg, max_len - strlen(response));
        strncat(response, "'", max_len - strlen(response));
        return strlen(response);
    }

    // This should be unreachable
    strncat(response, "NACK:Unknown error", max_len);
    return strlen(response);
}

static char* itoa(int value, char* string) {
    // including stdio.h to get sprintf overflows the rom
    char tmp[33];
    char* tmp_ptr = tmp;
    char* sp = string;
    unsigned int digit;
    unsigned int remaining;
    bool sign;

    if ( string == NULL ) {
        return 0;
    }

    sign = (value < 0);
    if (sign) {
        remaining = -value;
    } else {
        remaining = (unsigned int)value;
    }

    while (remaining || tmp_ptr == tmp) {
        digit = remaining % 10;
        remaining /= 10;
        *tmp_ptr = digit + '0';
        tmp_ptr++;
    }

    if (sign) {
        *sp = '-';
        sp++;
    }

    // string is in reverse at this point
    while (tmp_ptr > tmp) {
        tmp_ptr--;
        *sp = *tmp_ptr;
        sp++;
    }
    *sp = '\0';

    return string;
}
