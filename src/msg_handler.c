#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "msg_handler.h"
#include "servo.h"
#include "global_vars.h"
#include "i2c.h"

static char* itoa(int value, char* string);

static void append_str(char* dest, const char* src, int dest_max_len) {
    strncat(dest, src, dest_max_len - strlen(dest));
}
static char* get_next_arg(char* response, const char* err_msg, int max_len) {
    char* next_arg = strtok(NULL, ":");
    if (next_arg == NULL) {
        strncat(response, err_msg, max_len);
        return NULL;
    }
    return next_arg;
}

void handle_msg(char* buf, char* response, int max_len) {
    // max_len is the maximum length of the string that can be fitted in buf
    // so the buffer must be at least max_len+1 long
    char temp_str[12] = {0};  // for doing itoa conversions
    response[0] = '\0';  // make a blank string

    char* next_arg = strtok(buf, ":");
    if (strcmp(next_arg, "SERVO") == 0) {
        next_arg = get_next_arg(response, "NACK:Missing servo number", max_len);
        if(next_arg == NULL) {return;}

        unsigned long int servo_num;

        if (strcmp(next_arg, "I?") == 0) {
            // Get stored current value
            append_str(response, itoa(board_current_ma, temp_str), max_len);
            return;
        } else if (strcmp(next_arg, "V?") == 0) {
            // Get stored voltage value
            append_str(response, itoa(board_voltage_mv, temp_str), max_len);
            return;
        } else if (isdigit((int)next_arg[0])) {
            servo_num = strtoul(next_arg, NULL, 10);
            // bounds check
            if (servo_num >= NUM_SERVOS) {
                append_str(response, "NACK:Invalid servo number", max_len);
                return;
            }
        } else {
            append_str(response, "NACK:Missing servo number", max_len);
            return;
        }

        next_arg = get_next_arg(response, "NACK:Missing servo command", max_len);
        if(next_arg == NULL) {return;}

        if (strcmp(next_arg, "SET") == 0) {
            next_arg = get_next_arg(response, "NACK:Missing servo setpoint", max_len);
            if(next_arg == NULL) {return;}
            if (!isdigit((int)next_arg[0])) {
                append_str(response, "NACK:Invalid servo setpoint", max_len);
                return;
            }

            unsigned long int servo_val = strtoul(next_arg, NULL, 10);

            // bounds check
            if (servo_val < MIN_SERVO_PULSE || servo_val > MAX_SERVO_PULSE) {
                append_str(response, "NACK:Invalid servo setpoint", max_len);
                return;
            }
            // Set servo value
            servo_set_pos((uint8_t) servo_num, (uint16_t) servo_val);

            append_str(response, "ACK", max_len);
            return;
        } else if (strcmp(next_arg, "GET?") == 0) {
            // Get servo value
            append_str(response, itoa(servo_get_pos((uint8_t)servo_num), temp_str), max_len);
            return;
        } else if (strcmp(next_arg, "DISABLE") == 0) {
            // Disable servo_num
            servo_disable((uint8_t)servo_num);

            append_str(response, "ACK", max_len);
            return;
        } else {
            append_str(response, "NACK:Unknown servo command", max_len);
            return;
        }
    } else if (strcmp(next_arg, "*IDN?") == 0) {
        // Identifier string: manufacturer, board name, asset tag, version
        append_str(response, "Student Robotics:" BOARD_NAME_SHORT ":", max_len);
        append_str(response, (const char *)SERIALNUM_BOOTLOADER_LOC, max_len);
        append_str(response, ":" FW_VER, max_len);
        return;
    } else if (strcmp(next_arg, "*STATUS?") == 0) {
        append_str(response, i2c_timed_out ? "1" : "0", max_len);  // I2C is timed out
        append_str(response, ":", max_len);
        append_str(response, detected_power_good ? "1" : "0", max_len);  // power good
        return;
    } else if (strcmp(next_arg, "*RESET") == 0) {
        servo_reset();

        append_str(response, "ACK", max_len);
        return;
    } else if (strcmp(next_arg, "*DBG") == 0) {
        next_arg = get_next_arg(response, "NACK:Missing debug command", max_len);
        if(next_arg == NULL) {return;}
        if (strcmp(next_arg, "SERVOS") == 0) {
            next_arg = get_next_arg(response, "NACK:Missing phase number", max_len);
            if(next_arg == NULL) {return;}
            unsigned long int phase_num;
            if (isdigit((int)next_arg[0])) {
                phase_num = strtoul(next_arg, NULL, 10);
                // bounds check
                if ((phase_num == 0) || (phase_num > NUM_SERVO_PHASES)) {
                    append_str(response, "NACK:Invalid phase number", max_len);
                    return;
                } else {
                    servo_step_t* steps = get_servo_steps(phase_num);
                    if (steps == NULL) {
                        append_str(response, "NACK:failed to read servo steps", max_len);
                        return;
                    }
                    append_str(response, next_arg, max_len);
                    for (uint8_t i = 0; i < SERVO_STEPS_PER_PHASE; i++) {
                        append_str(response, ":", max_len);
                        append_str(response, itoa(steps[i].idx, temp_str), max_len);  // idn
                        append_str(response, steps[i].rising ? "U" : "D", max_len);  // rising
                        append_str(response, steps[i].enabled ? "1" : "0", max_len);  // enabled
                        append_str(response, ":", max_len);
                        append_str(response, itoa(steps[i].next_steps, temp_str), max_len);  // steps
                    }
                    return;
                }
            } else {
                append_str(response, "NACK:Missing phase number", max_len);
                return;
            }
        } else {
            append_str(response, "NACK:Unknown debug command", max_len);
            return;
        }
    } else if (strcmp(next_arg, "ECHO") == 0) {
        next_arg = strtok(NULL, ":");

        append_str(response, next_arg, max_len);
        return;
    } else {
        append_str(response, "NACK:Unknown command: '", max_len);
        append_str(response, next_arg, max_len);
        append_str(response, "'", max_len);
        return;
    }

    // This should be unreachable
    append_str(response, "NACK:Unknown error", max_len);
    return;
}

static char* itoa(int value, char* string) {
    // string must be a buffer of at least 12 chars
    // including stdio.h to get sprintf overflows the rom
    char tmp[11];
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
