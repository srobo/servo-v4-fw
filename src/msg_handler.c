#include <stdlib.h>
#include <string.h>

#include "msg_handler.h"

int parse_msg(char* buf, char* response, int max_len)
{
    response[0] = '\0';  // make a blank string

    char* next_arg = strtok(buf, ":");
    if (strcmp(next_arg, "ECHO") == 0) {
        next_arg = strtok(NULL, ":");

        strncat(response, next_arg, max_len);
        return strlen(response);
    } else {
        strncat(response, "NACK:Unknown command", max_len);
        return strlen(response);
    }
}
