#include <stdlib.h>
#include <string.h>

#include "msg_handler.h"

int parse_msg(char* buf, int len, char* response, int max_len)
{
    (void)max_len;
    /// TODO check the response fits

    memcpy(response, buf, len);
    response[len - 1] = '\n';  // revert null terminator to new line
    return len;
}
