#include "Agv_core/utils.h"

void LOG(const char* name, const char* fmt, ...) {
    va_list ap;

    if (name && name[0] != '\0') {
        printf("[%s] ", name);
    }
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);

    putchar('\n');
}