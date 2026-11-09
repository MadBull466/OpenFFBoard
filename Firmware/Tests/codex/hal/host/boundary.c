#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

void p12_unsupported(const char *operation)
{
    fprintf(stderr, "P12 unsupported peripheral access: %s\n", operation);
    abort();
}

void assert_failed(uint8_t *file, uint32_t line)
{
    fprintf(stderr, "P12 vendor parameter assertion: %s:%u\n", file, (unsigned)line);
    abort();
}
