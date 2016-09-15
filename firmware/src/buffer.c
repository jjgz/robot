#include "buffer.h"
#include "system_definitions.h"

CharBuffer buffer_new(unsigned length) {
    CharBuffer buff;
    buff.length = length;
    buff.buff = malloc(buff.length);
    return buff;
}

void buffer_free(CharBuffer *buffer) {
    free(buffer->buff);
    buffer->buff = 0;
}
