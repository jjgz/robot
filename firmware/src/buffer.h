#ifndef _BUFFER_H
#define _BUFFER_H

typedef struct {
    char *buff;
    unsigned length;
} CharBuffer;

CharBuffer buffer_new(unsigned length);
void buffer_free(CharBuffer *buffer);

#endif
