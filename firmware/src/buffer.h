#ifndef _BUFFER_H
#define _BUFFER_H

typedef struct {
    char *buff;
    unsigned length;
} CharBuffer;

typedef struct {
  float x,y;  
}Point;

typedef struct{
    Point *buff;
    unsigned length;
}PointBuffer;

#endif
