#include "hw3.h"

void function3(const char*where)
{
    int err = glGetError();
    if (err) fprintf(stderr, "GLUT error: %s at [%s]\n", gluErrorString(err),where);
}