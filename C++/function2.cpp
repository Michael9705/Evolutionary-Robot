#include "hw3.h"

void function2(const char*format,...){
    va_list args;
    va_start(args, format);
    vfprintf(stderr, format, args); // print stand err and exit
    va_end(args);
    exit(1);
}