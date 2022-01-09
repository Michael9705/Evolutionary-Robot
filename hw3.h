#ifndef hw3
# define hw3

#include<iostream>
#include<vector>
#include<cstdio>
#include<numeric>
#include<stdarg.h>
#include<stdint.h>
#include<strings.h>
#include<ctime>
#include<cmath>
#include<fstream>
#include<cstdlib>
#include<GLUT/glut.h>

#define LENGTH 8192

extern "C"{
    unsigned int funciton1(const char*file);
    void function2(const char*format,...);
    void function3(const char*where);
}

#endif
