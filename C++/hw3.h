#ifndef hw3
#define hw3

#include<iostream>
#include<vector>
#include<cstdio>
#include<numeric>
#include<stdarg.h>


#include<ctime>
#include<cmath>
#include<fstream>
#include<cstdlib>
#include<GLUT/glut.h>

#define LENGTH 8192


#define Sin(th) sin(M_PI/180*(th))
#define Cos(th) cos(M_PI/180*(th))

extern "C"{
    unsigned int getTexture(const char*file);
    void function2(const char*format,...);
    void function3(const char*where);
}

#endif
