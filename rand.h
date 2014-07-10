/**
	File name: rand.h
	Description: header file of rand.c
	Date: July 21, 2006
	Maker: Jaehoon Paul Jeong
	E-mail: jjeong@cs.umn.edu
*/

#ifndef __RAND_H__
#define __RAND_H__

#include <math.h>
#include "smpl.h"

//extern double drand48();

#if defined(_WIN32)
#include <stdlib.h>
#define drand48() (((float) rand())/((float) RAND_MAX))
#define srand48(x) srand(x)
#endif

//typedef double real;
#define then

/* function declarations */
int stream(int n);
long seed(long Ik, int n);
double uniform(double a, double b);

int smpl_random(int i, int n); //return a random integer with uniform distribution
int smpl_random_v1(int i, int n);
int smpl_random_v2(int i, int n);

real expntl(real x);
real erlang(real x, real s);
real hyperx(real x, real s);
real normal(real x, real s);

#endif
