/**********************************************************************/
/*                           File "smpl.h"                            */
/*  Includes, Defines, & Extern Declarations for Simulation Programs  */
/**********************************************************************/

#ifndef __SMPL_H__
#define __SMPL_H__

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stddef.h> //size_t
#include <stdlib.h> //exit()

typedef double real;

#define then

//#define nl 256       /* element pool length                 */
//@ modified by jaehoon, 01/08/2005
//@ nl increase for processing fast arrivals of open-loop traffic

/* element pool length */ 
//#define nl (256*10000)   
#define nl (256*30000)

//#define nl (256*10000)   /* element pool length                 */ 
//#define nl (256*1000)   /* element pool length                 */ //=> [01/08/08] issues an error of "Empty Pool Element"
//#define nl (256*100)   /* element pool length                 */ //=> works
#define ns 256       /* namespace length                    */
#define pl 58        /* printer page length   (lines used   */
#define sl 23        /* screen page length     by 'smpl')   */
#define FF 12        /* form feed                           */

//extern real Lq(), U(), B(), time();
//extern char *fname(), *mname();
//extern FILE *sendto();

//extern real uniform(), expntl(), erlang(), hyperx(), normal();
//extern long seed();

/* function declarations */
void smpl(int m, char *s);
void reset();
int save_name(char *s, int m);
char *mname();
char *fname(int f);
void schedule(int ev, real te, int tkn);
int cause(int *ev,int *tkn);
//void cause(int *ev,int *tkn);
//real time();
real smpl_time();
int cancel(int ev);
int facility(char *s,int n);
int request(int f, int tkn, int pri);
int preempt(int f, int tkn, int pri);
void release(int f, int tkn);
int status(int f);
int inq(int f);
real U(int f);
real B(int f);
real Lq(int f);
void trace(int n);
//void pause();
void smpl_pause();
void error(int n, char *s);
void report();
void reportf();
int lns(int i);
void endpage();
void newpage();
FILE *sendto(FILE *dest);

///////////////////

#endif

