/*
 * utils.h : header file for utils.c
 */

/* Macros ***********************************/

//#define min(a,b) ((a) <= (b) ? (a) : (b))
//#define max(a,b) ((a) >= (b) ? (a) : (b))
#define sqr(a)   ((a)*(a))
#define M_PI 3.14159265359f
#define M_PI_2  1.57079632679489661923
#define 	M_SQRT2   1.41421356237309504880
#define 	M_SQRT1_2   0.70710678118654752440

/* Function Declarations ********************/

void Normalize (double *v);
void CrossProduct (double *a, double *b, double *c);
void AddVector (double *out, double *x, double *y);
void SubtractVector (double *out, double *x, double *y);
void CrossX (double *a, double *b);
void CrossY (double *a, double *b);
void CrossZ (double *a, double *b);
double InnerProduct (double *a, double *b);
float ran0 (int *idum);
float gasdev (int *idum);
void pexit (char *s);
