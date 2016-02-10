/*
 * utils.c : This file contains a number of utility routines for the
 * structural recovery program.
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifndef RAND_MAX
#include <limits.h>
#define RAND_MAX INT_MAX
#endif

void Normalize (double *v)
{
  double s = sqrt (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  
  v[0] = v[0] / s;  v[1] = v[1] / s;  v[2] = v[2] / s;
}

void CrossProduct (double *a, double *b, double *c)
{
  a[0] = b[1]*c[2] - b[2]*c[1];
  a[1] = b[2]*c[0] - b[0]*c[2];
  a[2] = b[0]*c[1] - b[1]*c[0];
}

void AddVector (double *out, double *x, double *y)
{
  out[0] = x[0] + y[0];  out[1] = x[1] + y[1];  out[2] = x[2] + y[2];
}

void SubtractVector (double *out, double *x, double *y)
{
  out[0] = x[0] - y[0];  out[1] = x[1] - y[1];  out[2] = x[2] - y[2];
}

void CrossX (double *a, double *b)
{
  a[0] = 0.0;
  a[1] = -b[2];
  a[2] = b[1];
}

void CrossY (double *a, double *b)
{
  a[0] = b[2];
  a[1] = 0.0;
  a[2] = -b[0];
}

void CrossZ (double *a, double *b)
{
  a[0] = -b[1];
  a[1] = b[0];
  a[2] = 0.0;
}

double InnerProduct (double *a, double *b)
{
  double output = 0.0;
  int j;

  for (j=0; j < 3; ++j)
    output += a[j]*b[j];

  return (output);
}

/*
 * ran0 : lifted from Numerical Recipes in C : Returns a uniform deviate
 * between 0.0 and 1.0 using the system supplied rand function. Set idum
 * to any negative value to initialize or reinitialize the system.
 */

float ran0 (int *idum)
{
  static float maxran;
  static int y, v[97];
  static int iff=0;
  int j;
  
  if (*idum < 0 || iff == 0) {
    iff = 1;
    maxran = RAND_MAX;
    srand(*idum);
    *idum = 1;
    for (j=0; j < 97; ++j) y = rand();
    for (j=0; j < 97; ++j) v[j] = rand();
    y = rand();
  }

  j = 97.0*(y/maxran);
  if (j > 96) j = 96;

  y = v[j];
  v[j] = rand();
  return (y/maxran);
}

/*
 * gasdev : Lifted from Numerical Recipes this routine returns a normally
 * distributed deviate with zero mean and unit variance using ran0 
 * as the source of uniform deviates.
 */

float gasdev (int *idum)
{
  static int iset=0;
  static float gset;
  float fac, r, v1, v2;
  
  if (!iset) {
    do {
      v1 = 2.0*ran0(idum)-1.0;
      v2 = 2.0*ran0(idum)-1.0;
      r = v1*v1 + v2*v2;
    } while (r >= 1.0);
    fac = sqrt (-2.0*log(r)/r);
    gset = v1*fac;
    iset = 1;
    return (v2*fac);
  } else {
    iset = 0;
    return (gset);
  }
}

void pexit (char *s)
{
  printf ("%s \n", s);
  exit (1);
}
