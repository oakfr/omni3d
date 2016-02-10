/*
 * mat3x3.c : This file contains a set of utility routines that are used
 * to manipulate 3 x 3  matrices.
 */

#include "mat3x3.h"

int mat_mult (double A[3][3], double B[3][3], double C[3][3])
{
  int i, j, k;
  
  for (i=0; i < 3; ++i)
    for (j=0; j < 3; ++j)
      for (k=0, C[i][j]=0.0; k < 3; ++k)
	C[i][j] += A[i][k] * B[k][j];
}

int mat_vec_mult (double A[3][3], double b[3], double c[3])
{
  int i, j;

  for (i=0; i < 3; ++i)
    for (j=0, c[i]=0.0; j < 3; ++j)
      c[i] += A[i][j] * b[j];
}

int mat_sqr (double A[3][3], double sA[3][3])
{
  double At[3][3];
  
  mat_transpose (A, At);
  mat_mult (At, A, sA);  /* N.B. At*A not equal to A*At */
}

int mat_transpose (double A[3][3], double At[3][3])
{
  int i, j;

  for (i=0; i < 3; ++i)
    for (j=0; j < 3; ++j)
      At[j][i] = A[i][j];
}

double mat_mag (double A[3][3])
{
  int i, j;
  double res=0.0;

  for (i=0; i < 3; ++i)
    for (j=0; j < 3; ++j)
      res += (A[i][j] * A[i][j]);

  return (res);
}
