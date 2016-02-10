/*
 * svd.c : This file defines a reasonable C interface to the LINPACK
 * function dsvdc
 *
 *  Arguments :
 *
 *   int nrows, ncols : Self Explanatory (nrows >= ncols)
 *
 *   double *A : The input matrix A
 *
 *   double *U : The Left singular matrix (ncols x nrows)
 *
 *   double *w : The singular values of A in descending order (ncols)
 *
 *   double *V : The right singular matrix (ncols x ncols)
 *
 *   This routine performs the factorization:
 *              t
 *         A = U  diag(w) V 
 *
 *   N.B. The matrices A, U and V will all be stored in row major order
 *        i.e. they will be standard C matrices
 *
 */

#define MAX_ROWS 500
#define MAX_COLS 30

int svd (int nrows, int ncols, double *A, double *U, double *w, double *V)
{ 
  double cA[MAX_ROWS*MAX_COLS];
  double work[MAX_COLS], e[MAX_COLS];
  int i, j, info, job;
  int ldx, n, p, ldu, ldv;

  /* check input parameters */
  if ((nrows < 0) || (ncols < 0) || 
      (nrows > MAX_ROWS) || (ncols > MAX_COLS) ||
      (nrows < ncols))
    return (-1);
  
  /* convert A to column major form */
  for (i=0; i < nrows; ++i)
    for (j=0; j < ncols; ++j)
      cA[j*nrows + i] = A[i*ncols + j];

  /* call dsvdc_ */
  job = 21;
  ldx = nrows;
  n = nrows;
  p = ncols;
  ldu = nrows;
  ldv = ncols;
  dsvdc_ (cA, &ldx, &n, &p,
	  w, e, U, &ldu, V, &ldv,
	  work, &job, &info);
  
  return (0);
}
