/*
 * ev.c : This file provides a simple C front end to the EISPACK routine rs
 *
 * Arguments :
 *
 *   int n : The dimension of the matrix
 *
 *   double *A : a real symmettric n x n matrix
 *
 *   double *w : The eigenvalues of A in ascending order
 *
 *   double *Z : An n x n matrix containing the eigenvectors of A 
 *  
 *   This routine finds the eigenvalues and eigenvectors associated with
 *   a real symmetrric matrix.
 *
 *           t
 *      A = Z  diag(w) Z
 *
 * N.B. The matrices A and Z will be stored in row major order
 *      i.e. they will be standard C matrices
 *
 */

#define MAX_N 10

int ev (int n, double *A, double *w, double *Z)
{
  int nm=n, matz=1, ierr;
  double fv1[MAX_N], fv2[MAX_N];

  if ((n < 0) || (n > MAX_N)) return (-1);

  rs_ (&nm, &n, A, w, &matz, Z, fv1, fv2, &ierr);

  return (0);
}
