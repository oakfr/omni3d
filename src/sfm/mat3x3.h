/*
 * mat3x3.h : header file for mat3x3.c
 */

int mat_mult (double A[3][3], double B[3][3], double C[3][3]);
int mat_vec_mult (double A[3][3], double b[3], double c[3]);
int mat_sqr (double A[3][3], double sA[3][3]);
int mat_transpose (double A[3][3], double At[3][3]);
double mat_mag (double A[3][3]);
