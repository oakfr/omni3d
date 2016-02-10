/*
 * LH88.c : This file implements the linear SFM algorithm
 * proposed by Weng, Liu, Huang and Ahuja in their 1988 CVPR
 * paper.
 */

/*
 * PROVISO : This implementation assumes that the essential matrices
 * E1 , E2 and E3 all have rank 3. There are certain special camera 
 * configurations where this assumption is violated, in which case
 * this implementation will not produce the correct result. In their
 * CVPR paper Weng, Liu, Huang and Ahuja propose a method for handling
 * these special cases by computing the ranks of these matrices. We found
 * these rank tests to be quite unreliable in the presence of image noise.
 * So in our simulation experiments we simply restricted ourselves to camera
 * configurations where all the resulting essential matrices were safely
 * rank three.
 */

/* define WEIGHTING as 1 to add scale factors */
#define  WEIGHTING   1

#define  NOISY       0

#include <math.h>
#include "utils.h"
#include "quaternions.h"
#include "mat3x3.h"
#include "SFM.h"
#include "polygon.h"

#undef A

/* Forward Declarations ********************************************/

int GetEssentialMatrices (double E1[3][3], double E2[3][3], double E3[3][3]);
int GetTranslationDirection (double E1[3][3], double E2[3][3], double E3[3][3],
			     double U[3]);
int FormMatrices (double A[3][3],  double M[3][3],  double T[3],
		  double E1[3][3], double E2[3][3], double E3[3][3],
		  double U[3]);
int GetRotations (double qp[4], double qm[4], double M[3][3]);

int ChooseSigns (int *s1, int *s2,
		 double E1[3][3], double E2[3][3], double E3[3][3],
		 double Rp[4], double Rm[4], double T[3], double magT,
		 double Sp[4], double Sm[4], double U[3], double magU);
double ComputeSignError (int s1, int s2,
			 double E1[3][3], double E2[3][3], double E3[3][3],
			 double R[4], double T[3], double magT,
			 double S[4], double U[3], double magU);

int SolveStereo (double R[4], double T[3], double magT,
		 double S[4], double U[3], double magU);
int ConvertToLine (double *v, double *d, line *TheLine);

/* Main routine ****************************************************/

int LH88 ()
{
  double E1[3][3],  E2[3][3],  E3[3][3];
  double E1t[3][3], E2t[3][3], E3t[3][3];
  double Rp[4], Rm[4], T[3], Sp[4], Sm[4], U[3];
  double *R, *S;
  double magT, magU;
  double A[3][3], M[3][3], q[4];
  int s1, s2;

  GetEssentialMatrices (E1, E2, E3);

  mat_transpose (E1, E1t);  mat_transpose (E2, E2t);  mat_transpose (E3, E3t);

  GetTranslationDirection (E1,  E2,  E3,  U);
  GetTranslationDirection (E1t, E2t, E3t, T);

  FormMatrices (A, M, T, E1, E2, E3, U);
  magU = sqrt (mat_mag (A) / 2.0);

  GetRotations (Rp, Rm, M);

  FormMatrices (A, M, U, E1t, E2t, E3t, T);
  magT = sqrt (mat_mag (A) / 2.0);

  GetRotations (Sm, Sp, M); /* order is different because of sign change */

#if NOISY
  print_solns (Rp, Rm, magT, T, Sp, Sm, magU, U);
#endif

  ChooseSigns (&s1, &s2, E1, E2, E3, Rp, Rm, T, magT, Sp, Sm, U, magU);

#if NOISY
  printf ("s1 = %d, s2 = %d\n", s1, s2);
#endif

  if (s1 == 1) R = Rp; else { magU = -magU; R = Rm; }
  if (s2 == 1) S = Sp; else { magT = -magT; S = Sm; }

  /* Fill in the data structures */  
  SolveStereo (R, T, magT, S, U, magU);

  /* Camera position 0 : the origin */
  TheCameraPositions[0]->q[0] = 1.0;
  TheCameraPositions[0]->q[1] = 0.0;
  TheCameraPositions[0]->q[2] = 0.0;
  TheCameraPositions[0]->q[3] = 0.0;

  TheCameraPositions[0]->t[0] = 0.0;
  TheCameraPositions[0]->t[1] = 0.0;
  TheCameraPositions[0]->t[2] = 0.0;

  /* Camera position 1 */
  QuaternionConjugate (R, q);
  T[0] *= -magT;  T[1] *= -magT;  T[2] *= -magT;
  RotateQuaternion (TheCameraPositions[1]->t, q, T);

  TheCameraPositions[1]->q[0] = R[0];
  TheCameraPositions[1]->q[1] = R[1];
  TheCameraPositions[1]->q[2] = R[2];
  TheCameraPositions[1]->q[3] = R[3];

  /* Camera position 2 */
  QuaternionConjugate (S, q);
  U[0] *= -magU;  U[1] *= -magU;  U[2] *= -magU;
  RotateQuaternion (TheCameraPositions[2]->t, q, U);

  TheCameraPositions[2]->q[0] = S[0];
  TheCameraPositions[2]->q[1] = S[1];
  TheCameraPositions[2]->q[2] = S[2];
  TheCameraPositions[2]->q[3] = S[3];

  /* Scaling the answers appropriately */
  ScaleLH88 (magT, magU);
}

int print_solns (double Rp[4], double Rm[4], double magT, double T[3],
		 double Sp[4], double Sm[4], double magU, double U[3])
{
  printf ("\nRecovered solutions ***************\n");

  printf ("T = "); PrintVector (T);
  printf ("magT = %12.3e\n", magT);

  printf ("U = "); PrintVector (U);
  printf ("magU = %12.3e\n", magU);

  printf ("Rp = "); PrintQuaternion (Rp);
  printf ("Rm = "); PrintQuaternion (Rm);

  printf ("\n");

  printf ("Sp = "); PrintQuaternion (Sp);
  printf ("Sm = "); PrintQuaternion (Sm);
}

int PrintVector (double *t)
{
  printf ("t0 : %8.3f  t1 : %8.3f  t2 : %8.3f \n", t[0], t[1], t[2]);
}

int PrintMatrix (char *legend, double A[3][3])
{
  int i, j;

  printf ("\n*** %s ******\n\n", legend);

  for (i=0; i < 3; ++i) {
    for (j=0; j < 3; ++j)
      printf (" %12.3e ", A[i][j]);
    printf ("\n");
  }
  
  printf ("\n");
}

int GetEssentialMatrices (double E1[3][3], double E2[3][3], double E3[3][3])
{
  int i, j, k, index;
  double *m0, *m1, *m2;
  double l0, l1, l2, scale;
  double A[3*MAX_N][27], U[27][3*MAX_N], w[27], V[27][27], C[9], J[3][3];

  /* Form SVD matrix */
  for (i=0; i < n; ++i) {
    m0 = TheEdges[0][i]->m;  l0 = TheEdges[0][i]->length;
    m1 = TheEdges[1][i]->m;  l1 = TheEdges[1][i]->length;
    m2 = TheEdges[2][i]->m;  l2 = TheEdges[2][i]->length;
    scale = sqrt (1 / ((1/l0) + (1/l1) + (1/l2)));

    for (j=0; j < 3; ++j)
      for (k=0; k < 3; ++k)
#if WEIGHTING
	C[3*j+k] = scale*m1[j]*m2[k];
#else
	C[3*j+k] = m1[j]*m2[k];
#endif

    J[0][0] =    0.0;  J[0][1] = -m0[2];  J[0][2] =  m0[1];
    J[1][0] =  m0[2];  J[1][1] =    0.0;  J[1][2] = -m0[0];
    J[2][0] = -m0[1];  J[2][1] =  m0[0];  J[2][2] =    0.0;

    for (j=0; j < 3; ++j)
      for (k=0; k < 27; ++k)
	A[i*3+j][k] = J[j][(k/9)] * C[(k%9)];
  }

  /* perform SVD */
  svd (3*n, 27, A, U, w, V);

  /* choose eignenvector corresponding to smallest magnitude eigenvalue */
  /* pack up matrices */
  for (i=0,index=0; i < 3; ++i)
    for (j=0; j < 3; ++j, ++index) {
      E1[i][j] = V[26][  index ];
      E2[i][j] = V[26][ 9+index];
      E3[i][j] = V[26][18+index];
    }
}

int GetTranslationDirection (double E1[3][3], double E2[3][3], double E3[3][3],
			     double U[3])
{
  int i;
  double sE1[3][3], sE2[3][3], sE3[3][3];
  double w1, V1[3], w2, V2[3], w3, V3[3];
  double A[3][3], sA[3][3], w[3], Z[3][3];

  mat_sqr (E1, sE1);  mat_sqr (E2, sE2);  mat_sqr (E3, sE3);

  ev (3, sE1, w, Z);
  V1[0] = Z[0][0];  V1[1] = Z[0][1];  V1[2] = Z[0][2];
  w1 = (w[1] - w[0]);

  ev (3, sE2, w, Z);
  V2[0] = Z[0][0];  V2[1] = Z[0][1];  V2[2] = Z[0][2];
  w2 = (w[1] - w[0]);

  ev (3, sE3, w, Z);
  V3[0] = Z[0][0];  V3[1] = Z[0][1];  V3[2] = Z[0][2];
  w3 = (w[1] - w[0]);

#if WEIGHTING
  A[0][0] = w1*V1[0];  A[0][1] = w1*V1[1];  A[0][2] = w1*V1[2];
  A[1][0] = w2*V2[0];  A[1][1] = w2*V2[1];  A[1][2] = w2*V2[2];
  A[2][0] = w3*V3[0];  A[2][1] = w3*V3[1];  A[2][2] = w3*V3[2];
#else
  A[0][0] = V1[0];  A[0][1] = V1[1];  A[0][2] = V1[2];
  A[1][0] = V2[0];  A[1][1] = V2[1];  A[1][2] = V2[2];
  A[2][0] = V3[0];  A[2][1] = V3[1];  A[2][2] = V3[2];
#endif

  mat_sqr (A, sA);
  ev (3, sA, w, Z);

  U[0] = Z[0][0];  U[1] = Z[0][1];  U[2] = Z[0][2];
}

int FormMatrices (double A[3][3],  double M[3][3],  double T[3],
		  double E1[3][3], double E2[3][3], double E3[3][3],
		  double U[3])
{
  double c1[3], c2[3], c3[3], temp[3];

  mat_vec_mult (E1, U, temp);
  CrossProduct (c1, T, temp);

  mat_vec_mult (E2, U, temp);
  CrossProduct (c2, T, temp);

  mat_vec_mult (E3, U, temp);
  CrossProduct (c3, T, temp);

  A[0][0] = c1[0];  A[0][1] = c2[0];  A[0][2] = c3[0];
  A[1][0] = c1[1];  A[1][1] = c2[1];  A[1][2] = c3[1];
  A[2][0] = c1[2];  A[2][1] = c2[2];  A[2][2] = c3[2];

  CrossProduct (M[0], T, c1);
  CrossProduct (M[1], T, c2);
  CrossProduct (M[2], T, c3);
}

int GetRotations (double qp[4], double qm[4], double Mt[3][3])
{
  int i;
  double A[4][4], w[4], Z[4][4], M[3][3];

  mat_transpose (Mt, M);

  A[0][0] = M[0][0] + M[1][1] + M[2][2];

  A[1][1] = M[0][0];
  A[2][2] = M[1][1];
  A[3][3] = M[2][2];

  A[0][1] = A[1][0] = (M[2][1] - M[1][2]) / 2;
  A[0][2] = A[2][0] = (M[0][2] - M[2][0]) / 2;
  A[0][3] = A[3][0] = (M[1][0] - M[0][1]) / 2;

  A[1][2] = A[2][1] = (M[0][1] + M[1][0]) / 2;
  A[1][3] = A[3][1] = (M[0][2] + M[2][0]) / 2;

  A[2][3] = A[3][2] = (M[1][2] + M[2][1]) / 2;

  ev (4, A, w, Z);

  for (i=0; i < 4; ++i) { qp[i] = Z[0][i]; qm[i] = Z[3][i]; }

  NormalizeQuaternion (qp);  NormalizeQuaternion (qm);
}

int ChooseSigns (int *s1, int *s2,
		 double E1[3][3], double E2[3][3], double E3[3][3],
		 double Rp[4], double Rm[4], double T[3], double magT,
		 double Sp[4], double Sm[4], double U[3], double magU)
{
  double temp, best;

  /* 
   * for each sign assignment compute its correspondence to the
   * essential matrices. Choose the one with the smallest error.
   */

  *s1 = *s2 = 1;
  best = ComputeSignError ( 1,  1, E1, E2, E3, Rp, T, magT, Sp, U, magU);

  temp = ComputeSignError ( 1, -1, E1, E2, E3, Rp, T, magT, Sm, U, magU);
  if (temp < best) { *s1 = 1; *s2 = -1; best = temp; }

  temp = ComputeSignError (-1,  1, E1, E2, E3, Rm, T, magT, Sp, U, magU);
  if (temp < best) { *s1 = -1; *s2 = 1; best = temp; }

  temp = ComputeSignError (-1, -1, E1, E2, E3, Rm, T, magT, Sm, U, magU);
  if (temp < best) { *s1 = -1; *s2 = -1; best = temp; }
}

double ComputeSignError (int s1, int s2,
			 double E1[3][3], double E2[3][3], double E3[3][3],
			 double R[4], double T[3], double magT,
			 double S[4], double U[3], double magU)
{
  int i, j;
  double dE1[3][3], dE2[3][3], dE3[3][3];
  double R1[3], R2[3], R3[3];
  double S1[3], S2[3], S3[3];
  double a[3];

  a[0] = 1.0;  a[1] = 0.0;  a[2] = 0.0;
  RotateQuaternion (R1, R, a);
  RotateQuaternion (S1, S, a);

  a[0] = 0.0;  a[1] = 1.0;  a[2] = 0.0;
  RotateQuaternion (R2, R, a);
  RotateQuaternion (S2, S, a);

  a[0] = 0.0;  a[1] = 0.0;  a[2] = 1.0;
  RotateQuaternion (R3, R, a);
  RotateQuaternion (S3, S, a);  

  for (i=0; i < 3; ++i)
    for (j=0; j < 3; ++j) {
      dE1[i][j] = E1[i][j] - (s1*magU*R1[i]*U[j] - s2*magT*T[i]*S1[j]);
      dE2[i][j] = E2[i][j] - (s1*magU*R2[i]*U[j] - s2*magT*T[i]*S2[j]);
      dE3[i][j] = E3[i][j] - (s1*magU*R3[i]*U[j] - s2*magT*T[i]*S3[j]);
    }

  return (mat_mag (dE1) + mat_mag (dE2) + mat_mag (dE3));
}

int SolveStereo (double R[4], double T[3], double magT,
		 double S[4], double U[3], double magU)
{
  int i, pos, neg;
  double Rt[4], St[4], A[3][3], sA[3][3], magd, a[3];
  double w[3], Z[3][3];
  double *m0, *m1, *m2;
  double v[3], d[3];

  /* Compute inverse rotations */
  QuaternionConjugate (R, Rt);
  QuaternionConjugate (S, St);

  pos = neg = 0;

  for (i=0; i < n; ++i) {
    m0 = TheEdges[0][i]->m;
    m1 = TheEdges[1][i]->m;
    m2 = TheEdges[2][i]->m;

    /* Compute the line direction */
    A[0][0] = m0[0];  A[0][1] = m0[1];  A[0][2] = m0[2];

    RotateQuaternion (A[1], Rt, m1);
    RotateQuaternion (A[2], St, m2);

    mat_sqr (A, sA);

    ev (3, sA, w, Z);

    v[0] = Z[0][0];  v[1] = Z[0][1];  v[2] = Z[0][2];

    /* compute d */
    CrossProduct (d, m0, v);
    Normalize (d);

    RotateQuaternion (a, R, d);
    magd = (magT * InnerProduct (m1, T)) / InnerProduct (m1, a);
  
    RotateQuaternion (a, S, d);
    magd += (magU * InnerProduct (m2, U)) / InnerProduct (m2, a);

    magd = (-magd / 2);

    d[0] *= magd;  d[1] *= magd;  d[2] *= magd;

    if (d[2] > 0.0) ++pos; else ++neg;

    TheLines[i] = (line *) malloc (sizeof (line));
    ConvertToLine (v, d, TheLines[i]);
  }

  if (pos > neg) { /* Invert the answers */
    for (i=0; i < 3; ++i) { T[i] = -T[i];  U[i] = -U[i]; }

    for (i=0; i < n; ++i) {
      TheLines[i]->a = -TheLines[i]->a;  TheLines[i]->b = -TheLines[i]->b;
    }
  }
}

int ConvertToLine (double *v, double *d, line *TheLine)
{
  double a[3], b[3], theta, q[4], s;

  if (v[2] < 0.0) { v[0] = -v[0]; v[1] = -v[1]; v[2] = -v[2]; }

  /* compute a possible rotation */
  a[0] = 0.0;  a[1] = 0.0;  a[2] = 1.0;
  CrossProduct (b, a, v);

  s = VectorNorm (b);
  if (s >  1.0) s = 1.0;
  theta = asin (s);
  Normalize (b);
  b[0] *= theta;  b[1] *= theta;  b[2] *= theta;
  
  VectorToQuaternion (b, TheLine->q);

  QuaternionConjugate (TheLine->q, q);
  RotateQuaternion (a, q, d);
  TheLine->a = a[0];  TheLine->b = a[1];
}

int ScaleLH88 (double magT, double magU)
{
  int i, j;
  double scale1, scale2, scale;

  scale1 = VectorNorm (TrueCameraPositions[1].t) / fabs (magT);
  scale2 = VectorNorm (TrueCameraPositions[2].t) / fabs (magU);

  scale = (scale1 + scale2) / 2;

  for (j=1; j < 3; ++j) {
    TheCameraPositions[j]->t[0] *= scale;
    TheCameraPositions[j]->t[1] *= scale;
    TheCameraPositions[j]->t[2] *= scale;
  }
  
  for (i=0; i < n; ++i) {
    TheLines[i]->a *= scale;  TheLines[i]->b *= scale;
  }
}
