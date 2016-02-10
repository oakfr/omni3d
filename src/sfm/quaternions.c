/*
 * quaternions.c : This file ontains code that is used to manipulate
 * unit quaternions.
 */

#include <stdio.h>
#include <math.h>
#include "utils.h"
#include "quaternions.h"

void VectorToQuaternion (double *w, double *q)
{
  double alpha, scale;

  alpha = VectorNorm (w);

  q[0] = cos (alpha/2);

  if (alpha != 0.0) {
    scale = (sin (alpha/2) / alpha);
    q[1] = w[0] * scale;
    q[2] = w[1] * scale;
    q[3] = w[2] * scale;
  } else {
    q[1] = q[2] = q[3] = 0.0;
  }
}

void QuaternionToVector (double *q, double *w)
{
  double alpha, scale;

  scale = VectorNorm (q+1);  
  alpha = 2 * (atan2 (scale, fabs (q[0])));

  if (scale != 0.0) {
    scale = alpha / scale;
    if (q[0] < 0.0) scale = -scale;
    w[0] = scale * q[1];
    w[1] = scale * q[2];
    w[2] = scale * q[3];
  } else {
    w[0] = w[1] = w[2] = 0.0;
  }
}

void NormalizeQuaternion (double *q)
{
  double scale;

  scale = sqrt ( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
  
  q[0] /= scale;  q[1] /= scale;  q[2] /= scale;  q[3] /= scale;
}

void RotateQuaternion (double *b, double *q, double *a)
{
  double *u = q+1, u0 = q[0];
  double c[3], scale;

  CrossProduct (c, u, a);
  scale = 2*u0;
  b[0] = scale*c[0];   b[1] = scale*c[1];   b[2] = scale*c[2];
  
  scale = 2 * InnerProduct (u, a);
  b[0] += scale*u[0];  b[1] += scale*u[1];  b[2] += scale*u[2];

  scale = u0*u0 - InnerProduct (u, u);
  b[0] += scale*a[0];  b[1] += scale*a[1];  b[2] += scale*a[2];
}

void RotateVector (double *b, double *w, double *a)
{
  double alpha, s, v, u[3], c[3], d[3];

  alpha = VectorNorm (w);

  if (alpha == 0.0) {
    b[0] = a[0];  b[1] = a[1];  b[2] = a[2];
    return;
  }

  u[0] = w[0]/alpha;  u[1] = w[1]/alpha;  u[2] = w[2]/alpha;

  s = sin (alpha);
  v = 1 - cos (alpha);

  b[0] = a[0];  b[1] = a[1];  b[2] = a[2];

  CrossProduct (c, u, a);
  b[0] += s*c[0];  b[1] += s*c[1];  b[2] += s*c[2];

  CrossProduct (d, u, c);
  b[0] += v*d[0];  b[1] += v*d[1];  b[2] += v*d[2];
}

void RotateAngleVector (double *b, double alpha, double *u, double *a)
{
  double s, v, c[3], d[3];

  s = sin (alpha);
  v = 1 - cos (alpha);

  b[0] = a[0];  b[1] = a[1];  b[2] = a[2];

  CrossProduct (c, u, a);
  b[0] += s*c[0];  b[1] += s*c[1];  b[2] += s*c[2];

  CrossProduct (d, u, c);
  b[0] += v*d[0];  b[1] += v*d[1];  b[2] += v*d[2];
}

void QuaternionConjugate (double *q, double *q_prime)
{
  q_prime[0] = q[0];
  q_prime[1] = - q[1];
  q_prime[2] = - q[2];
  q_prime[3] = - q[3];
}

void QuaternionMultiply (double *result, double *q1, double *q2)
{
  double *u1 = q1+1, *u2 = q2+1, c[3];

  result[0] = q1[0]*q2[0] - InnerProduct (u1, u2);

  CrossProduct (c, u1, u2);
  result[1] = c[0] + q1[0]*q2[1] + q2[0]*q1[1];
  result[2] = c[1] + q1[0]*q2[2] + q2[0]*q1[2];
  result[3] = c[2] + q1[0]*q2[3] + q2[0]*q1[3];

  NormalizeQuaternion (result);
}

double QuaternionNorm (double *q)
{
  return (2 * atan2 (VectorNorm (q+1), fabs (q[0])));
}

double VectorNorm (double *w)
{
  return (sqrt (w[0]*w[0] + w[1]*w[1] + w[2]*w[2]));
}

double QuaternionDistance (double *q1, double *q2)
{
  double q3[4], q4[4];

  QuaternionConjugate (q2, q3);
  QuaternionMultiply (q4, q1, q3);
  return (QuaternionNorm (q4));
}

void PrintQuaternion (double *q)
{
  printf ("u0 : %8.3f  u1 : %8.3f  u2 : %8.3f  u3 : %8.3f \n",
	  q[0], q[1], q[2], q[3]);
}

void RandomQuaternion (double *q, double maxtheta)
{
  double w[3];
  int idum = 1;

  if ((maxtheta < 0.0) || (maxtheta > M_PI)) maxtheta = M_PI;

  do {
    w[0] = 2.0*ran0 (&idum) - 1.0;
    w[1] = 2.0*ran0 (&idum) - 1.0;
    w[2] = 2.0*ran0 (&idum) - 1.0;
  } while (VectorNorm (w) > 1.0);

  w[0] *= maxtheta;  w[1] *= maxtheta;  w[2] *= maxtheta;

  VectorToQuaternion (w, q);
}

int MakeQuaternion (double *q, double angle, char axis)
{
  double c, s;

  c = cos (angle/2);  s = sin (angle/2);

  switch (axis) {
    case 'x' : q[0] = c; q[1] = s;   q[2] = 0.0; q[3] = 0.0; break;
    case 'y' : q[0] = c; q[1] = 0.0; q[2] = s;   q[3] = 0.0; break;
    case 'z' : q[0] = c; q[1] = 0.0; q[2] = 0.0; q[3] = s;   break;
  }

  return 0;
}
