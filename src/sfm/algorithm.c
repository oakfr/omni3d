/*
 * algorithm.c : This file contains some routines that are used to recover the
 * structure of a scene from a set of edge correspondences in multiple
 * images.
 */

/* Includes ****************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "SFM.h"
#include "utils.h"
#include "quaternions.h"
#include "polygon.h"
#include "noise.h"

#pragma warning(disable: 4715)
#pragma warning(disable: 4716)

/* Forward Declarations *****************************/

int ReconstructPt (double *pt, double *v, double *d, double x);
int ComputeLineInterval (position *pos, edge *TheEdge, double *v, double *d,
			 double *x1, double *x2);
double ComputeClosestPoint (position *pos, double x, double y, 
			    double *v, double *d);

/* Globals ******************************************/

edge *TheEdges[MAX_M][MAX_N];
line *TheLines[MAX_N];
position *TheCameraPositions[MAX_M];
segment TheSegments[MAX_N];
int m, n, I, J;
char FrozenParameter;

/* Subroutines *************************************/

void ComputeV (double *v, line TheLine)
{
  double z[3];
  
  z[0] = 0.0;  z[1] = 0.0;  z[2] = 1.0;

  RotateQuaternion (v, TheLine.q, z);
}

void ComputeD (double *d, line TheLine)
{
  double x[3];
  
  x[0] = TheLine.a;  x[1] = TheLine.b;  x[2] = 0.0;

  RotateQuaternion (d, TheLine.q, x);
}

void ComputeM (double *m, line TheLine, position ThePosition)
{
  double v[3], d[3], a[3];

  ComputeV (v, TheLine);
  ComputeD (d, TheLine);
  SubtractVector (d, d, ThePosition.t);
  CrossProduct (a, v, d);
  RotateQuaternion (m, ThePosition.q, a);
}

char IndexToAxis (int index)
{
  switch (index) {
    case ALPHA : return ('x');  break;
    case BETA  : return ('y');  break;
    case GAMMA : return ('z');  break;
    default : pexit ("Error in IndexToAxis"); break;
  }
}

void UpdateLine_q (double *q, double *step, double *newq)
{
  double w[3], stepq[4], tempq[4];

  /* exp{J(R[theta*x + omega*y])}R = R * exp{J(theta*x + omega*y)} */

  w[0] = step[0];  w[1] = step[1];  w[2] = 0.0;

  VectorToQuaternion (w, stepq);
  QuaternionMultiply (tempq, q, stepq);

  newq[0] = tempq[0];
  newq[1] = tempq[1];
  newq[2] = tempq[2];
  newq[3] = tempq[3];
}

void UpdatePosition_q (double *q, double *step, double *newq)
{
  double stepq[4], tempq[4];

  VectorToQuaternion (step, stepq);
  QuaternionMultiply (tempq, stepq, q);

  newq[0] = tempq[0];
  newq[1] = tempq[1];
  newq[2] = tempq[2];
  newq[3] = tempq[3];
}

double v_error (line l, segment s)
{
  double v[3], v2[3], c;

  ComputeV (v, l);

  v2[0] = s.pt2[0] - s.pt1[0];
  v2[1] = s.pt2[1] - s.pt1[1];
  v2[2] = s.pt2[2] - s.pt1[2];

  Normalize (v2);
  c = InnerProduct (v, v2);

  return (1.0 - c*c);
}

double LineError (line l, segment s)
{
  double v[3], d[3], f[3], temp[3], p[3], q[3], a, b, c, length;
  
  ComputeV (v, l);
  ComputeD (d, l);

  SubtractVector (f, s.pt2, s.pt1);
  length = VectorNorm (f);

  a = InnerProduct (f, v);

  p[0] = f[0] - a*v[0];
  p[1] = f[1] - a*v[1];
  p[2] = f[2] - a*v[2];

  SubtractVector (q, s.pt1, d);
  a = InnerProduct (s.pt1, v);

  q[0] -= a*v[0];
  q[1] -= a*v[1];
  q[2] -= a*v[2];

  a = InnerProduct (p, p);
  b = 2*InnerProduct (p, q);
  c = InnerProduct (q, q);

  return (sqrt (a/3 + b/2 + c) / length);
}

double R_error (position p, position true_p)
{
  double tempq[4], stepq[4];

  QuaternionConjugate (true_p.q, tempq);
  QuaternionMultiply (stepq, tempq, p.q);

  return (QuaternionNorm (stepq));
}

double T_error (position p, position true_p)
{
  double a[3];

  SubtractVector (a, p.t, true_p.t);
  
  return (VectorNorm (a));
}

/*
 * routines used for reporting on the performance of the algorithm.
 */

int Report_R_errors ()
{
  double temp, sum;
  int j;
  
  printf ("\n Errors in R\n\n");

  for (j=1, sum=0.0; j < m; ++j) {
    temp = R_error (*TheCameraPositions[j], TrueCameraPositions[j]);
    printf ("Error in R[%d] = %12.3e\n", j, temp);
    sum += temp;
  }

  printf ("\nAverage error in R = %12.3e radians\n", sum / (m-1));
}

int Report_T_errors ()
{
  double temp, sum;
  int j;
   
  printf ("\n Errors in T\n\n");

  for (j=1, sum=0.0; j < m; ++j) {
    temp = T_error (*TheCameraPositions[j], TrueCameraPositions[j]);
    printf ("Error in T[%d] = %12.3e\n", j, temp);
    sum += temp;
  }

  printf ("\n Average error in T = %12.3e mm.\n", sum / (m-1));
}

int Report_v_errors ()
{
  double temp, sum;
  int i;
  
  printf ("\n Errors in v\n\n");

  for (i=0, sum=0.0; i < n; ++i) {
    temp = v_error (*TheLines[i], TrueSegments[i]);
    printf ("Error in v[%d] = %12.3e\n", i, temp);
    sum += temp;
  }

  printf ("\n Average error in v = %12.3e\n", sum / n);
}

int Report_line_errors ()
{
  double temp, sum;
  int i;
  
  printf ("\n Line Errors\n\n");

  for (i=0, sum=0.0; i < n; ++i) {
    temp = LineError (*TheLines[i], TrueSegments[i]);
    printf ("Error in line %d = %12.3e\n", i, temp);
    sum += temp;
  }

  printf ("\n Average error in lines = %12.3e mm.\n", sum / n);
}

double AverageRerror ()
{
  int j;
  double sum = 0.0;

  for (j=1; j < m; ++j)
    sum += R_error (*TheCameraPositions[j], TrueCameraPositions[j]);

  return (sum / (m-1));
}

double AverageTerror ()
{
  int j;
  double sum = 0.0;

  for (j=1; j < m; ++j)
    sum += T_error (*TheCameraPositions[j], TrueCameraPositions[j]);

  return (sum / (m-1));
}

double AverageVerror ()
{
  int i;
  double sum = 0.0;

  for (i=0; i < n; ++i)
    sum += v_error (*TheLines[i], TrueSegments[i]);

  return (sum / n);
}

double AverageLineError ()
{
  int i;
  double sum = 0.0;

  for (i=0; i < n; ++i)
    sum += LineError (*TheLines[i], TrueSegments[i]);

  return (sum / n);
}

int CheckRErrors (position *pos_buffer, double r_error)
{
  int j;
  
  for (j=1; j < m; ++j)
    if (R_error (*TheCameraPositions[j], pos_buffer[j]) > r_error)
      return (1);
  
  return (0);
}

int CheckTErrors (position *pos_buffer, double t_error)
{
  int j;
  double dt[3];

  for (j=1; j < m; ++j) {
    SubtractVector (dt, TheCameraPositions[j]->t, pos_buffer[j].t);
    if ((fabs(dt[0]) > t_error) || 
	(fabs(dt[1]) > t_error) ||
	(fabs(dt[2]) > t_error))
      return (1);
  }
  
  return (0);
}

/*
 * Routines used for reconstructing the endpoints of the recovered lines.
 */

int ReconstructEndPoints (segment *TheSegments)
{
  double x1, x2, maxima, minima;
  int i, j, first;
  double v[3], d[3];

  for (i=0; i < n; ++i) {
    ComputeV (v, *TheLines[i]);
    ComputeD (d, *TheLines[i]);
    first = 1;
    for (j=0; j < m; ++j) {
      if (TheCameraPositions[j] && TheEdges[j][i]) {
	if (first) {
	  ComputeLineInterval (TheCameraPositions[j], TheEdges[j][i], v, d,
			       &minima, &maxima);
	  first = 0;
	} else {
	  ComputeLineInterval (TheCameraPositions[j], TheEdges[j][i], v, d,
			       &x1, &x2);
	  maxima = max(maxima, x2);
	  minima = min(minima, x1);
	}
      }
    }

    ReconstructPt (TheSegments[i].pt1, v, d, minima);
    ReconstructPt (TheSegments[i].pt2, v, d, maxima);
  }
}

int ReconstructPt (double *pt, double *v, double *d, double x)
{
  pt[0] = (x * v[0]) + d[0];
  pt[1] = (x * v[1]) + d[1];
  pt[2] = (x * v[2]) + d[2];
}

int ComputeLineInterval (position *pos, edge *TheEdge, double *v, double *d,
			 double *x1, double *x2)
{
  double temp;

  *x1 = ComputeClosestPoint (pos, TheEdge->x1, TheEdge->y1, v, d);
  *x2 = ComputeClosestPoint (pos, TheEdge->x2, TheEdge->y2, v, d);

  if (*x2 < *x1) { temp = *x1;  *x1 = *x2;  *x2 = temp; }
}

double ComputeClosestPoint (position *pos, double x, double y,
			    double *v, double *d)
{
  double q[4], u[3], w[3];
  double a, b, c;

  w[0] = x;  w[1] = y;  w[2] = -1.0;
  QuaternionConjugate (pos->q, q);
  RotateQuaternion (u, q, w);
  Normalize (u);

  SubtractVector (w, d, pos->t);

  a = InnerProduct (u, v);
  b = InnerProduct (v, w);
  c = InnerProduct (u, w);

  return ( (b - a*c) / (a*a - 1.0) );
}


int TransformPt (double *out, position *pos, double *in)
{
  double temp[3];
  
  SubtractVector (temp, in, pos->t);
  RotateQuaternion (out, pos->q, temp);
}

/*
 * CheckReconstruction : This function is used to check whether the
 * reconstructed figure corresponds to the image data.
 */

int CheckReconstruction (double pixel_error)
{
  int i, j;
  double v[3], d[3], x, pt[3], temp[3], p, q, i_error;

  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i) {
      
      ComputeV (v, *TheLines[i]);
      ComputeD (d, *TheLines[i]);
      
      if (TheEdges[j][i] && TheCameraPositions[j]) {
	
	x = ComputeClosestPoint (TheCameraPositions[j],
				 TheEdges[j][i]->x1, TheEdges[j][i]->y1,
				 v, d);
	
	ReconstructPt (temp, v, d, x);
	TransformPt (pt, TheCameraPositions[j], temp);

	if (pt[2] > -10.0) return (1);

	p = - pt[0] / pt[2];	q = - pt[1] / pt[2];

	i_error = ImageError (TheEdges[j][i]->x1, TheEdges[j][i]->y1, p, q);
	if (i_error > pixel_error) return (1);
	
	
	x = ComputeClosestPoint (TheCameraPositions[j],
				 TheEdges[j][i]->x2, TheEdges[j][i]->y2,
				 v, d);

	
	ReconstructPt (temp, v, d, x);
	TransformPt (pt, TheCameraPositions[j], temp);

	if (pt[2] > -10.0) return (1);

	p = - pt[0] / pt[2];	q = - pt[1] / pt[2];
	
	i_error = ImageError (TheEdges[j][i]->x2, TheEdges[j][i]->y2, p, q);
	if (i_error > pixel_error) return (1);
	
      }
    }

  return (0);
}

int FreeTheEdges ()
{
  int i, j;

  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i)
      if (TheEdges[j][i])
	free (TheEdges[j][i]);
}

int FreeTheCameraPositions ()
{
  int j;

  for (j=0; j < m; ++j)
    free (TheCameraPositions[j]);
}

int FreeTheLines ()
{
  int i;

  for (i=0; i < n; ++i)
    free (TheLines[i]);
}
