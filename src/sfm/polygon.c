/*
 * polygon.c : This file contains a set of routines that are used to 
 * generate synthetic images of a transparent polygon.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
 
#include "SFM.h"
#include "utils.h"
#include "quaternions.h"

#include "polygon.h"

position TrueCameraPositions[MAX_M];
segment  TrueSegments[MAX_N];

#define RADIUS   250.0
#define DEPTH    1000.0

#define PT(v, x, y, z) {(v)[0] = (x); (v)[1] = (y); (v)[2] = (z);}

segment MakeSegment (double *pt1, double *pt2)
{
  segment s;

  s.pt1[0] = pt1[0];  s.pt1[1] = pt1[1];  s.pt1[2] = pt1[2];
  s.pt2[0] = pt2[0];  s.pt2[1] = pt2[1];  s.pt2[2] = pt2[2];

  return (s);
}

int MakeSquare (segment *segs, double startx, double starty, 
		double dx, double dy)
{
  double p1[3], p2[3], p3[3], p4[3];
  
  PT(p1, startx,     starty,     0.0);
  PT(p2, startx+dx,  starty,     0.0);
  PT(p3, startx+dx,  starty+dy,  0.0);
  PT(p4, startx,     starty+dy,  0.0);

  segs[0] = MakeSegment (p1, p2);
  segs[1] = MakeSegment (p2, p3);
  segs[2] = MakeSegment (p3, p4);
  segs[3] = MakeSegment (p4, p1);
}

int MakeSquares ()
{
  int i;
  double c[3];

  n = 16;

  MakeSquare (TrueSegments,    -250.0, -250.0, 200.0, 200.0);
  MakeSquare (TrueSegments+4,     0.0, -250.0, 200.0, 200.0);
  MakeSquare (TrueSegments+8,  -250.0,   50.0, 200.0, 200.0);
  MakeSquare (TrueSegments+12,    0.0,   50.0, 200.0, 200.0);

  c[0] = 0.0;  c[1] = 0.0;  c[2] = -DEPTH;
  for (i=0; i < n; ++i) {
    AddVector (TrueSegments[i].pt1, TrueSegments[i].pt1, c);
    AddVector (TrueSegments[i].pt2, TrueSegments[i].pt2, c);
  }
}

int cube12 ()
{
  int i;
  double p1[3], p2[3], p3[3], p4[3], p5[3], p6[3], p7[3], p8[3], c[3];

  n = 12;
  
  PT(p1,  100.0, -100.0,  100.0);
  PT(p2,  100.0, -100.0, -100.0);
  PT(p3, -100.0, -100.0, -100.0);
  PT(p4, -100.0, -100.0,  100.0);
  
  PT(p5,  100.0,  100.0,  100.0);
  PT(p6,  100.0,  100.0, -100.0);
  PT(p7, -100.0,  100.0, -100.0);
  PT(p8, -100.0,  100.0,  100.0);

  TrueSegments[0] = MakeSegment (p1, p2);
  TrueSegments[1] = MakeSegment (p2, p3);
  TrueSegments[2] = MakeSegment (p3, p4);
  TrueSegments[3] = MakeSegment (p4, p1);

  TrueSegments[4] = MakeSegment (p1, p5);
  TrueSegments[5] = MakeSegment (p2, p6);
  TrueSegments[6] = MakeSegment (p3, p7);
  TrueSegments[7] = MakeSegment (p4, p8);

  TrueSegments[8] = MakeSegment (p5, p6);
  TrueSegments[9] = MakeSegment (p6, p7);
  TrueSegments[10] = MakeSegment (p7, p8);
  TrueSegments[11] = MakeSegment (p8, p5);

  c[0] = 0.0;  c[1] = 0.0;  c[2] = -DEPTH;
  for (i=0; i < n; ++i) {
    AddVector (TrueSegments[i].pt1, TrueSegments[i].pt1, c);
    AddVector (TrueSegments[i].pt2, TrueSegments[i].pt2, c);
  }
}

int RotatedCubes (int count)
{
  int i, j;
  double q[4];
  double p1[3], p2[3], p3[3], p4[3], p5[3], p6[3], p7[3], p8[3], c[3];
  segment cube[12];
  
  PT(p1,  100.0, -100.0,  100.0);
  PT(p2,  100.0, -100.0, -100.0);
  PT(p3, -100.0, -100.0, -100.0);
  PT(p4, -100.0, -100.0,  100.0);
  
  PT(p5,  100.0,  100.0,  100.0);
  PT(p6,  100.0,  100.0, -100.0);
  PT(p7, -100.0,  100.0, -100.0);
  PT(p8, -100.0,  100.0,  100.0);

  cube[0] = MakeSegment (p1, p2);
  cube[1] = MakeSegment (p2, p3);
  cube[2] = MakeSegment (p3, p4);
  cube[3] = MakeSegment (p4, p1);

  cube[4] = MakeSegment (p1, p5);
  cube[5] = MakeSegment (p2, p6);
  cube[6] = MakeSegment (p3, p7);
  cube[7] = MakeSegment (p4, p8);

  cube[8] = MakeSegment (p5, p6);
  cube[9] = MakeSegment (p6, p7);
  cube[10] = MakeSegment (p7, p8);
  cube[11] = MakeSegment (p8, p5);

  for (i=0, n=0; i < count; ++i, n += 12) {
    RandomQuaternion (q, -1.0);
    for (j=0; j < 12; ++j) {
      RotateQuaternion (TrueSegments[n+j].pt1, q, cube[j].pt1);
      RotateQuaternion (TrueSegments[n+j].pt2, q, cube[j].pt2);
    }
  }

  c[0] = 0.0;  c[1] = 0.0;  c[2] = -DEPTH;
  for (i=0; i < n; ++i) {
    AddVector (TrueSegments[i].pt1, TrueSegments[i].pt1, c);
    AddVector (TrueSegments[i].pt2, TrueSegments[i].pt2, c);
  }     
}

int cube36 ()
{
  int i;
  double  p1[3],  p2[3],  p3[3],  p4[3],  p5[3],  p6[3],  p7[3],  p8[3], c[3];
  double  p9[3], p10[3], p11[3], p12[3], p13[3], p14[3], p15[3], p16[3];
  double p17[3], p18[3], p19[3], p20[3], p21[3], p22[3], p23[3], p24[3];

  n = 36;

  PT(p1,  40.0, -90.0,  90.0);
  PT(p2,  90.0, -40.0,  90.0);
  PT(p3,  90.0, -90.0,  40.0);

  PT(p4,  40.0, -90.0,  -90.0);
  PT(p5,  90.0, -40.0,  -90.0);
  PT(p6,  90.0, -90.0,  -40.0);

  PT(p7,  -40.0, -90.0, -90.0);
  PT(p8,  -90.0, -40.0, -90.0);
  PT(p9,  -90.0, -90.0, -40.0);

  PT(p10,  -40.0, -90.0, 90.0);
  PT(p11,  -90.0, -40.0, 90.0);
  PT(p12,  -90.0, -90.0, 40.0);

  PT(p13,  40.0, 90.0,  90.0);
  PT(p14,  90.0, 40.0,  90.0);
  PT(p15,  90.0, 90.0,  40.0);

  PT(p16,  40.0, 90.0,  -90.0);
  PT(p17,  90.0, 40.0,  -90.0);
  PT(p18,  90.0, 90.0,  -40.0);

  PT(p19,  -40.0, 90.0, -90.0);
  PT(p20,  -90.0, 40.0, -90.0);
  PT(p21,  -90.0, 90.0, -40.0);

  PT(p22,  -40.0, 90.0, 90.0);
  PT(p23,  -90.0, 40.0, 90.0);
  PT(p24,  -90.0, 90.0, 40.0);

  TrueSegments[0] = MakeSegment (p1, p2);
  TrueSegments[1] = MakeSegment (p2, p3);
  TrueSegments[2] = MakeSegment (p3, p1);

  TrueSegments[3] = MakeSegment (p4, p5);
  TrueSegments[4] = MakeSegment (p5, p6);
  TrueSegments[5] = MakeSegment (p6, p4);

  TrueSegments[6] = MakeSegment (p7, p8);
  TrueSegments[7] = MakeSegment (p8, p9);
  TrueSegments[8] = MakeSegment (p9, p7);

  TrueSegments[9] = MakeSegment (p10, p11);
  TrueSegments[10] = MakeSegment (p11, p12);
  TrueSegments[11] = MakeSegment (p12, p10);

  TrueSegments[12] = MakeSegment (p13, p14);
  TrueSegments[13] = MakeSegment (p14, p15);
  TrueSegments[14] = MakeSegment (p15, p13);

  TrueSegments[15] = MakeSegment (p16, p17);
  TrueSegments[16] = MakeSegment (p17, p18);
  TrueSegments[17] = MakeSegment (p18, p16);

  TrueSegments[18] = MakeSegment (p19, p20);
  TrueSegments[19] = MakeSegment (p20, p21);
  TrueSegments[20] = MakeSegment (p21, p19);

  TrueSegments[21] = MakeSegment (p22, p23);
  TrueSegments[22] = MakeSegment (p23, p24);
  TrueSegments[23] = MakeSegment (p24, p22);

  TrueSegments[24] = MakeSegment (p3, p6);
  TrueSegments[25] = MakeSegment (p9, p12);
  TrueSegments[26] = MakeSegment (p1, p10);
  TrueSegments[27] = MakeSegment (p4, p7);

  TrueSegments[28] = MakeSegment (p15, p18);
  TrueSegments[29] = MakeSegment (p21, p24);
  TrueSegments[30] = MakeSegment (p13, p22);
  TrueSegments[31] = MakeSegment (p16, p19);
  
  TrueSegments[32] = MakeSegment (p2, p14);
  TrueSegments[33] = MakeSegment (p5, p17);
  TrueSegments[34] = MakeSegment (p8, p20);
  TrueSegments[35] = MakeSegment (p11, p23);

  c[0] = 0.0;  c[1] = 0.0;  c[2] = -DEPTH;
  for (i=0; i < n; ++i) {
    AddVector (TrueSegments[i].pt1, TrueSegments[i].pt1, c);
    AddVector (TrueSegments[i].pt2, TrueSegments[i].pt2, c);
  } 
}

int RotatePt (double *out, double *in, double *c, double *q)
{
  double temp[3];

  SubtractVector (temp, in, c);
  RotateQuaternion (out, q, temp);
  AddVector (out, out, c);
}

int RotateSegments ()
{
  int i, idum = 1;
  double c[3], q[4];

  RandomQuaternion (q, -1.0);

  c[0] = 0.0;  c[1] = 0.0;  c[2] = -DEPTH;
  
  for (i=0; i < n; ++i) {
    RotatePt (TrueSegments[i].pt1, TrueSegments[i].pt1, c, q);
    RotatePt (TrueSegments[i].pt2, TrueSegments[i].pt2, c, q);
  }
}

int RandomPoint (double *p, double size)
{
  int idum = 1;

  p[0] = (2.0*ran0(&idum) - 1.0) * size;
  p[1] = (2.0*ran0(&idum) - 1.0) * size;
  p[2] = (2.0*ran0(&idum) - 1.0) * size;
}

int GenerateRandomLines (int n)
{
  int i;
  double c[3];

  c[0] = 0.0;  c[1] = 0.0;  c[2] = -DEPTH;

  for (i=0; i < n; ++i) {
    RandomPoint (TrueSegments[i].pt1, 100.0); 
    RandomPoint (TrueSegments[i].pt2, 100.0);

    AddVector (TrueSegments[i].pt1, TrueSegments[i].pt1, c);
    AddVector (TrueSegments[i].pt2, TrueSegments[i].pt2, c);
  }
}

int CircularStereoPositions2 (int m, double radius)
{
  int j;
  double theta, dtheta;
  position *q;
  double l, q1[4], q2[4], alpha, beta;

  /* l = length of line from camera position to center of fixation */
  l = sqrt (radius*radius + DEPTH*DEPTH);

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  /* note the first position always defines the frame of reference */  
  q->q[0] = 1.0;  q->q[1] = q->q[2] = q->q[3] = 0.0;
  q->t[0] = q->t[1] = q->t[2] = 0.0;

  TheCameraPositions[0]  = q;
  TrueCameraPositions[0] = *q;

  /* Generate Circular stereo positions */
  dtheta = (2*M_PI)/(m-1);
  for (j=1, theta=dtheta; j < m; ++j, theta += dtheta) {
    q = (position*) malloc (sizeof(position));
    if (!q) pexit ("Memory Error in GenerateCameraPositions");

    q->t[0] = radius * sin (theta);
    q->t[1] = radius * cos (theta);
    q->t[2] = 0.0;

    alpha = atan2 (q->t[0], DEPTH);
    beta  = atan2 (q->t[1], l);

    MakeQuaternion (q1, -alpha, 'y');
    MakeQuaternion (q2,  beta,  'x');

    QuaternionMultiply (q->q, q1, q2);
    
    TheCameraPositions[j]  = q;
    TrueCameraPositions[j] = *q;
  }
}

int CircularStereoPositions (int m)
{
  int j;
  double theta, dtheta;
  position *q;
  double l, q1[4], q2[4], alpha, beta;

  /* l = length of line from camera position to center of fixation */
  l = sqrt (RADIUS*RADIUS + DEPTH*DEPTH);

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  /* note the first position always defines the frame of reference */  
  q->q[0] = 1.0;  q->q[1] = q->q[2] = q->q[3] = 0.0;
  q->t[0] = q->t[1] = q->t[2] = 0.0;

  TheCameraPositions[0]  = q;
  TrueCameraPositions[0] = *q;

  /* Generate Circular stereo positions */
  dtheta = (2*M_PI)/(m-1);
  for (j=1, theta=dtheta; j < m; ++j, theta += dtheta) {
    q = (position*) malloc (sizeof(position));
    if (!q) pexit ("Memory Error in GenerateCameraPositions");

    q->t[0] = RADIUS * sin (theta);
    q->t[1] = RADIUS * cos (theta);
    q->t[2] = 0.0;

    alpha = atan2 (q->t[0], DEPTH);
    beta  = atan2 (q->t[1], l);

    MakeQuaternion (q1, -alpha, 'y');
    MakeQuaternion (q2,  beta,  'x');

    QuaternionMultiply (q->q, q1, q2);
    
    TheCameraPositions[j]  = q;
    TrueCameraPositions[j] = *q;
  }
}

int TrinocularStereo ()
{
  position *q;
  double y, z, q1[4], q2[4], alpha, beta;

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  /* note the first position always defines the frame of reference */  
  q->q[0] = 1.0;  q->q[1] = q->q[2] = q->q[3] = 0.0;
  q->t[0] = q->t[1] = q->t[2] = 0.0;

  TheCameraPositions[0]  = q;
  TrueCameraPositions[0] = *q;

  z = DEPTH - (sqr(RADIUS*1.5) / (2*DEPTH));
  y = sqrt (sqr(DEPTH) - sqr(z));

  /* Generate Trinocular stereo positions */

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");
  
  q->t[0] = RADIUS * (sqrt(3.0) / 2.0);
  q->t[1] = y;
  q->t[2] = z - DEPTH;

  alpha = atan2 (q->t[0], z);
  beta  = atan2 (q->t[1], DEPTH);
  
  MakeQuaternion (q1, -alpha, 'y');
  MakeQuaternion (q2,  beta,  'x');
  
  QuaternionMultiply (q->q, q1, q2);
  
  TheCameraPositions[1]  = q;
  TrueCameraPositions[1] = *q;

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");
  
  q->t[0] = -RADIUS * (sqrt(3.0) / 2.0);
  q->t[1] = y;
  q->t[2] = z - DEPTH;
  
  MakeQuaternion (q1,  alpha, 'y');
  MakeQuaternion (q2,  beta,  'x');
  
  QuaternionMultiply (q->q, q1, q2);
  
  TheCameraPositions[2]  = q;
  TrueCameraPositions[2] = *q;

  m = 3;
}

position FlyingPosition (double theta_z, double theta_y, double r, double *c)
{
  position p;
  double q1[4], q2[4], q3[4], z[3], v[3];

  MakeQuaternion (q1, theta_y, 'y');
  MakeQuaternion (q2, theta_z, 'z');

  QuaternionMultiply (q3, q2, q1);
  z[0] = 0.0;  z[1] = 0.0;  z[2] = 1.0;
  RotateQuaternion (v, q3, z);

  QuaternionConjugate (q3, p.q);
  p.t[0] = r * v[0];
  p.t[1] = r * v[1];
  p.t[2] = r * v[2];

  AddVector (p.t, p.t, c);

  return (p);
}


int FlyingCameraPositions ()
{
  double c[3];
  position *q;

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  /* note the first position always defines the frame of reference */  
  q->q[0] = 1.0;  q->q[1] = q->q[2] = q->q[3] = 0.0;
  q->t[0] = q->t[1] = q->t[2] = 0.0;

  TheCameraPositions[0]  = q;
  TrueCameraPositions[0] = *q;

  c[0] = c[1] = 0.0; c[2] = -DEPTH;


  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  *q = FlyingPosition (0.0, (M_PI/4), 1000.0, c);

  TheCameraPositions[1]  = q;
  TrueCameraPositions[1] = *q;

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  *q = FlyingPosition (0.0, (M_PI/2), 1000.0, c);

  TheCameraPositions[2]  = q;
  TrueCameraPositions[2] = *q;

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  *q = FlyingPosition (0.0, -(M_PI/4), 1000.0, c);

  TheCameraPositions[3]  = q;
  TrueCameraPositions[3] = *q;

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  *q = FlyingPosition (0.0, -(M_PI/2), 1000.0, c);

  TheCameraPositions[4]  = q;
  TrueCameraPositions[4] = *q;


  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  *q = FlyingPosition (M_PI_2, (M_PI/4), 1000.0, c);

  TheCameraPositions[5]  = q;
  TrueCameraPositions[5] = *q;

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  *q = FlyingPosition (M_PI_2, (M_PI/2), 1000.0, c);

  TheCameraPositions[6]  = q;
  TrueCameraPositions[6] = *q;

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  *q = FlyingPosition (M_PI_2, -(M_PI/4), 1000.0, c);

  TheCameraPositions[7]  = q;
  TrueCameraPositions[7] = *q;

  q = (position*) malloc (sizeof(position));
  if (!q) pexit ("Memory Error in GenerateCameraPositions");

  *q = FlyingPosition (M_PI_2, -(M_PI/2), 1000.0, c);

  TheCameraPositions[8]  = q;
  TrueCameraPositions[8] = *q;

  m = 9;
}


edge *ProjectSegment (position q, segment p)
{
  edge *TheEdge;
  double a[3], b[3];

  /* N.B. this routine currently assumes that every line 
   * is visible in every image */

  TheEdge = (edge*) malloc (sizeof(edge));
  if (!TheEdge) pexit ("Memory Error in ProjectSegment");

  /* project pt1 */

  SubtractVector (a, p.pt1, q.t);
  RotateQuaternion (b, q.q, a);

  if (b[2] == 0.0) pexit ("Arithmetic error in ProjectSegment");
  TheEdge->x1 = -b[0] / b[2];  TheEdge->y1 = -b[1] / b[2];

  /* project pt2 */

  SubtractVector (a, p.pt2, q.t);
  RotateQuaternion (b, q.q, a);

  if (b[2] == 0.0) pexit ("Arithmetic error in ProjectSegment");
  TheEdge->x2 = -b[0] / b[2];  TheEdge->y2 = -b[1] / b[2];

  /* calculate edge length */

  TheEdge->length = sqrt (sqr(TheEdge->x2 - TheEdge->x1) + 
			  sqr(TheEdge->y2 - TheEdge->y1));

  /* m = (x1 y1 -1) ^ (x2 y2 -1) */
  TheEdge->m[0] = TheEdge->y2 - TheEdge->y1;
  TheEdge->m[1] = TheEdge->x1 - TheEdge->x2;
  TheEdge->m[2] = (TheEdge->x1)*(TheEdge->y2) - (TheEdge->x2)*(TheEdge->y1);

  Normalize (TheEdge->m);

  return (TheEdge);
}

int GenerateEdges ()
{
  int i, j;

  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i)
      TheEdges[j][i] = ProjectSegment (TrueCameraPositions[j],
				       TrueSegments[i]);

	return 0;
}
