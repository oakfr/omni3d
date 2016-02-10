/*
 * stageCD.c : This file contains the routines that are used to implement
 * the last two stages of the SFM process. Notice that both stages work
 * by minimizing the same objective function, O2.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "sym_mat.h" 
#include "SFM.h"
#include "utils.h"
#include "quaternions.h"
#include "algorithm.h"
#include "LieCalculus.h"

#pragma warning(disable: 4101)
#pragma warning(disable: 4033)
#pragma warning(disable: 4715)
#pragma warning(disable: 4716)


/* Function Declarations ********************************/

int FN_C   (double *x, double *res);
int GRAD_C (double *x, double *g);
int HESS_C (double *x, double *H);
int UPDATE_C (double *x, double *step, double *newx);

int FN_D   (double *x, double *res);
int GRAD_D (double *x, double *g);
int HESS_D (double *x, double *H);
int UPDATE_D (double *x, double *step, double *newx);

double Gradient2 (edge *TheEdge, position *ThePosition, line *TheLine,
		  int index);
double Hessian2 (edge *TheEdge, position *ThePositions, line *TheLine,
		 int index_x, int index_y);
double QuadraticFormA (edge TheEdge, double *v1, double *v2);
double QuadraticFormB (double *v1, double *v2);
int DerivativeM (edge TheEdge, position ThePosition, line TheLine,
		 int index, double *dm_dx);
int DoubleDerivativeM (edge TheEdge, position ThePosition, line TheLine,
		       int index_x, int index_y, double *dm_dxdy);

int DerivativeV (line TheLine, int index, double *dv_dx);
int DoubleDerivativeV (line TheLine, int index_x, int index_y, 
		       double *dv_dxdy);
int DerivativeD (line TheLine, int index, double *dd_dx);
int DoubleDerivativeD (line TheLine, int index_x, int index_y,
		       double *dd_dxdy);

/** Shared routines *************************************/

double Error2 (edge *TheEdge, position *ThePosition, line *TheLine)
{
  double m[3], h1, h2, norm;

  if (!TheEdge || !ThePosition || !TheLine) return (0.0);
  
  ComputeM (m, *TheLine, *ThePosition);

  return (QuadraticFormA (*TheEdge, m, m) / QuadraticFormB (m, m));
}

double Gradient2 (edge *TheEdge, position *ThePosition, line *TheLine,
		  int index)
{
  double m[3], dm_dx[3], term1, term2, temp;

  if (!TheLine || !ThePosition || !TheEdge) return (0.0);
  
  ComputeM (m, *TheLine, *ThePosition);
  DerivativeM (*TheEdge, *ThePosition, *TheLine, index, dm_dx);

  temp = QuadraticFormB (m, m);
  term1 = (temp*QuadraticFormA (*TheEdge, m, dm_dx));
  term2 = (QuadraticFormA(*TheEdge, m, m) * QuadraticFormB(m, dm_dx));

  return (2*(term1 - term2)/(temp*temp));
}

double Hessian2 (edge *TheEdge, position *ThePosition, line *TheLine,
		 int index_x, int index_y)
{
  double m[3], dm_dx[3], dm_dy[3], dm_dxdy[3];
  double term1, term2, term3, temp;

  if (!TheLine || !ThePosition || !TheEdge) return (0.0);
  
  ComputeM (m, *TheLine, *ThePosition);
  DerivativeM (*TheEdge, *ThePosition, *TheLine, index_x, dm_dx);
  DerivativeM (*TheEdge, *ThePosition, *TheLine, index_y, dm_dy);
  DoubleDerivativeM (*TheEdge, *ThePosition, *TheLine,
		     index_x, index_y, dm_dxdy);

  temp = QuadraticFormB (m, m);

  term1 = QuadraticFormB (m, m)*QuadraticFormA (*TheEdge, m, dm_dx) -
	  QuadraticFormA (*TheEdge, m, m)*QuadraticFormB (m, dm_dx);
  term1 *= (-8*QuadraticFormB(m, dm_dy)/(temp*temp*temp));

  term2 = 2*QuadraticFormB (m, dm_dy)*QuadraticFormA (*TheEdge, m, dm_dx) +
          QuadraticFormB (m, m)*QuadraticFormA (*TheEdge, dm_dy, dm_dx)   -
          2*QuadraticFormA (*TheEdge, m, dm_dy)*QuadraticFormB (m, dm_dx) -
	  QuadraticFormA (*TheEdge, m, m)*QuadraticFormB (dm_dy, dm_dx);
  term2 *= 2/(temp*temp);

  term3 = QuadraticFormB (m, m)*QuadraticFormA (*TheEdge, m, dm_dxdy) -
	  QuadraticFormA (*TheEdge, m, m)*QuadraticFormB (m, dm_dxdy);
  term3 *= 2/(temp*temp);

  return (term1 + term2 + term3);
}

double QuadraticFormA (edge TheEdge, double *v1, double *v2)
{
  double h1, h2, h3, h4, l;
  
  l = TheEdge.length;

  h1 = (TheEdge.x1)*v1[0] + (TheEdge.y1)*v1[1] - v1[2];
  h2 = (TheEdge.x2)*v1[0] + (TheEdge.y2)*v1[1] - v1[2];

  h3 = (TheEdge.x1)*v2[0] + (TheEdge.y1)*v2[1] - v2[2];
  h4 = (TheEdge.x2)*v2[0] + (TheEdge.y2)*v2[1] - v2[2];

  return (l*(h1*(h3 + 0.5*h4) + h2*(h3*0.5 + h4))/3);
}

double QuadraticFormB (double *v1, double *v2)
{
  return (v1[0]*v2[0] + v1[1]*v2[1]);
}

int DerivativeM (edge TheEdge, position ThePosition, line TheLine,
		 int index, double *dm_dx)
{
  double v[3], d[3], a[3], b[3], c[3], dv_dx[3], dd_dx[3], w[3];

  ComputeV (v, TheLine);
  ComputeD (d, TheLine);

  switch (index) {
    case THETA :
    case OMEGA :
      DerivativeV (TheLine, index, dv_dx);
      DerivativeD (TheLine, index, dd_dx);
      SubtractVector (d, d, ThePosition.t);
      CrossProduct (a, dv_dx, d);
      CrossProduct (b, v, dd_dx);
      AddVector (a, a, b);
      RotateQuaternion (dm_dx, ThePosition.q, a);
      break;

    case A :
      a[0] = 0.0;  a[1] = 1.0;  a[2] = 0.0;
      RotateQuaternion (b, TheLine.q, a);
      RotateQuaternion (dm_dx, ThePosition.q, b);
      break;

    case B :
      a[0] = -1.0;  a[1] = 0.0;  a[2] = 0.0;
      RotateQuaternion (b, TheLine.q, a);
      RotateQuaternion (dm_dx, ThePosition.q, b);
      break;

    case ALPHA :
    case BETA :
    case GAMMA :
      SubtractVector (d, d, ThePosition.t);
      CrossProduct (a, v, d);
      RotateQuaternion (b, ThePosition.q, a);
      DerivR (dm_dx, b, IndexToAxis (index));
      break;

    case TX :
      a[0] = -1.0;  a[1] = 0.0;  a[2] = 0.0;
      CrossProduct (b, v, a);
      RotateQuaternion (dm_dx, ThePosition.q, b);
      break;

    case TY :
      a[0] = 0.0;  a[1] = -1.0;  a[2] = 0.0;
      CrossProduct (b, v, a);
      RotateQuaternion (dm_dx, ThePosition.q, b);
      break;

    case TZ :
      a[0] = 0.0;  a[1] = 0.0;  a[2] = -1.0;
      CrossProduct (b, v, a);
      RotateQuaternion (dm_dx, ThePosition.q, b);
      break;

    default :
      pexit ("Bad Parameters to DerivativeM");
      break;
  }
}

int DoubleDerivativeM (edge TheEdge, position ThePosition, line TheLine,
		       int index_x, int index_y, double *dm_dxdy)
{
  int index;
  double v[3], d[3], dv_dx[3], dv_dy[3], dd_dx[3], dd_dy[3];
  double dv_dxdy[3], dd_dxdy[3], a[3], b[3], c[3];

  if (index_y < index_x) { index=index_x; index_x=index_y; index_y=index; }

  dm_dxdy[0] = 0.0;  dm_dxdy[1] = 0.0;  dm_dxdy[2] = 0.0;

  ComputeV (v, TheLine);
  ComputeD (d, TheLine);

  /* Case 1 */
  if ( (index_x==THETA || index_x==OMEGA) &&
       (index_y==THETA || index_y==OMEGA || index_y==A || index_y==B) ) {
    DerivativeV (TheLine, index_x, dv_dx);
    DerivativeV (TheLine, index_y, dv_dy);
    DoubleDerivativeV (TheLine, index_x, index_y, dv_dxdy);

    DerivativeD (TheLine, index_x, dd_dx);
    DerivativeD (TheLine, index_y, dd_dy);
    DoubleDerivativeD (TheLine, index_x, index_y, dd_dxdy);
    
    SubtractVector (d, d, ThePosition.t);

    CrossProduct (a, dv_dxdy, d);
    CrossProduct (b, dv_dx, dd_dy);
    AddVector (a, a, b);
    CrossProduct (b, dv_dy, dd_dx);
    AddVector (a, a, b);
    CrossProduct (b, v, dd_dxdy);
    AddVector (a, a, b);

    RotateQuaternion (dm_dxdy, ThePosition.q, a);
  }

  /* Case 2 */
  if ( (index_x == ALPHA || index_x == BETA || index_x == GAMMA) &&
       (index_y == ALPHA || index_y == BETA || index_y == GAMMA) ) {
    SubtractVector (d, d, ThePosition.t);
    CrossProduct (a, v, d);
    RotateQuaternion (b, ThePosition.q, a);
    DoubleDerivR (dm_dxdy, b, IndexToAxis (index_x), IndexToAxis (index_y));
  }

  /* Case 3 */
  if ( (index_x == ALPHA || index_x == BETA || index_x == GAMMA) &&
       (index_y == TX || index_y == TY || index_y == TZ) ) {

    switch (index_y) {
      case TX : a[0] = -1.0;  a[1] =  0.0;  a[2] =  0.0;  break;
      case TY : a[0] =  0.0;  a[1] = -1.0;  a[2] =  0.0;  break;
      case TZ : a[0] =  0.0;  a[1] =  0.0;  a[2] = -1.0;  break;
    }

    CrossProduct (b, v, a);
    RotateQuaternion (c, ThePosition.q, b);
    DerivR (dm_dxdy, c, IndexToAxis (index_x));    
  }

  /* Case 4 */
  if ( (index_x==THETA || index_x==OMEGA || index_x==A || index_x==B) &&
       (index_y==ALPHA || index_y==BETA || index_y==GAMMA) ) {
    DerivativeV (TheLine, index_x, dv_dx);
    DerivativeD (TheLine, index_x, dd_dx);
    
    SubtractVector (d, d, ThePosition.t);
    CrossProduct (a, dv_dx, d);
    CrossProduct (b, v, dd_dx);
    AddVector (a, a, b);
    RotateQuaternion (b, ThePosition.q, a);
    DerivR (dm_dxdy, b, IndexToAxis (index_y));
  }

  /* Case 5 */
  if ( (index_x == THETA || index_x == OMEGA) &&
       (index_y == TX || index_y == TY || index_y == TZ) ) {

    switch (index_y) {
      case TX : a[0] = -1.0;  a[1] =  0.0;  a[2] =  0.0;  break;
      case TY : a[0] =  0.0;  a[1] = -1.0;  a[2] =  0.0;  break;
      case TZ : a[0] =  0.0;  a[1] =  0.0;  a[2] = -1.0;  break;
    }
    
    DerivativeV (TheLine, index_x, dv_dx);
    CrossProduct (b, dv_dx, a);
    RotateQuaternion (dm_dxdy, ThePosition.q, b);
  }
}

int DerivativeV (line TheLine, int index, double *dv_dx)
{
  double a[3];

  dv_dx[0] = dv_dx[1] = dv_dx[2] = 0.0;

  switch (index) {
    case THETA :
      a[0] = 0.0;  a[1] = -1.0;  a[2] = 0.0;
      RotateQuaternion (dv_dx, TheLine.q, a);
      break;

    case OMEGA :
      a[0] = 1.0;  a[1] = 0.0;  a[2] = 0.0;
      RotateQuaternion (dv_dx, TheLine.q, a);
      break;
  }
}

int DoubleDerivativeV (line TheLine, int index_x, int index_y, double *dv_dxdy)
{
  dv_dxdy[0] = dv_dxdy[1] = dv_dxdy[2] = 0.0;

  if ((index_x == THETA && index_y == THETA) ||
      (index_x == OMEGA && index_y == OMEGA)) {
    ComputeV (dv_dxdy, TheLine);
    dv_dxdy[0] = - dv_dxdy[0];
    dv_dxdy[1] = - dv_dxdy[1];
    dv_dxdy[2] = - dv_dxdy[2];
  }
}

int DerivativeD (line TheLine, int index, double *dd_dx)
{
  double a[3];

  dd_dx[0] = dd_dx[1] = dd_dx[2] = 0.0;

  switch (index) {
    case THETA :
      a[0] = 0.0;  a[1] = 0.0;  a[2] = TheLine.b;
      RotateQuaternion (dd_dx, TheLine.q, a);      
      break;

    case OMEGA :
      a[0] = 0.0;  a[1] = 0.0;  a[2] = -TheLine.a;
      RotateQuaternion (dd_dx, TheLine.q, a);      
      break;

    case A :
      a[0] = 1.0;  a[1] = 0.0;  a[2] = 0.0;
      RotateQuaternion (dd_dx, TheLine.q, a);      
      break;

    case B :
      a[0] = 0.0;  a[1] = 1.0;  a[2] = 0.0;
      RotateQuaternion (dd_dx, TheLine.q, a);      
      break;
  }
}


int DoubleDerivativeD (line TheLine, int index_x, int index_y, double *dd_dxdy)
{
  double a[3];

  dd_dxdy[0] = dd_dxdy[1] = dd_dxdy[2] = 0.0;

  if (index_x == THETA && index_y == THETA) {
    a[0] = 0.0;  a[1] = -TheLine.b;  a[2] = 0.0;
    RotateQuaternion (dd_dxdy, TheLine.q, a);
  }

  if (index_x == OMEGA && index_y == OMEGA) {
    a[0] = -TheLine.a;  a[1] = 0.0;  a[2] = 0.0;
    RotateQuaternion (dd_dxdy, TheLine.q, a);
  }

  if ((index_x == THETA && index_y == OMEGA) ||
      (index_x == OMEGA && index_y == THETA)) {
    a[0] = TheLine.b / 2.0;  a[1] = TheLine.a / 2.0;  a[2] = 0.0;
    RotateQuaternion (dd_dxdy, TheLine.q, a);
  }

  if ((index_x == THETA && index_y == B) ||
      (index_x == B && index_y == THETA)) {
    a[0] = 0.0;  a[1] = 0.0;  a[2] = 1.0;
    RotateQuaternion (dd_dxdy, TheLine.q, a);
  }

  if ((index_x == OMEGA && index_y == A) ||
      (index_x == A && index_y == OMEGA)) {
    a[0] = 0.0;  a[1] = 0.0;  a[2] = -1.0;
    RotateQuaternion (dd_dxdy, TheLine.q, a);
  }
}

int eliminate_row (double *H, int index, int n)
{
  int i, j;

  if (index >= n) return;

  for (i=index; i < (n-1); ++i) {
    for (j=0; j < index; ++j)
      H[elt(i, j)] = H[elt(i+1, j)];
    for (j=index; j <= i; ++j)
      H[elt(i, j)] = H[elt(i+1, j+1)];
  }
}

/** STAGE C routines ************************************/

/*
 * InitStageC () : This routine performs a linear least squares calculation 
 * to obtain starting estimates for a, b, tx, ty and tz for the whole system.
 */

int InitStageC ()
{
  int i, j;
  double p, q, r, s, t;
  double f[3], l[3], e[3], tempq[4];
  double b[2*MAX_N + 3*(MAX_M-1)], x[2*MAX_N + 3*(MAX_M-1)];
  double *H, *L, maxadd, scale, m_error;

  H = make_sym_matrix (2*n + 3*(m-1));
  L = make_sym_matrix (2*n + 3*(m-1));

  for (i=0; i < (2*n + 3*(m-1)); ++i)
    for (j=0; j <= i; ++j)
      H[elt(i, j)] = 0.0;
  
  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i) 
      if (TheEdges[j][i]) {
	QuaternionConjugate (TheCameraPositions[j]->q, tempq);
	RotateQuaternion (f, tempq, TheEdges[j][i]->m);
	
	e[0] = 1.0;  e[1] = 0.0;  e[2] = 0.0;
	RotateQuaternion (l, TheLines[i]->q, e);
	p = InnerProduct (f, l);
	
	e[0] = 0.0;  e[1] = 1.0;  e[2] = 0.0;
	RotateQuaternion (l, TheLines[i]->q, e);
	q = InnerProduct (f, l);

	r = - f[0];   s = - f[1];   t = - f[2];

	/* scale the errors */
	m_error = TheEdges[j][i]->m_error;
	p /= m_error; q /= m_error; r /= m_error; s /= m_error; t /= m_error;

	H[elt(2*i, 2*i)]      +=  p*p;
	H[elt(2*i, 2*i+1)]    +=  p*q;
	H[elt(2*i+1, 2*i+1)]  +=  q*q;

	if (j) {
	  H[elt(2*i, 2*n+3*(j-1))]      +=  p*r;
	  H[elt(2*i, 2*n+3*(j-1)+1)]    +=  p*s;
	  H[elt(2*i, 2*n+3*(j-1)+2)]    +=  p*t;

	  H[elt(2*i+1, 2*n+3*(j-1))]    +=  q*r;
	  H[elt(2*i+1, 2*n+3*(j-1)+1)]  +=  q*s;
	  H[elt(2*i+1, 2*n+3*(j-1)+2)]  +=  q*t;

	  H[elt(2*n+3*(j-1),   2*n+3*(j-1))]    +=  r*r;
	  H[elt(2*n+3*(j-1),   2*n+3*(j-1)+1)]  +=  r*s;
	  H[elt(2*n+3*(j-1),   2*n+3*(j-1)+2)]  +=  r*t;
	  H[elt(2*n+3*(j-1)+1, 2*n+3*(j-1)+1)]  +=  s*s;
	  H[elt(2*n+3*(j-1)+1, 2*n+3*(j-1)+2)]  +=  s*t;
	  H[elt(2*n+3*(j-1)+2, 2*n+3*(j-1)+2)]  +=  t*t;
	}
      }
  
  /* solve the linear least squares problem */
 
  /* extract b vector */

  /* eliminate row */
  switch (FrozenParameter) {
    case 'x':
      scale = TheCameraPositions[m-1]->t[0];
      for (i=0; i < (2*n + 3*(m-1)); ++i)
	b[i] = scale * H[elt(i, (2*n + 3*(m-1) - 3))];
      b[2*n + 3*(m-1) - 3] = b[2*n + 3*(m-1) - 2];
      b[2*n + 3*(m-1) - 2] = b[2*n + 3*(m-1) - 1];
      eliminate_row (H, (2*n + 3*(m-1) - 3), (2*n + 3*(m-1)));
      break;

    case 'y' :
      scale = TheCameraPositions[m-1]->t[1];
      for (i=0; i < (2*n + 3*(m-1)); ++i)
	b[i] = scale * H[elt(i, (2*n + 3*(m-1) - 2))];
      b[2*n + 3*(m-1) - 2] = b[2*n + 3*(m-1) - 1];
      eliminate_row (H, (2*n + 3*(m-1) - 2), (2*n + 3*(m-1)));
      break;

    case 'z' :
      scale = TheCameraPositions[m-1]->t[2];
      for (i=0; i < (2*n + 3*(m-1)); ++i) 
	b[i] = scale * H[elt(i, (2*n + 3*(m-1) - 1))];
      break;
  }

  CHOLDECOMP ((2*n + 3*(m-1) - 1), H, 0.0, 5.5e-17, L, &maxadd);
  if (maxadd != 0.0) pexit ("InitStageC : Matrix H not positive definite");

  CHOLSOLVE ((2*n + 3*(m-1) - 1), b, L, x);

  /* Fill the data structures */
  FillStructs_C (x);

//  free (H);
//  free (L);
}

int REPORT_C (int itncount, double *x,
	      double fc, double *gc, double *H,
	      int nfev, int njev, int nhev)
{
  int i;
  double max;

  for (i=1, max = fabs (gc[0]); i < (2*n + 3*(m-1) - 1); ++i)
    if (fabs (gc[i]) > max) max = fabs (gc[i]);

  printf ("%5d %12.5e %12.5e %5d %5d %5d\n", itncount, fc, max,
	  nfev, njev, nhev);
}

int StageC ()
{
  int i, j, k;
  double *H, *L, g[2*MAX_N+3*(MAX_M-1)];
  double typx[2*MAX_N+3*(MAX_M-1)], x[2*MAX_N+3*(MAX_M-1)], f;
  int nfev, njev, nhev, info;

  H = make_sym_matrix (2*n + 3*(m-1));
  L = make_sym_matrix (2*n + 3*(m-1));

  InitStageC ();

  /* initialize x */
  for (i=0; i < n; ++i) {
    x[2*i]   = TheLines[i]->a;
    x[2*i+1] = TheLines[i]->b;
  }
  
  for (j=1; j < (m-1); ++j)
    for (k=0; k < 3; ++k)
      x[2*n+3*(j-1)+k] = TheCameraPositions[j]->t[k];
  
  switch (FrozenParameter) {
    case 'x' :
      x[2*n+3*(m-2)]   = TheCameraPositions[(m-1)]->t[1];
      x[2*n+3*(m-2)+1] = TheCameraPositions[(m-1)]->t[2];
      break;

    case 'y' :
      x[2*n+3*(m-2)]   = TheCameraPositions[(m-1)]->t[0];
      x[2*n+3*(m-2)+1] = TheCameraPositions[(m-1)]->t[2];
      break;

    case 'z' :
      x[2*n+3*(m-2)]   = TheCameraPositions[(m-1)]->t[0];
      x[2*n+3*(m-2)+1] = TheCameraPositions[(m-1)]->t[1];
      break;
  }

  /* intitialize typx */
  for (i=0; i < (2*n + 3*(m-1)); ++i) typx[i] = 100.0;

#ifdef REPORT

  UMDRIVER ((2*n + 3*(m-1) - 1), (2*n + 3*(m-1) - 1),
            FN_C, GRAD_C, HESS_C, UPDATE_C,
	    REPORT_C,
            typx, 0.01,
            1.0e-8, 1.0e-8, 100.0, 50,
            x, &f, g,
            &nfev, &njev, &nhev, &info,
            L, H);

#else

  UMDRIVER ((2*n + 3*(m-1) - 1), (2*n + 3*(m-1) - 1),
            FN_C, GRAD_C, HESS_C, UPDATE_C,
	    (void*) 0x00,
            typx, 0.01,
            1.0e-8, 1.0e-8, 100.0, 50,
            x, &f, g,
            &nfev, &njev, &nhev, &info,
            L, H);

#endif
  
  FillStructs_C (x);

//  free (H);
//  free (L);

  return (info);
}

int FN_C (double *x, double *res)
{
  int i, j;

  FillStructs_C (x);
  
  *res = 0.0;
  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i)
      *res += Error2 (TheEdges[j][i], TheCameraPositions[j], TheLines[i]);
}

int GRAD_C (double *x, double *g)
{
  int i, j, index;

  FillStructs_C (x);

  for (i=0; i < (2*n + 3*(m-1)); ++i) g[i] = 0.0;

  for (j=0; j < m; ++j) 
    for (i=0; i < n; ++i) {
      for (index=A; index <= B; ++index)
	g[2*i + (index - A)] += Gradient2 (TheEdges[j][i],
					   TheCameraPositions[j],
					   TheLines[i],
					   index);
      
      if (j) {
	for (index=TX; index <= TZ; ++index)
	  g[2*n + (j-1)*3 + (index - TX)] += Gradient2 (TheEdges[j][i],
							TheCameraPositions[j],
							TheLines[i],
							index);
      }
    }

  switch (FrozenParameter) {
    case 'x' :
      g[2*n + 3*(m-2)]   = g[2*n + 3*(m-2)+1];
      g[2*n + 3*(m-2)+1] = g[2*n + 3*(m-2)+2];
      break;

    case 'y' :
      g[2*n + 3*(m-2)+1] = g[2*n + 3*(m-2)+2];
      break;

    case 'z' :
      break;
  }
}

int HESS_C (double *x, double *H)
{
  int i, j, index_x, index_y;

  FillStructs_C (x);

  for (i=0; i < (2*n + 3*(m-1)); ++i) 
    for (j=0; j <= i; ++j)
      H[elt(i, j)] = 0.0;
  
  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i) {
      
      for (index_x=A; index_x <= B; ++index_x)
	for (index_y=A; index_y <= index_x; ++index_y)
	  H[elt(2*i+(index_x-A), 2*i+(index_y-A))] +=
	    Hessian2 (TheEdges[j][i],
		      TheCameraPositions[j],
		      TheLines[i],
		      index_x, index_y);
      
      if (j) {
	for (index_x=A; index_x <= B; ++index_x)
	  for (index_y=TX; index_y <= TZ; ++index_y)
	    H[elt(2*i+(index_x-A), 2*n+(j-1)*3+(index_y-TX))] +=
	      Hessian2 (TheEdges[j][i],
			TheCameraPositions[j],
			TheLines[i],
			index_x, index_y);
	
	for (index_x=TX; index_x <= TZ; ++index_x)
	  for (index_y=TX; index_y <= index_x; ++index_y)
	    H[elt(2*n+(j-1)*3+(index_x-TX), 2*n+(j-1)*3+(index_y-TX))] +=
	      Hessian2 (TheEdges[j][i],
			TheCameraPositions[j],
			TheLines[i],
			index_x, index_y);
      }
    }
  
  switch (FrozenParameter) {
    case 'x' : eliminate_row (H, (2*n + 3*(m-2)),   (2*n + 3*(m-1)));  break;
    case 'y' : eliminate_row (H, (2*n + 3*(m-2)+1), (2*n + 3*(m-1)));  break;
    case 'z' : break;
  }

}

int UPDATE_C (double *x, double *step, double *newx)
{
  int i;

  for (i=0; i < (2*n + 3*(m-1) - 1); ++i)
    newx[i] = x[i] + step[i];
}

int FillStructs_C (double *x)
{
  double *ptr;
  int i, j;

  /* fill in data structures */
  for (i=0, ptr = x; i < n; ++i, ptr += 2) {
    TheLines[i]->a = ptr[0];
    TheLines[i]->b = ptr[1];
  }

  for (j=1; j < (m-1); ++j, ptr += 3) {
    TheCameraPositions[j]->t[0] = ptr[0];
    TheCameraPositions[j]->t[1] = ptr[1];
    TheCameraPositions[j]->t[2] = ptr[2];
  }

  switch (FrozenParameter) {
    case 'x' :
      TheCameraPositions[m-1]->t[1] = ptr[0];
      TheCameraPositions[m-1]->t[2] = ptr[1];
      break;
    case 'y' :
      TheCameraPositions[m-1]->t[0] = ptr[0];
      TheCameraPositions[m-1]->t[2] = ptr[1];
      break;
    case 'z' :
      TheCameraPositions[m-1]->t[0] = ptr[0];
      TheCameraPositions[m-1]->t[1] = ptr[1];
      break;
    }
}

int CheckAnswerC ()
{
  int i, j;
  double sum1=0.0, sum2=0.0;

  /* compute residuals across the data set */
  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i) 
      if (TheEdges[j][i]) {
	sum1 += Error2 (TheEdges[j][i], TheCameraPositions[j], TheLines[i]);
	sum2 += TheEdges[j][i]->l_error;
      }

  if (sum1 > sum2) return (1); else return (0);
}

/** STAGE D routines ************************************/

int REPORT_D (int itncount, double *x,
	     double fc, double *gc, double *H,
	     int nfev, int njev, int nhev)
{
  int i;
  double max;

  for (i=1, max = fabs (gc[0]); i < (4*n + 6*(m-1) - 1); ++i)
    if (fabs (gc[i]) > max) max = fabs (gc[i]);

  printf ("%5d %12.5e %12.5e %5d %5d %5d\n", itncount, fc, max,
	  nfev, njev, nhev);

/* 
    if (!(itncount%10)) ANALYZE_HESS_D (H);
*/
}

int ANALYZE_HESS_D (double *H)
{
  int i, j, index_x, index_y;
  double temp;
  double avg1, max1, avg2, max2, avg3, max3, avg4, max4, avg5, max5;
  double avg6, max6, avg7, max7, avg8, max8, avg9, max9, avg10, max10;
  
  avg1 = avg2 = avg3 = avg4 = avg5 = avg6 = avg7 = avg8 = avg9 = avg10 = 0.0;
  max1 = max2 = max3 = max4 = max5 = max6 = max7 = max8 = max9 = max10 = 0.0;

  for (i=0; i < n; ++i) {

    for (index_x = THETA; index_x <= OMEGA; ++index_x)
      for (index_y = THETA; index_y <= index_x; ++index_y) {
	temp = fabs (H[elt(4*i + (index_x - THETA), 4*i + (index_y - THETA))]);
	avg1 += temp; 
	if (temp > max1) max1 = temp;
      }
    
    for (index_x = THETA; index_x <= OMEGA; ++index_x)
      for (index_y = A; index_y <= B; ++index_y) {
	temp = fabs (H[elt(4*i + (index_x - THETA), 4*i + (index_y - THETA))]);
	avg2 += temp; 
	if (temp > max2) max2 = temp;
      }

    for (index_x = A; index_x <= B; ++index_x)
      for (index_y = A; index_y <= index_x; ++index_y) {
	temp = fabs (H[elt(4*i + (index_x - THETA), 4*i + (index_y - THETA))]);
	avg3 += temp; 
	if (temp > max3) max3 = temp;
      }
  }

  for (j=1; j < m; ++j)
    for (i=0; i < n; ++i) {

      for (index_x = THETA; index_x <= OMEGA; ++index_x)
	for (index_y = ALPHA; index_y <= GAMMA; ++index_y) {
	  temp = 
	    fabs (H[elt(4*i+(index_x-THETA), 4*n+6*(j-1)+(index_y-ALPHA))]);
	  avg4 += temp; 
	  if (temp > max4) max4 = temp;
	}
      
      for (index_x = A; index_x <= B; ++index_x)
	for (index_y = ALPHA; index_y <= GAMMA; ++index_y) {
	  temp = 
	    fabs (H[elt(4*i+(index_x-THETA), 4*n+6*(j-1)+(index_y-ALPHA))]);
	  avg5 += temp; 
	  if (temp > max5) max5 = temp;
	}
      
      for (index_x = THETA; index_x <= OMEGA; ++index_x)
	for (index_y = TX; index_y <= TZ; ++index_y) {
	  temp = 
	    fabs (H[elt(4*i+(index_x-THETA), 4*n+6*(j-1)+(index_y-ALPHA))]);
	  avg6 += temp; 
	  if (temp > max6) max6 = temp;
	}
      
      for (index_x = A; index_x <= B; ++index_x)
	for (index_y = TX; index_y <= TZ; ++index_y) {
	  temp = 
	    fabs (H[elt(4*i+(index_x-THETA), 4*n+6*(j-1)+(index_y-ALPHA))]);
	  avg7 += temp; 
	  if (temp > max7) max7 = temp;
	}
    }

  
  for (j=1; j < m; ++j) {
    
    for (index_x = ALPHA; index_x <= GAMMA; ++index_x)
      for (index_y = ALPHA; index_y <= index_x; ++index_y) {
	temp = 
	fabs(H[elt(4*n+6*(j-1)+(index_x-ALPHA), 4*n+6*(j-1)+(index_y-ALPHA))]);
	avg8 += temp; 
	if (temp > max8) max8 = temp;
      }
    
    for (index_x = ALPHA; index_x <= GAMMA; ++index_x)
      for (index_y = TX; index_y <= TZ; ++index_y) {
	temp = 
	fabs(H[elt(4*n+6*(j-1)+(index_x-ALPHA), 4*n+6*(j-1)+(index_y-ALPHA))]);
	avg9 += temp; 
	if (temp > max9) max9 = temp;
      }
    
    for (index_x = TX; index_x <= TZ; ++index_x)
      for (index_y = TX; index_y <= index_x; ++index_y) {
	temp = 
        fabs(H[elt(4*n+6*(j-1)+(index_x-ALPHA), 4*n+6*(j-1)+(index_y-ALPHA))]);
	avg10 += temp; 
	if (temp > max10) max10 = temp;
      }
  }

  printf ("Case1 : x in (theta omega) y in (theta omega)\n");
  printf ("avg1 = %12.3e,  max1 = %12.3e\n", avg1/(3*n), max1);
   
  printf ("Case2 : x in (theta omega) y in (a b)\n");
  printf ("avg2 = %12.3e,  max2 = %12.3e\n", avg2/(4*n), max2);
      
  printf ("Case3 : x in (a b) y in (a b)\n");
  printf ("avg3 = %12.3e,  max3 = %12.3e\n", avg3/(3*n), max3);
    

  printf ("Case4 : x in (theta omega) y in (alpha beta gamma)\n");
  printf ("avg4 = %12.3e,  max4 = %12.3e\n", avg4/(6*(m-1)*n), max4);
   
  printf ("Case5 : x in (a b) y in (alpha beta gamma)\n");
  printf ("avg5 = %12.3e,  max5 = %12.3e\n", avg5/(6*(m-1)*n), max5);
   
  printf ("Case6 : x in (theta omega) y in (tx ty tz)\n");
  printf ("avg6 = %12.3e,  max6 = %12.3e\n", avg6/(6*(m-1)*n), max6);
   
  printf ("Case7 : x in (a b) y in (tx ty tz)\n");
  printf ("avg7 = %12.3e,  max7 = %12.3e\n", avg7/(6*(m-1)*n), max7);
   

  printf ("Case8 : x in (alpha beta gamma) y in (alpha beta gamma)\n");
  printf ("avg8 = %12.3e,  max8 = %12.3e\n", avg8/(6*(m-1)), max8);
  
  printf ("Case9 : x in (alpha beta gamma) y in (tx ty tz)\n");
  printf ("avg9 = %12.3e,  max9 = %12.3e\n", avg9/(6*(m-1)), max9);
   
  printf ("Case10 : x in (tx ty tz) y in (tx ty tz)\n");
  printf ("avg10 = %12.3e,  max10 = %12.3e\n", avg10/(6*(m-1)), max10);
}

int StageD ()
{
  int i, j, k;
  double *H, *L, g[4*MAX_N+6*(MAX_M-1)];
  double typx[4*MAX_N+6*(MAX_M-1)], x[6*MAX_N+7*(MAX_M-1)], f;
  int nfev, njev, nhev, info;

  H = make_sym_matrix (4*n + 6*(m-1));
  L = make_sym_matrix (4*n + 6*(m-1));

  /* initialize x */
  for (i=0; i < n; ++i) {
    x[6*i]   = TheLines[i]->q[0];
    x[6*i+1] = TheLines[i]->q[1];
    x[6*i+2] = TheLines[i]->q[2];
    x[6*i+3] = TheLines[i]->q[3];

    x[6*i+4] = TheLines[i]->a;
    x[6*i+5] = TheLines[i]->b;
  }
  
  for (j=1; j < m; ++j) {
    x[6*n+7*(j-1)]   = TheCameraPositions[j]->q[0];
    x[6*n+7*(j-1)+1] = TheCameraPositions[j]->q[1];
    x[6*n+7*(j-1)+2] = TheCameraPositions[j]->q[2];
    x[6*n+7*(j-1)+3] = TheCameraPositions[j]->q[3];

    x[6*n+7*(j-1)+4] = TheCameraPositions[j]->t[0];
    x[6*n+7*(j-1)+5] = TheCameraPositions[j]->t[1];
    x[6*n+7*(j-1)+6] = TheCameraPositions[j]->t[2];
  }
  
  switch (FrozenParameter) {
    case 'x' :
      x[6*n+7*(m-2)+4] = x[6*n+7*(m-2)+5];
      x[6*n+7*(m-2)+5] = x[6*n+7*(m-2)+6];
      break;

    case 'y' :
      x[6*n+7*(m-2)+5] = x[6*n+7*(m-2)+6];
      break;

    case 'z' :
      break;
  }

  /* intitialize typx */
  for (i=0; i < n; ++i) {
    typx[4*i]   = typx[4*i+1] = 0.01;  /* theta and omega */
    typx[4*i+2] = typx[4*i+3] = 100.0; /* a b */
  }

  for (j=1; j < m; ++j) {
    /* alpha, beta, gamma */
    typx[4*n+6*(j-1)]   = typx[4*n+6*(j-1)+1] = typx[4*n+6*(j-1)+2] = 0.01;
    /* tx, ty, tz */
    typx[4*n+6*(j-1)+3] = typx[4*n+6*(j-1)+4] = typx[4*n+6*(j-1)+5] = 100.0;
  }

#ifdef REPORT

  UMDRIVER ((4*n + 6*(m-1) - 1), (6*n + 7*(m-1) - 1),
            FN_D, GRAD_D, HESS_D, UPDATE_D,
	    REPORT_D,
            typx, 0.01,
            1.0e-8, 1.0e-8, 100.0, 100,
            x, &f, g,
            &nfev, &njev, &nhev, &info,
            L, H);
  
#else

  UMDRIVER ((4*n + 6*(m-1) - 1), (6*n + 7*(m-1) - 1),
            FN_D, GRAD_D, HESS_D, UPDATE_D,
	    (void*) 0x00,
            typx, 0.01,
            1.0e-8, 1.0e-8, 100.0, 100,
            x, &f, g,
            &nfev, &njev, &nhev, &info,
            L, H);

#endif

  FillStructs_D (x);

//  free (H);
//  free (L);

  return (info);
}

int FN_D (double *x, double *res)
{
  int i, j;

  FillStructs_D (x);
  
  *res = 0.0;
  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i)
      *res += Error2 (TheEdges[j][i], TheCameraPositions[j], TheLines[i]);
}

int GRAD_D (double *x, double *g)
{
  int i, j, index;

  FillStructs_D (x);

  for (i=0; i < (4*n + 6*(m-1)); ++i) g[i] = 0.0;

  for (j=0; j < m; ++j) 
    for (i=0; i < n; ++i) {
      for (index=THETA; index <= B; ++index)
	g[4*i + (index - THETA)] += Gradient2 (TheEdges[j][i],
					       TheCameraPositions[j],
					       TheLines[i],
					       index);
      
      if (j) {
	for (index=ALPHA; index <= TZ; ++index)
	  g[4*n + (j-1)*6 + (index-ALPHA)] += Gradient2 (TheEdges[j][i],
							 TheCameraPositions[j],
							 TheLines[i],
							 index);
      }
    }
  
  switch (FrozenParameter) {
    case 'x' :
      g[4*n+6*(m-2)+3] = g[4*n+6*(m-2)+4];
      g[4*n+6*(m-2)+4] = g[4*n+6*(m-2)+5];
      break;

    case 'y' :
      g[4*n+6*(m-2)+4] = g[4*n+6*(m-2)+5];
      break;

    case 'z' :
      break;
  }
}

int HESS_D (double *x, double *H)
{
  int i, j, index_x, index_y;

  FillStructs_D (x);

  for (i=0; i < (4*n + 6*(m-1)); ++i) 
    for (j=0; j <= i; ++j)
      H[elt(i, j)] = 0.0;
  
  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i) {
      
      for (index_x=THETA; index_x <= B; ++index_x)
	for (index_y=THETA; index_y <= index_x; ++index_y)
	  H[elt(4*i+(index_x-THETA), 4*i+(index_y-THETA))] +=
	    Hessian2 (TheEdges[j][i],
		      TheCameraPositions[j],
		      TheLines[i],
		      index_x, index_y);
      
      if (j) {
	for (index_x=THETA; index_x <= B; ++index_x)
	  for (index_y=ALPHA; index_y <= TZ; ++index_y)
	    H[elt(4*i+(index_x-THETA), 4*n+(j-1)*6+(index_y-ALPHA))] +=
	      Hessian2 (TheEdges[j][i],
			TheCameraPositions[j],
			TheLines[i],
			index_x, index_y);
	
	for (index_x=ALPHA; index_x <= TZ; ++index_x)
	  for (index_y=ALPHA; index_y <= index_x; ++index_y)
	    H[elt(4*n+(j-1)*6+(index_x-ALPHA), 4*n+(j-1)*6+(index_y-ALPHA))] +=
	      Hessian2 (TheEdges[j][i],
			TheCameraPositions[j],
			TheLines[i],
			index_x, index_y);
      }
    }
  
  switch (FrozenParameter) {
    case 'x' : eliminate_row (H, (4*n + 6*(m-2) + 3), (4*n + 6*(m-1)));  break;
    case 'y' : eliminate_row (H, (4*n + 6*(m-2) + 4), (4*n + 6*(m-1)));  break;
    case 'z' : break;
  }

}

int UPDATE_D (double *x, double *step, double *newx)
{
  double *ptr1, *ptr2, *ptr3;
  int i, j;
  
  ptr1=x;  ptr2=step;  ptr3=newx;
  for (i=0; i < n; ++i, ptr1+=6, ptr2+=4, ptr3+=6) {
    UpdateLine_q (ptr1, ptr2, ptr3);
    ptr3[4] = ptr1[4] + ptr2[2];
    ptr3[5] = ptr1[5] + ptr2[3];
  }

  for (j=1; j < m; ++j, ptr1+=7, ptr2+=6, ptr3+=7) {
    UpdatePosition_q (ptr1, ptr2, ptr3); 
    ptr3[4] = ptr1[4] + ptr2[3];
    ptr3[5] = ptr1[5] + ptr2[4];
    ptr3[6] = ptr1[6] + ptr2[5];
  }
}

int FillStructs_D (double *x)
{
  double *ptr;
  int i, j;

  /* fill in data structures */
  for (i=0, ptr = x; i < n; ++i, ptr += 6) {
    TheLines[i]->q[0] = ptr[0];
    TheLines[i]->q[1] = ptr[1];
    TheLines[i]->q[2] = ptr[2];
    TheLines[i]->q[3] = ptr[3];

    TheLines[i]->a = ptr[4];
    TheLines[i]->b = ptr[5];
  }

  for (j=1; j < (m-1); ++j, ptr += 7) {
    TheCameraPositions[j]->q[0] = ptr[0];
    TheCameraPositions[j]->q[1] = ptr[1];
    TheCameraPositions[j]->q[2] = ptr[2];
    TheCameraPositions[j]->q[3] = ptr[3];

    TheCameraPositions[j]->t[0] = ptr[4];
    TheCameraPositions[j]->t[1] = ptr[5];
    TheCameraPositions[j]->t[2] = ptr[6];
  }

  TheCameraPositions[m-1]->q[0] = ptr[0];
  TheCameraPositions[m-1]->q[1] = ptr[1];
  TheCameraPositions[m-1]->q[2] = ptr[2];
  TheCameraPositions[m-1]->q[3] = ptr[3];

  switch (FrozenParameter) {
    case 'x' :
      TheCameraPositions[m-1]->t[1] = ptr[4];
      TheCameraPositions[m-1]->t[2] = ptr[5];
      break;
    case 'y' :
      TheCameraPositions[m-1]->t[0] = ptr[4];
      TheCameraPositions[m-1]->t[2] = ptr[5];
      break;
    case 'z' :
      TheCameraPositions[m-1]->t[0] = ptr[4];
      TheCameraPositions[m-1]->t[1] = ptr[5];
      break;
  }
}
