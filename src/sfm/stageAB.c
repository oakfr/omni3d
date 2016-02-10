/*
 * stageAB.c : This file contains the routines that are used to implement
 * the first two stages of the SFM process. Notice that both stages work
 * by minimizing the same objective function, O1.
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

#define SCALE0  100.0

/* Specify a Levenberg Marquardt Hessian */
#define LM       1

/* Function Declarations ********************************/

int FN_A   (double *x, double *res);
int GRAD_A (double *x, double *g);
int HESS_A (double *x, double *H);

int FN_B   (double *x, double *res);
int GRAD_B (double *x, double *g);
int HESS_B (double *x, double *H);
int UPDATE_B (double *x, double *step, double *newx);

/** Shared routines *************************************/

double Error1 (edge *TheEdge, position *ThePosition, line *TheLine)
{
  double v[3], newv[3], temp;

  if (!TheEdge || !ThePosition || !TheLine) return (0.0);

  ComputeV (v, *TheLine);
  RotateQuaternion (newv, ThePosition->q, v);
  temp = InnerProduct (newv, TheEdge->m);

  return (sqr(temp) / (TheEdge->m_error));
}

double Deriv1 (edge *TheEdge, position *ThePosition, line *TheLine,
	       int index)
{
  double v[3], newv[3], dv[3], hat[3];

  if (!TheEdge || !ThePosition || !TheLine) return (0.0);

  switch (index) {
    case THETA :
      hat[0] = 0.0;  hat[1] = -1.0;  hat[2] = 0.0;
      RotateQuaternion (dv, TheLine->q, hat);
      RotateQuaternion (newv, ThePosition->q, dv);
      return (InnerProduct (newv, TheEdge->m));
      break;

    case OMEGA :
      hat[0] = 1.0;  hat[1] = 0.0;  hat[2] = 0.0;
      RotateQuaternion (dv, TheLine->q, hat);
      RotateQuaternion (newv, ThePosition->q, dv);
      return (InnerProduct (newv, TheEdge->m));
      break;

    case ALPHA :
      ComputeV (v, *TheLine);
      RotateQuaternion (newv, ThePosition->q, v);
      DerivR (dv, newv, 'x');
      return (InnerProduct (dv, TheEdge->m));
      break;

    case BETA :
      ComputeV (v, *TheLine);
      RotateQuaternion (newv, ThePosition->q, v);
      DerivR (dv, newv, 'y');
      return (InnerProduct (dv, TheEdge->m));
      break;

    case GAMMA :
      ComputeV (v, *TheLine);
      RotateQuaternion (newv, ThePosition->q, v);
      DerivR (dv, newv, 'z');
      return (InnerProduct (dv, TheEdge->m));
      break;

    default :
      pexit ("Bad parameters to Deriv1");
      break;
  }

  return 0.0;
}

int FillStructs_B (double *x)
{
  double *ptr;
  int i, j;

  /* fill in data structures */
  for (i=0, ptr = x; i < n; ++i, ptr += 4) {
    TheLines[i]->q[0] = ptr[0];
    TheLines[i]->q[1] = ptr[1];
    TheLines[i]->q[2] = ptr[2];
    TheLines[i]->q[3] = ptr[3];
  }

  for (j=1; j < m; ++j, ptr += 4) {
    TheCameraPositions[j]->q[0] = ptr[0];
    TheCameraPositions[j]->q[1] = ptr[1];
    TheCameraPositions[j]->q[2] = ptr[2];
    TheCameraPositions[j]->q[3] = ptr[3];
  }

  return 0;
}

double DoubleDeriv1 (edge *TheEdge, position *ThePosition, line *TheLine,
		     int index_x, int index_y)
{
  int index;
  double dv[3], v[3], newv[3], hat[3], temp1;

  if (!TheEdge || !ThePosition || !TheLine) return (0.0);

  if (index_y < index_x) {
    index = index_x; index_x = index_y; index_y = index;
  }

  ComputeV (v, *TheLine);
  RotateQuaternion (newv, ThePosition->q, v);
  temp1 = InnerProduct (newv, TheEdge->m);

  if ((index_x == THETA) && (index_y == THETA) ||
      (index_x == OMEGA) && (index_y == OMEGA))
    return (-2.0 * sqr(temp1));

  if ((index_x == THETA) && (index_y == OMEGA))
    return (0.0);

  if (((index_x == THETA) || (index_x == OMEGA)) &&
      ((index_y == ALPHA) || (index_y == BETA) || (index_y == GAMMA))) {

    if (index_x == THETA) {
      hat[0] = 0.0; hat[1] = -1.0; hat[2] = 0.0;
    } else {
      hat[0] = 1.0; hat[1] =  0.0; hat[2] = 0.0;
    }

    RotateQuaternion (v, TheLine->q, hat);
    RotateQuaternion (newv, ThePosition->q, v);

    DerivR (dv, newv, IndexToAxis (index_y));

    return (2.0 * temp1 * InnerProduct (dv, TheEdge->m));
  }

  if ( ((index_x == ALPHA) || (index_x == BETA) || (index_x == GAMMA)) &&
       ((index_y == ALPHA) || (index_y == BETA) || (index_y == GAMMA)) ) {
    
    ComputeV (v, *TheLine);
    RotateQuaternion (newv, ThePosition->q, v);

    DoubleDerivR (dv, newv, IndexToAxis (index_x), IndexToAxis (index_y));

    return (2.0 * temp1 * InnerProduct (dv, TheEdge->m));
  }

  pexit ("Bad Parameters to DoubleDeriv1");

  return 0.0;
}

double Gradient1 (edge *TheEdge, position *ThePosition, line *TheLine,
		  int index)
{
  double v[3], newv[3], temp1, temp2;

  if (!TheEdge || !ThePosition || !TheLine) return (0.0);

  ComputeV (v, *TheLine);
  RotateQuaternion (newv, ThePosition->q, v);
  temp1 = InnerProduct (newv, TheEdge->m);

  temp2 = Deriv1 (TheEdge, ThePosition, TheLine, index);

  return ( (2.0*temp1*temp2) / (TheEdge->m_error) );
}

double Hessian1 (edge *TheEdge, position *ThePosition, line *TheLine,
		int index_x, int index_y)
{
  double temp1, temp2;

  if (!TheEdge || !ThePosition || !TheLine) return (0.0);

  temp1 = Deriv1 (TheEdge, ThePosition, TheLine, index_x);
  temp2 = (index_x == index_y) ? temp1 :
    Deriv1 (TheEdge, ThePosition, TheLine, index_y);

  /* The Levenberg Marquardt approximation for the Hessian appears to yield
   * a better minimizations, possibly because the matrix is guaranteed to be 
   * positive semi-definite. 
   */

#ifdef LM
  return ( (2.0*temp1*temp2) / (TheEdge->m_error) );	  
#else
  temp3 = DoubleDeriv1 (TheEdge, ThePosition, TheLine, index_x, index_y);
  return ( (2.0*temp1*temp2 + temp3) / (TheEdge->m_error) );
#endif

}

/** STAGE A routines ************************************/


int FillStructs_A (double *x)
{
  TheLines[I]->q[0] = x[0];
  TheLines[I]->q[1] = x[1];
  TheLines[I]->q[2] = x[2];
  TheLines[I]->q[3] = x[3];

  return 0;
}

/* 
 * This function is used to  check whether we have ended up at one of the
 * two other stationary points.
 */

int CheckAnswerA (double *q)
{
  double newq1[4], newq2[4], stepq[4], min_f, temp1, temp2;

  FN_A (q, &min_f);

  stepq[0] = M_SQRT1_2; stepq[1] = 0.0; stepq[2] = M_SQRT1_2; stepq[3] = 0.0;
  QuaternionMultiply (newq1, q, stepq);

  FN_A (newq1, &temp1);

  stepq[0] = M_SQRT1_2; stepq[1] = M_SQRT1_2; stepq[2] = 0.0; stepq[3] = 0.0;
  QuaternionMultiply (newq2, q, stepq);

  FN_A (newq2, &temp2);

  if ((temp1 < min_f) && (temp1 <= temp2)) {
    q[0] = newq1[0];  q[1] = newq1[1];  q[2] = newq1[2];  q[3] = newq1[3];
  }

  if ((temp2 < min_f) && (temp2 <= temp1)) {
    q[0] = newq2[0];  q[1] = newq2[1];  q[2] = newq2[2];  q[3] = newq2[3];
  }

  return 0;
}

int StageA ()
{
  double *H, *L, g[2];
  double typx[2], x[4], f;
  int nfev, njev, nhev, info;

  H = make_sym_matrix (2);
  L = make_sym_matrix (2);

  for (I=0; I < n; ++I) {
    TheLines[I] = (line*) malloc (sizeof(line));
    if (!TheLines[I]) pexit ("Memory error in StageA");
    
    /* initialize x */
    x[0] = 1.0; x[1] = 0.0; x[2] = 0.0; x[3] = 0.0;

    NormalizeQuaternion (x);

    /* intitialize typx */
    typx[0] = typx[1] = 1.0;
    
    UMDRIVER (2, 4,
	      FN_A, GRAD_A, HESS_A, UpdateLine_q,
	      (void*) 0x00,
	      typx, 0.01,
	      1.0e-8, 1.0e-6, 5.0, 50,
	      x, &f, g,
	      &nfev, &njev, &nhev, &info,
	      L, H);

    CheckAnswerA (x);
    
    FillStructs_A (x);    
  }

  //free (H);
  //free (L);

  return 0;
}

int FN_A (double *x, double *res)
{
  int j;

  FillStructs_A (x);

  *res = SCALE0 * Error1 (TheEdges[0][I], TheCameraPositions[0], TheLines[I]);
  
  for (j=1; j < m; ++j)
    *res += Error1 (TheEdges[j][I], TheCameraPositions[j], TheLines[I]);

  return 0;
}

int GRAD_A (double *x, double *g)
{
  int j;

  FillStructs_A (x);

  g[0] = SCALE0 * Gradient1 (TheEdges[0][I], 
			     TheCameraPositions[0], 
			     TheLines[I],
			     THETA);
  g[1] = SCALE0 * Gradient1 (TheEdges[0][I], 
			     TheCameraPositions[0], 
			     TheLines[I],
			     OMEGA);

  for (j=1; j < m; ++j) {
    g[0] += Gradient1 (TheEdges[j][I], 
		       TheCameraPositions[j], 
		       TheLines[I],
		       THETA);
    g[1] += Gradient1 (TheEdges[j][I], 
		       TheCameraPositions[j], 
		       TheLines[I],
		       OMEGA);
  }

  return 0;
}

int HESS_A (double *x, double *H)
{
  int j;

  FillStructs_A (x);

  H[elt(0,0)] = SCALE0 * Hessian1 (TheEdges[0][I], 
				   TheCameraPositions[0], 
				   TheLines[I],
				   THETA, THETA);
  H[elt(0,1)] = SCALE0 * Hessian1 (TheEdges[0][I], 
				   TheCameraPositions[0], 
				   TheLines[I],
				   OMEGA, THETA);
  H[elt(1,1)] = SCALE0 * Hessian1 (TheEdges[0][I], 
				   TheCameraPositions[0], 
				   TheLines[I],
				   OMEGA, OMEGA);

  for (j=1; j < m; ++j) {
    H[elt(0,0)] += Hessian1 (TheEdges[j][I], 
			     TheCameraPositions[j], 
			     TheLines[I],
			     THETA, THETA);
    H[elt(0,1)] += Hessian1 (TheEdges[j][I], 
			     TheCameraPositions[j], 
			     TheLines[I],
			     OMEGA, THETA);
    H[elt(1,1)] += Hessian1 (TheEdges[j][I], 
			     TheCameraPositions[j], 
			     TheLines[I],
			     OMEGA, OMEGA);
  }

  return 0;
}

/** STAGE B routines ************************************/

int REPORT_B (int itncount, double *x,
	      double fc, double *gc, double *H,
	      int nfev, int njev, int nhev)
{
  int i;
  double max;

  for (i=1, max=fabs(gc[0]); i < (2*n + 3*(m-1)); ++i)
    if (fabs (gc[i]) > max) max = fabs (gc[i]);

  printf ("%5d %12.5e %5d %5d %5d %12.5e\n", itncount, fc, nfev, njev, nhev,
	  max);

  return 0;
}

int StageB ()
{
  int i, j, k;
  double *H, *L, g[2*MAX_N+3*MAX_M];
  double typx[2*MAX_N+3*(MAX_M-1)], x[4*MAX_N+4*(MAX_M-1)], f;
  int nfev, njev, nhev, info;

  H = make_sym_matrix (2*n + 3*(m-1));
  L = make_sym_matrix (2*n + 3*(m-1));

  /* initialize x */
  for (i=0; i < n; ++i)
    for (k=0; k < 4; ++k)
      x[4*i+k] = TheLines[i]->q[k];

   for (j=1; j < m; ++j)
    for (k=0; k < 4; ++k)
      x[4*n+4*(j-1)+k] = TheCameraPositions[j]->q[k];

  /* intitialize typx */
  for (i=0; i < (2*n + 3*(m-1)); ++i) typx[i] = 1.0;

#ifdef REPORT

  UMDRIVER ((2*n + 3*(m-1)), (4*n + 4*(m-1)),
            FN_B, GRAD_B, HESS_B, UPDATE_B,
            REPORT_B,
            typx, 0.01,
            1.0e-8, 1.0e-8, 5.0, 50,
            x, &f, g,
            &nfev, &njev, &nhev, &info,
            L, H);

#else

  UMDRIVER ((2*n + 3*(m-1)), (4*n + 4*(m-1)),
            FN_B, GRAD_B, HESS_B, UPDATE_B,
            (void*) 0x00,
            typx, 0.01,
            1.0e-8, 1.0e-8, 5.0, 50,
            x, &f, g,
            &nfev, &njev, &nhev, &info,
            L, H);

#endif  

  FillStructs_B (x);

 // free (H);
 // free (L);

  return info;
}

int UPDATE_B (double *x, double *step, double *newx)
{
  double *ptr1, *ptr2, *ptr3;
  int i, j;
  
  ptr1=x;  ptr2=step;  ptr3=newx;
  for (i=0; i < n; ++i, ptr1+=4, ptr2+=2, ptr3+=4)
    UpdateLine_q (ptr1, ptr2, ptr3);

  for (j=1; j < m; ++j, ptr1+=4, ptr2+=3, ptr3+=4)
    UpdatePosition_q (ptr1, ptr2, ptr3); 

  return 0;
}

int FN_B (double *x, double *res)
{
  int i, j;

  FillStructs_B (x);
  
  *res = 0.0;
  for (i=0; i < n; ++i)
    for (j=0; j < m; ++j)
      *res += Error1 (TheEdges[j][i], TheCameraPositions[j], TheLines[i]);

	return 0;
}

int GRAD_B (double *x, double *g)
{
  int i, j, index;

  FillStructs_B (x);

  for (i=0; i < (2*n + 3*(m-1)); ++i) g[i] = 0.0;

  for (j=0; j < m; ++j) 
    for (i=0; i < n; ++i) {
      for (index=THETA; index <= OMEGA; ++index)
	g[2*i + (index - THETA)] += Gradient1 (TheEdges[j][i],
					       TheCameraPositions[j],
					       TheLines[i],
					       index);
      
      if (j) {
	for (index=ALPHA; index <= GAMMA; ++index)
	  g[2*n + (j-1)*3 + (index-ALPHA)] += Gradient1 (TheEdges[j][i],
							 TheCameraPositions[j],
							 TheLines[i],
							 index);
      }
    }

	return 0;
}

int HESS_B (double *x, double *H)
{
  int i, j, index_x, index_y;

  FillStructs_B (x);

  for (i=0; i < (2*n + 3*(m-1)); ++i) 
    for (j=0; j <= i; ++j)
      H[elt(i, j)] = 0.0;
  
  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i) {
      
      for (index_x=THETA; index_x <= OMEGA; ++index_x)
	for (index_y=THETA; index_y <= index_x; ++index_y)
	  H[elt(2*i+(index_x-THETA), 2*i+(index_y-THETA))] +=
	    Hessian1 (TheEdges[j][i],
		      TheCameraPositions[j],
		      TheLines[i],
		      index_x, index_y);
      
      if (j) {
	for (index_x=THETA; index_x <= OMEGA; ++index_x)
	  for (index_y=ALPHA; index_y <= GAMMA; ++index_y)
	    H[elt(2*i+(index_x-THETA), 2*n+(j-1)*3+(index_y-ALPHA))] +=
	      Hessian1 (TheEdges[j][i],
			TheCameraPositions[j],
			TheLines[i],
			index_x, index_y);
	
	for (index_x=ALPHA; index_x <= GAMMA; ++index_x)
	  for (index_y=ALPHA; index_y <= index_x; ++index_y)
	    H[elt(2*n+(j-1)*3+(index_x-ALPHA), 2*n+(j-1)*3+(index_y-ALPHA))] +=
	      Hessian1 (TheEdges[j][i],
			TheCameraPositions[j],
			TheLines[i],
			index_x, index_y);
      }
    }

	return 0;
}

int CheckAnswerB ()
{
  int i, j;
  double sum1=0.0, sum2=0.0;

  /* compute residuals across the data set */
  for (j=0; j < m; ++j)
    for (i=0; i < n; ++i) 
      if (TheEdges[j][i]) {
	sum1 += Error1 (TheEdges[j][i], TheCameraPositions[j], TheLines[i]);
	sum2 += 1.0;
      }
  
	  if (sum1 > sum2) {
		  printf("Check answer B: NOK (error = %f > residual = %f)\n",sum1,sum2);
		  return 1; 
	  } else {
		  printf("Check answer B: OK (error = %f < residual = %f)\n",sum1,sum2);
		  return 0;
	  }
}
