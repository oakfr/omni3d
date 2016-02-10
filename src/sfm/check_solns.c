/*
 * check_solns.c : This file implements a simple database that is used
 * to keep track of the various solutions that have been pursued.
 */

#include <stdio.h>
#include <stdlib.h>
#include "SFM.h"
#include "utils.h"
#include "quaternions.h"

#define MAX_SOLNS    50
#define Q_DISTANCE   1.0e-2
//#define Q_DISTANCE   3.0

int nsolns = 0;
double *Solns[MAX_SOLNS];

int ClearSolns ()
{
  int i;

//  for (i=0; i < nsolns; ++i)
//    free (Solns[i]);

  nsolns = 0;
}

int CheckSolns ()
{
  int i, j;

  for (i=0; i < nsolns; ++i)
	  if (CompareSolns (Solns[i], TheCameraPositions)) {
		  printf("Check Solns failed [%d] : \n",i);
		  PrintQuaternion(Solns[i]);
	      return (1);
	  }

  /* record the solution */
  if (nsolns < MAX_SOLNS) {
    if (!(Solns[nsolns] = malloc ((m-1)*4*sizeof(double))))
      pexit ("Memory error in CheckSolns");
    for (j=1; j < m; ++j)
      CopyQuaternion (TheCameraPositions[j]->q, Solns[nsolns] + 4*(j-1));
    ++nsolns;
  }
  
  return (0);
}

int CopyQuaternion (double *q1, double *q2)
{
  q2[0] = q1[0];  q2[1] = q1[1];  q2[2] = q1[2];  q2[3] = q1[3];
}

int CompareSolns (double *Q, position *ThePositions)
{
  int j;

  for (j=1; j < m; ++j) {
	  if (QuaternionDistance (TheCameraPositions[j]->q, (Q + 4*(j-1)))
		  > Q_DISTANCE) {
		  printf("\tcompare solns failed: distance between camera quaternion\n\t");
		  PrintQuaternion(TheCameraPositions[j]->q);
		  printf("\n\tand estimate\n\t");
		  PrintQuaternion(Q + 4*(j-1));
		  printf("\n\t = %f is larger than threshold %f\n",QuaternionDistance (TheCameraPositions[j]->q, (Q + 4*(j-1))),Q_DISTANCE);
		  return (0);
	  }
	  
	  return (1);
  }
}
