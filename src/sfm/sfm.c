/*
 * sfm.c : This file runs the structure form motion algorithm on real
 * data. The program takes two arguments, the name of the log file
 * containing all the calibrated image measurements and the name of
 * the output file to write the 3D reconstruction results to. See
 * the code in LogFile.c to understand the format of this output file.
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "SFM.h"
#include "algorithm.h"
#include "utils.h"
#include "polygon.h"
#include "noise.h"
#include "quaternions.h"

#include "sym_mat.h"

#define DEGREES_TO_RADS (M_PI / 180.0)

#define MAX_RANSAC_TRIALS 1000

int main (int argc, char *argv[])
{
  int i,j;
  int _ransac_trials;

  double center_error = 10.0, image_error = 0.5, r_error = 20.0;
  //double center_error = 40.0, image_error = 12.0, r_error = 120.0;
  position ThePositions[MAX_M];

  /* Setting SFM parameters image_error and r_error: 
   * These parameters should reflect your confidence in the image 
   * measurements and your initial guess for the camera orientations. 
   */

/*
  image_error = 1.5;
  r_error = 5.0;
*/

  if (argc != 3) {
    printf ("%s takes two arguments\n", argv[0]);
    printf ("  arg1 : The name of the edge file\n");
    printf ("  arg2 : The name of the output file\n");
    //exit (0);
  } 

  //init_edges (argv[1], center_error, image_error);
  srand(time(NULL));

  _ransac_trials = 0;

  for (i=0;i<MAX_RANSAC_TRIALS;i++) {
	  init_edges ("blocks.log", center_error, image_error,_ransac_trials);

	  /*
	 init_camera_positions ();
	   */

	  /* Perform SFM minimization */
	 if (SFM (r_error * DEGREES_TO_RADS, M_SQRT2 * (center_error + image_error)) == 0)
		 break;

	 printf("RANSAC TRIAL %d\n",_ransac_trials);

	 _ransac_trials++;
  }

  //FreeTheEdges ();
  //FreeTheCameraPositions ();
  //FreeTheLines ();

  /* Dumping the results */
  ReconstructEndPoints (TheSegments);
  for (j=0; j < m; ++j) ThePositions[j] = *(TheCameraPositions[j]);

  if (WriteStructureFile ("blocks.3D", 1, n, m,
			  TheSegments, ThePositions, 0x0, 0x0))
    printf ("Couldn't open %s for output\n", "blocks.3D");
  
  print_segments ();
  print_camera_positions ();
}

#define MAX_TRIES 60

int SFM (double r_error, double pixel_error)
{
  int info, i, j, try=0;
  position pos_buffer[MAX_M];

  /* Copy camera positions into a buffer for safekeeping */  
  for (j=1; j < m; ++j) pos_buffer[j] = *(TheCameraPositions[j]);
  
  printf("***************************************\n");
  printf("***************************************\n");
  printf("*** CAMERA POSITIONS ***\n\n");
  print_camera_positions();
  printf("***************************************\n");
  printf("***************************************\n");
  
  ClearSolns ();

  while (1) {

    printf ("\n try number %d\n\n", try);
    if (++try > MAX_TRIES) {
		printf("*** SFM FAILED.\n");
		return 1; // FAILURE
	}

    /* generate initial camera positions */
    for (j=1; j < m; ++j) {
      *(TheCameraPositions[j]) = pos_buffer[j];
      AddCameraPositionNoise (TheCameraPositions[j], r_error, 0.0);
    }
    
    /****** STAGE A *********************************/
    
    printf ("\n Stage A \n\n");
   
    StageA ();
 
    /****** STAGE B *********************************/
    
    printf ("\n Stage B \n\n");
    
    info = StageB ();

    printf ("\n info = %d\n", info);

	/*printf("***************************************\n");
	printf("***************************************\n");
	printf("*** CAMERA POSITIONS ***\n\n");
	print_camera_positions();
	printf("***************************************\n");
	printf("***************************************\n");
	*/

    if (CheckAnswerB ()) continue;

    if (CheckSolns ()) continue;

    /****** STAGE C *********************************/
    
    printf ("\n Stage C \n\n");

    InitStageC ();
    
    info = StageC ();
    
    printf ("\n info = %d\n", info);
 
    if (CheckAnswerC ()) continue;
    if (CheckReconstruction (pixel_error)) continue; else break;
	break;

  }

  /****** STAGE D *********************************/

  printf ("\n Stage D \n\n");

  info = StageD ();

  printf ("\n info = %d\n", info);

  printf("*** SFM successful\n");

  return 0; //SUCCESS
}

position *init_position ()
{
  position *q;

  q = (position *) malloc (sizeof(position));
  if (!q) pexit ("Memory error in init_camera_positions");
  
  q->q[0] = 1.0; q->q[1] = q->q[2] = q->q[3] = 0.0;
  q->t[0] = q->t[1] = q->t[2] = 0.0;

  return (q);
}

int init_edges (char *filename, double center_error, double image_error, int counter)
{
  int i, j, index;
  int _i,_j,_p;
  double x1, y1, x2, y2;
  double alpha, beta, qa[4], qb[4];
  FILE *TheFile;
  char input[200];
  edge *TheEdge;

  /* Initialize the edges */
  for (j=0; j < MAX_M; ++j)
    for (i=0; i < MAX_N; ++i)
      TheEdges[j][i] = (edge*) NULL;

  TheFile = fopen (filename, "r");
  if (!TheFile) {
    printf ("Couldn't open %s\n", filename);
    exit (0);
  }

  fgets (input, 200, TheFile);
  if (input[0] != '#' || input[1] != '^') {
    printf ("Bad edge file \n");
    exit (0);
  }
  
  TheCameraPositions[0] = init_position ();

  m = n = 0;
  while (fgets (input, 200, TheFile)) {
    if (sscanf (input, "%d %le %le %le %le", &index, &x1, &y1, &x2, &y2)==5) {

	  if (index+1 > n)
		  n = index+1;

      if (n >= MAX_N) pexit ("Too many lines in data set\n");

      TheEdge = (edge*) malloc (sizeof(edge));
      if (!TheEdge) pexit ("Memory Error in init_edges");
      
      TheEdge->x1 = x1;  TheEdge->y1 = y1;
      TheEdge->x2 = x2;  TheEdge->y2 = y2;

      /* calculate edge length */
      TheEdge->length = sqrt (sqr(TheEdge->x2 - TheEdge->x1) + 
			      sqr(TheEdge->y2 - TheEdge->y1));
      
      /* m = (x1 y1 -1) ^ (x2 y2 -1) */
      TheEdge->m[0] = TheEdge->y2 - TheEdge->y1;
      TheEdge->m[1] = TheEdge->x1 - TheEdge->x2;
      TheEdge->m[2] = (TheEdge->x1)*(TheEdge->y2)-(TheEdge->x2)*(TheEdge->y1);
      
      Normalize (TheEdge->m);

      ComputeMError (TheEdge, center_error, image_error);

      if (TheEdges[m][index]) {
	MergeEdges (TheEdges[m][index], TheEdge);	
	ComputeMError (TheEdges[m][index], center_error, image_error);
	free (TheEdge);
      } else
	TheEdges[m][index] = TheEdge;
    }
	
    if (sscanf (input, "#+ alpha %lf beta %lf", &alpha, &beta)==2) {
		
		MakeQuaternion (qa, (DEGREES_TO_RADS * alpha), 'x');
		MakeQuaternion (qb, (DEGREES_TO_RADS * beta),  'y');
		QuaternionMultiply (TheCameraPositions[m]->q, qb, qa);
    }
	
    if (input[0] == '#' && input[1] == '!')
		if (++m >= MAX_M) pexit ("Too many camera positions");
		
		if (input[0] == '#' && input[1] == '^') {
			TheCameraPositions[m] = init_position ();
			
			// randomly generate initial guesses for camera orientations
			_p = (int)(sqrt(MAX_RANSAC_TRIALS));

			_i = (int)(counter / _p);
			_j = counter % _p;



			alpha = (((double)_i/_p) -0.5) * 200.0;
			beta = (((double)_j/_p) -0.5) * 200.0;
			
			//printf("alpha: %f   beta: %f\n",alpha,beta);
			
			if (m>0) {
				MakeQuaternion (qa, (DEGREES_TO_RADS * alpha), 'x');
				MakeQuaternion (qb, (DEGREES_TO_RADS * beta),  'y');
				QuaternionMultiply (TheCameraPositions[m]->q, qb, qa);
			}
		}
  }
  
  /* Select a frozen parameter */
  FrozenParameter = 'y';
  TheCameraPositions[m-1]->t[1] = 200.0;

  fclose (TheFile);
}

int MergeEdges (edge *e1, edge *e2)
{
  double x, y, sumx, sumx2, sumy, sumy2, sumxy, p, q, r, s, lambda;
  double mx, my, mz, temp1, temp2;

  x = (e1->x1);  y = (e1->y1);
  sumx = x;  sumx2 = x*x;  sumy = y;  sumy2 = y*y;  sumxy = x*y;

  x = (e1->x2);  y = (e1->y2);
  sumx += x;  sumx2 += x*x;  sumy += y;  sumy2 += y*y;  sumxy += x*y;

  x = (e2->x1);  y = (e2->y1);
  sumx += x;  sumx2 += x*x;  sumy += y;  sumy2 += y*y;  sumxy += x*y;

  x = (e2->x2);  y = (e2->y2);
  sumx += x;  sumx2 += x*x;  sumy += y;  sumy2 += y*y;  sumxy += x*y;
  
  p = sumx2 - (sumx*sumx)/4;
  q = sumxy - (sumx*sumy)/4;
  r = sumy2 - (sumy*sumy)/4;
  
  lambda = ( (p+r) - sqrt( (p-r)*(p-r) + 4*q*q ) ) / 2;
  
  mx = (lambda - r - q);    my = (p + q - lambda);
  
  s = sqrt(mx*mx + my*my);
  if (s != 0.0) { mx /= s; my /= s; }
  if (my < 0) { mx = -mx; my = -my; }
  mz = (mx*sumx + my*sumy) / 4;
  
  /* compute endpoints */
  s = mx*(e1->y1) - my*(e1->x1);
  temp1 = temp2 = s;

  s = mx*(e1->y2) - my*(e1->x2);
  if (s > temp1) temp1 = s;
  if (s < temp2) temp2 = s;

  s = mx*(e2->y1) - my*(e2->x1);
  if (s > temp1) temp1 = s;
  if (s < temp2) temp2 = s;

  s = mx*(e2->y2) - my*(e2->x2);
  if (s > temp1) temp1 = s;
  if (s < temp2) temp2 = s;

  /* Fill in endpoints */
  e1->x1 = mx*mz - temp1*my;
  e1->y1 = my*mz + temp1*mx;

  e1->x2 = mx*mz - temp2*my;
  e1->y2 = my*mz + temp2*mx;

  /* calculate edge length */
  e1->length = sqrt (sqr(e1->x2 - e1->x1) + sqr(e1->y2 - e1->y1));
  
  /* m = (x1 y1 -1) ^ (x2 y2 -1) */
  e1->m[0] = e1->y2 - e1->y1;
  e1->m[1] = e1->x1 - e1->x2;
  e1->m[2] = (e1->x1)*(e1->y2)-(e1->x2)*(e1->y1);
  
  Normalize (e1->m);

  return 0;
}

#if 0
int init_camera_positions ()
{
  int j;
  position *q;

  for (j=0; j < m; ++j) {
    q = (position *) malloc (sizeof(position));
    if (!q) pexit ("Memory error in init_camera_positions");
    
    q->q[0] = 1.0; q->q[1] = q->q[2] = q->q[3] = 0.0;
    q->t[0] = q->t[1] = q->t[2] = 0.0;

    TheCameraPositions[j] = q;
  }

  /* Select a frozen parameter */
  FrozenParameter = 'y';
  TheCameraPositions[m-1]->t[1] = 200.0;

  return 0;
}
#endif

int print_segments ()
{
  int i;

  for (i=0; i < n; ++i) {
    //printf ("\nsegment #%d\n", i);
    //printf ("x1: %12.5f  y1: %12.5f  z1: %12.5f\n", 
	printf ("%12.5f   %12.5f   %12.5f\n", 
	    TheSegments[i].pt1[0], TheSegments[i].pt1[1],
	    TheSegments[i].pt1[2]);
    //printf ("x2: %12.5f  y2: %12.5f  z2: %12.5f\n", 
	printf (" %12.5f   %12.5f   %12.5f\n", 
	    TheSegments[i].pt2[0], TheSegments[i].pt2[1],
	    TheSegments[i].pt2[2]);
  }

  return 0;
}

int print_camera_positions ()
{
  int j;

  for (j=0; j < m; ++j) {
    printf ("\nposition # %d\n", j);
    printf ("q[0] = %8.5e, q[1] = %8.5e, q[2] = %8.5e, q[3] = %8.5e\n",
	    TheCameraPositions[j]->q[0], TheCameraPositions[j]->q[1],
	    TheCameraPositions[j]->q[2], TheCameraPositions[j]->q[3]);
    printf ("t[0] = %8.5e, t[1] = %8.5e, t[2] = %8.5e\n",
	    TheCameraPositions[j]->t[0], TheCameraPositions[j]->t[1],
	    TheCameraPositions[j]->t[2]);
  }

  return 0;
}
