/*
 * expt7.c : This program is used to compare the performance of the LH88
 * and Taylor Kriegman algorithms under additive image noise.
 */

#include <stdio.h>
#include <math.h>
#include "SFM.h"
#include "algorithm.h"
#include "utils.h"
#include "polygon.h"
#include "noise.h"
#include "quaternions.h"

#include "sym_mat.h"

#define M_PI 3.14159265359f
#define DEGREES_TO_RADS (M_PI / 180.0)

int main ()
{
  double center_x, center_y, q[4];
  double r_error, temp[3];
  int i, j, idum = 1;
  position ThePositions[MAX_M];
  int itn, tries;
  double center_error, image_error;
  double TK_Rerror, TK_Terror, TK_LineError, TK_Verror;
  double LH_Rerror, LH_Terror, LH_LineError, LH_Verror;
  
  /* Generate the segments */
  RotatedCubes (3);

/*
  cube36 ();
*/

/*
  n = 15;
  GenerateRandomLines (n);
*/

/*
  MakeSquares ();
*/
  
  /* Apply a random rotation to the segments */
  RotateSegments ();

  printf ("\n## n = %d\n", n);

  for (image_error=0.5; image_error <= 3.0; image_error += 0.5) {
    for (itn=0; itn < 50; ++itn) {

      /* Generate the camera positions */
      TrinocularStereo ();

      /* Generate the images */
      GenerateEdges ();
      
      /* Add errors to the image measurements */
      
      /* Generate a random camera center error */
/*
      center_x = (2.0 * ran0 (&idum) - 1.0) * 5.0;
      center_y = (2.0 * ran0 (&idum) - 1.0) * 5.0;
*/     

      center_x = center_y = 0.0;
      
      for (j=0; j < m; ++j)
	for (i=0; i < n; ++i) {
	  AddImageNoise (TheEdges[j][i], center_x, center_y, image_error);
	  ComputeMError (TheEdges[j][i], 3.0, image_error+0.5);
	}

      /* Taylor Kriegman algorithm */
      r_error = 4.0;
      
      /* Add errors to the camera positions */
      for (j=1; j < m; ++j) {
	*(TheCameraPositions[j]) = TrueCameraPositions[j];
	AddCameraPositionNoise (TheCameraPositions[j],
				r_error * DEGREES_TO_RADS, 100.0);
      }
      
      /* Select a frozen parameter */
      FrozenParameter = 'y';
      (TheCameraPositions[m-1])->t[1] = TrueCameraPositions[m-1].t[1];
 
      tries = SFM (r_error * DEGREES_TO_RADS, image_error+7.0);

      TK_Rerror = AverageRerror ();
      TK_Terror = AverageTerror ();
      TK_LineError = AverageLineError ();
      TK_Verror = AverageVerror ();

      FreeTheLines ();

      /* Liu Huang Algorithm */
      LH88 ();

      LH_Rerror = AverageRerror ();
      LH_Terror = AverageTerror ();
      LH_LineError = AverageLineError ();
      LH_Verror = AverageVerror ();

      printf ("TK %4d %8.3f %12.3e %12.3e %12.3e %12.3e %d\n",
	       itn, image_error, 
	       TK_Rerror, TK_Terror, TK_LineError, TK_Verror, tries);
      printf ("LH %4d %8.3f %12.3e %12.3e %12.3e %12.3e\n", itn, image_error, 
	       LH_Rerror, LH_Terror, LH_LineError, LH_Verror);
      fflush (stdout);

      FreeTheEdges ();
      FreeTheCameraPositions ();
      FreeTheLines ();
    }
      
    printf ("\n");  /* spacing line */
  }
}


#define MAX_TRIES 25

int SFM (double r_error, double pixel_error)
{
  int j, try=0;
  position pos_buffer[MAX_M];

  /* Copy camera positions into a buffer for safekeeping */  
  for (j=1; j < m; ++j) pos_buffer[j] = *(TheCameraPositions[j]);
  
  while (1) {

    ++try;

    if (try > MAX_TRIES) break;

    /* generate initial camera positions */
    for (j=1; j < m; ++j) {
      *(TheCameraPositions[j]) = pos_buffer[j];
      AddCameraPositionNoise (TheCameraPositions[j], r_error, 0.0);
    }
    
    /****** STAGE A *********************************/
   
    StageA ();
 
    /****** STAGE B *********************************/
    
    StageB ();

    if (CheckAnswerB ()) continue;
 
    /****** STAGE C *********************************/
    
    StageC ();

    /****** STAGE D *********************************/

    StageD ();
  
    if (CheckAnswerC ()) continue;
    if (CheckReconstruction (pixel_error)) continue; else break;
  }

  return (try);
}
