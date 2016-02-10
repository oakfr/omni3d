/*
 * noise.c : This file contains the routines that are used to add errors
 * to the synthetic data.
 */

#include <math.h>
#include "SFM.h"
#include "quaternions.h"
#include "utils.h"

/* Defines *****************************************/

/* FOCAL_LENGTH : Focal length o f the camera in mm. */
#define FOCAL_LENGTH     1.0

/* N_PIXELS : The width/height of the image in pixels */
#define N_PIXELS         512

/* FIELD_OF_VIEW : the complete field of view of the camera in deg. */
#define FIELD_OF_VIEW     30

/* MAX_POINTS : Maximum number of points in any edge */
#define MAX_POINTS          800

#define ran1  (2.0*ran0 (&idum) - 1.0)

static int first = 1;
static double pixel_size;
 
int AddCameraPositionNoise (position *ThePos, double r_error, double t_error)
{
  int idum = 1;
  double stepq[4], newq[4];

  /* select a random rotation */

  RandomQuaternion (stepq, r_error);
  QuaternionMultiply (newq, stepq, ThePos->q);

  ThePos->q[0] = newq[0];
  ThePos->q[1] = newq[1];
  ThePos->q[2] = newq[2];
  ThePos->q[3] = newq[3];

  ThePos->t[0] += t_error * ran1;
  ThePos->t[1] += t_error * ran1;
  ThePos->t[2] += t_error * ran1;

  return 0;
}

/*
 * AddImageNoise : This function adds noise to the image edges by adding random
 * errors to the endpoints and a user specified camera center bias. All of the
 * error parameters to this function are specified in pixels.
 */

int AddImageNoise (edge *TheEdge, double center_x, double center_y,
		   double pixel_error)
{
  int idum = 1;

  if (first) {
    /* calculate the pixel size */
    pixel_size = (2 * FOCAL_LENGTH * tan(FIELD_OF_VIEW * M_PI / 360.0));
    pixel_size /= N_PIXELS;
    first = 0;
  }

  TheEdge->x1 += (center_x + pixel_error * ran1) * pixel_size;
  TheEdge->y1 += (center_y + pixel_error * ran1) * pixel_size;

  TheEdge->x2 += (center_x + pixel_error * ran1) * pixel_size;
  TheEdge->y2 += (center_y + pixel_error * ran1) * pixel_size;

  /* calculate edge length */
  TheEdge->length = sqrt (sqr(TheEdge->x2 - TheEdge->x1) + 
			  sqr(TheEdge->y2 - TheEdge->y1));

  /* m = (x1 y1 -1) ^ (x2 y2 -1) */
  TheEdge->m[0] = TheEdge->y2 - TheEdge->y1;
  TheEdge->m[1] = TheEdge->x1 - TheEdge->x2;
  TheEdge->m[2] = (TheEdge->x1)*(TheEdge->y2) - (TheEdge->x2)*(TheEdge->y1);

  TheEdge->l_error = TheEdge->m_error = 100.0;

  Normalize (TheEdge->m);

  return 0;
}

int ComputeMError (edge *TheEdge, double center_error, double image_error)
{
  double m[3], e[3];

  /* N.B. image_error and center_error are given in pixels */

  if (first) {
    /* calculate the pixel size */
    pixel_size = (2 * FOCAL_LENGTH * tan(FIELD_OF_VIEW * M_PI / 360.0));
    pixel_size /= N_PIXELS;
    first = 0;
  }

  m[0] = TheEdge->y2 - TheEdge->y1;
  m[1] = TheEdge->x1 - TheEdge->x2;
  m[2] = (TheEdge->x1)*(TheEdge->y2) - (TheEdge->x2)*(TheEdge->y1);

  e[0] = e[1] = 2 * image_error;
  e[2] = fabs (TheEdge->x1 - TheEdge->x2) * center_error +
         fabs (TheEdge->y1 - TheEdge->y2) * center_error + 
         (fabs (TheEdge->x1) + fabs (TheEdge->y1)) * image_error +
         (fabs (TheEdge->x2) + fabs (TheEdge->y2)) * image_error;

  TheEdge->m_error = sqr((VectorNorm (e) * pixel_size) / VectorNorm (m));
  TheEdge->l_error = sqr((image_error + center_error) * pixel_size) *
                     TheEdge->length;

  return 0;
}

double ImageError (double x1, double y1, double x2, double y2)
{
  double dx, dy;

  if (first) {
    /* calculate the pixel size */
    pixel_size = (2 * FOCAL_LENGTH * tan(FIELD_OF_VIEW * M_PI / 360.0));
    pixel_size /= N_PIXELS;
    first = 0;
  }

  dx = x1 - x2;
  dy = y1 - y2;

  return (sqrt(dx*dx + dy*dy) / pixel_size);
}
