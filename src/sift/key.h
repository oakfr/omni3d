/************************************************************************
  Copyright (c) 2004. David G. Lowe, University of British Columbia.
  This software is being made available for research purposes only
  (see file LICENSE for conditions).  This notice must be retained on
  all copies and modified versions of this software.
*************************************************************************/
#ifndef _SIFT_H
#define _SIFT_H

/* From the standard C libaray: */
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>


/*------------------------- Macros and constants  -------------------------*/

/* Following defines TRUE and FALSE if not previously defined. */
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define MAX_SIFT_FEATURES 4000

/* Value of PI, rounded up, so orientations are always in range [0,PI]. */
#define PI_SIFT 3.1415927

#define ABS(x)    (((x) > 0) ? (x) : (-(x)))
#define MAX_SIFT(x,y)  (((x) > (y)) ? (x) : (y))
#define MIN_SIFT(x,y)  (((x) < (y)) ? (x) : (y))

/* Given the name of a structure, NEW allocates space for it in the
   given pool (see util.c) and returns a pointer to the structure.
*/
#define NEW(s,pool) ((struct s *) MallocPool(sizeof(struct s),pool))

/* Assign a unique number to each pool of storage needed for this application. 
*/
#define PERM_POOL  0     /* Permanent storage that is never released. */
#define IMAGE_POOL 1     /* Data used only for the current image. */
#define KEY_POOL   2     /* Data for set of keypoints. */

/* These constants specify the size of the index vectors that provide
   a descriptor for each keypoint.  The region around the keypoint is
   sampled at OriSize orientations with IndexSize by IndexSize bins.
   VecLength is the length of the resulting index vector.
*/
#define OriSize 8
#define IndexSize 4
#define VecLength (IndexSize * IndexSize * OriSize)

/*---------------------------- Structures -------------------------------*/

/* Data structure for a double image.
*/
typedef struct ImageSt {
    int rows, cols;          /* Dimensions of image. */
    double **pixels;          /* 2D array of image pixels. */
} *SiftImage;


/* This structure describes a keypoint that has been found in an image.
*/
typedef struct KeypointSt {
    double row, col;      /* Row, column location relative to input image.  */
    double scale;         /* Scale (in sigma units for smaller DOG filter). */
    double ori;           /* Orientation in radians (-PI to PI). */
    unsigned char *ivec; /* Vector of gradient samples for indexing.*/
    struct KeypointSt *next;   /* Links all keypoints for an image. */
} *Keypoint;


/*------------------------------- Externals -----------------------------*/

extern const int MagFactor;
extern const double GaussTruncate;


/*-------------------------- Function prototypes -------------------------*/
/* The are prototypes for the external functions that are shared
   between files.
*/

/* Only interface needed to key.c. */
Keypoint GetKeypoints(SiftImage image);

/* Following are from util.c */
void *MallocPool(int size, int pool);
void FreeStoragePool(int pool);
double **AllocMatrix(int rows, int cols, int pool);
SiftImage CreateImage(int rows, int cols, int pool);
SiftImage CopyImage(SiftImage image, int pool);
SiftImage DoubleSize(SiftImage image);
SiftImage HalfImageSize(SiftImage image);
void SubtractImage(SiftImage im1, SiftImage im2);
void GradOriImages(SiftImage im, SiftImage grad, SiftImage ori);
void GaussianBlur(SiftImage image, double sigma);
void SolveLeastSquares(double *solution, int rows, int cols, double **jacobian,
		       double *errvec, double **sqarray);
void SolveLinearSystem(double *solution, double **sq, int size);
double DotProd(double *v1, double *v2, int len);

int compute_sift_features( SiftImage im, double *features );

#endif
