/************************************************************************
  Copyright (c) 2004. David G. Lowe, University of British Columbia.
  This software is being made available for research purposes only
  (see file LICENSE for conditions).  This notice must be retained on
  all copies and modified versions of this software.
*************************************************************************/

/* main.c:
   This file contains the main program to read an image, find keypoints,
   and output them to a file.
*/

#include "key.h"



/* -------------------- Local function prototypes ------------------------ */
SiftImage ReadPGM(FILE *fp);
void SkipComments(FILE *fp);
void WritePGM(FILE *fp, SiftImage image);
void DrawKeypoints(SiftImage im, Keypoint keys);
void TransformLine(SiftImage im, Keypoint k, double x1, double y1, double x2,
		   double y2);
void DrawLine(SiftImage image, int r1, int c1, int r2, int c2);
void WriteKeypoints(FILE *fp, Keypoint keys);


/*----------------------------- Routines ----------------------------------*/


/* Top level routine.  Read a PGM file, detect the keypoints, and
   either output to file or draw them on the image, and write out a
   PGM file.
*/
/*int main (int argc, char **argv)
{
    int display = FALSE, arg = 0;
    SiftImage im;
    Keypoint keys;

    // Parse command line arguments.
    while (++arg < argc) {
      if (! strcmp(argv[arg], "-display")) 
	display = TRUE;
      else {
	fprintf(stderr, "Invalid command line argument: %s\n", argv[arg]);
	exit(1);
      }
    }

    fprintf(stderr, "Finding keypoints...\n");
    im = ReadPGM(stdin);
    keys = GetKeypoints(im);   // Call to detect keypoints.

    if (display) {
      DrawKeypoints(im, keys);
      WritePGM(stdout, im);
      fprintf(stderr, "PGM file output.\n");
    } else
      WriteKeypoints(stdout, keys);

    return 0;
}*/

/* main sift entry point */
int compute_sift_features( SiftImage im, double *features )
{

	//SiftImage im;
	Keypoint keys;
	Keypoint k;
	int count = 0;

	int n = 0;

	//FILE *fg = fopen("toto.pgm", "w");
	//FILE *fp = fopen(pgm_file,"r");
	//im = ReadPGM(fp); // read the image
	//WritePGM( fg, im);
	//fclose(fp);
	//fclose(fg);

	keys = GetKeypoints(im);   // Call to detect keypoints.

    for (k = keys; k != NULL; k = k->next)
		n++;
	
	for (k = keys; k != NULL; k = k->next ) {
		
		features[4*count+0] = k->row;
		features[4*count+1] = k->col;
		features[4*count+2] = k->scale;
		features[4*count+3] = k->ori;
		count++;
		
		if ( count >= MAX_SIFT_FEATURES )
			break;
	}
	
	printf("%d sift features detected.\n", n);

	return n;
}

/* Read a PGM file from the given file pointer and return it as a
   floating point SiftImage structure.  See "man pgm" for details on PGM
   file format.  This handles only the usual 8-bit "raw" PGM format.
   Use xv or the PNM tools (such as pnmdepth) to convert from other
   formats.
*/
SiftImage ReadPGM(FILE *fp)
{
  int char1, char2, width, height, max, c1, c2, c3, r, c;
  SiftImage image;

  char1 = fgetc(fp);
  char2 = fgetc(fp);
  SkipComments(fp);
  c1 = fscanf(fp, "%d", &width);
  SkipComments(fp);
  c2 = fscanf(fp, "%d", &height);
  SkipComments(fp);
  c3 = fscanf(fp, "%d", &max);

  if (char1 != 'P' || (char2 != '5' && char2 != '2') || c1 != 1 || c2 != 1 || c3 != 1 ||
      max > 255) {
    fprintf(stderr, "ERROR: Input is not a standard raw PGM file.\n"
	    "Use xv or PNM tools to convert file to 8-bit PGM format.\n");
  }
  fgetc(fp);  /* Discard exactly one byte after header. */

  /* Create floating point image with pixels in range [0.0,1.0]. */
  image = CreateImage(height, width, PERM_POOL);
  for (r = 0; r < height; r++) {
	  for (c = 0; c < width; c++) {
		  double val = ((double) fgetc(fp)) / 255.0;
		image->pixels[r][c] = val;
		//printf("%f ", val);
	  }
	  //printf("\n");
	}


  return image;
}


/* Skip white space including any comments. PGM files allow a comment
   starting with '#' to end-of-line.
*/
void SkipComments(FILE *fp)
{
    int ch;

    fscanf(fp," ");      /* Skip white space. */
    while ((ch = fgetc(fp)) == '#') {
      while ((ch = fgetc(fp)) != '\n'  &&  ch != EOF)
	;
      fscanf(fp," ");
    }
    ungetc(ch, fp);      /* Replace last character read. */
}


/* Write an SiftImage in PGM format to the file fp.
*/
void WritePGM(FILE *fp, SiftImage image)
{
    int r, c, val;

    fprintf(fp, "P5\n%d %d\n255\n", image->cols, image->rows);

    for (r = 0; r < image->rows; r++) {
      for (c = 0; c < image->cols; c++) {
	val = (int) (255.0 * image->pixels[r][c]);
	fputc(MAX_SIFT(0, MIN_SIFT(255, val)), fp);
     		printf("%f ", val);
	  }
	  printf("\n");
	}
}


/* Draw an arrow for each keypoint onto the image.
*/
void DrawKeypoints(SiftImage im, Keypoint keys)
{
    int count = 0;
    Keypoint k;

    for (k = keys; k != NULL; k = k->next) {
      
      /* Draw 3 lines creating a horizontal arrow of unit length in
	 (row,col) coordinates.  Each line will be then be transformed
	 according to keypoint parameters.
      */
      TransformLine(im, k, 0.0, 0.0, 0.0, 1.0);  /* Main shaft of arrow. */
      TransformLine(im, k, 0.1, 0.85, 0.0, 1.0); 
      TransformLine(im, k, -0.1, 0.85, 0.0, 1.0);
      count++;
    }
    fprintf(stderr, "%d keypoints found.\n", count);
}


/* Draw the given line in the image, but first translate, rotate, and
   scale according to the keypoint parameters.
*/
void TransformLine(SiftImage im, Keypoint k, double row1, double col1, double row2,
		   double col2)
{
    int r1, c1, r2, c2;
    double s, c, len;

    /* The scaling of the unit length arrow is set to half the width
       of the region used to compute the keypoint descriptor.
    */
    len = 0.5 * IndexSize * MagFactor * k->scale;

    /* Rotate the points by k->ori. */
    s = sin(k->ori);
    c = cos(k->ori);

    /* Apply transform. */
    r1 = (int) (k->row + len * (c * row1 - s * col1) + 0.5);
    c1 = (int) (k->col + len * (s * row1 + c * col1) + 0.5);
    r2 = (int) (k->row + len * (c * row2 - s * col2) + 0.5);
    c2 = (int) (k->col + len * (s * row2 + c * col2) + 0.5);

    /* Discard lines that have any portion outside of image. */
    if (r1 >= 0 && r1 < im->rows && c1 >= 0 && c1 < im->cols && 
	r2 >= 0 && r2 < im->rows && c2 >= 0 && c2 < im->cols)
      DrawLine(im, r1, c1, r2, c2);
}

/* Draw a white line from (r1,c1) to (r2,c2) on the image.  Both points
   must lie within the image.
*/
void DrawLine(SiftImage image, int r1, int c1, int r2, int c2)
{
    int i, dr, dc, temp;

    if (r1 == r2 && c1 == c2)  /* Line of zero length. */
      return;

    /* Is line more horizontal than vertical? */
    if (ABS(r2 - r1) < ABS(c2 - c1)) {

      /* Put points in increasing order by column. */
      if (c1 > c2) {
	temp = r1; r1 = r2; r2 = temp;
	temp = c1; c1 = c2; c2 = temp;
      }
      dr = r2 - r1;
      dc = c2 - c1;
      for (i = c1; i <= c2; i++)
	image->pixels[r1 + (i - c1) * dr / dc][i] = 1.0;

    } else {

      if (r1 > r2) {
	temp = r1; r1 = r2; r2 = temp;
	temp = c1; c1 = c2; c2 = temp;
      }
      dr = r2 - r1;
      dc = c2 - c1;
      for (i = r1; i <= r2; i++)
	image->pixels[i][c1 + (i - r1) * dc / dr] = 1.0;
    }
}


/* Write set of keypoints to a file in ASCII format.  
   The file format starts with 2 integers giving the total number of
     keypoints, and size of descriptor vector for each keypoint. Then
     each keypoint is specified by 4 floating point numbers giving
     subpixel row and column location, scale, and orientation (in 
     radians from -PI to PI).  Then the descriptor vector for each
     keypoint is given as a list of integers in range [0,255].
*/
void WriteKeypoints(FILE *fp, Keypoint keys)
{
    int i, count = 0;
    Keypoint k;

    for (k = keys; k != NULL; k = k->next)
      count++;

    /* Output total number of keypoints and VecLength. */
    fprintf(fp, "%d %d\n", count, VecLength);
    
    /* Output data for each keypoint. */
    for (k = keys; k != NULL; k = k->next) {
      fprintf(fp, "%4.2f %4.2f %4.2f %4.3f", k->row, k->col, k->scale,
	      k->ori);
      for (i = 0; i < VecLength; i++) {
	if (i % 20 == 0)
	  fprintf(fp, "\n");
	fprintf(fp, " %d", (int) k->ivec[i]);
      }
      fprintf(fp, "\n");
    }

    /* Write message to terminal. */
    fprintf(stderr, "%d keypoints found.\n", count);
}
