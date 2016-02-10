/*
 * noise.h : Header file for noise.c.
 */
int AddCameraPositionNoise (position *ThePos, double r_error, double t_error);
int AddImageNoise (edge *TheEdge, double center_x, double center_y,
		   double pixel_error);
int ComputeMError (edge *TheEdge, double center_error, double image_error);
double ImageError (double x1, double y1, double x2, double y2);
