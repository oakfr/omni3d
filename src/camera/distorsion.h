#include "basic.h"
#include "geom/geom3D.h"
#include "camera/camera.h"

void distortion (unsigned char *pixels, unsigned char *pixels_new, int width, int height,Intrinsic intr);
void undistortion (unsigned char *pixels, unsigned char *pixels_new, int width, int height,Intrinsic intr);
Edge2D rectifyEdge (Edge2D edge, int width, int height,Intrinsic intr);

