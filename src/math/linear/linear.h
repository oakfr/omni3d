#include "basic.h"
#include "VLd.h"

CvMat* createCvMatrix (Matd A);
Matd createVLMatrix (CvMat *C);

Vecd linearSolver (Matd A);
Vecd linearSolver (Matd A, Matd G, int r);
void SVDFactorization2 (Matd A, Matd &U, Matd &V, Matd &D);

Mat3d crossProductMatrix (Vec3d a);
Mat3d simpleProductMatrix (Vec3d a, Vec3d b);
Matd simpleProductMatrix (Vecd a, Vecd b);
double Det (Vecd a, Vecd b, Vecd c);
void RQFactorization(Mat3d &A, Mat3d &Q, Mat3d &R);
Mat3d getRotationFromCameraMatrix (Matd P);
Vec3d getCenterPointFromCameraMatrix (Matd P);
void resetMatrix (Matd &A);
Mat3d flip ();
Mat3d sub3x3Matrix (Matd P);
Matd sub3x3Matrix (Mat3d P);

void printMatrix (FILE *f,Matd A);
void printMatrix (Matd A);
void printMatrix (Vecd A);
Matd convertToMatd (Mat3d A);
Matd diagonalMatrix (Vecd V);

//Mat3d findNormalizationMatrix (edge2DVector edges);
void normalizeDataPoint (Vec3d &point, Mat3d H);
void normalizeDataLine (Vec3d &line, Mat3d H);
void denormalizeVector (Vecd &t, Mat3d H1, Mat3d H2, Mat3d H3);
void denormalizeDataPoint (Vec3d &point, Mat3d H);

