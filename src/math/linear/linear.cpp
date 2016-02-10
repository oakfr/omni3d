#include "linear.h"
#include "Factor.h"

CvMat* createCvMatrix (Matd A)
{
	int nrows = A.Rows();
	int ncols = A.Cols();

	CvMat *Acv = cvCreateMat (nrows, ncols, CV_32FC1);

	for (int i=0;i<nrows;i++) {
		for (int j=0;j<ncols;j++) {
			cvSetReal2D (Acv,i,j,A[i][j]);
		}
	}

	return Acv;
}

Matd createVLMatrix (CvMat *C)
{
	int nrows = C->rows;
	int ncols = C->cols;

	Matd A (nrows,ncols);

	for (int i=0;i<nrows;i++) {
		for (int j=0;j<ncols;j++) {
			A[i][j] = cvGetReal2D(C,i,j);
		}
	}

	return A;
}

void SVDFactorization2 (Matd A, Matd &U, Matd &V, Matd &D)
{

	int nrows = A.Rows();
	int ncols = A.Cols();

	CvMat *Acv = createCvMatrix(A);

	CvMat *Wcv = cvCreateMat (ncols, ncols, CV_32FC1);

	CvMat *Ucv = cvCreateMat (ncols, nrows, CV_32FC1);

	CvMat *Vcv = cvCreateMat (ncols, ncols, CV_32FC1);

	cvSVD (Acv, Wcv, Ucv, Vcv, CV_SVD_MODIFY_A | CV_SVD_U_T | CV_SVD_V_T);

	U = trans (createVLMatrix(Ucv));

	V = trans (createVLMatrix(Vcv));

	D = createVLMatrix(Wcv);
}


/* Zissermann Algorithm A3.5 pp. 564
 *
 * Given a matrix A that has as many rows as columns, find x that minimizes
 * |Ax| subject to |x|=1
 *
 */

Vecd linearSolver (Matd A)
{

	assert (A.Rows() >= A.Cols());

	Matd U,V;
	Matd D;

	// find the SVD decomposition of A
	SVDFactorization2(A,U,V,D);	

	// x is the last column of V
	return col(V,V.Cols()-1);
}


/* Zissermann Algorithm A3.7 pp. 566
 *
 * Given a matrix A that has as many rows as columns, find x that minimizes
 * |Ax| subject to |x|=1 and x = Gz where G has rank r
 *
 * This function returns z.
 */

Vecd linearSolver (Matd A, Matd G, int r)
{
	// compute the SVD of G
	Matd U,V;
	Matd D;

	SVDFactorization2(G,U,V,D);
	
	// take the first r columns or U
	Matd Up = sub (U,0,0,U.Rows(),r);

	// find the unit vector xp that minimizes |AUpx| using algorithm A3.5
	Vecd xp = linearSolver(A*Up);

	// take the first r columns of V
	Matd Vp = sub (V,0,0,V.Rows(),r);

	// take the r x r block of D
	Matd Dp = sub(D,0,0,r,r);

	// check
	//printf("D matrix:\n");
	//printMatrix(Dp);
	
	// return z
	return Vp * inv(Dp) * xp;
}

// VL provides QR factorization but not RQ factorization!
// The following algorithm works only if A is the 3x3 left hand side of a camera matrix
// see http://wwwnavab.in.tum.de/twiki/pub/Chair/TeachingWs04CV/solution07WS0405.pdf for details
//
void RQFactorization(Mat3d &A, Mat3d &Q, Mat3d &R)
{
	Matd Q1,R1;

	Mat3d I = flip();

	QRFactorization(sub3x3Matrix(trans(A)*I),Q1,R1);

	Q = I * trans(sub3x3Matrix(Q1));

	R = I * trans(sub3x3Matrix(R1)) * I;

	printf("RQ decomposition:\n");

	printf("A = \n");
	printMatrix(A);
	printf("R = \n");
	printMatrix(R1);
	printf("Q = \n");
	printMatrix(Q1);

	printf("difference A-RQ:\n");
	printMatrix(A - R*Q);
}

// gives a flip matrix
Mat3d flip ()
{
	Mat3d I;
	int n=3;

	for (int i=0;i<n;i++) {
		for (int j=0;j<n;j++) {
			if (i == n-1-j)
				I[i][j] = 1.0;
			else
				I[i][j] = 0.0;
		}
	}

	return I;
}

Mat3d crossProductMatrix (Vec3d a)
{
	Mat3d A;

	A[0][0] = A[1][1] = A[2][2] = 0.0;

	A[0][1] = -a[2];
	A[0][2] = a[1];
	A[1][0] = a[2];
	A[1][2] = -a[0];
	A[2][0] = -a[1];
	A[2][1] = a[0];

	return A;
}

Mat3d simpleProductMatrix (Vec3d a, Vec3d b)
{
	Mat3d A;

	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			A[i][j] = a[i] * b[j];
		}
	}

	return A;
}

Matd simpleProductMatrix (Vecd a, Vecd b)
{
	Matd A;

	assert(a.Elts() == b.Elts());

	int n = a.Elts();

	for (int i=0;i<n;i++) {
		for (int j=0;j<n;j++) {
			A[i][j] = a[i] * b[j];
		}
	}

	return A;
}

Mat3d getRotationFromCameraMatrix (Matd P)
{
	Mat3d K,R;

	Mat3d M = sub3x3Matrix(P);

	printf("decomposing:\n");
	printMatrix(M);

	RQFactorization(M,R,K);

	// K is the intrinsic camera matrix

	// R is the rotation matrix
	return R;
}

Mat3d sub3x3Matrix (Matd P)
{
	return Mat3d (P[0][0],P[0][1],P[0][2],P[1][0],P[1][1],P[1][2],P[2][0],P[2][1],P[2][2]);
}

Matd sub3x3Matrix (Mat3d P)
{
	return Matd (3,3,P[0][0],P[0][1],P[0][2],P[1][0],P[1][1],P[1][2],P[2][0],P[2][1],P[2][2]);
}

double Det (Vecd a, Vecd b, Vecd c)
{
	Mat3d A;

	col(A,0) = a;
	col(A,1) = b;
	col(A,2) = c;

	return det(A);
}

Vec3d getCenterPointFromCameraMatrix (Matd P)
{
	double x = Det(col(P,1),col(P,2),col(P,3));
	double y = -Det(col(P,0),col(P,2),col(P,3));
	double z = Det(col(P,0),col(P,1),col(P,3));
	double t = -Det(col(P,0),col(P,1),col(P,2));

	x /= t;  
	y /= t;
	z /= t;

	return Vec3d (x,y,z);
}

void resetMatrix (Matd &A)
{
	int n = A.Rows();
	int p = A.Cols();

	for (int i=0;i<n;i++)
		for (int j=0;j<p;j++)
			A[i][j] = 0.0;
}

Matd convertToMatd (Mat3d A)
{
	Matd B (3,3);

	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			B[i][j] = A[i][j];
		}
	}

	return B;
}

void printMatrix (FILE *f,Matd A)
{
	for (int i=0;i<A.Rows();i++) {
		for (int j=0;j<A.Cols();j++) {
		fprintf(f,"%.4f ",A[i][j]);
		}
		fprintf(f,"\n");
	}
	fprintf(f,"\n");
}

void printMatrix (Matd A)
{
	for (int i=0;i<A.Rows();i++) {
		for (int j=0;j<A.Cols();j++) {
		printf("%.4f ",A[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

void printMatrix (Vecd A)
{
	for (int i=0;i<A.Elts();i++) {
		printf("%.4f ",A[i]);
	}
	printf("\n");
}

Matd diagonalMatrix (Vecd V)
{
	int n = V.Elts();

	Matd A(n,n);
	resetMatrix(A);

	for (int i=0;i<n;i++)
		A[i][i] = V[i];

	return A;
}

// find normalization matrix based on Hartley & Zisserman pp.92-93:
// - translate points so that centroid is at the origin
// - scale points so that the average distance from (0,0) is sqrt(2)
/*Mat3d findNormalizationMatrix (edge2DVector edges)
{

	// find translation
	Vec2d t0(0.0,0.0);

	edge2DVector::iterator iter;
	for (iter=edges.begin();iter!=edges.end();iter++) {
		t0 -= iter->_a + iter->_b;
	}

	t0 /= 2*edges.size();

	// translate the points
	for (iter=edges.begin();iter!=edges.end();iter++) {
		iter->_a += t0;
		iter->_b += t0;
	}

	// find scaling
	double d = 0.0;

	for (iter=edges.begin();iter!=edges.end();iter++) {
		d += len(iter->_a) + len(iter->_b);
	}

	d /= 2*edges.size();

	// check result for centroid
	Vec2d centroid(0.0,0.0);
	for (iter=edges.begin();iter!=edges.end();iter++) {
		centroid += iter->_a + iter->_b;
	}
	printf("centroid: %.4f %.4f\n",centroid[0],centroid[1]);

	// apply scaling
	for (iter=edges.begin();iter!=edges.end();iter++) {
		iter->_a *= sqrt(2.0)/d;
		iter->_b *= sqrt(2.0)/d;
	}

	// check result for average distance
	d = 0.0;
	for (iter=edges.begin();iter!=edges.end();iter++) {
		d += len(iter->_a) + len(iter->_b);
	}

	d /= 2*edges.size();
	
	printf("average distance: %.4f\n",d);

	// generate matrix
	double alpha = sqrt(2.0)/d;

	Mat3d A(1.0,0.0,t0[0],0.0,1.0,t0[1],0.0,0.0,1.0);

	return alpha * A;
}*/

// normalize data point as described in Hartley & Zisserman Alg 15.1 pp.383
void normalizeDataPoint (Vec3d &point, Mat3d H)
{
	point = H*point;
}

// normalize data point as described in Hartley & Zisserman Alg 15.1 pp.383
void denormalizeDataPoint (Vec3d &point, Mat3d H)
{
	point = inv(H)*point;
}

// normalize data line as described in Hartley & Zisserman Alg 15.1 pp.383
void normalizeDataLine (Vec3d &line, Mat3d H)
{
	line = trans(inv(H))*line;
}

// denormalize tensor vector as described in Hartley & Zisserman Alg 15.1 pp.383
void denormalizeVector (Vecd &vector, Mat3d H1, Mat3d H2, Mat3d H3)
{
	Vecd new_vector = vector;

	int i,j,k,r,s,t;

	Matd inv_H2 = inv(H2), inv_H3 = inv(H3);

	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			for (k=0;k<3;k++) {

				double d = 0.0;

				for (r=0;r<3;r++) {
					for (s=0;s<3;s++) {
						for (t=0;t<3;t++) {

							d += vector[9*r+3*s+t] * H1[r][i] * inv_H2[j][s] * inv_H3[k][t];
						}
					}
				}

				new_vector[9*i+3*j+k] = d;

			}
		}
	}

	vector = new_vector;
}
