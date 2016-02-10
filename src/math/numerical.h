#ifndef _NUMERICAL_H__
#define _NUMERICAL_H__

#include "basic.h"
#include "linear/linear.h"

#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1+(IM-1)/NTAB)
#define RNMX (1.0-EPS)

class TrifocalTensor
{
public:

	Vecd _t; // vector of size 27  [t1_11 t1_12 ... t1_33    t2_11 ... t2_33   t3_11 ... t3_33]

	Vec3d _e1, _e2; // epipoles

	Vecd _a; // vector containing the camera matrix entries [aij ... bik]

	Mat3d _F21, _F31; // fundamental matrices

	Matd _P1, _P2, _P3; // camera matrices 

	//TrifocalTensor (Vecd t) {assert(t.Elts() == 27); _t = t;}
	TrifocalTensor() { _t = Vecd(27,0.0);}
	//TrifocalTensor (Matd, Matd, Matd);

	Mat3d Ti(int n); // n in [0,1,2]

	void getFundamentalMatrices ();
	void getCameraMatrices ();
	Mat3d mult (Vec3d e, bool transposed);
	void getCameraCenterPoint (Vec3d &C1, Vec3d &C2, Vec3d &C3);
	void getCameraRotationMatrix (Mat3d &R1, Mat3d &R2, Mat3d &R3);
	Vecd TrifocalTensor::tensorFromCameraMatrices (Matd P1, Matd P2);

};

class Quaternion {
public:
	double _s;
	Vec3d _v;
	
	Quaternion() { _s=1; _v=Vec3d(0,0,0);};
	Quaternion(double s, Vec3d v) {_s = s; _v = v;};
	Quaternion (Vec3d axis, double angle);
	Quaternion inv() {return Quaternion(_s,-_v);}
	Quaternion plusPi() { return Quaternion( -_s, -_v); }
	Quaternion Quaternion::operator*(const Quaternion &q);
	Quaternion operator+ (const Quaternion &q) {return Quaternion( _s + q._s, _v + q._v );}
	Quaternion bar();
	Vec3d rotate (Vec3d v);
	Mat3d toRotationMatrix ();
	void Quaternion::toAxisAngle (Vec3d &axis);
	void print() {Vec3d v; toAxisAngle( v ); LOG(LEVEL_INFO, "%f %f %f\n",v[0],v[1],v[2]);}
	Quaternion normalize();
};

void normAngle( double &angle );

int solve8thDegree (std::vector<double> &coeffs, std::vector<double> &solutions, double a, double b, int int_res);
double solve8thDegree (std::vector<double> &coeffs, double a, double b, double resolution);
double eval8thDegree (std::vector<double> coeffs, double a);

bool solve3rdDegree (double p, double q, double &x1, double &x2, double &x3);
double eval3rdDegree (double p, double q, double x);
Vec3d intersectRayPlane (Vec3d P, Vec3d u, Vec3d Q, Vec3d n, double &lambda);

void middleLine ( Vec3d lp1, Vec3d ldir1, Vec3d lp2, Vec3d ldir2, Vec3d &lp3, Vec3d &ldir3 );
Vec3d projectPointLine( Vec3d p, Vec3d lp, Vec3d ldir );
double OverlapPlaneLine( Vec3d a, Vec3d b, Vec3d p, Vec3d q );

bool minRotation2Planes2Lines( Vec3d e1a, Vec3d e1b, Vec3d e2a, Vec3d e2b, Vec3d l1a, Vec3d l1b, Vec3d l2a, Vec3d l2b, double max_error, double &overlap, Vec3d &rotation );
bool minRotationFourPoints( const Vec3d e11, const Vec3d e12, const Vec3d e21, const Vec3d e22, Vec3d &rotation);

bool intersectPlanePlane (Vec3d P, Vec3d u, Vec3d Q, Vec3d v, Vec3d &point, Vec3d &dir);
double Distance (Vec3d point, Vec3d line_point, Vec3d line_dir);
int intersectPlaneCube (Vec3d P, Vec3d n, Vec3d C, double w, std::vector<Vec3d> &vertices);
double Acos (double a);
double Asin (double a);
int roundi (double a);
bool xor (bool a, bool b);
Vec3d fromSphericToEuler (Vec3d origin,SphericalCoord spheric);
double toDegrees (double a);
double toRadians (double a);
static double inchTomm (double length) {return length * 25.4;}
static double mmToInch (double length) {return length / 25.4;}

void closest_segment_approach (Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4, Vec3d &pc1, Vec3d &pc2 );


Vec2d ladybugflip( Vec2d point, int h, double zoom );

// from numerical recipes in C, chapters 6.1, 6.2
double gammln( double xx );
void gcf(double *gammcf, double a, double x, double *gln);
void gser(double *gamser, double a, double x, double *gln);
double gammq(double a, double x);

// random number generators
double ran1(long *idum);
double gasdev(long *idum);
void testRandomGenerators();

Quaternion random_rotation( long &idum );

#endif
