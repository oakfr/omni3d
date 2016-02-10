#ifndef IMPROVED_MATH_HPP
#define IMPROVED_MATH_HPP

// improved_math.hpp defines basic mathematical objects like vectors and matrices.
// Entries are doubles throughout.
#define  M_PI   3.14159265358979323846f 

class vec2d;
class vec3d;
class vec4d;
class row3d;
//class row4d;
class mat33;
class mat34;
class mat44;

extern vec3d linePlaneIntersect(vec3d x1, vec3d x2, vec3d x3, vec3d y1, vec3d y2);
extern vec3d linePlaneIntersect(vec3d x1, vec3d x2, vec3d x3, vec3d y1, vec3d y2, double *tp);
extern vec3d linePlaneIntersect(vec4d plane, vec3d picks[]);
extern vec4d planeByPointNormal(vec3d point, vec3d normal);
extern vec3d dropPerpendicular(vec3d x, vec3d y1, vec3d y2);
extern mat44 translationMatrix(vec3d x);
extern mat44 rotationMatrixByAxisAngle(vec3d axis, double angle);
extern vec4d quatFromAA(vec3d axis, double angle);
extern mat33 quatToRotationMatrix(vec4d q);
extern vec4d rotationMatrixToQuat(mat33 m);

// vector * scalar
extern vec2d operator* (vec2d v, double a);
extern vec3d operator* (vec3d v, double a);
extern vec4d operator* (vec4d v, double a);

// scalar * vector
extern vec2d operator* (double a, vec2d v);
extern vec3d operator* (double a, vec3d v);
extern vec4d operator* (double a, vec4d v);

// matrix * scalar
extern mat33 operator* (mat33 m, double s);
extern mat44 operator* (mat44 m, double s);

// matrix * vector
extern vec3d operator* (mat33 m, vec3d v);
extern vec3d operator* (mat34 m, vec4d v);
extern vec4d operator* (mat44 m, vec4d v);

// vector * row
extern mat33 operator* (vec3d v, row3d r);

// matrix * matrix
extern mat33 operator* (mat33 m, mat33 n);
extern mat44 operator* (mat44 m, mat44 n);

class mat33 {
public:
  double m[3][3];

  mat33() {}
  mat33(double mc[]) {
    for (int x = 0; x < 3; x++)
      for (int y = 0; y < 3; y++)
	m[y][x] = mc[y * 3 + x];
  }
  mat33(double m11, double m12, double m13,
	double m21, double m22, double m23,
	double m31, double m32, double m33) {
    m[0][0] = m11;
    m[0][1] = m12;
    m[0][2] = m13;
    m[1][0] = m21;
    m[1][1] = m22;
    m[1][2] = m23;
    m[2][0] = m31;
    m[2][1] = m32;
    m[2][2] = m33;
  }
  double det();
  mat33 operator+(mat33 m);
  mat33 operator-(mat33 m);
  mat33 inverse();
  mat33 transpose();
  vec3d column(int n);
  void setcolumn(int n, vec3d c);
  void print();
};

class mat34 {
public:
  double m[3][4];

  mat34() {}
  mat34(double m11, double m12, double m13, double m14,
	double m21, double m22, double m23, double m24,
	double m31, double m32, double m33, double m34) {
    m[0][0] = m11;
    m[0][1] = m12;
    m[0][2] = m13;
    m[0][3] = m14;
    m[1][0] = m21;
    m[1][1] = m22;
    m[1][2] = m23;
    m[1][3] = m24;
    m[2][0] = m31;
    m[2][1] = m32;
    m[2][2] = m33;
    m[2][3] = m34;
  }
};

class mat44 {
public:
  double m[4][4];

  mat44() {}
  mat44(double m11, double m12, double m13, double m14,
	double m21, double m22, double m23, double m24,
	double m31, double m32, double m33, double m34,
	double m41, double m42, double m43, double m44) {
    m[0][0] = m11;
    m[0][1] = m12;
    m[0][2] = m13;
    m[0][3] = m14;
    m[1][0] = m21;
    m[1][1] = m22;
    m[1][2] = m23;
    m[1][3] = m24;
    m[2][0] = m31;
    m[2][1] = m32;
    m[2][2] = m33;
    m[2][3] = m34;
    m[3][0] = m41;
    m[3][1] = m42;
    m[3][2] = m43;
    m[3][3] = m44;
  }

  double det();
  mat44 inverse();
  vec4d column(int n);
  void setcolumn(int n, vec4d c);
  void print();
};

class vec2d {
public:
  double v[2];

  double norm();
  double sqnorm();
  double dot(vec2d x);
  vec2d() {}
  vec3d tohc();
  vec2d(double x, double y) { v[0] = x; v[1] = y; }
  vec2d normalize();
  vec2d operator+(vec2d v);
  vec2d operator+=(vec2d v);
  vec2d operator-(vec2d w);
  void print();
};

class vec3d {
public:
  double v[3];

  double norm();
  double sqnorm();
  double dot(vec3d x);
  vec3d cross(vec3d x);
  row3d transpose();
  vec3d() {}
  vec3d(double x, double y, double z) { v[0] = x; v[1] = y; v[2] = z; }
  vec2d unhc();
  vec4d tohc();
  vec3d normalize();
  vec3d operator+(vec3d v);
  vec3d operator-(vec3d w);
  void print();
};

class vec4d {
public:
  double v[4];

  double norm();
  double sqnorm();
  vec4d() {}
  vec4d(double x, double y, double z, double w) { v[0] = x; v[1] = y; v[2] = z; v[3] = w; }
  vec4d normalize();
  vec4d operator+(vec4d v);
  vec4d operator-(vec4d v);
  vec4d quatMult(vec4d v);
  vec4d quatDiv(vec4d v);
  vec4d quatConj();
  vec3d unhc();
  void print();
};

class row3d {
public:
  double r[3];

  row3d(double x, double y, double z);
};

#endif
