#include <cassert>
#include <cmath>
#include <iostream>

#include "impmath.hpp"

//using namespace std;

void mat33::print() {
  int i, j;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      std::cout << m[i][j] << "  ";
    }
    std::cout << std::endl;
  }
}

void mat44::print() {
  int i, j;
  for (i = 0; i < 4; i++) {
    for (j = 0; j < 4; j++) {
      std::cout << m[i][j] << "  ";
    }
    std::cout << std::endl;
  }
}

void vec3d::print() {
  int i;
  std::cout << "(";
  for (i = 0; i < 3; i++) {
    std::cout << v[i];
    if (i < 2) std::cout << ", ";
  }
  std::cout << ")" << std::endl;
}

void vec2d::print() {
  int i;
  std::cout << "(";
  for (i = 0; i < 2; i++) {
    std::cout << v[i];
    if (i < 1) std::cout << ", ";
  }
  std::cout << ")" << std::endl;
}

void vec4d::print() {
  int i;
  std::cout << "(";
  for (i = 0; i < 4; i++) {
    std::cout << v[i];
    if (i < 3) std::cout << ", ";
  }
  std::cout << ")" << std::endl;
}

double vec2d::norm() {
  return sqrt(v[0] * v[0] + v[1] * v[1]);
}

double vec3d::norm() {
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double vec4d::norm() {
  return sqrt(v[0] * v[0] + v[1] * v[1]
	      + v[2] * v[2] + v[3] * v[3]);
}

double vec2d::sqnorm() {
  return v[0] * v[0] + v[1] * v[1];
}

double vec3d::sqnorm() {
  return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

double vec4d::sqnorm() {
  return v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3];
}

vec2d vec2d::operator- (vec2d w) {
  return vec2d(v[0] - w.v[0], v[1] - w.v[1]);
}

vec3d vec3d::operator- (vec3d w) {
  return vec3d(v[0] - w.v[0], v[1] - w.v[1], v[2] - w.v[2]);
}

vec4d vec4d::operator- (vec4d w) {
  return vec4d(v[0] - w.v[0], v[1] - w.v[1], v[2] - w.v[2], v[3] - w.v[3]);
}

vec4d vec4d::operator+ (vec4d w) {
  return vec4d(v[0] + w.v[0], v[1] + w.v[1], v[2] + w.v[2], v[3] + w.v[3]);
}

vec3d vec3d::operator+ (vec3d w) {
  return vec3d(v[0] + w.v[0], v[1] + w.v[1], v[2] + w.v[2]);
}

vec2d vec2d::operator+ (vec2d w) {
  return vec2d(v[0] + w.v[0], v[1] + w.v[1]);
}

vec2d vec2d::operator+= (vec2d w) {
  v[0] += w.v[0];
  v[1] += w.v[1];
  return *this;
}

vec2d vec3d::unhc() {
  assert(v[2] != 0);
  return vec2d(v[0] / v[2],
	       v[1] / v[2]);
}

vec3d vec4d::unhc() {
  assert(v[3] != 0);
  return vec3d(v[0] / v[3],
	       v[1] / v[3],
	       v[2] / v[3]);
}

vec4d vec3d::tohc() {
  return vec4d(v[0], v[1], v[2], 1);
}

vec3d vec2d::tohc() {
  return vec3d(v[0], v[1], 1);
}

vec4d operator* (mat44 m, vec4d v) {
  int k, n;
  vec4d ret;

  for (k = 0; k < 4; k++) {
    double s = 0;
    for (n = 0; n < 4; n++) {
      s += m.m[k][n] * v.v[n];
    }
    ret.v[k] = s;
  }
  return ret;
}

vec3d operator* (mat34 m, vec4d v) {
  int k, n;
  vec3d ret;

  for (k = 0; k < 3; k++) {
    double s = 0;
    for (n = 0; n < 4; n++) {
      s += m.m[k][n] * v.v[n];
    }
    ret.v[k] = s;
  }
  return ret;
}

vec3d operator* (mat33 m, vec3d v) {
  int k, n;
  vec3d ret;

  for (k = 0; k < 3; k++) {
    double s = 0;
    for (n = 0; n < 3; n++) {
      s += m.m[k][n] * v.v[n];
    }
    ret.v[k] = s;
  }
  return ret;
}

vec2d operator* (vec2d v, double a) {
  return vec2d(v.v[0] * a,
	       v.v[1] * a);
}

vec3d operator* (vec3d v, double a) {
  return vec3d(v.v[0] * a,
	       v.v[1] * a,
	       v.v[2] * a);
}

vec4d operator* (vec4d v, double a) {
  return vec4d(v.v[0] * a,
	       v.v[1] * a,
	       v.v[2] * a,
	       v.v[3] * a);
}

vec2d operator* (double a, vec2d v) {
  return vec2d(v.v[0] * a,
	       v.v[1] * a);
}

vec3d operator* (double a, vec3d v) {
  return vec3d(v.v[0] * a,
	       v.v[1] * a,
	       v.v[2] * a);
}

vec4d operator* (double a, vec4d v) {
  return vec4d(v.v[0] * a,
	       v.v[1] * a,
	       v.v[2] * a,
	       v.v[3] * a);
}

mat44 mat44::inverse() {
  mat33 tmp;
  mat44 ret;
  double det = 0;
  int i, j, k, n;
  int tx, ty;
  
  // compute determinant and adjoint matrix
  // row of inverse matrix
  for (i = 0; i < 4; i++) {
    // column of inverse matrix
    for (j = 0; j < 4; j++) {

      // fill the matrix used to compute the cofactor
      tx = 0;
      // row we're looking at
      for (k = 0; k < 4; k++) {
	if (k != j) {
	  // column we're looking at
	  ty = 0;
	  for (n = 0; n < 4; n++) {
	    if (n != i) {
	      tmp.m[ty][tx] = m[k][n];
	      ty++;
	    }
	  }
	  tx++;
	}
      }

      double cofactor = tmp.det() * (((i + j) % 2 == 0) ? 1 : -1);
      ret.m[i][j] = cofactor;

      if (j == 0) {
	det += cofactor * m[0][i];
      }
    }
  }

  printf("asserting...\n");
  assert(det != 0);
  
  // divide by determinant
  for (i = 0; i < 4; i++) {
    for (j = 0; j < 4; j++) {
      ret.m[i][j] /= det;
    }
  }

  return ret;
}

mat33 mat33::transpose() {
  mat33 ret;
  int i, j;
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      ret.m[i][j] = m[j][i];
  return ret;
}

mat33 mat33::inverse() {
  int i, j;
  mat44 bigmat, inv;
  mat33 ret;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++)
      bigmat.m[i][j] = m[i][j];
    bigmat.m[3][i] = bigmat.m[i][3] = 0;
  }
  bigmat.m[3][3] = 1;
  inv = bigmat.inverse();
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++)
      ret.m[i][j] = inv.m[i][j];
  }
  return ret;
}

double mat33::det() {
  return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
	  - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
	  + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]));
}

mat33 operator* (mat33 m, double s) {
  mat33 ret;
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      ret.m[i][j] = m.m[i][j] * s;
  return ret;
}

mat44 operator* (mat44 m, double s) {
  mat44 ret;
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      ret.m[i][j] = m.m[i][j] * s;
  return ret;
}

vec2d vec2d::normalize() {
  return (*this) * (1.0 / norm());
}

vec3d vec3d::normalize() {
  return (*this) * (1.0 / norm());
}

vec4d vec4d::normalize() {
  return (*this) * (1.0 / norm());
}

double vec3d::dot(vec3d x) {
  return v[0] * x.v[0] + v[1] * x.v[1] + v[2] * x.v[2];
}

double vec2d::dot(vec2d x) {
  return v[0] * x.v[0] + v[1] * x.v[1];
}

row3d::row3d(double x, double y, double z) {
  r[0] = x;
  r[1] = y;
  r[2] = z;
}

row3d vec3d::transpose() {
  return row3d(v[0], v[1], v[2]);
}

mat33 operator* (vec3d v, row3d r) {
  mat33 ret;
  int x, y;

  for (y = 0; y < 3; y++)
    for (x = 0; x < 3; x++)
      ret.m[y][x] = v.v[y] * r.r[x];

  return ret;
}

mat33 mat33::operator+(mat33 ma) {
  mat33 ret;
  int x, y;

  for (x = 0; x < 3; x++)
    for (y = 0; y < 3; y++)
      ret.m[x][y] = m[x][y] + ma.m[x][y];

  return ret;
}

mat33 mat33::operator-(mat33 ma) {
  mat33 ret;
  int x, y;

  for (x = 0; x < 3; x++)
    for (y = 0; y < 3; y++)
      ret.m[x][y] = m[x][y] - ma.m[x][y];

  return ret;
}

vec3d vec3d::cross(vec3d x) {
  return vec3d(v[1] * x.v[2] - v[2] * x.v[1],
	       v[2] * x.v[0] - v[0] * x.v[2],
	       v[0] * x.v[1] - v[1] * x.v[0]);
}

vec4d vec4d::quatDiv(vec4d x) {
  return quatMult(x.quatConj()) * (1 / x.sqnorm());
}

vec4d vec4d::quatMult(vec4d x) {
  return vec4d(v[3] * x.v[0] + v[0] * x.v[3] + v[1] * x.v[2] - v[2] * x.v[1],
	       v[3] * x.v[1] + v[1] * x.v[3] + v[2] * x.v[0] - v[0] * x.v[2],
	       v[3] * x.v[2] + v[2] * x.v[3] + v[0] * x.v[1] - v[1] * x.v[0],
	       v[3] * x.v[3] - v[0] * x.v[0] - v[1] * x.v[1] - v[2] * x.v[2]);
}

vec4d vec4d::quatConj() {
  return vec4d(v[0], -v[1], -v[2], -v[3]);
}

mat33 operator* (mat33 m, mat33 n) {
  mat33 ret;

  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      ret.m[y][x] = 0;
      for (int k = 0; k < 3; k++)
	ret.m[y][x] += m.m[y][k] * n.m[k][x];
    }
  }

  return ret;
}

mat44 operator* (mat44 m, mat44 n) {
  mat44 ret;

  for (int x = 0; x < 4; x++) {
    for (int y = 0; y < 4; y++) {
      ret.m[y][x] = 0;
      for (int k = 0; k < 4; k++)
	ret.m[y][x] += m.m[y][k] * n.m[k][x];
    }
  }

  return ret;
}

vec3d linePlaneIntersect(vec4d plane, vec3d picks[]) {
  // find 3 points on the plane

  vec3d n(plane.v[0], plane.v[1], plane.v[2]);
  vec3d n2 = n.normalize();

  vec3d pc = n2 * (-plane.v[3]) * (1 / n.norm());
  
  vec3d p1 = pc + n2.cross(vec3d(1, 0, 0));
  vec3d p2 = pc + n2.cross(vec3d(0, 1, 0));
  vec3d p3 = pc + n2.cross(vec3d(0, 0, 1));

  return linePlaneIntersect(p1, p2, p3, picks[0], picks[1]);
}

vec4d planeByPointNormal(vec3d point, vec3d normal) {
  vec4d ret;
  for (int i = 0; i < 3; i++)
    ret.v[i] = normal.v[i];
  ret.v[3] = -point.dot(normal);
  return ret;
}

vec3d linePlaneIntersect(vec3d x1, vec3d x2, vec3d x3, vec3d x4, vec3d x5) {
  return linePlaneIntersect(x1, x2, x3, x4, x5, NULL);
}

vec3d linePlaneIntersect(vec3d x1, vec3d x2, vec3d x3, vec3d x4, vec3d x5, double *tp) {
  mat44 num(1, 1, 1, 1,
	    x1.v[0], x2.v[0], x3.v[0], x4.v[0],
	    x1.v[1], x2.v[1], x3.v[1], x4.v[1],
	    x1.v[2], x2.v[2], x3.v[2], x4.v[2]);
  mat44 den(1, 1, 1, 0,
	    x1.v[0], x2.v[0], x3.v[0], x5.v[0] - x4.v[0],
	    x1.v[1], x2.v[1], x3.v[1], x5.v[1] - x4.v[1],
	    x1.v[2], x2.v[2], x3.v[2], x5.v[2] - x4.v[2]);
  
  // parametric value
  double t = -num.det() / den.det();

  if (tp != NULL) *tp = t;

  return x4 + (x5 - x4) * t;
}

double mat44::det() {
  mat33 tmp;
  double det = 0;
  int i, j, k, n;
  int tx, ty;
  
  // compute determinant and adjoint matrix
  // row of inverse matrix
  for (i = 0; i < 4; i++) {
    // column of inverse matrix
    for (j = 0; j < 1; j++) {

      // fill the matrix used to compute the cofactor
      tx = 0;
      // row we're looking at
      for (k = 0; k < 4; k++) {
	if (k != j) {
	  // column we're looking at
	  ty = 0;
	  for (n = 0; n < 4; n++) {
	    if (n != i) {
	      tmp.m[ty][tx] = m[k][n];
	      ty++;
	    }
	  }
	  tx++;
	}
      }
      double cofactor = tmp.det() * (((i + j) % 2 == 0) ? 1 : -1);
      det += cofactor * m[0][i];
    }
  }

  return det;
}

vec3d mat33::column(int n) {
  return vec3d(m[0][n], m[1][n], m[2][n]);
}

vec4d mat44::column(int n) {
  return vec4d(m[0][n], m[1][n], m[2][n], m[3][n]);
}

void mat33::setcolumn(int n, vec3d c) {
  for (int i = 0; i < 3; i++)
    m[i][n] = c.v[i];
}

void mat44::setcolumn(int n, vec4d c) {
  for (int i = 0; i < 4; i++)
    m[i][n] = c.v[i];
}

vec3d dropPerpendicular(vec3d x, vec3d y1, vec3d y2) {
  vec3d a = x - y1;
  vec3d b = y2 - y1;
  return y1 + b * (a.dot(b) / b.sqnorm());
}

mat44 translationMatrix(vec3d x) {
  mat44 ret;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      ret.m[i][j] = (i == j) ? 1 : 0;
  ret.setcolumn(3, x.tohc());
  return ret;
}

mat44 rotationMatrixByAxisAngle(vec3d axis, double angle) {
  mat44 ret;
  vec4d q = quatFromAA(axis, angle);
  mat33 x = quatToRotationMatrix(q);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      ret.m[i][j] = x.m[i][j];
    ret.m[i][3] = ret.m[3][i] = 0;
  }
  ret.m[3][3] = 1;
  return ret;
}

vec4d quatFromAA(vec3d axis, double angle) {
  vec4d ret;
  vec3d a = axis.normalize();
  for (int i = 0; i < 3; i++)
    ret.v[i] = a.v[i] * sin(angle / 2);
  ret.v[3] = cos(angle / 2);
  return ret;
}

vec4d rotationMatrixToQuat(mat33 m) {
  double s;
  vec4d ret;

  s = 0.5 * sqrt(m.m[0][0] + m.m[1][1] + m.m[2][2] + 1);
  
  ret.v[3] = s;
  ret.v[0] = (m.m[2][1] - m.m[1][2]) / (4 * s);
  ret.v[1] = (m.m[0][2] - m.m[2][0]) / (4 * s);
  ret.v[2] = (m.m[1][0] - m.m[0][1]) / (4 * s);

  ret = ret * (1.0 / ret.norm());

  return ret;
}

mat33 quatToRotationMatrix(vec4d q) {
  // Turn a quaternion into a rotation matrix with the usual convention
  // assumes quaternion is represented (vector, scalar) AND THAT
  // IT IS NORMALIZED

  mat33 ret;

  ret.m[0][0] = 1.0 - 2.0 * (q.v[1] * q.v[1] + q.v[2] * q.v[2]);
  ret.m[1][0] = 2.0 * (q.v[0] * q.v[1] + q.v[2] * q.v[3]);
  ret.m[2][0] = 2.0 * (q.v[2] * q.v[0] - q.v[1] * q.v[3]);
  
  ret.m[0][1] = 2.0 * (q.v[0] * q.v[1] - q.v[2] * q.v[3]);
  ret.m[1][1] = 1.0 - 2.0 * (q.v[2] * q.v[2] + q.v[0] * q.v[0]);
  ret.m[2][1] = 2.0 * (q.v[1] * q.v[2] + q.v[0] * q.v[3]);

  ret.m[0][2] = 2.0 * (q.v[2] * q.v[0] + q.v[1] * q.v[3]);
  ret.m[1][2] = 2.0 * (q.v[1] * q.v[2] - q.v[0] * q.v[3]);
  ret.m[2][2] = 1.0 - 2.0 * (q.v[1] * q.v[1] + q.v[0] * q.v[0]);

  return ret;
}

