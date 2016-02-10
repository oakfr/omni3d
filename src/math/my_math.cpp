// implementation for my_math.h - see that file for more details

#include <iostream>
//#include <FL/glut.H>
#include <GL/glut.h>
#include <GL/glu.h>
#include "my_math.hpp"
#include <math.h>

//using namespace std;

pix wc::toPixel() {
  pix ret;

  GLdouble mvm[16];
  GLdouble pm[16];
  GLint vpt[4];

  glGetDoublev(GL_MODELVIEW_MATRIX, mvm);
  glGetDoublev(GL_PROJECTION_MATRIX, pm);
  glGetIntegerv(GL_VIEWPORT, vpt);
  
  gluProject(x, y, z, mvm, pm, vpt, &ret.x, &ret.y, &ret.z);

  return ret;
}

ndc pix::toNDC(vp v) {
  ndc ret;

  ret.x = -1 + 2 * ((x - v.x) / v.w);
  ret.y = -1 + 2 * ((y - v.y) / v.h);
  return ret;
}

// this function doesn't appear to work correctly
// wc pix::toWc(double depth) {
//  wc ret;
// 
//  GLdouble mvm[16];
//   GLdouble pm[16];
//  GLint vpt[4];
//
//  glGetDoublev(GL_MODELVIEW_MATRIX, mvm);
//  glGetDoublev(GL_PROJECTION_MATRIX, pm);
//  glGetIntegerv(GL_VIEWPORT, vpt);
//  
//  gluUnProject(x, y, depth, mvm, pm, vpt, &ret.x, &ret.y, &ret.z);
// }

vp::vp() {
  GLint vpt[4];
  glGetIntegerv(GL_VIEWPORT, vpt);

  x = vpt[0];
  y = vpt[1];
  w = vpt[2];
  h = vpt[3];
}

vec::vec() {
  x = y = z = w = 0;
}

vec::vec(double x_, double y_, double z_) {
  x = x_;
  y = y_;
  z = z_;
}

vec::vec(double x_, double y_, double z_, double w_) {
  x = x_;
  y = y_;
  z = z_;
  w = w_;
}

double vec::norm() {
  return sqrt(x*x + y*y + z*z + w*w);
}

vec vec::normalize() {
  //assert (norm() != 0);
  return (*this)*(1/norm());
}

vec vec::operator*(double s) {
  return vec(x * s, y * s, z * s, w * s);
}

vec vec::operator+(vec v) {
  return vec(v.x + x, v.y + y, v.z + z, v.w + w);
}

vec vec::operator-(vec v) {
  return vec(x - v.x, y - v.y, z - v.z, w - v.w);
}

vec vec::cross(vec v) {
  return vec(y * v.z - z * v.y,
	     z * v.x - x * v.z,
	     x * v.y - y * v.x);
}

double vec::dot(vec v) {
  return x * v.x + y * v.y + z * v.z + w * v.w;
}

wc vec::toWc() {
  return wc(x, y, z);
}

// 036
// 147
// 258
double det3Mat(GLdouble *mat) {
  return (mat[0] * (mat[4] * mat[8] - mat[5] * mat[7])
	  - mat[3] * (mat[1] * mat[8] - mat[2] * mat[7])
	  + mat[6] * (mat[1] * mat[5] - mat[2] * mat[4]));
}

// 0 4 8  12
// 1 5 9  13
// 2 6 10 14
// 3 7 11 15
void invertGLMatrix(GLdouble *in, GLdouble *out) {
  GLdouble tmp[9];
  double det = 0;
  int i, j, k, m;
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
	  for (m = 0; m < 4; m++) {
	    if (m != i) {
	      tmp[tx * 3 + ty] = in[m * 4 + k];
	      ty++;
	    }
	  }
	  tx++;
	}
      }

      double cofactor = det3Mat(tmp) * (((i + j) % 2 == 0) ? 1 : -1);
      out[j * 4 + i] = cofactor;

      if (j == 0) {
	det += cofactor * in[4 * i];
      }
    }
  }

  // divide by determinant
  for (i = 0; i < 4; i++) {
    for (j = 0; j < 4; j++) {
      out[j * 4 + i] /= det;
    }
  }
}

wc ndc::toWc(double depth) {
  wc ret;

  GLdouble mvm[16];
  GLdouble pm[16];

  glGetDoublev(GL_MODELVIEW_MATRIX, mvm);
  glGetDoublev(GL_PROJECTION_MATRIX, pm);

  GLdouble imvm[16];
  GLdouble ipm[16];
  invertGLMatrix(mvm, imvm);
  invertGLMatrix(pm, ipm);

  GLdouble v[4] = {x, y, depth, 1};
  GLdouble a[4], b[4];

  mult4MatBy4Vec(ipm, v, a);
  mult4MatBy4Vec(imvm, a, b);

  ret.x = b[0] / b[3];
  ret.y = b[1] / b[3];
  ret.z = b[2] / b[3];

  return ret;
}

void mult4MatBy4Vec(GLdouble *m, GLdouble *v, GLdouble *out) {
  out[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8] + v[3] * m[12];
  out[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9] + v[3] * m[13];
  out[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10] + v[3] * m[14];
  out[3] = v[0] * m[3] + v[1] * m[7] + v[2] * m[11] + v[3] * m[15];
}

double det4Mat(GLdouble *in) {
  GLdouble tmp[9];
  double det = 0;
  int j, k, m;
  int tx, ty;

  // use expansion by minors on first row
  
  // column
  for (j = 0; j < 4; j++) {

    // fill the matrix used to compute the cofactor
    tx = 0;
    // row we're looking at
    for (k = 0; k < 4; k++) {
      if (k != 0) {
	// column we're looking at
	ty = 0;
	for (m = 0; m < 4; m++) {
	  if (m != j) {
	    tmp[tx * 3 + ty] = in[m * 4 + k];
	    ty++;
	  }
	}
	tx++;
      }
    }

    double cofactor = det3Mat(tmp) * ((j % 2 == 0) ? 1 : -1);

    det += cofactor * in[4 * j];
  }
  
  return det;
}

vec linePlaneIntersect(vec x1, vec x2, vec x3, vec x4, vec x5, double *tp) {
  // perform a line-plane intersection as described on
  // http://mathworld.wolfram.com/Line-PlaneIntersection.html
      
  GLdouble num[16] = {1, x1.x, x1.y, x1.z,
		      1, x2.x, x2.y, x2.z,
		      1, x3.x, x3.y, x3.z,
		      1, x4.x, x4.y, x4.z};
  GLdouble den[16] = {1, x1.x, x1.y, x1.z,
		      1, x2.x, x2.y, x2.z,
		      1, x3.x, x3.y, x3.z,
		      0, x5.x - x4.x, x5.y - x4.y, x5.z - x4.z};
      
  // parametric value
  double t = -det4Mat(num) / det4Mat(den);
  //      cout << "t: " << t << endl;
  if (tp != NULL) *tp = t;
  return x4 + (x5 - x4) * t;
  //      cout << "Pick: (" << pick.x << ", " << pick.y << ", " << pick.z << ")" << endl;
}

vec linePlaneIntersect(vec x1, vec x2, vec x3, vec x4, vec x5) {
  return linePlaneIntersect(x1, x2, x3, x4, x5, NULL);
}
