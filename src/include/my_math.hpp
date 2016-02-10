#ifndef MY_MATH_H
#define MY_MATH_H

#include <GL/gl.h>
#define  M_PI   3.14159265358979323846f 

class vec;
class wc;
class ndc;
class pix;
class vp;

extern double det3Mat(GLdouble *mat);
extern double det4Mat(GLdouble *mat);
extern void invertGLMatrix(GLdouble *in, GLdouble *out);
extern void mult4MatBy4Vec(GLdouble *mat, GLdouble *v, GLdouble *out);
extern vec linePlaneIntersect(vec x1, vec x2, vec x3, vec x4, vec x5);
extern vec linePlaneIntersect(vec x1, vec x2, vec x3, vec x4, vec x5, double *t);

// general vector
class vec {
 public:
  vec();
  vec(double, double, double);
  vec(double, double, double, double);
  double x, y, z, w;
  double norm();
  vec normalize();
  vec operator*(double s);
  vec operator+(vec v);
  vec operator-(vec v);
  vec cross(vec v);
  double dot(vec v);
  wc toWc();
};

// world coordinate
class wc : public vec {
 public:
  wc() {}
  wc(double x_, double y_, double z_) {
    x = x_;
    y = y_;
    z = z_;
  }
  pix toPixel();
};

// normalized device coordinate
class ndc : public vec {
 public:
  wc toWc(double depth);
};

// pixel coordinate
class pix : public vec {
 public:
  pix() {}
  pix(double x_, double y_) { x = x_; y = y_; }
  ndc toNDC(vp v);
  //  wc toWc(double depth);
};

// viewport
class vp {
 public:
  vp();
  vp(double xc, double yc, double wc, double hc) {
    x = xc; y = yc; w = wc; h = hc;
  }
  double x, y, w, h;
};

#endif
