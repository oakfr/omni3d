#ifndef CAMERA_MATH_HPP
#define CAMERA_MATH_HPP

#include "impmath.hpp"

class cam;
class pose;

extern vec2d project(vec3d world_coords, pose& p, cam& c);
extern vec3d unproject(vec2d image_coords, pose& p, cam& c);
extern pose matrixToPose(mat44 m);

// if q is a normalized rotation quaternion, return the inverse rotation
extern vec4d invQuatRot(vec4d& q);

class cam {
public:
  // intrinsic parameters - no skew yet
  double ax, ay, x0, y0;

  cam() { }
  cam(double a, double b, double c, double d) { ax = a; ay = b; x0 = c; y0 = d; }
  mat34 toMatrix();
  mat33 toMatrix33();
};

class pose {
public:
  pose() { }
  pose(vec3d a, vec4d b) { p = a; q = b; }

  // position of camera and rotation
  // that _world_points_ must undergo
  // to be correctly transformed - this
  // is a quaternion.  Translation happens first.
  // quaternion is represented (vector, scalar)
  vec3d p;
  vec4d q;

  mat44 toMatrix();
  vec3d viewDir();
};  

#endif
