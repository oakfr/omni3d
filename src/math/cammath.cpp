#include <iostream>

#include "cammath.hpp"

//using namespace std;

mat34 cam::toMatrix() {
  mat34 ret;

  ret.m[0][0] = ax;
  ret.m[0][1] = 0;
  ret.m[0][2] = -x0;
  ret.m[0][3] = 0;

  ret.m[1][0] = 0;
  ret.m[1][1] = ay;
  ret.m[1][2] = -y0;
  ret.m[1][3] = 0;

  ret.m[2][0] = 0;
  ret.m[2][1] = 0;
  ret.m[2][2] = -1;
  ret.m[2][3] = 0;

  return ret;
}

mat33 cam::toMatrix33() {
  mat33 ret;

  ret.m[0][0] = ax;
  ret.m[0][1] = 0;
  ret.m[0][2] = -x0;

  ret.m[1][0] = 0;
  ret.m[1][1] = ay;
  ret.m[1][2] = -y0;

  ret.m[2][0] = 0;
  ret.m[2][1] = 0;
  ret.m[2][2] = -1;

  return ret;
}

mat44 pose::toMatrix() {
  int i, j;
  mat44 ret;

  mat33 r = quatToRotationMatrix(q);
  vec3d a = r * p;

  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++)
      ret.m[i][j] = r.m[i][j];
    ret.m[i][3] = -a.v[i];
  }

  ret.m[3][0] = ret.m[3][1] = ret.m[3][2] = 0;
  ret.m[3][3] = 1;

  return ret;
}

vec2d project(vec3d world_pt, pose& p, cam& c) {
  vec4d k = world_pt.tohc();
  vec4d j = p.toMatrix() * k;
  vec3d h = c.toMatrix() * j;
  return h.unhc();
}

vec3d unproject(vec2d image_coords, pose& p, cam& c) {
  int i, j;
  mat33 cammat;
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      cammat.m[i][j] = c.toMatrix().m[i][j];
	  return (p.toMatrix().inverse() * (cammat.inverse() * image_coords.tohc()).tohc()).unhc();
}

vec4d invQuatRot(vec4d& q) {
  return vec4d(-q.v[0], -q.v[1], -q.v[2], q.v[3]);
}

vec3d pose::viewDir() {
  return quatToRotationMatrix(q).inverse() * vec3d(0, 0, -1);
}

pose matrixToPose(mat44 m) {
  pose ret;
  mat44 m2 = m * (1 / m.m[3][3]);

  mat33 rot;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      rot.m[i][j] = m2.m[i][j];
  ret.q = rotationMatrixToQuat(rot);
  ret.p = rot.inverse() * m2.column(3).unhc() * -1;
  return ret;
}
