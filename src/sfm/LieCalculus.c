/*
 * LieCalculus.c : This file implements some routines that are used to
 * perform calculus on the Lie group SO (3).
 */


void DerivR (double *dv, double *v, char axis)
{
  switch (axis) {
    case 'x' : dv[0] = 0.0;    dv[1] = -v[2];  dv[2] = v[1];   break; 
    case 'y' : dv[0] = v[2];   dv[1] = 0.0;    dv[2] = -v[0];  break; 
    case 'z' : dv[0] = -v[1];  dv[1] = v[0];   dv[2] = 0.0;    break; 
  }
}

void DoubleDerivR (double *dv, double *v, char axis1, char axis2)
{
  if (axis1 == 'x') {
    switch (axis2) {
      case 'x' : dv[0] = 0.0;     dv[1] = -v[1];   dv[2] = -v[2];   break; 
      case 'y' : dv[0] = v[1]/2;  dv[1] = v[0]/2;  dv[2] = 0.0;     break; 
      case 'z' : dv[0] = v[2]/2;  dv[1] = 0.0;     dv[2] = v[0]/2;  break; 
    }
    return;
  }

  if (axis1 == 'y') {
    switch (axis2) {
      case 'x' : dv[0] = v[1]/2;  dv[1] = v[0]/2;  dv[2] = 0.0;     break; 
      case 'y' : dv[0] = -v[0];   dv[1] = 0.0;     dv[2] = -v[2];   break; 
      case 'z' : dv[0] = 0.0;     dv[1] = v[2]/2;  dv[2] = v[1]/2;  break; 
    }
    return;
  }

  if (axis1 == 'z') {
    switch (axis2) {
      case 'x' : dv[0] = v[2]/2;  dv[1] = 0.0;     dv[2] = v[0]/2;  break; 
      case 'y' : dv[0] = 0.0;     dv[1] = v[2]/2;  dv[2] = v[1]/2;  break; 
      case 'z' : dv[0] = -v[0];   dv[1] = -v[1];   dv[2] = 0.0;     break; 
    }
    return;
  }
}
