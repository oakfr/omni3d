#ifndef _BASIC_H__
#define _BASIC_H__

/* defines */
#define  M_PI   3.14159265358979323846f
#define M_PI_2		1.57079632679489661923
#define VERTEX 0
#define EDGE 1
#define FACE 2
#define PROJECT_HOME "C:/omni3d"
#define DATABASE_HOME "C:/omni3d/data"
#define EPSILON 0.000001
#define SQRT3 1.73205080757

#define ITMAX 100 //Maximum allowed number of iterations.
#define EPS 1.0e-7 //Relative accuracy.
#define FPMIN 1.0e-30

#define DEFAULT_LADYBUG_ID 5040012

#pragma warning(disable: 4786)

/* includes */

#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
//#include <FL/glut.H>
#include <GL/glut.h>
#include <math.h>
#include <cv.h>
#include <vector>
#include "highgui.h"
#include <string>
#include <algorithm>
#include "impmath.hpp"
#include "VLf.h"
#include "VLd.h"
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "rudeconfig.h"
#include "timer/my_timer.h"
#include <map>
#include<functional>

#include <FL/Fl_Gl_Window.h>
#include <Fl/Fl.h>
#include <FL/Fl_Menu_Bar.H>
#include <Fl/Fl_Double_Window.h>
#include <FL/Fl_Button.H>

#include "mytrace.h"

static void* fonts[7] = {
GLUT_BITMAP_9_BY_15, 
GLUT_BITMAP_8_BY_13,
GLUT_BITMAP_TIMES_ROMAN_10, 
GLUT_BITMAP_TIMES_ROMAN_24, 
GLUT_BITMAP_HELVETICA_10,
GLUT_BITMAP_HELVETICA_12 ,
GLUT_BITMAP_HELVETICA_18 
};

class SphericalCoord {
public:
	double rho, phi, theta, pitch;

	SphericalCoord() {rho = phi = theta = pitch = 0;}
	SphericalCoord(double a, double b, double c, double d) {rho=a; phi=b; theta=c; pitch=d;}
	SphericalCoord (Vec3d);
	void init(double r) {rho = r; phi = 3*M_PI/4; theta = M_PI/4; pitch=0;}
	void print() {printf("Spherical coord: rho= %f,phi= %f,theta= %f,pitch= %f\n",rho,phi,theta,pitch);}
	double geodesicDistance(SphericalCoord sp);
};

class Eye_pose {
public:
	Vec3d eye;
	Vec3d target;
	Vec3d up;

	/* constructor */
	Eye_pose() {};
	Eye_pose (Vec3d a, Vec3d b, Vec3d c) {eye=a; target=b; up=c;}
	void print() {printf("[%f,%f,%f] [%f,%f,%f] [%f,%f,%f]",eye[0],eye[1],eye[2],target[0],target[1],target[2],up[0],up[1],up[2]);}

};

enum inputStateType { none, rotating, translating, scaling, climbing, pitching, draggingcpoint };

enum Event { NODE_CHANGED, NEW_FRAME, PRIMARY_LOST, VALIDATION_SELECT, BACKUP_SELECT, LUT_SELECT, POSE_COMPUTE, SM_ERROR };

typedef std::vector<double> doubleVector;
typedef std::vector<int> intVector;
typedef std::vector<bool> boolVector;

static int width_image = 720;
static int height_image = 480;

#endif
