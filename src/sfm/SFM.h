/*
 * SFM.h : This file defines the major data structures that are used by 
 * this Structure from Motion algorithm.
 */

/* Typedefs ****************************************/

typedef struct {
  double q[4];
  double t[3];
} position;

typedef struct {
  double q[4];
  double a, b;
} line;

typedef struct {
 /* The endpoints of the edge in mm. */
  double x1, y1, x2, y2;
  /* length of the edge in mm. */
  double length;  
  /* the normal vector */
  double m[3];
  /* m_error = sin of the angular error in m */
  double m_error, l_error;
} edge;

typedef struct {
  double pt1[3];
  double pt2[3];
} segment;

/* Defines ****************************************/

#define MAX_M    30
#define MAX_N    60

/* DEFINES ******************************************/

#define THETA    0
#define OMEGA    1
#define A        2
#define B        3
#define ALPHA    4
#define BETA     5
#define GAMMA    6
#define TX       7
#define TY       8
#define TZ       9

/* Globals ******************************************/

extern edge *TheEdges[MAX_M][MAX_N];
extern line *TheLines[MAX_N];
extern position *TheCameraPositions[MAX_M];
extern segment TheSegments[MAX_N];
extern int m, n, I, J;
extern char FrozenParameter;
