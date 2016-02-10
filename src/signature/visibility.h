#include "basic.h"
#include "viewer/viewer.h"
#include "model/model.h"

#define BUFSIZE_FEEDBACK 65536		// size of the buffer for the visibility computation

bool processHits (GLint hits, GLfloat *buffer, GLfloat *pixels, Model *model, Vec3d position, int w, 
				  int h,Edge *edge, double STEP);
bool processHits (GLint hits, GLfloat *buffer, GLfloat *pixels, Vec3d position, int w, int h);

bool checkValue (GLfloat *pixels, double x, double y, GLfloat feedbackZ, int w, int h, int error);
GLfloat getVal (GLfloat *pixels, int x, int y, int w, int h);
void writeVisibility(Vec3d position, Model *model, edgeVector &edges, const char *file, const char *file_sub, const char *file_list);
bool readVisibility (Vec3d position, Model *model, edgeVector &edges, const char *file,const char *file_sub, const char *file_list);
bool testVisibility (Vec3d position, const char *file_list);
void cleanupVisibleLines( Vec3d position, edgeVector &lines, double min_subtended_angle, double min_dihedral_angle, int max_lines );

