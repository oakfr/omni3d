#ifndef _VIEWER_HPP_
#define _VIEWER_HPP_

#include "basic.h"
#include "geom/geom3D.h"
#include "util/util.h"
#include "math/numerical.h"
#include "math/histogram.h"

#define BUF_SIZE 512 // buffer size for GL_SELECT mode

class Viewer;

enum ViewerType {WALKTHROUGH, OUTSIDE};

class Viewer {
public:
	
	int _width,_height;
	int _windowId;
	bool _valid;
	
	// 3d stuff
	Vec3d _eye,_target,_unit,_up;
	SphericalCoord _spheric;
	int _anchorx,_anchory;
	int _istate;
	double _znear, _zfar;
	double _fov;
	GLenum _mode; // GL_SELECT, GL_RENDER
	GLuint *_selectBuffer;
	ViewerType _type;

	Vec3d _save_eye; // useful when switching from outside view to walkthrough view
	SphericalCoord _save_spheric; // useful when switching from outside view to walkthrough view

	Viewer() {_width=0; _height=0; _valid = false;}
	~Viewer();

	void set(int w, int h, int id);
	void clearWindow(bool black);
	void setWindow();
	void drawImage (IplImage *img, int x, int y, bool color,bool flip);
	void drawEdge (Edge2D edge, const float color[3], int lineWidth);
	void drawEdge (Edge edge, const float color[3], int lineWidth);
	void drawEdge (Edge *edge, const float color[3], int lineWidth) {drawEdge(*edge,color,lineWidth);}
	void drawTriangle (Vec3d p1, Vec3d p2, Vec3d p3, const float color[3]);
	void drawPoint( Vec3d point, int point_width, const float color[3], double blending) ;
	int drawPoints (vec2dVector points, int x, int y, double zoom, const float color[3], double blending, bool flipHoriz, \
						 bool flipVert, double h, double w);
	void drawEdges (edge2DVector edges, int x, int y, double zoom, const float color[3], bool flipVert, bool flipHoriz, \
		double h, double w, int cameraId);
	void drawEdges (edge2DVector edges, int x, int y, double zoom, const float color[3], bool flipVert, bool flipHoriz, \
		double h, double w, int cameraId, int linewidth);
	void drawEdgesColorCoded (edge2DVector edges, int x, int y, double zoom, bool flipVert, bool flipHoriz, \
		double h, double w, int cameraId, int linewidth);
	void flipEdges (edge2DVector &edges, int height);
	void drawFaces (faceVector &faces, const float color[3], bool fill);
	void drawEdges (edgeVVector &edges, const float color[3], int width, bool wireframe);
	void drawEdges (edgeVector &edges, const float color[3], int width, bool wireframe);
	void drawEdgeBoxes (edgeVector &edges, double width);
	void drawEdgesSelectId (edge2DVector edges, int x, int y, double zoom, const float color[3], bool flipVert, bool flipHoriz, double h,\
								double w, edgePlaneVector edgeplanes, int cameraId, int linewidth);
	void drawEdgesSelectId (edge2DVector edges, int x, int y, double zoom, const float color[3], bool flipEdge, bool flipVert, double h,\
								double w, edgeVector &lines, int cameraId, int linewidth);
	void drawTriangle (Vec2d p1, Vec2d p2, Vec2d p3, const float color[3]);
	void drawBox (Vec2d p1, Vec2d p2, Vec2d p3, Vec2d p4, const float color[3]);
	void drawBox (Vec3d position, double size, const float color[3], bool fill);
	void drawBox (Vec2d p1, Vec2d p2, double transparency, const float color[3]);
	void drawBoxes (std::vector<Vec2d> &vector, const float color[3], int size);
	void drawBox (int x1, int y1, int x2, int y2, int line_width, const float color[3]);

	void init (Vec3d target, double rho, double phi, double theta, double fov, double znear, double zfar, ViewerType);
	void setup2D ();
	void setup2D (int x, int y); // to be called in GL_SELECT mode only!
	void setup3D();
	void setup3D (int x, int y); // to be called in GL_SELECT mode only!
	void setup3D_topview();
	Vec3d raycast (int winx, int winy);
	void drawAxis (Vec3d position, double radius);

	void rotateViewPoint (int x, int y, double sensitivity);
	void scaleViewPoint (int x, int y, double sensitivity);
	void translateViewPoint (int x, int y, double sensitivity);
	void motion (int x, int y, double sens_trans, double sens_rot, double sens_scale);
	void mouse (int button, int state, int x, int y, int glutModifier);
	void refresh() {setWindow(); glutPostRedisplay();}
	void glRasterPos2D(int x, int y);
	void drawEdgePlanes (edgePlaneVector edgeplanes, Vec3d center, Quaternion rotation, double radius, int activeSensor, int color_code, int linewidth);
	void drawEdgePlane (EdgePlane edgeplane, Vec3d center, Quaternion rotation, double radius, int color_code, int linewidth);
	void drawEdgePlanesDetails (edgePlaneVector edgeplanes, Vec3d center, Quaternion rotation, double radius, int activeSensor, int color_code, int linewidth);
	void drawEdgePlaneDetails (EdgePlane edgeplane, Vec3d center, Quaternion rotation, double radius, int color_code, int linewidth);

	void screenshot (const char *filename);
	void screenshot (const char *filename, int x1, int y1, int dx, int dy);
	void Viewer::screenshot (int timestamp, int x1, int y1, int dx, int dy);
	void drawSphere (Vec3d center, double radius, const float color[3], int slices, int stacks);
	void Viewer::drawCone( Vec3d center, Vec3d dir, double base_radius, const float color[3], int slices, int stacks ) ;
	void Viewer::drawCircle( Vec3d center, Vec3d normal, double radius, const float color[3], int line_width, int slices );
	void Viewer::drawEllipse(int x, int y, float xradius, float yradius, const float color[3]);

	void drawLine (Vec3d a, Vec3d b, const float color[3], int width);
	void drawLine (Vec3d a, Vec3d b, int width);
	void drawLine (Vec2d a, Vec2d b, const float color[3], int width);
	void displayMsg (double x, double y, const float colors[3], int font, const char *string, ...);
	void displayMsg (double x, double y, char *string, \
		const float colors[3], int font);
	GLint setMode (GLenum mode);
	void processHits (GLint hits, intVector &hitVector);
	void processVideoHits (GLint hits, intVector &hitVector);
	void stopPicking(intVector &hitVector);
	void stopVideoPicking(intVector &hitVector);
	void drawEdgesFeedback (edgeVector &edges, double step);
	void switchType();
	void setFill(bool fill);	
	void glLighting( Vec3d position );
	void Viewer::drawSiftFeatures( std::vector< Vec4d > &points, int x, int y, double zoom, const float color[3], bool flipHoriz, \
						 bool flipVert, double h, double w);

	void drawUserMask( int x, int y, double zoom, std::vector< Vec2d > pts, bool flipVertical, int h );

	void Viewer::drawFace( Vec2d a, Vec2d b, Vec2d c, Vec2d d, int x, int y, double zoom, const float color[3], bool flipEdge, bool flipVert, double h, \
							double w );
	void Viewer::drawFace( Vec3d a, Vec3d b, Vec3d c, Vec3d d, Vec3d n, double thickness, const float color[3] ) ;

	void Viewer::drawFrustum ( ExtrinsicParameters pose, double radius, const float color[3] ) ;

	void drawCorrespondence( corres &c, ExtrinsicParameters pose, const float color[3] );
	void drawCorrespondenceInfo( corres &c, ExtrinsicParameters &pose, int x, int y, const float color[3] );

	void Viewer::drawHistogram( Histogram &hist, int x, int y, int w, int h, const float color[3] );

	void restoreView();
	
};

#endif
