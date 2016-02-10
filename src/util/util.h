#ifndef _UTIL_H__
#define _UTIL_H__

#include "basic.h"
#include "geom/geom3D.h"

// util.cpp
Vecd toHomg (Vec3d v);
Vec3d toNonHomg (Vecd v);

std::string util_toFilename (char *root, std::string str, std::string str2);
std::string util_toFilename (char *root, char *str, char *str2);
bool existFile (const char *filename) ;
std::string findNextAvailableFilename ();
void glDisplayMsg (double x, double y, char *string, int width, int height, \
				   const float colors[3],  int) ;
void glDisplayMsg (double x, double y, double num, int width, int height, \
				   const float color[3],  int) ;
void sleep_fps (double fps);
void glRectangle (int x, int y, int w, int h, const float color[3], bool full);
void glRectangle (int x, int y, int z, int w, int h, const float color[3], bool full);
void glPolygon (int n, int x, int y, int w, int h, const float color[3], bool full);
void readFaceFile (faceVector &vec, std::string filename);
bool isInside (int x, int y, int x0, int y0, int w, int h);
void mouseMove (int x, int y, int &anchorx, int &anchory, double &zoomMap,SphericalCoord&,int,int,int,int);
void mouseRotate ( int x, int y, int &anchorx, int &anchory, SphericalCoord &, int);
void screenSnapshot (int window,int,int, int w, int h, int &timestamp);
void glReset(int w, int h);
void insertPosition (std::vector<Vertex> &vector, int, Vertex v, Vertex w, Vertex z);
void glCircle (int x, int y, double r1, double r2, int width, int height) ;
void drawFrustum(Vec3d position, Vec3d target, Vec3d up, const float color[3], double focalLength, double size);
void drawFrustum(Eye_pose pose, const float color[3], double focalLength, double size);
void subdivideEdge (Edge edge, int id1, int id2, Vertex &a, Vertex &b, int SUBDIVISION_RESOLUTION);
void subdivideEdge (vertexVector &edgeVertices, Edge *edge);
Vertex getVertexFromId (int id, vertexVector &V);
Vertex* getVertexPtrFromId (int id, vertexVector &V);
Edge getEdgeFromId (int id, edgeVector &E);
Edge* getEdgePtrFromId (int id, edgeVector &E);
void readVertices (vertexVector &vector);
void readEdges (edgeVector &vector, vertexVector &vertex_vector);
void readFaces (faceVector &vector, vertexVector &vertex_vector);
void scaleEdges (edgeVector &edges, int X, int Y, double scale);
void processMousePressed (int x, int y, bool &allEdges, bool &visibleEdges);
int readVisibilityDiff (int id1, int id2);
void insertDouble (doubleVector &vector, double a, int max_size);
void writeVertices (vertexVector &V, const char *filename);
void readVertices (vertexVector &V, const char *filename);
void writeEdges (edgeVector &E, const char *filename);
void readEdges (edgeVector &E, vertexVector &V, faceVector &F, const char *filename);
void writeFaces (faceVector &F, const char *filename);
void readFaces (faceVector &F, vertexVector &V, const char *filename);
void extractXYZ (const std::string line, double *table);
void extractXYZ (const std::string line, Vec3d &v) ;
void extractXYZ (const std::string line, Vertex *table);
double roundUp (double _a) ;
int round (double val);
void printVertexVector (vertexVector V) ;
void printEdgeVector (edgeVector E) ;
void printTime(const char *text) ;
Vertex getSubVertex (Edge *edge, int pos, int SUBDIVISION_RESOLUTION);
void getline(char *buf,FILE *fp);
void fromGLfloatToChar (GLfloat *floats, unsigned char *chars, int w, int h);
Vec3d getNormal (Vec3d p1, Vec3d p2, Vec3d p3);
Mat3d fromEulerTo3x3Matrix (double rx, double ry, double rz);
Mat3d EulerXRotation (double theta);
Mat3d EulerYRotation (double theta);
Mat3d EulerZRotation (double theta);
void normAngle (double &angle);
Mat3d toMat3d (Matd P);
bool checkPlaneIntersection (Vec3d P1, Vec3d P11, Vec3d P12, Vec3d Q);
bool intersectPlanes (Vec3d P1, Vec3d P11, Vec3d P12, Vec3d P2, Vec3d P21, Vec3d P22, Vec3d &A1, Vec3d &A2, Vec3d &A3, Vec3d &A4);

void minMaxDihedralAngle( Edge *l1, Edge *l2, Vec3d p, double diam, double &min_angle, double &max_angle, double &max_polar_angle/*, int n_samples, double *samples*/  );
double maxPolarAngle( Vec3d l1a, Vec3d l1b, Vec3d l2a, Vec3d l2b );

bool minMaxPolarAngles( Edge *l1, Edge *l2, Vec3d p, double diam, double &min1, double &max1, double &min2, double &max2 );
void minMaxSubtendedAngle( Edge *line, Vec3d position, double diam, double &min_angle, double &max_angle );
void find_topk ( int *row, int n, int *values, int p );
void find_topk ( double *row, int n, double *values, int p );
double CompareTwoRotations( Quaternion q1, Quaternion q2 );
void computeVanishingPoints( edgePlaneVector &edgeplanes, vp_vector &vps );
void clusterVanishingPoints( vp_vector &vps, vp_vector &vpc, int k ) ;
double scoreSetsOfVanishingPoints( vp_vector &set1, vp_vector &set2, double angle_threshold );
void alignSetsOfVanishingPoints( vp_vector &set1, vp_vector &set2,  std::vector< ExtrinsicParameters > &solutions);
void writeEdgesVector (std::vector< std::vector<Edge2D> > edges, const char *filename, int nframes);
void readEdgesVector (std::vector< std::vector<Edge2D> > &edges, const char *filename);
double Distance (Vec3d a, Edge edge);
double Distance (Edge edge1, Edge edge2);
Edge Average (std::vector <Edge> edges);
bool readHomogeneousPoint (FILE *fp, Vec3d &point);
bool readHomogeneousPoint (FILE *fp, Vec3d &point);
bool readCameraMatrix (FILE *fp, Matd &A);
bool readCameraRotationMatrix (FILE *fp, Mat3d &A);
bool Exist (int a, std::vector<int> A);
Vec2d intersectLines (Vec2d a, Vec2d b, Vec2d c, Vec2d d);
bool merge2DEdges ( edge2DVector &edges, Edge2D &edge );
void detectHoughLines (IplImage *img, IplImage *hough_img, IplImage *temp_img, edge2DVector &edges);
bool removeElement (EdgePlane *edgeplane, edgePlaneVector &edgeplanes);
void selectNRandomInt (int N, int M, intVector &vector);

Mat3d rotationFromQuaternion (Vec3d w, double theta);
Mat3d Identity ();
Mat3d skew (Vec3d w);

void rotateCW (IplImage *src, IplImage *dst);
void rotateCCW (IplImage *src, IplImage *dst);
void flipHorizontal (IplImage *src, IplImage *dst);

void Free (edgeVVector vector);
void Free (edgeVector vector);
void Free (Edge edge);

void FFT (int *input, int N, doubleVector &real_vector, doubleVector &im_vector);
void Rotate (Vec3d center, Vec3d axis, double angle, Vec3d &point);

int fltk2gl_mousebutton (int button);
int fltk2gl_mousemodifier ();

bool cluster_poses( std::vector<ExtrinsicParameters> &poses, std::vector<ExtrinsicParameters> &top_poses, int K);
Vec3d average_translations( std::vector<ExtrinsicParameters> &poses );
Quaternion average_rotations( std::vector<ExtrinsicParameters> &poses );

// draw.cpp
void glLookAt (Vec3d position, Vec3d unit_target, Vec3d unit_up);
void drawCorners(CvPoint2D32f* corners, int corner_count, int window, const float color[3]) ;
void drawCorners(std::vector<Vertex> corners, int window, int window_width, int window_height, const float color[3]) ;
void drawBox (int x, int y, int r, const float colors[3]) ;
void glVertex (Vec3d v);
void glVertex (Vec3d v, Vec3d w);
void glVertex (Vec3d v1, Vec3d v2, Vec3d v3, Vec3d v4);
void glVertex (Vertex v, Vertex w) ;
void glPolygonMode (bool fill);
void glBox (Vec3d v, double r_x, double r_y, double r_z, bool fill, const float color[3]) ;
void glBox (Vec3d v, double radius, bool fill, const float color[3]);
void glBox (Vertex v, double r_x, double r_y, double r_z, bool fill, const float color[3]) ;
void glBox (Vertex *v, double r_x, double r_y, double r_z, bool fill, const float color[3]) ;
void glBox (Vertex v, double radius, bool fill, const float color[3]) ;
void glBox (Vertex *v, double radius, bool fill, const float color[3]) ;
void glColor(const float colors[3]);
void glBox2D (Vec3d v, double r_x, double r_y, bool fill, const float color[3]);
void glBox2D (CvPoint2D32f v, double r_x, double r_y, bool fill, const float color[3]);
void draw_simple (vertexVector &V, GLenum mode, const float color[3]);
void draw (vertexVector &V, int vertexId, int edgeId, GLenum mode, bool pickingMode, double scale, bool redraw, const float color[3]);
void draw_simple (edgeVector &E, GLenum mode, const float color[3]);
void draw (edgeVector &E, int vertexId, int edgeId, GLenum mode, bool pickingMode, double scale, bool redraw, const float color[3]);
void draw_simple (faceVector &F, GLenum mode, const float color[3]);
void draw (faceVector &F, int faceId, int edgeId, GLenum mode, bool pickingMode, bool redraw, const float color[3]);
void draw (vertVector &V, edgeVVector &E, double scale, Vec3d landmark, const float color_vertex[3], \
		   const float color_landmark[3], const float color_edges[3]) ;
void draw (vertVector &V, edgeVVector &E, double scale, const float colors1[3], const float colors2[3]);
void lighting (Vec3d lightsource);
void lighting (bool on);
void draw2DEdges(edge2DVector edges, const float color[3]);
void draw3DEdges (edgeVVector edges, const float color[3], int width);
void removeEdge (edge2DVector &edges, int id);
void glPyramid(Vec3d s, Vec3d a, Vec3d b, Vec3d c, Vec3d d, bool fill);
void draw2DEdge (Edge2D edge, const float color[3], int lineWidth);
void drawArcLine (Vec3d s, Vec3d a, Vec3d b, int n, int linewidth);
void score_to_color( double score, float color[3] );

int which_bag( int x, int y, int window_size, int size_x, int size_y, bool &up, bool &left ); // used by fast tracker and others
void which_bags_neighbor( int bag_id, std::vector< int > &bag_ids, int size_x, int size_y, bool up, bool left );

bool indexToColor( int n, int index, float *color );
bool colorToIndex( int n, int &index, float color[3] );

int intersectRayFaces( Vec3d center, Vec3d ray, faceVector &faces, Vec3d &point );
void RGBtoHSV( double r, double g, double b, double *h, double *s, double *v );
void HSVtoRGB( double h, double s, double v, double *r, double *g, double *b );

Vec2d flipEdgeDetector( Vec2d a, double h );
bool intersectEdgeplanesToLine( edgePlaneVector edgeplanes, Edge *line );

// file.cpp
bool isEmptyDirectory (char *dirname);
void emptyDirectory (char *dirname);
void printFileInfo (const char *filename);
time_t getModificationTime (const char *filename);
bool isFileOlder (const char *filename1, const char *filename2);
FILE *fileOpen (const char *filename, const char *mode);
void fileClose (FILE *f);
void fileErase (const char *filename);
bool fileExist (const char *filename);
void fileRemove (const char *filename);
void fileTouch (const char *filename);
bool fileScan (FILE *f, char *str, int n);
bool fileOffset (FILE *f, int n);
void createDataInfoFile (char *dirname, int w, int h, int nimages, double framerate, int synthetic);

class Weight;
typedef std::vector<Weight> weightVector;

class Weight {
public:
  int i,j,ii,jj;
  double weight;
  Vec3d position;
  
  Weight() {i=0;j=0;ii=0;jj=0;weight=0;position=Vec3d(0,0,0);}
  Weight(int a,int b,int c, int d,double w,Vec3d pos) {i=a;j=b;ii=c;jj=d;weight=w;position=pos;}
  bool operator== (Weight &w);
  void print(FILE *out) {fprintf(out,"%d %d %d %d %f ",i,j,ii,jj,weight);}
  void print() {print(stdout);}
  bool exist(weightVector vector);
};


//fposition
class FPosition {
public:
  int i,j;
  long int pos;
  bool valid;

  FPosition() {i=j=0; pos=0; valid=false;}
  FPosition(int _i,int _j,long int _pos,bool _v) {i=_i;j=_j;pos=_pos;valid=_v;}
};

class FPositionList {
public:
  std::vector<FPosition> lst;

  void insert(int i,int j,long int pos);
  FPosition get(int i,int j);
  void set (int i, int j, long int pos);
  int size() {return lst.size();}
  void clear() {lst.clear();}
};

#endif
