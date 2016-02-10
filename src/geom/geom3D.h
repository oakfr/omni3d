#ifndef _GEOM3D_H__
#define _GEOM3D_H__

#include "basic.h"
#include "colors.h"
#include "math/numerical.h"
#include <functional>

#define GEOM_RESOLUTION 0.005 // (inches) if two vertices are close enough, there are considered equal
#define NODE 3

#define COS_5  0.996194698091
#define COS_7 0.9925461516412
#define COS_10 0.984807753012
#define COS_15 0.965925826289
#define COS_20 0.939692620786
#define COS_30 0.866025403783
#define COS_85 0.087155742747
#define COS_80 0.173648177667
#define COS_25 0.906307787036

#define OCCLUSION_ANGLE_ERROR 0.017453292519 // 1 degree

#define DISTORTION_EDGE_STEP 10 // in pixels, step used to sample edges during distortion
#define MAINTENANCE_ANGULAR_SKEW_THRESHOLD COS_20
#define MAINTENANCE_ANGULAR_SEP_THRESHOLD COS_30

class Vertex;
class Edge;
class Edge2D;
class Face;

typedef std::vector<Vertex*> vertexVector;
typedef std::vector<Edge*> edgeVector;
typedef std::vector<Face*> faceVector;
typedef std::vector<Vertex> vertVector;
typedef std::vector<Edge> edgeVVector;
typedef std::vector<Edge2D> edge2DVector;
typedef std::vector<Edge2D>::iterator edge2DVectorIter;
typedef vertexVector::iterator vertexVectorIter;
typedef edgeVector::iterator edgeVectorIterator;

typedef std::vector<Vec2d> vec2dVector;
typedef std::vector<Vec3d> vec3dVector;


class Vec2dTable {
public:
	Vec2dTable() {m_w=m_h=0;}
	Vec2dTable(int w, int h);
	~Vec2dTable() {m_vector.clear();}
	
	int getWidth() {return m_w;}
	int getHeight() {return m_h;}
	Vec2d getElement(int i, int j) {assert(j<getWidth()); assert(i<getHeight()); return m_vector[i*getWidth()+j];}
	void setElement(int i, int j, Vec2d vec) {assert(j<getWidth()); assert(i<getHeight()); m_vector[i*getWidth()+j] = vec;}
	void rescale();
	void write(const char *filename);
	bool read (const char *filename);
	
protected:
	int m_w,m_h;
	std::vector<Vec2d> m_vector;
};

//////////////////////////////////////////////////////////
//                       class Vertex
//////////////////////////////////////////////////////////

class Vertex {
public:
	
	// ** variables **
	Vec3d _position;
	int _id;
	edgeVector _edges;
	double _weight;
	bool _visible;
	
	// ** accessors **
	Vec3d getPosition() {return _position;}
	void setPosition(Vec3d position) {_position = position;}
	int getId() {return _id;}
	double getWeight() {return _weight;}
	std::vector<Edge*> getEdges() {return _edges;}
	double getX() {return _position[0];}
	double getY() {return _position[1];}
	double getZ() {return _position[2];}
	void setX(double a) {_position[0] = a;}
	void setY(double a) {_position[1] = a;}
	void setZ(double a) {_position[2] = a;}
	int nedges() {return _edges.size();}

	// ** methods **
	Vertex ();
	Vertex (double _x, double _y, double _z);
	Vertex (double _x, double _y, double _z, int _id);
	Vertex (const Vertex &v); // canonical copy constructor
	Vertex& operator= (const Vertex &v) {_position = v._position; _id = v._id; _weight = v._weight; return *this;};
	~Vertex();
	
	void unit ();
	double length (); 
	double distance (Vertex *v);
	double distance (Vertex v);
	void scal (double r);
	double Dot (Vertex v);
	Vec3d Cross (Vertex u, Vertex v); 
	Vec3d Cross (Vertex v);
	void insertEdge (Edge edge);
	Vec3d add (Vertex v);
	Vec3d add (Vertex *v);
	Vec3d sub (Vertex v);
	Vec3d sub (Vertex *v);
	void mult (double d);
	void div (double d);
	void Vertex::rotate (Vec3d center, Vec3d axis, double angle);
	
	/*Vertex operator+(const Vertex &p) const;
	Vertex operator+(const int &p) const;
	Vertex operator-(const Vertex &p) const;
	Vertex operator*(const double d) const;
	Vertex operator/(const double d) const;*/
	bool operator==(const Vertex *v) const;
	bool operator==(const Vertex v) const;
	
	void glVertex() {glVertex3f(_position[0],_position[1],_position[2]);}
	void glVertex2D() {glVertex3f(_position[0],_position[1],0);}
	void glNormal() {glNormal3f(_position[0],_position[1],_position[2]);}
	bool contains (Edge *e) ;
	void insert (Edge *e) ;
	int n_visible_edges ();
	bool equal (Vertex *v) ;
	bool equal (Vertex v) ;
	bool belongs (Edge *e); 
	bool belongs (vertexVector vector);
	bool belongs_id (vertVector vector);
	bool isInside (Vertex a, Vertex b);
	void insert (vertexVector V_, vertexVector &V, Edge *e) ;
	bool isOrthogonal (Vertex *v1, Vertex *v2);
	int findVertexInList (vertexVector V);
	Vertex* sibling (Edge *e);
	Edge* sibling (Vertex *v);
	void print();
	void print(FILE *out);
	bool read (const char *s);
	bool read (FILE *fin);
	bool isAligned (Vertex *a, Vertex *b, Vertex *c);
	bool isOnSegment (Edge *e);
	void set (Vertex a);
	void set (double _x, double _y, double _z);
	double angle (Vertex a);
	void mult (Mat3d mat);
	void computeWeight();
	double vertexFunction (Edge *e1, Edge *e2);
	double vertexFunction (Edge *e1, Edge *e2, Edge *e3);
	Vec3d toVector (Edge *edge);
	double Vertex::angleFunction (double x);
	Vertex *next( Face *f );
	Vertex *prev( Face *f );
};

/*class WVertex : public Vertex {
public:
double _weight;
bool operator<(const WVertex &v);
};*/

// status for the maintenance phase
// ACCEPTED if, given the current camera pose, it finds a good match on the image.
// REJECTED if it does not find a good match. 
// REMOVED if it has been removed from the visibility set (NODE_CHANGED)
// UNKNOWN otherwise.
enum Status { ACCEPTED, PENDING, UNKNOWN };

//////////////////////////////////////////////////////////
//                       class Edge
//////////////////////////////////////////////////////////
class Edge {
public:
	
	// ** variables **
	Vertex *_a,*_b;
	Vec3d dir; // unit direction vector
	bool _horizontal, _vertical;
	faceVector _faces;
	int _id;
	double _weight;
	bool _visible;
	double _sharpness;
	bool _matched; // used during RANSAC matching
	bool _keep; // boolean for destructor
	bool _flat; // true if the edge is flat
	int _correspondenceId; // index in the table of correspondences
	Status status; // maintenance status

	// ** methods **
	Edge ();
	~Edge();
	Edge (Vec3d a, Vec3d b);
	Edge (Vertex *_a, Vertex *_b) ;
	Edge (int _id, Vertex *_a, Vertex *_b) ;
	Edge (Vertex _a, Vertex _b);
	Edge (int _id, Vertex _a, Vertex _b);
	Edge (int id, Vec3d a, Vec3d b);
	Edge (int id, Vec3d a, Vec3d b, bool visible);
	Vec3d getA() {return _a->getPosition();}
	Vec3d getB() {return _b->getPosition();}
	void setA( Vec3d a ) { _a->_position = a; }
	void setB( Vec3d a ) { _b->_position = a; }
	void info();
	Vec3d middlePoint() {return (_a->getPosition()+_b->getPosition())/2.0;}
	bool belongs (edgeVector E) ;
	bool equal (Edge *e) ;
	int findEdgeInList (edgeVector E) ;
	void rotateMidpoint (double alpha, Vec3d direction);
	void insert (edgeVector &E) ;
	int Insert (edgeVector &E, Edge *e) ;
	Vec3d centroid ();
	void printGeometry();
	void print();
	void print(FILE *out);
	void print(bool);
	void print(FILE *out,bool);
	bool read (FILE *fin, vertexVector &V, faceVector &F);
	bool operator==(const Edge e) const ;
	bool operator< (Edge &edge) {return (_weight > edge._weight);}
	void computeWeight (Vec3d position);
	double length();
	void draw(const float color[3], int linewidth);
	void draw(const float color[3]) {draw(color,1);}
	void glVertex();
	double angle(Edge edge);
	bool closest_approach(Edge edge, Edge &closest, bool &skew);
	bool skewness (Edge edge, Edge &closest, double &skewness);
	Vec3d toVector();
	void clear();
	void scale(double scale);
	double sharpness();
	bool sharp();
	bool testSkew (Edge *edge, double angular_threshold);
	bool testSkew (Edge *edge1, Edge *edge2, double angular_threshold);
	bool testPosition (Edge *edge, Vec3d &position, double angle_threshold);
	double shortest_distance( Vec3d p );


};

namespace std 
{
	struct greater<Edge*>
	{
		bool operator() (Edge const* e1, Edge const* e2) 
		{
			if (!e1)
				return false;
			if (!e2)
				return false;
			return (e1->_weight > e2->_weight);
		}
	};
};

enum EdgeType {DETECTED, CHAINED, REPROJECTED, CORRESPONDENCE};

class Edge2D {
public:
	
	// ** variables ** //
	Vec2d _a,_b;
	int _id;
	int _edgeplaneId; // edgeplane to which the 2D edge corresponds
	int _lineId;
	EdgeType _type;

	// ** methods ** //
	Edge2D() {_a = Vec2d(); _b = Vec2d(); _id = 0; _edgeplaneId = _lineId = -1; _type = DETECTED;}
	Vec2d midpoint() {return (_a+_b)/2;}
	Edge2D (Vec2d a, Vec2d b) { _a=a; _b=b; _id=-1; _edgeplaneId = _lineId = -1; _type = DETECTED;}
	Edge2D (Vec2d a, Vec2d b, int id) { _a=a; _b=b; _id=id; _edgeplaneId = _lineId = -1; _type = DETECTED;}
	double length() {return len(_a-_b);}
	Vec2d toVec() {return _b-_a;}
	double Dot (Edge2D edge) {return dot(toVec(),edge.toVec());}
	bool operator==(Edge2D edge) {return (((_a == edge._a) && (_b == edge._b)) || ((_b == edge._a) && (_a == edge._b)));}
	void print() {LOG(LEVEL_INFO, "%f,%f - %f,%f\n",_a[0],_a[1],_b[0],_b[1]);}
	void print( FILE *f ) { fprintf(f, "%f %f %f %f %d ", _a[0], _a[1], _b[0], _b[1], _id ); }
	bool read ( FILE *f ) { double a0, a1, b0, b1; int id; if ( fscanf(f, "%lf%lf%lf%lf%d", &a0, &a1, &b0, &b1, &id) != 5 ) return false;
	_a[0] = a0; _a[1] = a1; _b[0] = b0; _b[1] = b1; _id = id; return true;}

	double cosangle (Edge2D edge) {return dot(norm(toVec()),norm(edge.toVec()));}
	Edge2D flipOpenGL (double height) {return Edge2D(Vec2d(_a[0],height-_a[1]), Vec2d(_b[0],height-_b[1]), _id);}
	void flipEdgeDetector (double length);
	void flipHorizontal (double length);
	void flipVertical (double length);
	void glVertex() {glVertex2f(_a[0],_a[1]); glVertex2f(_b[0],_b[1]);}
	void draw(const float color[3],int);
	bool remove (edge2DVector &edges);
	bool connected (Edge2D &edge, double threshold_dist, double threshold_angle);
	bool checkInside (int x0, int y0, int x1, int y1);
	bool clip (int width, int height);
	bool intersect (Edge2D edge, Vec2d &result);
	bool operator< (Edge2D &edge) {return (length() < edge.length());}
	double overlap (Edge2D,double);
	bool testSimilarity (Edge2D edge, double angle_threshold, double dist_threshold);
	void scale(int x, int y, double zoom);
	bool checkBorder (int width, int height, double threshold);
	Vec3d normal();
	double distance (Vec2d m);
	double distance (Edge2D edge);
	Vec3d getLine();
	void roundInt(int w, int h);
	bool intersectSegments (Vec2d a, Vec2d b, double slope, double offset, Vec2d &new_point);
	void extend( Edge2D edge );
	double p2p_distance ( Edge2D &edge, Vec2d &a, Vec2d &b );

};

//////////////////////////////////////////////////////////
//                       class EdgeFeature
//////////////////////////////////////////////////////////
class EdgeFeature {
public:
	int _id; // ID
	int _sensorId; // camera sensor ID
	Edge2D _edge; // 2D edge on the image
	Vec3d _cop; // camera center of projection
	
	EdgeFeature() {_id=0; _sensorId=0; _edge = Edge2D();}
	EdgeFeature (int id, int sensorId, Edge2D edge, Vec3d cop) {_id=id; _sensorId=sensorId; _edge=edge; _cop=cop;}
	
};

typedef std::vector<EdgeFeature> edgeFeatureVector;


//////////////////////////////////////////////////////////
//                       class Face
//////////////////////////////////////////////////////////
class Face {
public:
	
	// ** variables **
	vertexVector _vertices;
	Vec3d _norm;
	int _id;
	bool _visible;
	bool _matched;
	float color[3]; // synthetic color
	bool _regular;

	std::vector<Vec3d> subdivision; //

	// ** methods **
	Face::Face() { color[0] = 0.0; color[1] = 0.0; color[2] = 0.0;};
	~Face() {};
	Face::Face( int id );
	Face::Face(int _id, Vertex *v1, Vertex *v2, Vertex *v3, Vertex *v4);
	Face::Face(int _id, Vertex *v1, Vertex *v2, Vertex *v3);
	bool equal (Face *f);
	bool belongs (faceVector vector);
	Face::Face (vertexVector vector, int faceId);
	Vec3d centroid ();
	Vec3d normal ();
	void print();
	void print(FILE *out);
	bool read (FILE *fin, vertexVector &V);
	void draw(bool fill, const float color[3]);
	void draw2D(bool fill, const float color[3]);
	void getAdjacentEdges( edgeVector &edges );
	bool inside( Vec3d point );
	bool equivalent( Face *f );
	double Face::area();
	bool occluding( Vec3d point, Vec3d view_point );
	
	bool Face::worldToBarycentric( Vec3d p, Vec2d &pbar );
	bool Face::barycentricToWorld( Vec2d pbar, Vec3d &point );
	bool Face::barycentricToIndex( Vec2d pbar, int level, int &i, int &j );
	bool Face::indexToBarycentric( int level, int i, int j, Vec2d &pbar ) ;
	void Face::subdivide( double resolution );
	bool Face::getSubdivisionVertices ( int id, double resolution, Vec3d &a, Vec3d &b, Vec3d &c, Vec3d &d );
	int Face::getSubdivisionIndex( Vec3d point, double resolution ) ;
	void Face::regular();
	int subdivisionSize( double resolution );

};

class DrawingEdge {
public:
	Vec3d p1,p2;
};

struct ModelBoundingBox {
	double diam;
	Vec3d box;
	double minX,maxX,minY,maxY,minZ,maxZ; // bounding box of the model
};

enum InitMode { LINE_LINE, ROTATION_VOTING, CORNERS, VANISHING_POINTS, EXHAUSTIVE_SEARCH, INIT_VERTICAL, NONE };

class ExtrinsicParameters {
public:

	ExtrinsicParameters() { translation = Vec3d(0,0,0); quaternion = Quaternion(); score = 0; mode = NONE; time ( &ctime ); id = -1;}
	Quaternion getRotation() {return quaternion;}
	Mat3d getRotationMatrix() {return quaternion.toRotationMatrix();}
	void setRotation (Quaternion q) {quaternion = q;}
	Vec3d getTranslation() {return translation;}
	void setTranslation (Vec3d t) {translation = t;}
	void setHeight( double h ) { translation[2] = h; }
	double getHeight() { return translation[2];}

	void print() {LOG(LEVEL_INFO, "POSE: %.3f %.3f %.3f   ",getTranslation()[0],getTranslation()[1], getTranslation()[2]); getRotation().print();}
	double distance_trans( const ExtrinsicParameters &p );
	double distance_rot( const ExtrinsicParameters &p );
	Vec3d getZ() { return getRotation().rotate( Vec3d(0,0,1) ); }
	bool write( FILE *f );
	bool read( FILE *f );

	double translation_error ( ExtrinsicParameters &pose ); // return the translation error between two poses
	double rotation_error ( ExtrinsicParameters &pose ); //return the rotation error between two rotations IN DEGREES

	void update_time() { time ( &ctime );}

	InitMode	get_mode()	{ return mode; }
	void		set_mode( InitMode m ) { mode = m;}

	std::string get_mode_string();
	time_t get_ctime() { return ctime;}

	double get_score() { return score; }
	void set_score(double s) { score = s;}

	bool operator<(const ExtrinsicParameters &p) const { return score > p.score;}

	int id; // used for clustering

protected:
	Vec3d translation;
	Quaternion quaternion;
	InitMode mode;
	double score;
	time_t ctime; // creation time for performance measurement
};

//static bool greater_pose( const ExtrinsicParameters &p1, const ExtrinsicParameters &p2 )
//{
//	return ( p1.score > p2.score );
//}

class EdgePlane {
public:
	Vec3d _a, _b, _s;
	Vec3d _normal;
	int _cameraId; // on which camera
	intVector _edgeIds; // from which 2D edges ( chaining => multiple 2D edges)
	int _chaineId; // what is the ID once chained
	int _uid; // unique ID
	double _length;
	bool _matched; // used during RANSAC matching

	Vec3d l_avg, r_avg, l_dev, r_dev; // color information in HSV format, left and right, average and std dev

	void print();
	bool operator<(EdgePlane &p) {return (_length>p._length);}
	bool operator==( const EdgePlane &c);

	EdgePlane();
	EdgePlane(Vec3d a, Vec3d b, Vec3d s, int cameraId, int edgeId, int uid);
	double score_alignement (EdgePlane ep);
	double score_alignement (Edge *line);
	Vec3d averageNormal (EdgePlane edge);
	bool testFrontality(Edge *edge);
	double distance (Edge *edge);
	double score (Edge *edge) {return fabs(dot(_normal,edge->dir));}
	bool merge (EdgePlane *edge);
	void findEndPoints(EdgePlane *edge, Vec3d &s, Vec3d &e);
	double separationAngle (EdgePlane *edge, Vec3d normal);
	double length();
	void print( FILE *f ) { fprintf(f, "%f %f %f %f %f %f %f %f %f %f %f %f %d %f", _a[0], _a[1], _a[2], _b[0], _b[1], _b[2], _s[0], _s[1], _s[2], 
		_normal[0], _normal[1], _normal[2], _cameraId, _length); }
	bool read ( FILE *f ) { double a0,a1,a2,b0,b1,b2,s0,s1,s2,n0,n1,n2,l; int id; if ( fscanf(f, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d%lf", 
		&a0,&a1,&a2,&b0,&b1,&b2,&s0,&s1,&s2,&n0,&n1,&n2,&id,&l) != 14 ) return false;
		_a[0] = a0; _a[1] = a1; _a[2] = a2; _b[0] = b0; _b[1] = b1; _b[2] = b2; _s[0] = s0; _s[1] = s1; _s[2] = s2; _normal[0] = n0; _normal[1] = n1;
		_normal[2] = n2; _cameraId = id; _length = l; return true;}

	void flip();
	double colorDistance( EdgePlane &ep );

	double solidAngle();
	double separation(EdgePlane *edge);
	double scoreAlignment (EdgePlane *edge);
	double span (EdgePlane *edge);
	bool overlap (EdgePlane *edge);
	double EdgePlane::overlap( EdgePlane &ep );
	double score_overlap ( EdgePlane &ep );
	double overlap (Edge *edge);
	double thickness (EdgePlane *edge);
	void fromCameraFrameToWorldFrame (Vec3d translation, Quaternion rotation);
	void fromWorldFrameToCameraFrame (Vec3d translation, Quaternion rotation);
	void fromCameraFrameToWorldFrame (ExtrinsicParameters came);
	void fromWorldFrameToCameraFrame (ExtrinsicParameters came);
	void rotate (Quaternion rotation);
	bool poles( EdgePlane ep, Vec3d &pole_1, Vec3d &pole_2 );

	void rotate(Vec3d axis, double angle);
	double angle (Edge *edge);
	double EdgePlane::angle( EdgePlane &plane );
	double angle() {return fabs(Acos(dot(_a,_b)));}
	bool minMaxPolarAngles( EdgePlane &ep, double &min1, double &max1, double &min2, double &max2 );
	double oriented_angle( EdgePlane &plane );
};

typedef std::vector<EdgePlane> edgePlaneVector;
typedef std::vector<EdgePlane>::iterator edgePlaneVectorIter;

typedef std::pair<Edge*,EdgePlane> Correspondence;
typedef std::vector<Correspondence> CorrespondenceVector;

typedef std::pair<Vec3d,int> vp; // vanishing point (a unit vector and a cluster ID)
typedef std::vector<vp> vp_vector; // vanishing point vector

typedef struct { int face_id; int i; int r, g, b; int hits; } color_hit;

typedef struct { Vec3d cc,a,b,c,d,n,o,c1,c2; } anchor; // an anchor is defined by an origin <cc> and a target (o,a,b,c,d) with a normal <n>

//////////////////////////////////////////////////////////////
// 
// a correspondence between a model line and an observed edge
//

enum corresStatus { _CONNECTED, _EXPECTED, _BLACKLISTED };

class corres {
public:

	Edge *line;
	edgePlaneVector eps;
	int age;
	int _id;
	int eid;
	doubleVector scores;
	double max_score;
	int notseen;
	int best_ep;
	double length; //length of the edge plane

	corres() {line = NULL; age = -1; _id = -1; max_score = -1.0; _status = _EXPECTED; best_ep = -1; eid=-1; notseen=0; length = 0.0;}
	corres( Edge *l) {line = l; age = -1; _id = -1; max_score = -1.0; _status = _EXPECTED; best_ep = -1; eid=-1; notseen=0;length = 0.0;}
	corres( Edge *l, EdgePlane p, corresStatus status) {line = l; eps.push_back(p); _status = status; age = 0; _id = -1; max_score = -1.0; best_ep = -1;eid=-1; notseen=0;length = 0.0;}
	corres( Edge *l, EdgePlane p, corresStatus status, double s) {line = l; eps.push_back(p); _status = status; age = 0; _id = -1; scores.push_back(s);max_score = -1.0; best_ep = -1;eid=-1; notseen=0;length = 0.0;}
	corres( Edge *l, EdgePlane p, corresStatus status, int _age, double s) {line = l; eps.push_back(p); _status = status; age = _age; _id = -1; scores.push_back(s);max_score = s; best_ep = -1;eid=-1; notseen=0;length = 0.0;}
	corresStatus status() { return _status; }
	void status( corresStatus status ) { _status = status; age = 0; }
	void print();
	void draw ( ExtrinsicParameters pose, const float color[3] );
	void drawInfo( int x, int y, const float color[3] );
	bool operator==( const corres &c );
	bool operator<(const corres &c);
	double computeScore( ExtrinsicParameters &pose );
	EdgePlane head() { return( eps.empty() ? EdgePlane() : eps[0]);}
	void clear() {eps.clear(); scores.clear(); age = 0;}
	bool valid() { return !eps.empty();}
	void update_eid( EdgePlane &ep ); // determine which edegplane among eps looks the most like ep
	void select_eid(); // select a new edgeplane (presumably for a new correspondence)

protected:
	corresStatus _status;
};

// light structure of a correspondence for pose computation
// i: index in vector cq (see state.h)
// j: number of edgeplanes in the correspondence (zero if no observations or if correspondence hasn't been computed yet)
// k: type of correspondence ( 0 = connected, 1 = expected, 2 = blacklisted)
// d: 1 if the correspondence has been updated, 0 otherwise
// age: age of the correspondence
//
class cr_pose {
public:
	int i, j, k, d, age;

	cr_pose( int _i, int _j, int _k, int _d, int _age ) { i=_i; j=_j; k=_k; d=_d; age=_age;}
	bool operator <(const cr_pose &t) const {
		return ( (k < t.k) || !(k < t.k) && (age > t.age) );
	}
};

static bool cr_pose_invalid( const cr_pose &t ) { return ( (t.d == 1) && ( t.j == 0 ) ); }
static bool corres_invalid( const corres &t ) { return t.eps.size() == 0; }
static bool corres_removed( const corres &t ) { return t.line->status == PENDING; }

typedef std::vector< corres > corresVector;

#endif
