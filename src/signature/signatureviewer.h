#ifndef _SIGNATUREVIEWER_H__
#define _SIGNATUREVIEWER_H__

#include "basic.h"
#include "geom/geom3D.h"
#include "model/model.h"
#include "signature/visibility.h"
#include "viewer/viewer.h"
#include "signature/signature.h"
#include <FL/Fl_Progress.H>
#include <FL/Fl_Gl_Window.h>
#include <Fl/Fl.h>
#include <FL/Fl_Menu_Bar.H>
#include <Fl/Fl_Double_Window.h>
#include <Fl/Fl_Window.h>


#define N_BEST_MATCHES 10 // the N best matches for signature matching
// node jump counter mechanism:
// to avoid multiple node jumps, the LUT declares a node jump only if the position 
// is consistently in the new node for FRAMES_JUMP frames
#define FRAMES_JUMP 10 // number of consecutive frames in a new node to declare a node jump

#define MIN_DISTANCE_RECOMPUTE 40.0 // minimum distance that has to be walked by the camera for visibility set to be recomputed

class SignatureHit {
public:
	Vec3d _p;
	double _score;

	SignatureHit() {};
	SignatureHit (Vec3d p, double s) {_p = p; _score = s;};
	bool operator< (SignatureHit &hit) {return (_score<hit._score);};
};

typedef std::vector<SignatureHit> signatureHitVector ;

class Node;

typedef std::vector< Node* > nodeVector;

class Node {
public:
	int _id; // unique ID
	Vec3d _pos; // 3D position
	long _file_pos; // file position
	long _signature_pos; // signature file position
	intVector _ns; // list of neighbor IDs
	intVector _rns; // list of real neighbors (taking occlusion into account)
	double affinity; // affinity with a camera signature
	bool visited;

	// constructors
	Node() { _id = -1; _pos = Vec3d(0,0,0); _file_pos = -1; affinity = 0.0; visited = false;};
	Node( Vec3d pos, int id ) { _pos = pos; _id = id; _file_pos = -1; affinity = 0.0; visited = false;};
	~Node() {};

	// methods
	bool Node::Read( FILE *f );
	bool Node::Write( FILE *f );
	bool Node::ReadVisibilitySet( Histogram &hist, edgeVector &lines, faceVector &faces, Model *model );
	bool Node::WriteVisibilitySet( edgeVector &lines, faceVector &faces, Model *model );
	bool Node::HasNeighbor( nodeVector &nodes, Vec3d position );
	void Node::InsertNeighbor( int id );
	void print();
};

typedef std::vector< Node* > nodeVector;
typedef std::vector< Node* >::iterator nodeVectorIterator;

namespace std 
{
	struct greater<Node*>
	{
		bool operator() (Node const* e1, Node const* e2) 
		{
			if (!e1)
				return false;
			if (!e2)
				return false;
			return (e1->affinity > e2->affinity);
		}
	};
};

class Region {

public:
	Region() { _score = 0; }

	nodeVector _nodes;
	double _score;
	ExtrinsicParameters _best_pose;

	void draw( Viewer &viewer, const float color[3], double blending );
};

typedef std::vector< Region > regionVector;

static bool greater_region( const Region &r1, const Region &r2 )
{
	return ( r1._score > r2._score );
}

////////////////////////////////////////////////
// the main lookup table class
//
class LUT {
public:
	
	LUT();
	~LUT() {};
	
	//** attributes **//
	Model *_model;
	Viewer _viewer;
	Viewer _viewer_3d;
	Viewer _viewer_3d_topview;
	Viewer _viewer_sig;
	Viewer _viewer_3dsubmap;
	Viewer _viewer_match;
	bool _draw_grid, _draw_pixels;
	int nframes;
	nodeVector nodes;
	regionVector regions;

	Node *_node; // current node
	int _depth; // depth in the node graph
	Histogram _node_histogram; // node histogram
	//LineFilter _line_filter; // a filter for visible lines

	int WIDTH,HEIGHT,GRID_SIZE_X,GRID_SIZE_Y;
	int REGION_SIZE; // integer specifying the size of a region (in terms of neighborhood)
	LengthUnit LENGTH_UNIT;

	ModelType TYPE;

	Vec3d _position;
	double GRID_STEP,EDGE_STEP,DEPTH_OF_FIELD;
	
	vp_vector _vps; // vanishing points

	int _window;
	edgeVector _lines; // visible lines
	faceVector _faces; // visible faces
	vertexVector _vertices; // visible corners

	edgeVector _visible_lines_horizontal, _visible_lines_vertical;//, ;
	Signature _signature;
	Signature _signature0;
	bool _fft;
	signatureHitVector _best_matches;
	double _currentScore; // this variable is for display only
	double _scaleFactor;
	bool _valid;
	edgeVector _selected_lines,_selected_lines_sphere;
	edgeVector _selected_lines_sphere_1;
	edgeVector _selected_lines_sphere_2;
	std::vector<Vec3d> _computed_positions;
	GLfloat *_pixels;
	GLubyte *_cpixels;
	bool _grid_computed;

	double LUT::inchToModelUnit( double length );

	std::map<std::pair<int,int>, double> book_min_dihedral; // keep a book of min and max dihedral angle between model lines
	std::map<std::pair<int,int>, double> book_max_dihedral; 
	std::map<int, double> book_max_subtended; // keep a book of max subtended angle of model lines
	void min_max_dihedral_angle( Edge *a, Edge *b, double h1, double h2, double &min_dihedral_angle, double &max_dihedral_angle, double &max_polar_angle );
	void max_subtended_angle( Edge *a, double h1, double h2, double &max_subtended_angle );
	void clear_dihedral_books() { book_min_dihedral.clear();book_max_dihedral.clear(); book_max_subtended.clear();} 

	///////////////////////////////////////////////////////
	// history
	//
	// store the history of the LUT table positions (for display)
	//
	std::vector< Vec3d > history;

	//** methods **//
	void resetFiles(bool new_geometry);
	bool lookup ( Vec3d position, double min_subtended_angle, double min_dihedral_angle, int max_lines, int depth );
	bool lookup( Node *node, double min_subtended_angle, double min_dihedral_angle, int max_lines, int depth ) ;
	void drawScene ();
	void init(int w, int h, std::string dirname);
	void model_picking_3d (int x, int y, bool wireframe, bool depth_test);
	void drawScene_3d(bool wireframe, bool depth_test, bool draw_visible_lines, bool texture, bool line_color_coding, int cameraId);
	void drawScene_3d_topview();
	void drawScene_3dsubmap();
	void keyboard ( unsigned char key, int x, int y );
	void mySpecialKeyFunc( int key, int x, int y  );
	void mouse_3d (int button, int state, int x, int y, int glutModifier);
	void mouse_3d_topview (int button, int state, int x, int y);
	void motion_3d (int x, int y);
	void motion_3d_topview (int x, int y);
	void automate(Fl_Progress *progress);
	void selectRandomLines (int N, edgeVector &select_lines, bool testSkew);
	void drawHistory( Viewer &viewer, const float color[3], int size );
	void LUT::computeVisibleSet ( Node *node, nodeVector &neighbors, int runid );
	bool LUT::compute( Node *node );
	bool LUT::valid_pose( ExtrinsicParameters &pose );
	Node* LUT::find_closest_node( Vec3d position );

	void clear();

	int LUT::checknodes ();
	void LUT::drawNode( Viewer &viewer, const float color1[3], const float color2[3], Node *node, bool active );
	void LUT::drawNodes( Viewer &viewer, const float color1[3], const float color2[3] );
	void LUT::computeNodes();
	int LUT::initNodes();
	int LUT::readNodes();
void LUT::findNodeNeighbors( Node *node, int depth, nodeVector &neighbors, bool active );
	void LUT::printNodes();

	void LUT::computeVanishingPoints();

	void LUT::InsertLines( edgeVector &src, edgeVector &dest );
	void LUT::InsertFaces( faceVector &src, faceVector &dest );
	void setLines( Status status );
	void LUT::resetLineWeights();

	void LUT::computeAngleHistogram( Node *node, Histogram &histogram );
void LUT::computeAngleHistogram( Vec3d position, edgeVector &lines, Histogram &histogram );

void LUT::sortNodes( Histogram &hist, nodeVector &nodes );

	void LUT::setupRegions();

};

// a structure for face visibility computation
struct FaceVis {
	int id;
	double min_z, max_z;
};

#endif
