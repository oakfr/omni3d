#ifndef _MODEL_H__
#define _MODEL_H__

#include "basic.h"
#include "geom/geom3D.h"
#include "util/util.h"
#include "viewer/viewer.h"

#define MODEL_ROOT "C:\\omni3d\\models"
//#define MODEL_FILE "model.log"
#define MODEL_GEOM_DIR "geom"
#define MODEL_VISI_DIR "visibility"
#define MODEL_VERTEX_FILE "geom\\vertices.txt"
#define MODEL_EDGE_FILE "geom\\edges.txt"
#define MODEL_FACE_FILE "geom\\faces.txt"
#define MODEL_INFO_FILE "geom\\info.txt"
#define MODEL_VISI_FILE "visibility\\edges.txt"
#define MODEL_SIGNATURE_FILE "visibility\\signature.txt"
#define MODEL_VISI_SUB_FILE "visibility\\subedges.txt"
#define MODEL_VISI_LIST_FILE "visibility\\pointers.txt"
#define MODEL_SIGN_LIST_FILE "visibility\\list_sign.txt"
#define MODEL_INV_LIST_FILE "visibility\\visibility_inv_clique.txt"
#define MODEL_NODE_FILE "visibility\\nodes.dat"
#define MODEL_LIST_FILE "model.list"

#define MODEL_CONFIG_FILE "config.ini"
#define MODEL_SIGN_FILE "visibility\\sig.txt"

#define TEXTURE_WIDTH  512
#define TEXTURE_HEIGHT 512

enum ModelType {AUTOCAD, INVENTOR};

enum LengthUnit { UNIT_INCH, UNIT_FOOT, UNIT_NONE };

//////////////////////////////////////////////////////////////////////////
//      class Clique
//
//  a clique of 3 3D lines (used for localization)
//
// a clique is OK iff (1) the 3 lines are skew pair-wise and (2) each reprojected line belongs to a different camera
// a clique is valid iff it is OK and passes the POSE_TEST and RANSAC_TEST successfully
// a clique cannot be valid without being OK.
/*class Clique;
typedef std::vector< Clique > cliqueVector;

class Clique {
public:
	Clique();
	Clique( Edge *e1, Edge *e2, Edge *e3, EdgePlane ep1, EdgePlane ep2, EdgePlane ep3, Vec3d &position, double threshold );
	Clique( corres ca, corres cb, corres cc, Vec3d &position, double threshold );
	bool valid() {return _valid;}
	void valid( bool v ) { _valid = v; }
	bool ok() {return _ok;}
	void ok( bool o ) { _ok = o; }
	double getQuality() {return quality;}
	void setQuality( double val ) {quality = val;}
	void draw (Viewer &viewer, ExtrinsicParameters pose, const float color[3]);
	void drawLines (Viewer &viewer, const float color[3]);
	void drawEdges (Viewer &viewer, ExtrinsicParameters pose, const float color[3]);
	bool checkOK( double );
	bool shareTwoLines( Clique &clique );
	bool accepted( cliqueVector &cliques );
	double reprojectionError( ExtrinsicParameters &pose );

	Edge *a, *b, *c;
	EdgePlane ea, eb, ec;

protected:
	bool _valid; 
	bool _ok; // true if skew test and geom test are passed
	double quality; // scores how good this clique is for localization
};
*/

//////////////////////////////////////////////////////////////////////////
//      class Model
//
//  a container for the 3D model (vertices, lines, faces)

class Model {
public:

	edgeVector _edges;
	vertexVector _vertices;
	faceVector _faces;
	
	Edge** _edge_table; // a table for which brings IDs and edges in correspondence - used in getEdge
	int _maxEdgeId;
	Vec3d _centroid;
	double _diameter;
	std::string _dirname;
	ModelType _type;
	std::string MODEL_FILE;
	Vec3d _bounding_box[2];
	double NORTH_ANGLE;
	long idum; // seed for random numbers generation
	int _global_offset;

	GLuint *_idtextures;
	IplImage *_textureImage;

	std::vector< color_hit >  _color_hits;
	int _lod; //level of details for colors

	void Model::writeColorHits();
	void Model::readColorHits();
	void Model::insertColorHit ( color_hit hit );
	//void Model::processColorHits();
	void Model::clearColorHits();
	void Model::drawColorHits();

	intVector _color_offset;
	double _color_resolution;

	Vec3d _homography_translation;
	Quaternion _homography_rotation;
	
	void Model::drawFaces ( bool texture );
	void Model::readTextures();
	void Model::readTextures( int id, Face *f );
	bool _read_texture; // true once the texture has been read

	double MODEL_RESOLUTION; //in model unit (inches, feet)
	double MODEL_MIN_EDGE_ANGLE; //min angle between adjacent faces

	Model() {_edge_table = NULL; NORTH_ANGLE = 0.0; MODEL_RESOLUTION = 1.0; idum = -1; _read_texture = false; _idtextures = NULL;};
	~Model();

	Vertex* Insert(Vec3d v);
	Vertex* Find(Vec3d v, vertexVector V);
	Edge* Insert(Vec3d a, Vec3d b);
	Edge* Insert(Vertex *a, Vertex *b, Face *f);
	Edge* Insert(Vec3d a, Vec3d b, Face *face);
	Face* Insert( Vec3d a, Vec3d b, Vec3d c );
	Face* Insert (Vertex *a, Vertex *b, Vertex *c);
	Edge* Insert(Vec3d a, Vec3d b, int id);
	Face* Insert (Vec3d a, Vec3d b, Vec3d c, Vec3d d);
	Face* Insert (Vertex *a, Vertex *b, Vertex *c, Vertex *d);
	Edge* Find (Vec3d a, Vec3d b, edgeVector V);
	void InsertCube (Vec3d origin, Vec3d dir1, Vec3d dir2, double l1, double l2, double l3);
	void InsertCube (std::vector<Vec3d> points, int index);
	void print();
	int readFile(const char *filename, const char*);
	int readAutocadFile(const char *filename, const char*);
	int readInventorFile (const char *filename, const char *dirname);
	int writeGeometry(const char*);
	int readGeometry(const char*);
	bool read (const char *dirname, ModelType type);
	Vertex* getVertex (int id);
	Edge* getEdge (int id);
	Face* getFace (int id);
	void computeCentroidAndDiameter();
	void reset();
	void clear();

	void Model::removeVertex( Vertex *v );
	void Model::removeEdge(Edge *e);
	void Model::removeFace( Face *f );
	void Model::mergeFaces();
	void Model::removeVertexDuplicates();
	void Model::removeEdgeDuplicates();
	void Model::mergeEdges();
	void Model::removeFaceDuplicates();
	void Model::cleanupModel();
	
	void updateModel();
	bool isOutside (Vec3d point);
	std::string toFilename (const char *name);
	void cleanup(bool geometry, bool nodes);
	double dimX() {return _bounding_box[1][0] - _bounding_box[0][0];}
	double dimY() {return _bounding_box[1][1] - _bounding_box[0][1];}
	double dimZ() {return _bounding_box[1][2] - _bounding_box[0][2];}
	void buildConnectivity();
	Vec3d random_position( double min_height, double max_height );

	void	make_icosahedron ( std::vector< Vec3d > &points );
	void	tessellate ( double resolution, std::vector< Vec3d > &points );
	Vertex* closest_vertex( Vec3d p );
	void	closest_cells( Vec3d p, intVector &cells );
	int		closest_cell( Vec3d p );

};

#endif
