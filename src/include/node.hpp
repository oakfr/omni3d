#include "basic.h"
#include "geom/geom3D.h"
#include "util/util.h"

#ifndef _SIGNT_H
#define _SIGNT_H

class SignatureEdge;
class Node;
class Grid;
class DeltaSignature;
class Pair;
class PairVector;

typedef std::vector< Node* > nodeVector;
typedef std::vector< SignatureEdge > signatureEdgeVector;
typedef std::vector< Pair > pairVector;

class Pair {
public:
	int _a,_b;
	
	Pair() {_a=0;_b=0;}
	Pair(int a, int b) {_a =a; _b=b;}

	bool isValid() { return (_a<_b);}	
	bool exist (pairVector vector);
	void display() {display(stdout);}
	void display (FILE *out) { fprintf(out,"%d-%d ",_a,_b);}
bool operator== (Pair &p);
bool operator> (Pair &p) ;
bool operator< (Pair &p) ;
PairVector Pair::subtract (Pair p);
PairVector Pair::subtract (PairVector vector);
};

class PairVector {
public:
	pairVector _pairs;

	PairVector() {};
	PairVector(int _a, int _b) {_pairs.push_back(Pair(_a,_b));}
	PairVector::PairVector(int _a, int _b, int _c, int _d); 
	int size() {return _pairs.size();}
	bool empty() {return _pairs.empty();}
	Pair last() { if (empty()) return Pair(0,0); else return _pairs[size()-1];}
	void clear() {_pairs.clear();}
	void merge();
	void insert(Pair pair);
void add (PairVector p);
Pair operator[] (int i) ;
bool PairVector::isValid();
PairVector PairVector::subtract (PairVector vector);
void display();
void display(FILE *out);
};

//////////////////////////////////////////////////////////
// class SignatureEdge
//////////////////////////////////////////////////////////
class SignatureEdge {
public:
  /// ** variables ** ///
  PairVector _items;
  int _id;
  
  /// ** methods ** ///
  SignatureEdge() {};
  //  SignatureEdge (int _id, Vertex _subId_a, Vertex _subId_b);
  SignatureEdge (int id, int _a, int _b);
  SignatureEdge (int id) {_id = id;}
  SignatureEdge (int id, PairVector items) {_id = id; _items = items;}
  void SignatureEdge::display();
//  void SignatureEdge::set (int _id, Vertex _subId_a, Vertex _subId_b);
  //void set (int id, int a, int b);

  Pair getItem(int i) {return _items._pairs[i];}
  bool exist (signatureEdgeVector vector);
  bool exist (Pair pair);
  void insert (Pair pair) {_items.insert(pair);}
void SignatureEdge::display(FILE *out);
};

//////////////////////////////////////////////////////////
// class Signature
//////////////////////////////////////////////////////////
class Signature {
public:
	/// ** variables ** ///
	//vertVector _vertices;
	intVector _vertices;
	signatureEdgeVector _edges;

	/// ** methods ** ///
	Signature() {};
	void clear() {_vertices.clear(); _edges.clear();}
	void display();
	void display(FILE *out);
	//void Insert (Vertex vertex);
	void insert (int id);
	//void Insert (int edgeId, Vertex subId_a, Vertex subId_b);
	void insert (int id, Pair pair);
void Signature::insert (int _id, PairVector pairs);
void Signature::insert (SignatureEdge se) {insert(se._id,se._items);}
void Signature::translateData (vertVector &vertices, edgeVVector &edges, vertexVector &V, edgeVector &E, int SUBDIVISION_RESOLUTION);

	bool exist (int vertexId);
	//  void Signature::Insert (int _id, signatureEdgeVector vect);
	//int  findVertex (int vertexId);
	//int  findEdge (int edgeId);
	//int  findSubEdge (SignatureEdge *sign, int sub_a, int sub_b);


};

//////////////////////////////////////////////////////////
// class DeltaSignature
//////////////////////////////////////////////////////////
class DeltaSignature {
public:

	/// ** variables ** ///
	Signature _positive;
	Signature _negative;

	void clear() {_positive.clear(); _negative.clear();}
	void DeltaSignature::opposite();
	/// ** methods ** ///
};

//////////////////////////////////////////////////////////
// class Node
//////////////////////////////////////////////////////////
class Node {
public:
  /// ** variables ** ///
  int _id;
  int _i,_j;
  Vec3d _position;
  long _file_position;
  Signature *_signature;
  nodeVector _neighbors;
  bool _visited;
  intVector _neighborIds;
  
  /// ** methods ** ///
  Node();
  Node(int i,int j, double x, double y, double z,int _id, long file_position);
  ~Node();
  void display();
  void display(FILE *out);
  long writeNodeData (const char *filename, const char *mode);
  void readNodeData(const char *filename, long pos);
  void transData (vertVector &V_out, edgeVVector &E_out);
  long writeNodeDelta (const char *filename, const char *mode, Node *neighbor, DeltaSignature delta);
  int readNodeDelta(const char *filename, DeltaSignature &delta, long int pos);
  DeltaSignature computeDelta (Node *neighbor);
  Node* Node::getNextNeighbor (Node *neighbor);
};

class Nodelist {
public:
};

//////////////////////////////////////////////////////////
// class Grid
//////////////////////////////////////////////////////////
class Grid {
public:
  /// ** variables ** ///

  const char *node_filename, *grid_filename, *node_delta_filename, *grid_delta_filename;

  nodeVector nodes;
  faceVector _F;
  edgeVector _E;
  vertexVector _V;
  int _window;

  /// ** methods ** ///
  Grid() {};
  Grid (const char *f1, const char *f2, const char *f3, const char *f4);
  Grid (vertexVector V, edgeVector E, faceVector F, int window);
  void insertNode (Node *node);
  void writeGridData (const char *filename,const char *mode,Node *node,long pos);
  void Grid::writeGridDelta(const char *filename, const char *mode,Node *node, Node *neighbor, long pos);
  long int Grid::readGridDelta (const char *filename, Node *node1, Node *node2);
  int readGridData (const char *gridFilename, FPositionList &fpos);
  Node* getNode (int i, int j, bool &found);
  bool Grid::findNode (int i, int j);
  void Grid::draw (GLenum mode, const float color[3]);
  void Grid::draw (weightVector vector, const float color[3], double maxWeight, double modelHeight);
};

#endif
