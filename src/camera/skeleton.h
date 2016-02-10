#ifndef _SKELETON_H__
#define _SKEKETON_H__

#include "basic.h"
#include "math/numerical.h"

// a corner is an image point along with a set of connected image edges
//

class corner {
public:

	corner();
	~corner();

	corner( double _x, double _y, double _z );

	void set_point( double _x, double _y, double _z ) { x = _x; y = _y; z = _z;};
	void set_cameraid ( int id ) { cameraid = id; };
	void insert_edge ( double x1, double y1, double z1, double x2, double y2, double z2, double min_angle );
	bool valid() { return _valid; } // more than two edges
	double getx() { return x;}
	double gety() { return y;}
	double getz() { return z;}
	int getcameraid() {return cameraid;}
	Vec3d getpoint() {return Vec3d(x,y,z);}
	double getweight() { return weight;}
	int nedges() {return edges.size()/6;}
	void print();

	void clear_edges() { edges.clear(); normal.clear(); _valid = false;}
	void getedge( int p, Vec3d &a, Vec3d &b );
	Vec3d getnormal( int p ) { return Vec3d( normal[3*p], normal[3*p+1], normal[3*p+2] ); }

	double dihedral_angle( int p, int q );
	double subtended_angle( int p );

private:

	int cameraid; 
	bool _valid;

	double x,y,z;			// image point
	std::vector< double > edges;	// list of edges end points
	std::vector< double > normal;   // list of normal vectors
	double weight; // the inverse of the SSD to edges end points
	
};


#endif
