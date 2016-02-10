#include "skeleton.h"

/* constructor
 */
corner::corner ()
{
	x = y = z = 0.0;
	cameraid = -1;
	weight = 0.0;

	_valid = false;
}

/* constructor
 */
corner::corner ( double _x, double _y, double _z)
{
	x = _x;
	y = _y;
	z = _z;

	weight = 0.0;

	cameraid = -1;
	_valid = false;
}

/* destructor
 */
corner::~corner ()
{
}

/* Insert an edge in the corner
 * min_angle: min angle (in radians) between two edges to validate a corner
 * warning: it is required that (x1,y1,z1) be the edge end point attached to the corner
 */
void corner::insert_edge ( double x1, double y1, double z1, double x2, double y2, double z2, double min_angle )
{
	edges.push_back( x1 );
	edges.push_back( y1 );
	edges.push_back( z1 );

	edges.push_back( x2 );
	edges.push_back( y2 );
	edges.push_back( z2 );

	Vec3d n1 = cross(  Vec3d(x1,y1,z1), Vec3d(x2,y2,z2) );
	if ( len(n1) < EPS )
		return;

	n1 = norm(n1);
	normal.push_back( n1[0] );
	normal.push_back( n1[1] );
	normal.push_back( n1[2] );

	// the corner is valid if at least two edges make an angle larger than <min_angle>
	if ( !_valid ) {

		for (int i=0;i<edges.size() / 6 - 1; i++) {
			Vec3d n2 = cross( Vec3d(edges[6*i],edges[6*i+1],edges[6*i+2]), Vec3d(edges[6*i+3],edges[6*i+4],edges[6*i+5]) );
			if ( len(n2) < EPS )
				continue;
			n2 = norm(n2);

			double angle = MIN( acos(dot(n1,n2)), acos(dot(n1,-n2)) );

			if ( angle > min_angle ) {
				_valid = true;
				return;
			}
		}
	}

	// compute the weight as the inverse of the SSD to edges end points

	double ssd = 0.0;
	int counter = 0;
	for (int i=0;i<edges.size() / 6;i++) {
		Vec3d a = Vec3d(edges[6*i  ],edges[6*i+1],edges[6*i+2]);
		Vec3d b = Vec3d(edges[6*i+3],edges[6*i+4],edges[6*i+5]);
		Vec3d p = Vec3d(x,y,z);
		ssd += MIN( len(a-p), len(b-p) );
		counter++;
	}

	if ( counter > 0 && ssd > 1E-8 )
		weight = (double)counter / sqrt(ssd);

}

/* print a list of corner to standard output
 */
void corner::print()
{
	printf("*** corner has %d edges ***\n", edges.size() / 6 );

	for (int i=0;i<edges.size() / 6; i++) {
		Vec3d n1 = cross( Vec3d(edges[6*i],edges[6*i+1],edges[6*i+2]), Vec3d(edges[6*i+3],edges[6*i+4],edges[6*i+5]) );
		if ( len(n1) < EPS )
			return;
		n1 = norm(n1);
		
		for (int j=i+1;j<edges.size() / 6; j++) {
			Vec3d n2 = cross( Vec3d(edges[6*j],edges[6*j+1],edges[6*j+2]), Vec3d(edges[6*j+3],edges[6*j+4],edges[6*j+5]) );
			if ( len(n2) < EPS )
				continue;
			n2 = norm(n2);

			double angle = MIN( acos(dot(n1,n2)), acos(dot(n1,-n2)) );

			printf("%f\n", toDegrees(angle));
		}
	}
}

/* return p-th edge in a corner
 */
void corner::getedge( int p, Vec3d &a, Vec3d &b )
{
	assert( p < edges.size() / 6 );

	a = Vec3d( edges[6*p  ], edges[6*p+1], edges[6*p+2] );
	b = Vec3d( edges[6*p+3], edges[6*p+4], edges[6*p+5] );
}

/* compute subtended angle between edge p and edge p+1
 */
double corner::subtended_angle ( int p )
{
	Vec3d a = Vec3d( edges[6*p  ], edges[6*p+1], edges[6*p+2] );
	Vec3d b = Vec3d( edges[6*p+3], edges[6*p+4], edges[6*p+5] );

	return Acos(dot( norm(a), norm(b) ) );
}

/* compute dihedral angle between edge p and edge q
 */
double corner::dihedral_angle( int p, int q )
{
	Vec3d n1 = getnormal( p );
	Vec3d n2 = getnormal( q );

	return dot(cross(n2,n1),getpoint()) > 0 ? Acos(dot(n1,n2)) : -Acos(dot(n1,n2));
}





