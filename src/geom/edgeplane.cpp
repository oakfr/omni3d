#include "basic.h"
#include "geom/geom3D.h"
#include "util/util.h"

/* constructor
 */
SphericalCoord::SphericalCoord(Vec3d a)
{
	rho = len(a);
	theta = atan(a[1]/a[0]);
	phi = acos(a[2]/rho);
}

/* compute geodesic distance between two coordinates
 */
double SphericalCoord::geodesicDistance (SphericalCoord sp)
{
	// Haversine formula
	// phi = co-latitude = M_PI/2 - latitude
	// theta = longitude

	double dlat = phi-sp.phi;
	double dlong = theta - sp.theta;
	double a = sin(dlat/2)*sin(dlat/2) + sin(phi) * sin(sp.phi) * sin(dlong/2) * sin(dlong/2);
	double c = 2*atan(sqrt(a)/sqrt(1-a));
	return c;
}

/* constructor
 */
EdgePlane::EdgePlane ()
{
	_a = _b = _s  = _normal = Vec3d(0,1,0);
	_cameraId = _chaineId = _uid = 0;
	_length = 0.0;
	_matched = false;
	l_avg = r_avg = l_dev = r_dev = Vec3d(0,0,0);
}

/* flip an edge plane
 */
void EdgePlane::flip()
{
	Vec3d temp = _b;
	_b = _a;
	_a = temp;
	_normal = - _normal;
}

/* return the distance between two edges in the color space
 */
double EdgePlane::colorDistance( EdgePlane &ep )
{
	if ( dot(_normal, ep._normal) > 0.0 ) {
		double d_avg = .5 * len(l_avg-ep.l_avg) + .5 * len(r_avg - ep.r_avg);
		d_avg /= SQRT3; // normalize from 0 to 1

		double d_dev = .5 * len(l_dev-ep.l_dev) + .5 * len(r_dev - ep.r_dev);
		d_dev /= SQRT3;

		return .5 * d_avg + .5 * d_dev;
	} else {
		double d_avg = .5 * len(l_avg-ep.r_avg) + .5 * len(r_avg - ep.l_avg);
		d_avg /= SQRT3; // normalize from 0 to 1

		double d_dev = .5 * len(l_dev-ep.r_dev) + .5 * len(r_dev - ep.l_dev);
		d_dev /= SQRT3;

		return .5 * d_avg + .5 * d_dev;
	}
}

/* constructor
 */
EdgePlane::EdgePlane (Vec3d a, Vec3d b, Vec3d s, int cameraId, int edgeId, int uid)
{
	_a = norm(a);
	_b = norm(b);
	_s = s;
	_cameraId = cameraId;
	_edgeIds.push_back(edgeId);
	_chaineId = -1;
	_uid = uid;
	_length = length();

	_normal = norm(cross(_a,_b));
	l_avg = r_avg = l_dev = r_dev = Vec3d(0,0,0);
}

/* from camera frame to world frame
 */
void EdgePlane::fromCameraFrameToWorldFrame (Vec3d translation, Quaternion rotation)
{
	//rotate(rotation);
	rotate(rotation);
	_s += translation;
}

/* from world frame to camera frame
 */
void EdgePlane::fromWorldFrameToCameraFrame (Vec3d translation, Quaternion rotation)
{
	_s -= translation;
	rotate(rotation.inv());
}

/* from camera frame to world frame
 */
void EdgePlane::fromCameraFrameToWorldFrame (ExtrinsicParameters came)
{
	rotate(came.getRotation());
	_s += came.getTranslation();
}

/* from camera frame to world frame
 */
void EdgePlane::fromWorldFrameToCameraFrame (ExtrinsicParameters came)
{
	_s -= came.getTranslation();
	rotate(came.getRotation().inv());
}

/* compute the min and max polar angles between the two edgeplanes
 * see edgeplanes.ppt for more info
 */
bool EdgePlane::minMaxPolarAngles( EdgePlane &ep, double &min1, double &max1, double &min2, double &max2 )
{
	Vec3d l11 = norm(_a); // unit end point directions
	Vec3d l12 = norm(_b);
	Vec3d l21 = norm(ep._a);
	Vec3d l22 = norm(ep._b);
	
	Vec3d n1 = _normal; // unit normal vectors
	Vec3d n2 = ep._normal;
	
	Vec3d u = cross(n1,n2); // intersection of the two edgeplanes
	
	if ( len(u) < EPS )  // if the two edgeplanes are not skew, skip it
		return false;
	
	u = norm(u); // normalized intersection
				
	double d11 = dot(l11,u); // dot products
	double d12 = dot(l12,u);
	double d21 = dot(l21,u);
	double d22 = dot(l22,u);
				
	double angle11 = MIN( Acos(d11), Acos(-d11) ); // polar angles
	double angle12 = MIN( Acos(d12), Acos(-d12) );
	double angle21 = MIN( Acos(d21), Acos(-d21) );
	double angle22 = MIN( Acos(d22), Acos(-d22) );
				
	min1 = MIN(min1,MIN(angle11,angle12));
	max1 = MAX(max1,MAX(angle11,angle12));
	min2 = MIN(min2,MIN(angle21,angle22));
	max2 = MAX(max2,MAX(angle21,angle22));

	return true;
}

/* rotate an edgeplane
 */
void EdgePlane::rotate (Quaternion q)
{
	//_s = rotation * _s;
	_a = q.rotate(_a);
	_b = q.rotate(_b);
	_normal = q.rotate(_normal);
}

/* rotate an edgeplane
 */
void EdgePlane::rotate (Vec3d axis, double angle)
{
	rotate(Quaternion(cos(angle/2),sin(angle/2)*norm(axis)));
}

/* return the angle between an edgeplane and a 3D line
 */
double EdgePlane::angle (Edge *edge)
{
	Vec3d n1 = _normal;
	Vec3d n2 = norm( cross( edge->getA() - _s, edge->getB() - _s ) );
	double a1 = Acos( dot( n1, n2 ) );
	double a2 = Acos( dot( n1, -n2 ) );

	return MIN( a1, a2 );
}

/* return the angle between two edgeplanes ( 0 < alpha < pi/2 )
 */
double EdgePlane::angle( EdgePlane &plane )
{
	Vec3d n1 = _normal;
	Vec3d n2 = plane._normal;

	Vec3d p = cross(n1,n2);

	double angle = Asin(len(p));

	return angle;
}

/* return the oriented angle between two edgeplanes ( 0 < alpha < pi )
 * for each edgeplane, we determine which normal points toward the other edgeplane
 */
double EdgePlane::oriented_angle( EdgePlane &plane )
{
	Vec3d n1 = _normal;
	Vec3d n2 = plane._normal;
	Vec3d e1 = (_a+_b)/2.0;
	Vec3d e2 = (plane._a + plane._b)/2.0;

	if ( dot(n1,e2) < 0 )
		n1 = -n1;

	if ( dot(n2,e1) < 0 )
		n2 = -n2;

	return Acos( dot( n1, -n2 ) );
}

/* Score the alignment between an edge plane and a 3D line
 * 0: bad, 1: good
 */
double EdgePlane::score_alignement (Edge *line)
{
	assert (line != NULL);

	double a = fabs(dot(_normal,norm(cross(norm(line->getA() - _s),norm(line->getB()- _s))))); // 0 < a < 1
	return a;
}

/* Score the alignment between two edgeplanes
 * 0: bad, 1: good
 */
double EdgePlane::score_alignement (EdgePlane ep)
{
	if ((dot(_a,ep._a) < 0) && (dot(_a,ep._b) < 0)) // if not in the same half-plane
		return 0;

	if ((dot(_b,ep._a) < 0) && (dot(_b,ep._b) < 0)) // if not in the same half-plane
		return 0;

	// otherwise, return angle between the two normal vectors
	return fabs(dot(_normal,ep._normal));
}

/* print out an edgeplane
 */
void EdgePlane::print()
{
	 LOG(LEVEL_DEBUG,"uid: %d cameraid: %d chaineid: %d (s,a,b,n): %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",_uid,_cameraId,_chaineId,_s[0],_s[1],_s[2],_a[0],_a[1],\
		 _a[2],_b[0],_b[1],_b[2],_normal[0],_normal[1],_normal[2]);
}

/* compute the solid angle of an edge plane
 * the solid angle is also the length on the unit sphere
 */
double EdgePlane::solidAngle()
{
	return Acos(dot(_a,_b));
}

/* compute the length of an edge plane
 */
double EdgePlane::length()
{
	return solidAngle();
}

/* compute the overlap between an edge plane and a 3D line segment
 */
double EdgePlane::overlap (Edge *edge)
{
	return OverlapPlaneLine(edge->getA() - _s, edge->getB() - _s, _a, _b);
}

/* Return the measurement of overlap between two edgeplanes
 */
double EdgePlane::score_overlap ( EdgePlane &ep )
{
	return OverlapPlaneLine(_a,_b,ep._a,ep._b);
}

/* Return the shortest distance between an edge plane and a 2D line segment
 */
double EdgePlane::distance (Edge *edge)
{
	double lambda;
	double distance = 0.0;

	intersectRayPlane(edge->_a->getPosition(),_normal,_s,_normal,lambda);
	distance += fabs(lambda)/len(edge->_a->getPosition()-_s);

	intersectRayPlane(edge->_b->getPosition(),_normal,_s,_normal,lambda);
	distance += fabs(lambda)/len(edge->_b->getPosition()-_s);

	return distance;
}


/* test frontality of a 3D line segment with respect to an image edge plane
 */
bool EdgePlane::testFrontality(Edge *edge)
{
	if ((dot(edge->_a->getPosition()-_s,_a)>0) && (dot(edge->_b->getPosition()-_s,_b)>0))
		return true;

	if ((dot(edge->_b->getPosition()-_s,_a)>0) && (dot(edge->_a->getPosition()-_s,_b)>0))
		return true;

	return false;
}

/* return overlap between line ep and current edge
 */
double EdgePlane::overlap( EdgePlane &ep )
{
	return OverlapPlaneLine(ep._a, ep._b, _a, _b);
}

/* return overlap between current edge plane and target edge plane
 */
bool EdgePlane::overlap (EdgePlane *edge)
{
	double alpha1, alpha2;
	Vec3d normal = averageNormal(*edge);

	alpha1 = Acos(dot(_a,edge->_a)) * sign(dot(normal,cross(_a,edge->_a)));
	alpha2 = Acos(dot(_a,edge->_b)) * sign(dot(normal,cross(_a,edge->_b)));
	if (sign(alpha1*alpha2) < 0)
		return true;

	alpha1 = Acos(dot(_b,edge->_a)) * sign(dot(normal,cross(_b,edge->_a)));
	alpha2 = Acos(dot(_b,edge->_b)) * sign(dot(normal,cross(_b,edge->_b)));
	if (sign(alpha1*alpha2) < 0)
		return true;

	alpha1 = Acos(dot(_a,edge->_b)) * sign(dot(normal,cross(_a,edge->_b)));
	alpha2 = Acos(dot(_b,edge->_b)) * sign(dot(normal,cross(_b,edge->_b)));
	if (sign(alpha1*alpha2) < 0)
		return true;

	alpha1 = Acos(dot(_a,edge->_a)) * sign(dot(normal,cross(_a,edge->_a)));
	alpha2 = Acos(dot(_b,edge->_a)) * sign(dot(normal,cross(_b,edge->_a)));
	if (sign(alpha1*alpha2) < 0)
		return true;

	return false;
}

/* separation angle between two edgeplanes with common normal
 */
double EdgePlane::separationAngle (EdgePlane *edge, Vec3d normal)
{
	double res = M_PI;

	double alpha1, alpha2;
	alpha1 = Acos(dot(_a,edge->_a)) * sign(dot(normal,cross(_a,edge->_a)));
	alpha2 = Acos(dot(_a,edge->_b)) * sign(dot(normal,cross(_a,edge->_b)));
	if (sign(alpha1*alpha2) < 0)
		return 0.0;
	else res = MIN(res,MIN(fabs(alpha1),fabs(alpha2)));

	alpha1 = Acos(dot(_b,edge->_a)) * sign(dot(normal,cross(_b,edge->_a)));
	alpha2 = Acos(dot(_b,edge->_b)) * sign(dot(normal,cross(_b,edge->_b)));
	if (sign(alpha1*alpha2) < 0)
		return 0.0;
	else res = MIN(res,MIN(fabs(alpha1),fabs(alpha2)));

	alpha1 = Acos(dot(_a,edge->_b)) * sign(dot(normal,cross(_a,edge->_b)));
	alpha2 = Acos(dot(_b,edge->_b)) * sign(dot(normal,cross(_b,edge->_b)));
	if (sign(alpha1*alpha2) < 0)
		return 0.0;
	else res = MIN(res,MIN(fabs(alpha1),fabs(alpha2)));

	alpha1 = Acos(dot(_a,edge->_a)) * sign(dot(normal,cross(_a,edge->_a)));
	alpha2 = Acos(dot(_b,edge->_a)) * sign(dot(normal,cross(_b,edge->_a)));
	if (sign(alpha1*alpha2) < 0)
		return 0.0;
	else res = MIN(res,MIN(fabs(alpha1),fabs(alpha2)));
	return res;
}

/* find the two most distant points between two edgeplanes
 */
void EdgePlane::findEndPoints(EdgePlane *edge, Vec3d &s, Vec3d &e)
{
	s = _a;
	e = edge->_a;
	double length = len(s-e);

	if (len(s-edge->_b)>length) {
		e = edge->_b;
		length = len(s-e);
	}

	if (len(_b-edge->_a)>length) {
		s = _b;
		e = edge->_a;
		length = len(s-e);
	}
	if (len(_b-edge->_b)>length) {
		s = _b;
		e = edge->_b;
	}
}

/* compute the span of an edge plane with respect to a given 3D line segment
 */
double EdgePlane::span (EdgePlane *edge)
{
	Vec3d a,b;
	findEndPoints(edge,a,b);
	return len(a-b);
}

/* Average normal vectors
 */
Vec3d EdgePlane::averageNormal (EdgePlane edge)
{
	Vec3d n1 = _normal;
	if (dot(_normal,edge._normal) < 0)
		n1 = -n1;

	return norm(_length*n1 + edge._length*edge._normal);
}

/* Score alignment
 */
double EdgePlane::scoreAlignment (EdgePlane *edge)
{
	Vec3d normal = averageNormal(*edge);
	double sin1 = _length*len(cross(normal,_normal));
	double sin2 = edge->_length*len(cross(normal,edge->_normal));

	return (sin1+sin2);
}

/* compute the poles of two edgeplanes
 */
bool EdgePlane::poles( EdgePlane ep, Vec3d &pole_1, Vec3d &pole_2 )
{
	pole_1 = cross( _normal, ep._normal );

	if ( len( pole_1 ) < EPS )
		return false;

	pole_1 = norm(pole_1);
	pole_2 = -pole_1;

	return true;
}

/* surface spanned by an edge plane and a 3D line segment
 */
double EdgePlane::thickness (EdgePlane *edge)
{
	double l1,l2,l3,l4;
	//intersectRayPlane(a,normal,Vec3d(0,0,0),normal,lambda);	
	intersectRayPlane(_a,edge->_normal,edge->_a,edge->_normal,l1);
	intersectRayPlane(_b,edge->_normal,edge->_a,edge->_normal,l2);
	intersectRayPlane(edge->_a,_normal,_a,_normal,l3);
	intersectRayPlane(edge->_b,_normal,_a,_normal,l4);
	return MAX(MAX(fabs(l1),fabs(l2)),MAX(fabs(l3),fabs(l4)))/span(edge);
}

/* separation angle between an edge plane and a 3D line segment
 */
double EdgePlane::separation(EdgePlane *edge)
{
	return 2.0*fabs(separationAngle(edge,averageNormal(*edge))/(solidAngle()+edge->solidAngle()));
}

/* Merge two edge planes
 */
bool EdgePlane::merge (EdgePlane *edge)
{

	// construct the new endpoints
	Vec3d normal = averageNormal(*edge);
	
	//first find the two end points
	Vec3d a,b;
	findEndPoints(edge,a,b);

	// project the two end points on the plane
	_a = a;
	_b = b;
	_length = length();
	
	// update the edgeIds
	for (int i=0;i<edge->_edgeIds.size();i++) {
		int id = edge->_edgeIds[i];
		bool found = false;
		for (int j=0;j<_edgeIds.size();j++) {
			if (id == _edgeIds[j]) {
				found = true;
				break;
			}
		}
		if (!found)
			_edgeIds.push_back(id);
	}

	_normal = norm(cross(_a,_b));

	return true;
}

bool EdgePlane::operator==( const EdgePlane &c)
{
	if ( _a != c._a )
		return false;

	if ( _b != c._b )
		return false;

	if ( _s != c._s )
		return false;

	if ( _cameraId != c._cameraId )
		return false;

	return true;
}


