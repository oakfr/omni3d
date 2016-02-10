#include "numerical.h"


//////////////////////////////////////////////////////////////
//     Quaternion
//////////////////////////////////////////////////////////////
Quaternion::Quaternion (Vec3d axis, double angle) {

	normAngle(angle);
	if ( len(axis) < EPS ) {
		_s = 1.0;
		_v = Vec3d(0,0,0);
	} else {
		_s = cos(angle/2); 
		_v = sin(angle/2) * norm(axis);
	}
}

Quaternion Quaternion::operator*(const Quaternion &q) 
{
	Quaternion r;

	r._s = _s*q._s-dot(_v,q._v);
	r._v = _s*q._v + q._s*_v + cross(_v,q._v);
	return r;
}

Quaternion Quaternion::bar() 
{
	Quaternion r;
	r._s = _s;
	r._v = -_v;
	return r;
}

Vec3d Quaternion::rotate(Vec3d v_in) 
{
	Quaternion p;
	p._s = 0;
	p._v = v_in;
	Quaternion q;
	q._s = _s;
	q._v = _v;
	Quaternion qbar = q.bar();
	Quaternion r;
	r = q*p;
	r = r*qbar;
	return r._v;
}

Quaternion Quaternion::normalize()
{
	double v = sqrt ( _s * _s + sqrlen(_v) );
	return  Quaternion( _s/v, _v/v );
}

Mat3d Quaternion::toRotationMatrix ()
{
	Mat3d R;
	double w = _s;
	double x = _v[0], y=_v[1], z=_v[2];

	R[0][0] = 1 - 2*y*y - 2*z*z;
	R[0][1] = 2*x*y + 2*w*z;
	R[0][2] = 2*x*z - 2*w*y;
	R[1][0] = 2*x*y - 2*w*z;
	R[1][1] = 1 - 2*x*x - 2*z*z;
	R[1][2] = 2*y*z + 2*w*x;
	R[2][0] = 2*x*z + 2*w*y;
	R[2][1] = 2*y*z - 2*w*x;
	R[2][2] = 1 - 2*x*x - 2*y*y;

	return R;
}

//assuming unit quaternion, return the rotation angle
// assume sin(theta/2) > 0
void Quaternion::toAxisAngle (Vec3d &axis)
{
	double angle = 2 * Acos(_s);
	normAngle(angle);

	if (fabs(1 - _s*_s) > EPSILON) {
		axis[0] = _v[0] / (sqrt(1 - _s*_s));
		axis[1] = _v[1] / (sqrt(1 - _s*_s));
		axis[2] = _v[2] / (sqrt(1 - _s*_s));
	} else {
		axis = Vec3d(0,0,1); // arbitrary axis, rotation angle = 0 [2*pi]
	}

	axis = angle * norm(axis);

	if ( len(axis) > M_PI ) {
		axis = -axis;
		axis = (2*M_PI - len(axis)) * norm(axis);
	}
}

//////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////

// make sure that angle is between 0 and 2*M_PI
void normAngle (double &angle) 
{
	while (angle < 0)
		angle += 2*M_PI;
	while (angle > 2*M_PI)
		angle -= 2*M_PI;
}

// solve for x^3 + p.x + q = 0
// if p>0, there is one and only one solution
//
bool solve3rdDegree (double p, double q, double &x1, double &x2, double &x3)
{
	bool singleSolution = false;
	
	double delta = p*p*p/27+q*q/4;
	
	if (delta >= 0) {
		double t1 = -q/2 - sqrt(delta);
		double t2 = -q/2 + sqrt(delta);
		
		double u = sign(t1)*powf(fabs(t1),(double)1.0/3);
		double v = sign(t2)*powf(fabs(t2),(double)1.0/3);
		
		x1 = u+v;
		//printf("X1 = %f... (delta = %f, p = %f, q = %f, t1 = %f, t2 = %f, u = %f, v = %f)\n",x1, delta,p,q, t1, t2, u, v);

		x2 = x3 = 0;
		singleSolution = true;
	} else {
		double th = -q/(2*sqrt(-p*p*p/27));
		th = MIN(MAX(th,-1.0),1.0);
		double ath = acos(th);
		
		assert(p<0);
		x1 = 2*sqrt(-p/3)*cos(ath/3);
		//printf("X1 = %f\n",x1);
		x2 = 2*sqrt(-p/3)*cos((ath+2*M_PI)/3);
		x3 = 2*sqrt(-p/3)*cos((ath+4*M_PI)/3);
		singleSolution = false;
	}
	
	return singleSolution;
}

// intersect the ray (P,u) with the plane (Q,n)
Vec3d intersectRayPlane (Vec3d P, Vec3d u, Vec3d Q, Vec3d n, double &lambda)
{
	u = norm(u);
	n = norm(n);
	lambda = dot(Q-P,n)/dot(u,n);
	return P + lambda * u;
}


// intersect the two planes defined by (P,u) and (Q,v) and returns the result in line (point,dir)
// if intersection does not exist, returns false; otherwise, return true
bool intersectPlanePlane (Vec3d P, Vec3d u, Vec3d Q, Vec3d v, Vec3d &point, Vec3d &dir)
{
	dir = cross(norm(u),norm(v));
	if (len(dir) < EPSILON)
		return false;

	dir = norm(dir);

	double lambda;
	point = intersectRayPlane(P,cross(u,dir),Q,v,lambda);
	return true;
}

// project a 3D point p onto a 3D line <lp,ldir>
// and return the projected point
//
Vec3d projectPointLine( Vec3d p, Vec3d lp, Vec3d ldir )
{
	ldir = norm(ldir);

	Vec3d a = lp;
	Vec3d b = lp + ldir;

	Vec3d w = cross( p - a, p - b );

	if ( len( w ) < EPS )  // if p is aligned with a and b
		return p;

	w = norm( w );

	Vec3d n = norm( cross( norm( b - a ), w ) ); // direction of the projection

	Vec3d s = p + dot(a-p,n) * n;

	LOG(LEVEL_INFO, "check projectPointLine: %f %f", dot(s-p,b-a), len(cross(b-s,a-s))); // both should be zero
	
	return s;
}

// return the overlap [0,1] between the edge plane (0,p,q)
// and the 3D line (a,b)
//
double OverlapPlaneLine( Vec3d a, Vec3d b, Vec3d p, Vec3d q )
{
	double lambda;
	a = norm(a);
	b = norm(b);
	Vec3d n = norm(cross(a,b));
	p = norm(intersectRayPlane(p,n,a,n,lambda));
	q = norm(intersectRayPlane(q,n,a,n,lambda));

	// determine whether a and b are inside the line span
	double angle_aq = dot(n,cross(a,q)) > 0 ? Acos(dot(a,q)) : 2*M_PI - Acos(dot(a,q));
	double angle_ap = dot(n,cross(a,p)) > 0 ? Acos(dot(a,p)) : 2*M_PI - Acos(dot(a,p));
	double angle_ab = Acos(dot(a,b));
	double angle_pq = Acos(dot(p,q));

	// both outside
	if ( angle_aq > angle_ab && angle_ap > angle_ab ) {
		double alpha = MIN(angle_aq-angle_ab,2*M_PI-angle_aq);			// if p and q are both close to a and b, don't attribute zero
		double beta = MIN(angle_ap-angle_ab,2*M_PI-angle_ap);			// but score linearly instead
		return MIN(1.0,MAX(0.0, 1.0 - 10.0*alpha/angle_ab - 10.0*beta/angle_ab)); 
	}

	// both inside
	if ( angle_aq < angle_ab && angle_ap < angle_ab )
		return 1.0; //fabs(angle_ap-angle_aq)  / angle_ab;

	// one inside, one outside
	if ( angle_ap < angle_ab ) { // p is inside
		if ( angle_aq - angle_ap > M_PI )
			return (angle_ap) * (angle_ap) / (angle_ab * angle_pq);
		else
			return (angle_ab - angle_ap) * (angle_ab - angle_ap) / (angle_ab * angle_pq);
	} else { // q is inside
		if ( angle_ap - angle_aq > M_PI )
			return (angle_aq * angle_aq) / (angle_ab * angle_pq);
		else
			return (angle_ab - angle_aq) * (angle_ab - angle_aq) / (angle_ab * angle_pq);
	}

	return -1.0;
}

// compute the minimal rotation that brings points (e11,e12) onto points (e21,e22)
// and write the solution in rotation
// return false if the computation is impossible
// at the end, the rotation is flipped if the angle > pi
bool minRotationFourPoints( const Vec3d e11, const Vec3d e12, const Vec3d e21, const Vec3d e22, Vec3d &rotation)
{
	rotation = Vec3d(0,0,0);

	// bring e11 onto e12
	Vec3d n = cross(e11,e21);

	double angle = Acos(dot(e11,e21));

	Quaternion q1 = Quaternion( n, angle );

	// rotate e11 and e12
	Vec3d e11p = q1.rotate( e11 );
	Vec3d e12p = q1.rotate( e12 );

	// verify that e11 is now aligned with e21
	if ( Acos(dot(e11p,e21)) > 1E-6 )
		return false;

	Vec3d n1 = (cross(e12p, e21));
	Vec3d n2 = (cross(e22 , e21));

	if ( len(n1) < EPS || len(n2) < EPS )
		return false;

	n1 = norm(n1);
	n2 = norm(n2);

	Vec3d n3 = norm(cross(n1, n2 ));

	if ( Acos(dot(n1,n2)) > M_PI )
		n3 = -n3;

	Quaternion q2 = Quaternion( n3, Acos(dot(n1,n2)) );

	// verify that e12 is now aligned with e22
	e12p = q2.rotate( e12p );
	e11p = q2.rotate( e11p );

	double check2 = Acos(dot(e12p,e22));
	double check1 = Acos(dot(e11p,e21));

	// convert to axis-angle representation
	(q2*q1).toAxisAngle( rotation );

	// flip if angle > pi
	if ( len(rotation) > M_PI ) {
		angle = 2*M_PI - len(rotation);
		rotation = -angle * norm(rotation);
	}

	return true;
}

// compute the minimal rotation that brings edges (e1a,e1b) and (e2a,e2b) onto lines (l1a,l1b) and (l2a,l2b)
// and write the solution in rotation
// return false if the computation is impossible (edges or lines aligned)
// edges are expressed in the camera coordinate frame, centered on the camera origin
// lines are expressed in the camera coordinate frame, centered on the camera origin
// at the end, the rotation is flipped if the angle > pi
// max_error is the maximum angular error allowed between edge 2 and line 2 after edge 1 and line 1 have been aligned
//
bool minRotation2Planes2Lines( Vec3d e1a, Vec3d e1b, Vec3d e2a, Vec3d e2b, Vec3d l1a, Vec3d l1b, Vec3d l2a, Vec3d l2b, double max_error, double &overlap, Vec3d &rotation )
{
	Quaternion best_rotation;
	
	if ( len(e1a) < 1E-7 || len(e1b) < 1E-7 || len(e2a) < 1E-7 || len(e2b) < 1E-7 )
		return false;

	if ( len(l1a) < 1E-7 || len(l1b) < 1E-7 || len(l2a) < 1E-7 || len(l2b) < 1E-7 )
		return false;

	e1a = norm(e1a); // normalize vectors
	e1b = norm(e1b);
	e2a = norm(e2a);
	e2b = norm(e2b);

	l1a = norm(l1a);
	l1b = norm(l1b);
	l2a = norm(l2a);
	l2b = norm(l2b);

	Vec3d ne1 = (cross(e1a,e1b)); // compute normal vectors
	Vec3d ne2 = (cross(e2a,e2b));
	Vec3d nl1 = (cross(l1a,l1b));
	Vec3d nl2 = (cross(l2a,l2b));

	if ( len(ne1) < 1E-7 || len(ne2) < EPS || len(nl1) < EPS || len(nl2) < EPS )
		return false;

	ne1 = norm(ne1);
	ne2 = norm(ne2);
	nl1 = norm(nl1);
	nl2 = norm(nl2);

	Vec3d n = cross(ne1,nl1); // axis of the first rotation - bringing e1 onto l1

	// compute the first rotation
	Quaternion q11, q12;

	if ( len(n) > EPS ) {  // if norm < eps, no rotation is required

		n = norm( n ); // normalize the axis vector
		
		double alpha = Acos(dot(ne1,nl1));

		q11 = Quaternion( n, alpha );  // this rotation brings e1 onto l1

		q12 = Quaternion( n, alpha + M_PI); // this rotation also brings e1 onto l1 !

		double check = len(cross(nl1,q11.rotate(ne1))); // check: should be zero
		check = len(cross(nl1,q12.rotate(ne1))); // check: should be zero
	}
//
//	q12.toAxisAngle( rotation );
//	return true;

	// try the first rotation

	Vec3d rne2 = q11.rotate( ne2 ); // apply first rotation to edge 2

	// find the rotation that brings e2 onto l2

	n = nl1; // axis of the rotation

	// at this point in time, rne2 and nl2 should make roughly the same angle with nl1
	double beta1 = Acos(dot(rne2,nl1));
	double beta2 = Acos(dot(nl2, nl1));

	bool skip = false;

	if ( fabs(beta1 - beta2) < max_error ) {
	} else if ( fabs(beta1 - (M_PI - beta2)) < max_error ) {
		nl2 = -nl2;
	} else {
		skip = true;
	}

	double lambda;
	Vec3d pne2, pnl2;
	Quaternion q21, q22, q;
	double check, temp = 0.0;
	overlap = 0.0;

	if ( !skip ) {

		pne2 = intersectRayPlane(rne2, nl1, Vec3d(0,0,0), nl1, lambda); // project rne2 and nl2 onto plane perp. to nl1

		pnl2 = intersectRayPlane( nl2, nl1, Vec3d(0,0,0), nl1, lambda);

		if ( len(pne2) > EPS && len(pnl2) > EPS ) {

			double alpha = Acos(dot(norm(pne2), norm(pnl2))); // find the angle of the rotation that brings pne2 onto pnl2
			if ( dot(cross(pne2,pnl2),n) < 0 )
				alpha = -alpha;

			q21 = Quaternion(n, alpha);  // this rotation brings rne2 onto nl2
			q22 = Quaternion( n, alpha + M_PI ); // this rotation also brings rne2 onto nl2

			check = len(cross(pnl2,q21.rotate(pne2))); // check: should be zero
			check = len(cross(pnl2,q22.rotate(pne2))); // check: should be zero
			check = len(cross(nl2,q21.rotate(rne2))); // check: should be zero
			check = len(cross(nl2,q22.rotate(rne2))); // check: should be zero

			// apply the global rotation to the two edges and compute the overlap
			
			q = q21 * q11;

			temp = OverlapPlaneLine(l1a, l1b, q.rotate(e1a), q.rotate(e1b)) * OverlapPlaneLine(l2a, l2b, q.rotate(e2a), q.rotate(e2b));

			if ( temp > overlap ) {
				overlap = temp;
				best_rotation = q;
			}

			// try the other rotation

			q = q22 * q11;

			temp = OverlapPlaneLine(l1a, l1b, q.rotate(e1a), q.rotate(e1b)) * OverlapPlaneLine(l2a, l2b, q.rotate(e2a), q.rotate(e2b));
			if ( temp > overlap ) {
				overlap = temp;
				best_rotation = q;
			}
		}
	}

	// **********************
	// try the other first rotation

	rne2 = q12.rotate( ne2 ); // apply first rotation to edge 2

	// at this point in time, rne2 and nl2 should make roughly the same angle with nl1
	beta1 = Acos(dot(rne2,nl1));
	beta2 = Acos(dot(nl2, nl1));

	skip = false;

	if ( fabs(beta1 - beta2) < max_error ) {
	} else if ( fabs(beta1 - (M_PI - beta2)) < max_error ) {
		nl2 = -nl2;
	} else {
		skip = true;
	}

	if ( !skip ) {
		pne2 = intersectRayPlane(rne2, nl1, Vec3d(0,0,0), nl1, lambda); // project rne2 and nl2 onto plane perp. to nl1

		pnl2 = intersectRayPlane( nl2, nl1, Vec3d(0,0,0), nl1, lambda);

		if ( len(pne2) > EPS && len(pnl2) > EPS ) {

			double alpha = Acos(dot(norm(pne2), norm(pnl2))); // find the angle of the rotation that brings pne2 onto pnl2
			if ( dot(cross(pne2,pnl2),n) < 0 )
				alpha = -alpha;

			q21 = Quaternion(n, alpha);  // this rotation brings rne2 onto nl2
			q22 = Quaternion( n, M_PI + alpha ); // this rotation also brings rne2 onto nl2

			check = len(cross(nl2,q21.rotate(rne2))); // check: should be zero
			check = len(cross(nl2,q22.rotate(rne2))); // check: should be zero

			// apply the global rotation to the two edges and compute the overlap
			
			q = q21 * q12;

			temp = OverlapPlaneLine(l1a, l1b, q.rotate(e1a), q.rotate(e1b)) * OverlapPlaneLine(l2a, l2b, q.rotate(e2a), q.rotate(e2b));
			if ( temp > overlap ) {
				overlap = temp;
				best_rotation = q;
			}

			// try the other rotation
			q = q22 * q12;
			temp = OverlapPlaneLine(l1a, l1b, q.rotate(e1a), q.rotate(e1b)) * OverlapPlaneLine(l2a, l2b, q.rotate(e2a), q.rotate(e2b));
			if ( temp > overlap ) {
				overlap = temp;
				best_rotation = q;
			}
		}
	}
	
	//***************************
	// test the best rotation
	if ( overlap < EPS )
		return false;

	// convert the rotation to axis angle
	best_rotation.toAxisAngle( rotation );

	// flip the rotation if angle > pi
	double minr_angle = len(rotation);
	if ( minr_angle > M_PI ) {   
		minr_angle = 2*M_PI - minr_angle;
		rotation = - norm(rotation) * minr_angle;
	}

	if ( minr_angle > M_PI )
		return false;
	else
		return true;
}

// ladybug flip and scale a point
Vec2d ladybugflip( Vec2d point, int h, double zoom )
{
	point /= zoom;
	double temp = point[1];
	point[1] = h - point[0];
	point[0] = temp;

	return point;
}

// return the closest segment approach (pc1,pc2) from two segments (p1,p2) and (p3,p4)
//
void closest_segment_approach (Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4, Vec3d &pc1, Vec3d &pc2 )
{
	Vec3d p13 = p1-p3;
	Vec3d p21 = p2-p1;
	Vec3d p43 = p4-p3;

	double d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
	double d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
	double d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
	double d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
	double d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];
	
	double denom = d2121 * d4343 - d4321 * d4321;

	if (abs(denom)<EPSILON) {
		pc1 = p1;
		pc2 = p3;
		if (len(p4-p1)<len(p3-p1))
		pc2 = p4;
		return;
	}

	double numer = d1343 * d4321 - d1321 * d4343;

	double mua = 0.0;
	double mub = 0.0;

	mua = numer / denom;
	mub = (d1343 + d4321 * mua) / d4343;

	pc1 = p1+mua*p21;
	pc2 = p3+mub*p43;

}

// return the middle line of two 3D lines, i.e the line defined by two middle points p and q
// where p and q are computed by projecting any two points of the first line onto the second line
// and taking the middle of the corresponding line segments
// the output is a line <lp3,ldir3>
//
void middleLine ( Vec3d lp1, Vec3d ldir1, Vec3d lp2, Vec3d ldir2, Vec3d &lp3, Vec3d &ldir3 )
{
	// pick a point on line 1 and project it on line 2
	Vec3d lp12_1 = projectPointLine( lp1, lp2, ldir2 );

	// take the middle segment
	Vec3d p = (lp1 + lp12_1) / 2.0;

	// pick another point on line 1 and project it on line 2
	Vec3d lp12_2 = projectPointLine( lp1 + 100.0 * ldir1, lp2, ldir2 );

	// take the middle point
	Vec3d q = (lp1 + 100.0 * ldir1 + lp12_2) / 2.0;

	// compute the corresponding line
	lp3 = p;

	if ( len( p-q ) < EPS )
		ldir3 = Vec3d(0,0,0); // the two lines where not skew!
	else
		ldir3 = norm(q-p);
}

double toRadians ( double a )
{
	return a * M_PI / 180.0;
}

double toDegrees (double a)
 {return a * 180.0/M_PI;}

// return the distance between a 3D point and a 3D line
double Distance (Vec3d point, Vec3d line_point, Vec3d line_dir)
{
	line_dir = norm(line_dir);
	Vec3d ut = cross(line_dir,point-line_point);
	if (len(ut)<EPSILON)
		return 0.0;
	ut = norm(ut);
	Vec3d vt = norm(cross(line_dir,ut));
	return fabs(dot(point-line_point,vt));
}
	
// intersect the plane (P,n) with the cube centered in Q and of width '2*w'
// solutions are written into 'vertices'
int intersectPlaneCube (Vec3d P, Vec3d n, Vec3d C, double w, std::vector<Vec3d> &vertices)
{
	double lambda;
	vertices.clear();
	Vec3d V = C + Vec3d(w,w,w);
	Vec3d solution;
	solution = intersectRayPlane(V,Vec3d(-1,0,0),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);
	solution = intersectRayPlane(V,Vec3d(0,-1,0),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);
	solution = intersectRayPlane(V,Vec3d(0,0,-1),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);

	V = C + Vec3d(-w,-w,w);
	solution = intersectRayPlane(V,Vec3d(1,0,0),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);
	solution = intersectRayPlane(V,Vec3d(0,1,0),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);
	solution = intersectRayPlane(V,Vec3d(0,0,-1),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);

	V = C + Vec3d(-w,w,-w);
	solution = intersectRayPlane(V,Vec3d(1,0,0),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);
	solution = intersectRayPlane(V,Vec3d(0,-1,0),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);
	solution = intersectRayPlane(V,Vec3d(0,0,1),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);

	V = C + Vec3d(w,-w,-w);
	solution = intersectRayPlane(V,Vec3d(-1,0,0),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);
	solution = intersectRayPlane(V,Vec3d(0,1,0),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);
	solution = intersectRayPlane(V,Vec3d(0,0,1),P,n,lambda);
	if ((lambda >= 0) && (lambda <= w))
		vertices.push_back(solution);

	return vertices.size();
}

double Acos (double a)
{
	return acos(MIN(MAX(-1.0,a),1.0));
}

double Asin (double a)
{
	return asin(MIN(MAX(-1.0,a),1.0));
}

double eval3rdDegree (double p, double q, double x)
{
	return x*x*x + p*x + q;
}

int roundi (double a)
{
	int _a = int(a);

	if (a-_a > 0.5)
		return _a+1;
	else
		return a;
}

bool xor (bool a, bool b)
{
	if (a && !b)
		return true;
	if (b && !a)
		return true;
	return false;
}

Vec3d fromSphericToEuler (Vec3d origin, SphericalCoord sp)
{
	return Vec3d (
	origin[0] - sp.rho * sin (sp.phi) * cos (sp.theta),
	origin[1] - sp.rho * sin (sp.phi) * sin (sp.theta),
	origin[2] - sp.rho * cos (sp.phi)
	);
}

// return the i-th matrix of the tensor, i=0,1,2
//
Mat3d TrifocalTensor::Ti (int n)
{
	int i,j;

	Mat3d T;

	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {

			T[i][j] = _t[9*n+3*i+j];
		}
	}

	return T;
}

// generates the three fundamental matrices from the epipoles
void TrifocalTensor::getFundamentalMatrices ()
{

	_F21 = crossProductMatrix(_e1) * mult(_e2,false);

	_F31 = crossProductMatrix(_e2) * mult(_e1,true);

}

//-- assuming that the matrix of the first camera is P = [ I | 0 ]
Vecd TrifocalTensor::tensorFromCameraMatrices (Matd P1, Matd P2)
{
	Vecd t(27,0.0);

	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			for (int k=0;k<3;k++) {

				t[9*i+3*j+k] = P1[j][i]*P2[k][3] - P1[j][3]*P2[k][i];
			}
		}
	}


	//sanity check
	/*_t = t;

	printf("tensor from camera matrices sanity check:\n\n");
	i=2;
	printMatrix(Ti(i));
	Vec3d ai = Vec3d(col(P1,i)[0],col(P1,i)[1],col(P1,i)[2]);
	Vec3d bi = Vec3d(col(P2,i)[0],col(P2,i)[1],col(P2,i)[2]);
	Vec3d a4 = Vec3d(col(P1,3)[0],col(P1,3)[1],col(P1,3)[2]);
	Vec3d b4 = Vec3d(col(P2,3)[0],col(P2,3)[1],col(P2,3)[2]);

	printf("P1:\n");
	printMatrix(P1);
	printf("P2:\n");
	printMatrix(P2);

	printf("ai: "); printMatrix(ai);
	printf("bi: "); printMatrix(bi);
	printf("a4: "); printMatrix(a4);
	printf("b4: "); printMatrix(b4);
	printMatrix(simpleProductMatrix(ai,b4) - simpleProductMatrix(a4,bi));
	printf("\n\n");*/

	return t;
}

//-- assuming that the matrix of the first camera is P = [ I | 0 ]
void TrifocalTensor::getCameraMatrices ()
{

	printf("** e1: %.4f %.4f %.4f\n",_e1[0],_e1[1],_e1[2]);
	printf("** e2: %.4f %.4f %.4f\n",_e2[0],_e2[1],_e2[2]);

	Vec3d e1 = norm(_e1);
	Vec3d e2 = norm(_e2);

	_P1 = Matd(3,4), _P2 = Matd(3,4), _P3 = Matd(3,4);
	resetMatrix(_P1);
	resetMatrix(_P2);
	resetMatrix(_P3);

	Matd I = Matd(3,3,vl_I);

	// _P1 = [ I | 0 ]
	sub(_P1,3,3) = I;
	col(_P1,3) = Vec3d (0.0, 0.0, 0.0);
	printf("Camera matrix P1 (from tensor and epipoles):\n");
	printMatrix(_P1);

	// _P2
	sub(_P2,3,3) = mult(e2,false);
	col(_P2,3) = e1;
	printf("Camera matrix P2 (from tensor and epipoles):\n");
	printMatrix(_P2);

	// _P3
	sub(_P3,3,3) = (simpleProductMatrix(e2,e2) - I) * mult(e1,true);
	col(_P3,3) = e2;
	printf("Camera matrix P3:\n");
	printMatrix(_P3);
}

Mat3d TrifocalTensor::mult (Vec3d e, bool transposed)
{
	Mat3d T1,T2,T3;

	if (transposed) {
		T1 = trans(Ti(0));
		T2 = trans(Ti(1));
		T3 = trans(Ti(2));
	} else {
		T1 = Ti(0);
		T2 = Ti(1);
		T3 = Ti(2);
	}

	Vecd v1, v2, v3;

	v1 = T1 * e;
	v2 = T2 * e;
	v3 = T3 * e;

	Mat3d D;
	col(D,0) = v1;
	col(D,1) = v2;
	col(D,2) = v3;

	return D;
}

void TrifocalTensor::getCameraCenterPoint (Vec3d &C1, Vec3d &C2, Vec3d &C3)
{
	C1 = getCenterPointFromCameraMatrix(_P1);
	C2 = getCenterPointFromCameraMatrix(_P2);
	C3 = getCenterPointFromCameraMatrix(_P3);
}

void TrifocalTensor::getCameraRotationMatrix (Mat3d &R1, Mat3d &R2, Mat3d &R3)
{
	R1 = getRotationFromCameraMatrix (_P1);
	R2 = getRotationFromCameraMatrix (_P2);
	R3 = getRotationFromCameraMatrix (_P3);
}

int solve8thDegree (std::vector<double> &coeffs, std::vector<double> &solutions, double a, double b, int int_res)
{
	assert(a<b);
	double resolution = 1.0/powf(10,int_res);

	double step = MAX((b-a)/10000,resolution);

	double x=a;

	while (x<b-step) {

		if (sign(eval8thDegree(coeffs,x)*eval8thDegree(coeffs,x+step)) > 0) {
			x += step;
			continue;
		}

		// there is a root between x and x+step
		solutions.push_back(solve8thDegree(coeffs,x,x+step,resolution));

		x += step;

	}

	return solutions.size();

}

double solve8thDegree (std::vector<double> &coeffs, double a, double b, double resolution)
{
	if ((b-a) > resolution) {

		double val1 = eval8thDegree(coeffs,a);
		double val2 = eval8thDegree(coeffs,(a+b)/2);
		double val3 = eval8thDegree(coeffs,b);

		if (sign(val1*val2) < 0) 
			return solve8thDegree(coeffs,a,(a+b)/2,resolution);
		else
			return solve8thDegree(coeffs,(a+b)/2,b,resolution);
	} else
		return (a+b)/2;
}

double eval8thDegree (std::vector<double> coeffs, double a)
{
	double a2 = a*a;
	double a3 = a2*a;
	double a4 = a2*a2;
	double a5 = a4*a;
	double a6 = a4*a2;
	double a7 = a4*a3;
	double a8 = a4*a4;
	return coeffs[0]+coeffs[1]*a+coeffs[2]*a2+coeffs[3]*a3+coeffs[4]*a4+coeffs[5]*a5+coeffs[6]*a6+coeffs[7]*a7+coeffs[8]*a8;
}

// from numerical recipes in C, chapters 6.1, 6.2
//
double gammln( double xx )
{
	double x,y,tmp,ser;
	static double cof[6]={76.18009172947146,-86.50532032941677,
		24.01409824083091,-1.231739572450155,
		0.1208650973866179e-2,-0.5395239384953e-5};
	int j;
	y=x=xx;
	tmp=x+5.5;
	tmp -= (x+0.5)*log(tmp);
	ser=1.000000000190015;
	for (j=0;j<=5;j++) ser += cof[j]/++y;
	return -tmp+log(2.5066282746310005*ser/x);
}

//Returns the incomplete gamma function Q(a, x) = 1 - P(a, x).
double gammq(double a, double x)
{
//void gcf(double *gammcf, double a, double x, double *gln);
//void gser(double *gamser, double a, double x, double *gln);
	//void nrerror(char error_text[]);
	
	double gamser,gammcf,gln;
	if (x < 0.0 || a <= 0.0) 
		LOG(LEVEL_ERROR,"Invalid arguments in routine gammq");
	if (x < (a+1.0)) { //Use the series representation
		gser(&gamser,a,x,&gln);
	return 1.0-gamser; //and take its complement.
	} else { //Use the continued fraction representation.
	gcf(&gammcf,a,x,&gln);
	return gammcf;
	}
}

//Returns the incomplete gamma function P(a, x) evaluated by its series representation as gamser.
//Also returns ln Ã(a) as gln.
void gser(double *gamser, double a, double x, double *gln)
{
	//double gammln(double xx);
	//void nrerror(char error_text[]);
	int n;
	double sum,del,ap;
	*gln=gammln(a);
	if (x <= 0.0) {
		if (x < 0.0) 
			LOG(LEVEL_ERROR,"x less than 0 in routine gser");
		*gamser=0.0;
		return;
	} else {
		ap=a;
		del=sum=1.0/a;
		for (n=1;n<=ITMAX;n++) {
			++ap;
			del *= x/ap;
			sum += del;
			if (fabs(del) < fabs(sum)*EPS) {
				*gamser=sum*exp(-x+a*log(x)-(*gln));
				return;
			}
		}
		LOG(LEVEL_ERROR,"a too large, ITMAX too small in routine gser");
		return;
	}
}


//Returns the incomplete gamma function Q(a, x) evaluated by its continued fraction representation
//as gammcf. Also returns lnÃ(a) as gln.
void gcf(double *gammcf, double a, double x, double *gln)
{
	//double gammln(double xx);
	//void nrerror(char error_text[]);
	int i;
	double an,b,c,d,del,h;
	*gln=gammln(a);
	b=x+1.0-a; //Set up for evaluating continued fraction
		//by modified Lentz’s method (§5.2)
		//with b0 = 0.
		c=1.0/FPMIN;
	d=1.0/b;
	h=d;
	for (i=1;i<=ITMAX;i++) { //Iterate to convergence.
		an = -i*(i-a);
	b += 2.0;
	d=an*d+b;
	if (fabs(d) < FPMIN) d=FPMIN;
	c=b+an/c;
	if (fabs(c) < FPMIN) c=FPMIN;
	d=1.0/d;
	del=d*c;
	h *= del;
	if (fabs(del-1.0) < EPS) break;
	}
	if (i > ITMAX) LOG(LEVEL_ERROR,"a too large, ITMAX too small in gcf");
	*gammcf=exp(-x+a*log(x)-(*gln))*h; //Put factors in front.
}


// uniform random number generator
// from numerical recipes in C - chapter 7.1
//
double ran1(long *idum)
// Minimal random number generator of Park and Miller with Bays-Durham shuffle and added
// safeguards. Returns a uniform random deviate between 0.0 and 1.0 (exclusive of the endpoint
// values). Call with idum a negative integer to initialize; thereafter, do not alter idum between
// successive deviates in a sequence. RNMX should approximate the largest floating value that is
// less than 1.
{
	int j;
	long k;
	static long iy=0;
	static long iv[NTAB];
	double temp;
	if (*idum <= 0 || !iy) { //Initialize.
		if (-(*idum) < 1) *idum=1; //Be sure to prevent idum = 0.
		else *idum = -(*idum);
		for (j=NTAB+7;j>=0;j--) { //Load the shuffle table (after 8 warm-ups).
			k=(*idum)/IQ;
			*idum=IA*(*idum-k*IQ)-IR*k;
			if (*idum < 0) *idum += IM;
			if (j < NTAB) iv[j] = *idum;
		}
		iy=iv[0];
	}
	k=(*idum)/IQ; //Start here when not initializing.
	*idum=IA*(*idum-k*IQ)-IR*k; //Compute idum=(IA*idum) % IM without over-
	if (*idum < 0) *idum += IM; //flows by Schrage’s method.
	j=iy/NDIV; //Will be in the range 0..NTAB-1.
	iy=iv[j]; //Output previously stored value and refill the
	iv[j] = *idum; //shuffle table.
	if ((temp=AM*iy) > RNMX) return RNMX; //Because users don’t expect endpoint values.
	else return temp;
}


// gaussian random number generator
// from numerical recipes in C - chapter 7.2
//
double gasdev(long *idum)
//Returns a normally distributed deviate with zero mean and unit variance, using ran1(idum)
//as the source of uniform deviates.
{
	double ran1(long *idum);
	static int iset=0;
	static double gset;
	double fac,rsq,v1,v2;
	if (*idum < 0) iset=0; //Reinitialize.
	if (iset == 0) { //We don’t have an extra deviate handy, so
		do {
			v1=2.0*ran1(idum)-1.0; //pick two uniform numbers in the square extending
			v2=2.0*ran1(idum)-1.0; //from -1 to +1 in each direction,
			rsq=v1*v1+v2*v2; //see if they are in the unit circle,
		} while (rsq >= 1.0 || rsq == 0.0); //and if they are not, try again.
		fac=sqrt(-2.0*log(rsq)/rsq);
		//Now make the Box-Muller transformation to get two normal deviates. Return one and
		//save the other for next time.
		gset=v1*fac;
		iset=1; //Set flag.
			return v2*fac;
	} else { //We have an extra deviate handy,
		iset=0; //so unset the flag,
		return gset; //and return it.
	}
}

// generate a random rotation
// idum is the seed for random number generation
//
Quaternion random_rotation( long &idum )
{
	Vec3d v(0,0,0);
	
	while ( len(v) < EPS ) {
		v = Vec3d(ran1(&idum) - 0.5, ran1(&idum) - 0.5, ran1(&idum)/2.0 + 0.5);
	}

	v = norm(v);

	double angle = ran1(&idum) * M_PI;

	return Quaternion( v, angle );
}


// test the random generators and output results in a file for matlab display
// in matlab, type "load ran1.txt" then "hist(ran1,100)"

void testRandomGenerators()
{
	long idum = -1;

	// uniform distribution
	FILE *f = fopen( "ran1.txt", "w" );
	
	for (int i=0;i<10000;i++) {
		double r = ran1( &idum );
		fprintf(f, "%f\n", r );
	}

	fclose( f );

	// gaussian distribution
	idum = -1;
	f = fopen( "gasdev.txt", "w" );
	
	for ( i=0;i<10000;i++) {
		double r = gasdev( &idum );
		fprintf(f, "%f\n", r );
	}

	fclose( f );

}

	
	

