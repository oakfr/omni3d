#include "basic.h"
#include "geom/geom3D.h"
#include "util/util.h"

Vec2dTable::Vec2dTable (int w, int h)
{
	m_w=w;
	m_h=h;

	for (int i=0;i<h;i++)
		for (int j=0;j<w;j++)
			m_vector.push_back(Vec2d(0,0));
}

void Vec2dTable::write (const char *filename)
{
	FILE *f = fopen(filename,"w");

	fprintf(f,"%d %d\n",m_w,m_h);
	Vec2d v;

	for (int i=0;i<m_h;i++) {
		for (int j=0;j<m_w;j++) {
			v = getElement(i,j);
			fprintf(f,"%f %f ",v[0],v[1]);
		}
		fprintf(f,"\n");
	}

	fclose(f);
}

bool Vec2dTable::read (const char *filename)
{
	FILE *f = fopen(filename,"r");

	if (f == NULL)
		return false;

	int w,h;
	double x,y;
	fscanf(f,"%d%d",&w,&h);
	m_w = w;
	m_h = h;

	for (int i=0;i<m_h;i++) {
		for (int j=0;j<m_w;j++) {
			fscanf(f,"%lf%lf",&x,&y);
			setElement(i,j,Vec2d(x,y));
		}
	}

	return true;
}

// rescale the table to (0,0) (m_w,m_h)
void Vec2dTable::rescale()
{	
	assert ((m_w > 0) && (m_h > 0));
	Vec2d v = getElement(0,0);
	int i,j;

	double min_w=v[0],max_w=v[0],min_h=v[1],max_h=v[1];

	for (i=0;i<m_h;i++) {
		for (j=0;j<m_w;j++) {
			v = getElement(i,j);
			min_w = MIN(min_w,v[0]);
			max_w = MAX(max_w,v[0]);
			min_h = MIN(min_h,v[1]);
			max_h = MAX(max_h,v[1]);
		}
	}

	double scale_w = m_w/(max_w-min_w);
	double scale_h = m_h/(max_h-min_h);

	printf("rescaling: translation [%f,%f]  scale: [%f,%f]\n",min_w,min_h,scale_w,scale_h);

	for (i=0;i<m_h;i++) {
		for (j=0;j<m_w;j++) {
			v = getElement(i,j)-Vec2d(min_w,min_h);
			v[0] = v[0] * scale_w;
			v[1] = v[1] * scale_h;
			setElement(i,j,v);
		}
	}
}





// ----------------------------------------------
// ------------ Vertex --------------------------

Vertex::Vertex ()
{
	_position = Vec3d(0,0,0);
	_id = 0;
	_weight = 0;
	_visible = false;
}

Vertex::Vertex (double _x, double _y, double _z) 
{
	_position = Vec3d(_x,_y,_z);
	_id = 0; _weight = 0;
	_visible = false;
}
Vertex::Vertex (double _x, double _y, double _z, int id) 
{
	_position = Vec3d(_x,_y,_z);
	_id = id; _weight = 0;
	_visible = false;
}

Vertex::Vertex (const Vertex &v)
{
	_position = v._position;
	_id = v._id;
	_weight = v._weight;
	for (int i=0;i<v._edges.size();i++)
		_edges.push_back(v._edges[i]);
	_visible = false;
}

Vertex::~Vertex()
{
}

void Vertex::unit () 
{
	_position = norm(_position);
}

Vec3d Vertex::add (Vertex v)
{
	return _position + v._position;
}

Vec3d Vertex::add (Vertex *v)
{
	return _position + v->_position;
}

Vec3d Vertex::sub (Vertex v)
{
	return _position - v._position;
}

Vec3d Vertex::sub (Vertex *v)
{
	return _position - v->_position;
}

void Vertex::mult (double d)
{
	_position *= d;
}

void Vertex::div (double d)
{
	_position /= d;
}

void Vertex::rotate (Vec3d center, Vec3d axis, double angle)
{
	Quaternion q(cos(angle/2),sin(angle/2)*axis);
	_position = center + q.rotate(_position-center);
}


/*Vertex Vertex::operator+(const Vertex &p) const 
{
	Vertex w = p;
	w._position = _position + p.position;
	_position = Vec3d(0,1,0) + Vec3d(0,0,1);
	return *this;
}

Vertex Vertex::operator+(const int &p) const 
{
	_position[2] += p;
	return *this;
}

Vertex Vertex::operator-(const Vertex &p) const 
{
	_position = _position - p._position;
	return *this;
}

Vertex Vertex::operator*(const double d) const 
{
	_position = _position * d;
	return *this;
}

Vertex Vertex::operator/(const double d) const 
{
	_position = _position / d;
	return *this;
}
*/

bool Vertex::operator==(const Vertex *v) const 
{
	return (_position == v->_position);
}

bool Vertex::operator==(const Vertex v) const 
{
	return (_position == v._position);
}

double Vertex::length () 
{
	return len(_position);
}

void Vertex::scal (double r)
{
	double fact = r/len(_position);
	_position *= fact;
}

double Vertex::distance (Vertex *v)
{
	return len(_position-v->_position);
}

double Vertex::distance (Vertex v)
{
	return len(_position-v._position);
}

double Vertex::Dot (Vertex v) 
{
	return dot(_position,v._position);
}

Vec3d Vertex::Cross (Vertex u, Vertex v) 
{
	return cross(u._position,v._position);
}

Vec3d Vertex::Cross (Vertex v) 
{
	return cross(_position,v._position);
}

bool Vertex::contains (Edge *e) 
{
	for (int i=0;i<_edges.size();i++) {
		if (e->equal(_edges[i])) return 1;
	}
	return 0;
}

int Vertex::n_visible_edges ()
{
	int counter = 0;

	for (int i=0;i<_edges.size();i++) {
		if ( _edges[i]->_visible )
			counter++;
	}

	return counter;
}

void Vertex::insert (Edge *e) 
{
	if (!this->contains(e)) 
		_edges.push_back(e);
}

bool Vertex::equal (Vertex *v) 
{
	return (_id == v->_id );
	//if (_position == v->_position) return 1;
	//if (distance(*v) < GEOM_RESOLUTION) 
	//	return 1;
	//return 0;
}

bool Vertex::equal (Vertex v) 
{
	if (_position == v._position) return 1;
	if (distance(v) < GEOM_RESOLUTION) 
		return 1;
	return 0;
}

bool Vertex::belongs (Edge *e) 
{
	if (equal(e->_a) || equal(e->_b)) return 1;
	return 0;
}

bool Vertex::belongs (vertexVector vector)
{
	for (int i=0;i<vector.size();i++) {
		if (this->equal(vector[i])) return 1;
	}
	return 0;
}

bool Vertex::belongs_id (vertVector vector)
{
	for (int i=0;i<vector.size();i++) {
		if (_id == vector[i]._id) return 1;
	}
	return 0;
}

bool Vertex::isInside (Vertex _a, Vertex _b)
{
	if ((_id > _a._id) && (_id < _b._id)) return true;
	return false;
}

int Vertex::findVertexInList (vertexVector V) 
{
	for (int i=0;i<V.size();i++) {
		if (equal(V[i])) return i;
	}
	return -1;
}

void Vertex::set (Vertex _a)
{
	_position = _a._position;
}

void Vertex::set (double _x, double _y, double _z)
{
	_position = Vec3d(_x,_y,_z);
}

bool Vertex::isOrthogonal (Vertex *v1, Vertex *v2)
{
	Vec3d w = v1->getPosition()-v2->getPosition();
	double d = dot(getPosition(),w);
	d /= len(w);
	d /= length();
	if (fabs(d) < 0.1) return 1;
	return 0;
}

bool Vertex::isAligned (Vertex *_a, Vertex *_b, Vertex *c)
{
	Vec3d r1 = norm(_a->sub(_b));
	Vec3d r2 = norm(_a->sub(c));
	double d = dot(r1,r2);
	if (fabs(d-1.0) < 0.001) return 1;
	return 0;
}

Vertex* Vertex::sibling (Edge *e) 
{
	if (equal(e->_a)) return e->_b;
	else if (equal(e->_b)) return e->_a;
	else {
		printf("Problem! ");
		return e->_a;
	}
}

Edge* Vertex::sibling (Vertex *v) 
{
	for (int i=0;i<_edges.size();i++) {
		if (v->belongs(_edges[i])) return _edges[i];
	}
	return NULL;
}

void Vertex::print(FILE *out)
{
	fprintf(out,"%d %.4f %.4f %.4f\n",_id,_position[0],_position[1],_position[2]);
}

bool Vertex::read (const char *s)
{
	return (sscanf(s,"%d%lf%lf%lf",&_id,&_position[0],&_position[1],&_position[2]) == 4);
}

bool Vertex::read (FILE *fin)
{
	int id;
	double a,b,c;
	int result = fscanf(fin,"%d%lf%lf%lf",&id,&a,&b,&c);
	if (result != 4)
		return false;
	_id = id;
	_position = Vec3d(a,b,c);
	_weight = 0;
	return true;
}

void Vertex::print() 
{
	print(stdout);
}

bool Vertex::isOnSegment (Edge *e)
{
	if (this->equal(e->_a)) return 0;
	if (this->equal(e->_b)) return 0;

	Vec3d w1 = norm(sub(e->_a));
	Vec3d w2 = norm(sub(e->_b));
	double d = dot(w1,w2);
	if (fabs(d+1) < 0.001) return 1;
	return 0;
}

double Vertex::angle (Vertex a)
{
	// return the angle between 'this' and a
	double d = dot(_position,a._position);
	d /= a.length() * length();
	return acos(d);
}

void Vertex::mult (Mat3d mat) {
	
	_position = mat * _position;
}

void Vertex::computeWeight()
{
	_weight=0.0;

	if (_edges.size() <= 1) { return;}

	if (_edges.size() == 2) {
		_weight = vertexFunction(_edges[0],_edges[1]);
		return;
	}

	Edge *edge1,*edge2,*edge3;
	edge1 = new Edge(); edge2 = new Edge(); edge3 = new Edge();
	unsigned int i=0,j=0,k=0;
	
	int counter=0;
	for (i=0;i<_edges.size();i++) {
		edge1 = _edges[i];

		for (j=0;j<_edges.size();j++) {
			edge2 = _edges[j];
			if (edge1 == edge2) continue;
			for (k=j+1;k<_edges.size();k++) {
				edge3 =  _edges[k];
				if (edge1 == edge3) continue;
				if (edge2 == edge3) continue;
				_weight += vertexFunction(edge1,edge2,edge3);
				counter++;
			}
		}
	}

	_weight /= counter;
}

double Vertex::angleFunction (double x)
{
	if (x<0.001) return 1.0;
	return (sin(x)*sin(x)/(x*x));
}


double Vertex::vertexFunction (Edge *e1, Edge *e2)
{
	
	Vec3d u1,u2;
	u1 = norm(toVector(e1));
	u2 = norm(toVector(e2));
	
	return angleFunction(dot(u1,u2));
}

double Vertex::vertexFunction (Edge *e1, Edge *e2, Edge *e3)
{
	Vec3d u1 = norm(toVector(e1)), u2 = norm(toVector(e2));
	Vec3d u3 = norm(toVector(e3));

	Vec3d cros = norm(cross(u1,u2));
	cros *= vertexFunction (e1,e2);
	
	return fabs(dot(cros,u3));
}

Vec3d Vertex::toVector (Edge *edge)
{
	Vertex *v = sibling(edge);
	Vec3d w = v->_position - _position;
	return w;
}

Vertex* Vertex::next( Face *f )
{
	for (int j=0;j<f->_vertices.size();j++) {
		if ( f->_vertices[j]->_id == _id ) {
			if ( j == f->_vertices.size()-1 )
				return f->_vertices[0];
			else
				return f->_vertices[j+1];
		}
	}

	return NULL;
}

Vertex* Vertex::prev( Face *f )
{
	for (int j=0;j<f->_vertices.size();j++) {
		if ( f->_vertices[j]->_id == _id ) {
			if ( j == 0 )
				return f->_vertices[f->_vertices.size()-1];
			else
				return f->_vertices[j-1];
		}
	}

	return NULL;
}

// ----------------------------------------------
// ------------ WVertex ----------------------------

//bool WVertex::operator<(const WVertex &v) {
//	return (_weight < v._weight);
//}
// ----------------------------------------------
// ------------ Edge2D --------------------------
bool Edge2D::connected (Edge2D &edge, double threshold_dist, double threshold_angle)
{
	Vec2d r1,r2,r3,r4;

	// threshold distance
	if (len(_a-edge._a) < threshold_dist) {
		r1 = _b; r2 = _a; r3 = edge._a; r4 = edge._b;
	} else if (len(_a-edge._b) < threshold_dist) {
		r1 = _b; r2 = _a; r3 = edge._b; r4 = edge._a;
	} else if (len(_b-edge._a) < threshold_dist) {
		r1 = _a; r2 = _b; r3 = edge._a; r4 = edge._b;
	} else if (len(_b-edge._b) < threshold_dist) {
		r1 = _a; r2 = _b; r3 = edge._b; r4 = edge._a;
	} else {
		return false;
	}

	// threshold angle
	double cosangle = dot(norm(r1-r2),norm(r4-r3));
	if (fabs(1-fabs(cosangle)) > threshold_angle)
		return false;

	edge = Edge2D(r1,r4,edge._id);
	return true;
}

void Edge2D::flipHorizontal (double length)
{
	_a[0] = length-_a[0];
	_b[0] = length-_b[0];
}

void Edge2D::flipVertical (double length)
{
	_a[1] = length-_a[1];
	_b[1] = length-_b[1];
}

void Edge2D::flipEdgeDetector(double length)
{
	double temp = _a[0];
	_a[0] = _a[1];
	_a[1] = length-temp;

	temp = _b[0];
	_b[0] = _b[1];
	_b[1] = length-temp;
}

void Edge2D::roundInt(int w, int h)
{
	_a[0] = MIN(w-1,round(_a[0]));
	_a[1] = MIN(h-1,round(_a[1]));
	_b[0] = MIN(w-1,round(_b[0]));
	_b[1] = MIN(h-1,round(_b[1]));
}

bool Edge2D::remove (edge2DVector &edges)
{
	edge2DVector::iterator iter;

	for (iter = edges.begin(); iter != edges.end(); iter++) {
		if (iter->_id == _id) {
			edges.erase(iter);
			return true;
		}
	}

	return false;
}

Vec3d Edge2D::normal()
{
	return norm(cross(Vec3d(_a[0],_a[1],1.0),Vec3d(_b[0],_b[1],1.0)));
}

Vec3d Edge2D::getLine()
{
	double x1 = _a[0], y1 = _a[1], x2 = _b[0], y2 = _b[1];

	if (fabs(x2-x1) < EPSILON) 
		return Vec3d(1.0,0.0,-x1);

	if (fabs(y2-y1) < EPSILON)
		return Vec3d(0.0,1.0,-y1);

	double a = 1.0;
	double b = (x1-x2)/(y2-y1);
	double c = - a*x1 - b*y1;
	
	return Vec3d(a,b,c);
}

// let us put 'p' the interesection
// solve for lambda1 and lambda2 such that:  ap = lambda1 x ab      cp = lambda2 x cd
bool Edge2D::intersect (Edge2D edge, Vec2d &result)
{
	double xa = _a[0], ya = _a[1], xb = _b[0], yb = _b[1];
	double xc = edge._a[0], yc = edge._a[1], xd = edge._b[0], yd = edge._b[1];

	double lambda2 = ((yb-ya)*(xa-xc) - (xb-xa)*(ya-yc))/((xd-xc)*(yb-ya) - (xb-xa)*(yd-yc));
	double lambda1 = (xd-xc)*lambda2/(xb-xa) - (xa-xc)/(xb-xa);

	result = _a + lambda1 * (_b-_a);
		
	return ((0<=lambda1) && (lambda1 <= 1.0) && (0 <= lambda2) && (lambda2 <= 1.0));
}

bool Edge2D::checkInside (int x0, int y0, int x1, int y1)
{
	bool t1 = ((x0 <= _a[0]) && (_a[0] <= x1) && (y0 <= _a[1]) && (_a[1] <= y1));
	bool t2 = ((x0 <= _b[0]) && (_b[0] <= x1) && (y0 <= _b[1]) && (_b[1] <= y1));
	return (t1 & t2);
}

bool Edge2D::clip (int width, int height)
{
	Vec2d res;
	
	if (checkInside(0,0,width-1,height-1))
		return true;
	
	if (_a[0] < 0) {		
		if (intersect( Edge2D(Vec2d(0,0),Vec2d(0,height), 0), res)) {
			_a = res;
			return true;
		}
	}
	
	if (_a[0] > width-1) {
		if (intersect( Edge2D(Vec2d(width,0),Vec2d(width,height), 0), res)) {
			_a = res;
			return true;
		}
	}
	
	if (_a[1] < 0) {
		if (intersect( Edge2D(Vec2d(0,0),Vec2d(width,0), 0), res)) {
			_a = res;
			return true;
		}
	}
	
	if (_a[1] > height-1) {
		if (intersect( Edge2D(Vec2d(0,height),Vec2d(width,height), 0), res)) {
			_a = res;
			return true;
		}
	}
	
	if (_b[0] < 0) {
		if (intersect( Edge2D(Vec2d(0,0),Vec2d(0,height), 0), res)) {
			_b = res;
			return true;
		}
	}
	
	if (_b[0] > width-1) {
		if (intersect( Edge2D(Vec2d(width,0),Vec2d(width,height), 0), res)) {
			_b = res;
			return true;
		}
	}
	
	if (_b[1] < 0) {
		if (intersect( Edge2D(Vec2d(0,0),Vec2d(width,0), 0), res)) {
			_b = res;
			return true;
		}
	}
	
	if (_b[1] > height-1) {
		if (intersect( Edge2D(Vec2d(0,height),Vec2d(width,height), 0), res)) {
			_b = res;
			return true;
		}
	}
	
	return false;
}

double Edge2D::overlap (Edge2D edge, double norm)
{
	double dist = len(midpoint()-edge.midpoint()) / norm;
	double a = fabs(cosangle(edge)) / (dist*dist);
	//printf("\t cosangle: %.3f  length: %.3f  a: %.3f\n",fabs(cosangle(edge)) ,len(midpoint()-edge.midpoint()),a);
	return a;
}

// return the distance between a 2D edge and a 2D point
double Edge2D::distance (Vec2d m)
{
	Vec2d u = norm(toVec());

	return fabs(m[0]*u[1]-m[1]*u[0] + u[0]*_a[1]-u[1]*_a[0]);
}
	
// return the average distance between two edges
double Edge2D::distance (Edge2D edge)
{
	return (distance(edge._a) + distance(edge._b))/2.0;
}
bool Edge2D::testSimilarity (Edge2D edge, double angle_threshold, double dist_threshold)
{
	double dist = distance(edge)/MAX(length(),edge.length());

	if (dist > dist_threshold)
		return false;

	double a = fabs(cosangle(edge));
	if (fabs(1-a) > angle_threshold)
		return false;

	return true;
}

void Edge2D::scale(int x, int y, double zoom)
{
	_a = _a/zoom + Vec2d(x,y);
	_b = _b/zoom + Vec2d(x,y);
}

void Edge2D::draw(const float color[3], int size)
{
	glColor3fv(color);
	glLineWidth(size);
	glBegin(GL_LINES);
	glVertex();
	glEnd();
	glLineWidth(1);
}

// return the point-to-point distance 
double Edge2D::p2p_distance ( Edge2D &edge, Vec2d &a, Vec2d &b )
{
	double d = 0.0;

	if ( len( _a - edge._a ) < len( _a - edge._b ) ) {
		a = _a - edge._a;
		b = _b - edge._b;
	} else {
		a = _a - edge._b;
		b = _b - edge._a;
	}

	return ( len ( a ) + len ( b ) );
}

// return true if one of the end point of the edge is getting close to the limits of the image (+/- threshold in %)
bool Edge2D::checkBorder (int width, int height, double threshold)
{
	// test first end point
	double d = fabs(_a[0]-width)/width;
	if (d < threshold) return true;
	d = fabs(_a[1]-height)/height;
	if (d < threshold) return true;

	// test second end point
	d = fabs(_b[0]-width)/width;
	if (d < threshold) return true;
	d = fabs(_b[1]-height)/height;
	if (d < threshold) return true;
	
	return false;
}

// extend the current edge to make it the same length as the input edge
void Edge2D::extend( Edge2D edge )
{
	Vec2d a = edge._a;
	Vec2d b = edge._b;
	Vec2d center = ( _a + _b ) / 2.0;

	double l = edge.length();

	if ( dot( a - center, _a - center ) > 0 )
		_a = center + len( a - center ) * norm( _a - center );
	if ( dot( b - center, _a - center ) > 0 )
		_a = center + len( b - center ) * norm( _a - center );

	if ( dot( a - center, _b - center ) > 0 )
		_b = center + len( a - center ) * norm( _b - center );
	if ( dot( b - center, _b - center ) > 0 )
		_b = center + len( b - center ) * norm( _b - center );
}
	
// ----------------------------------------------
// ------------ Edge ----------------------------
Edge::~Edge()
{
	if (_keep)
		return;

	delete _a;
	delete _b;
}

void Edge::info()
{
	dir = norm(_a->getPosition()-_b->getPosition());
	_horizontal = _vertical = false;
	if (fabs(dir[2]) < COS_85)
		_horizontal = true;
	else if ( (fabs(dir[0]) < COS_85) && (fabs(dir[1]) < COS_85) )
		_vertical = true;

	_correspondenceId = -1;
	_flat = false;
}

Edge::Edge ()
{
	_a = new Vertex();
	_b = new Vertex();
	_weight = 0;
	_id = 0;
	_visible = false;
	_a->_edges.push_back(this);
	_b->_edges.push_back(this);
	_sharpness = 0.0;
	_keep = false;
	info();
}

Edge::Edge (Vec3d a, Vec3d b)
{
	_a = new Vertex();
	_a->_position = a;
	_b = new Vertex();
	_b->_position = b;
	_visible = false;
	_a->_edges.push_back(this);
	_b->_edges.push_back(this);
	_sharpness = 0.0;
	_keep = false;
	info();
}

Edge::Edge (int id, Vec3d a, Vec3d b)
{
	_a = new Vertex();
	_a->_position = a;
	_b = new Vertex();
	_b->_position = b;
	_visible = false;
	
	_id = id;
	_a->_edges.push_back(this);
	_b->_edges.push_back(this);
	_sharpness = 0.0;
	_keep = false;
	info();
}

Edge::Edge (int id, Vec3d a, Vec3d b, bool visible)
{
	_a = new Vertex();
	_a->_position = a;
	_b = new Vertex();
	_b->_position = b;
	_visible = visible;
	
	_id = id;
	_a->_edges.push_back(this);
	_b->_edges.push_back(this);
	_sharpness = 0.0;
	_keep = false;
	info();
}

Edge::Edge (Vertex *a, Vertex *b) 
{
	_a = a;
	_b = b;
	_id = 0;
	_weight = 0;
	_visible = false;
	_a->_edges.push_back(this);
	_b->_edges.push_back(this);
	_sharpness = 0.0;
	_keep = false;
	info();
}

Edge::Edge (int id, Vertex *a, Vertex *b) 
{
	_a = a;
	_b = b;
	_weight = 0;
	_id = id;
	_visible = false;
	_a->_edges.push_back(this);
	_b->_edges.push_back(this);
	_sharpness = 0.0;
	_keep = false;
	info();
}

Edge::Edge (Vertex a, Vertex b)
{
	_id = 0;
	_a = new Vertex();
	*_a = a;
	_b = new Vertex();
	*_b = b;
	_weight=0;
	_visible = false;
	_sharpness = 0.0;
	_keep = false;
	info();
}

Edge::Edge (int id, Vertex a, Vertex b)
{
	_a = new Vertex();
	*_a = a;
	_b = new Vertex();
	*_b = b;
	_weight=0;
	_id = id;
	_visible = false;
	_sharpness = 0.0;
	_keep = false;
	info();
}

// return the shortest distance between an edge and a 3D point
double Edge::shortest_distance( Vec3d p )
{
	Vec3d a = _a->getPosition();
	Vec3d b = _b->getPosition();

	if ( length() < EPSILON )
		return len( p - a );

	Vec3d w = cross( p - a, p - b );

	if ( len( w ) < EPSILON )
		return 0.0;

	w = norm( w );

	Vec3d n = norm( cross( norm( b - a ), w ) );

	double d = fabs( dot( p - a, n ) );

	bool criteria = dot( cross( a - p, n ), cross( b - p, n ) ) < 0;

	if ( criteria ) 
		return d;
	else
		return MIN( len( p - a), len( p - b ) );
}

// the weight is defined as the subtended angle of the edge viewed from the <position>
void Edge::computeWeight(Vec3d position)
{
	_weight = fabs(Acos(dot(norm(getA()-position),norm(getB()-position))));

	//Vec3d A = _a->getPosition();
	//double lambda = dot(dir,position-A);
	//_weight =  1.0/len(position-(A+lambda*dir));
}

double Edge::sharpness ()
{
	if (_faces.size() != 2)
		return M_PI/2;

	Face *f1 = _faces[0];
	Face *f2 = _faces[1];

	Vec3d u1 = f1->normal();
	Vec3d u2 = f2->normal();

	return acos(MAX(-1.0,MIN(1.0,dot(u1,u2))));
}

bool Edge::sharp()
{
	double threshold = M_PI/10;

	if (fabs(_sharpness) < threshold)
		return false;

	if (fabs(M_PI-_sharpness) < threshold)
		return false;

	return true;
}

Vec3d Edge::toVector ()
{
	return (_b->getPosition() - _a->getPosition());
}

bool Edge::operator==(const Edge e) const 
{
	return (((e._a == _a)&&(e._b == _b)) || ((e._b == _a)&&(e._a == _b)));
}

bool Edge::equal (Edge *e) 
{
	if ((_a == e->_a) && (_b == e->_b)) return 1;
	if ((_b == e->_a) && (_a == e->_b)) return 1;
	return 0;
}

double Edge::angle(Edge edge)
{
	Vec3d u = norm(_b->getPosition()-_a->getPosition());
	Vec3d v = norm(edge._b->getPosition()-edge._a->getPosition());
	double angle = acos(MIN(1.0,MAX(-1.0,dot(u,v))));

	if (angle > M_PI/2)
		angle = M_PI-angle;

	assert ((angle>=0) & (angle<=M_PI/2));

	return angle;
}

void Edge::rotateMidpoint (double alpha, Vec3d axis)
{
	Vec3d a = _a->getPosition();
	Vec3d b = _b->getPosition();

	_a->rotate((a+b)/2,axis,alpha);
	_b->rotate((a+b)/2,axis,-alpha);
}

void Edge::scale(double scale)
{
	_a->setPosition(scale*_a->getPosition());
	_b->setPosition(scale*_b->getPosition());
}

// test the configuration of two edges with respect to a 3D position
// return false if test fails
bool Edge::testPosition (Edge *edge, Vec3d &position, double angle_threshold)
{
	Vec3d u1 =  (edge->getA() + edge->getB())/2.0 - position;
	Vec3d u2 =  (      getA() +       getB())/2.0 - position;
	u1[2] = 0.0;
	u2[2] = 0.0;
	u1 = norm( u1 );
	u2 = norm( u2 );

	if ( fabs(dot(u1,u2)) > angle_threshold )
		return false;

	return true;
}

bool Edge::testSkew (Edge *edge, double angular_threshold)
{
	Vec3d ldir1 = dir;
	Vec3d ldir2 = edge->dir;

	// if parallel, reject
	if (fabs(dot(ldir1,ldir2)) > angular_threshold)
		return false;

	// if coplanar, reject
	Vec3d u1 = _a->getPosition()-edge->_a->getPosition();
	if (len(u1)<EPSILON)
		return false;
	Vec3d u2 = _a->getPosition()-edge->_b->getPosition();
	if (len(u2)<EPSILON)
		return false;
	Vec3d u3 = _b->getPosition()-edge->_a->getPosition();
	if (len(u3)<EPSILON)
		return false;
	Vec3d u4 = _b->getPosition()-edge->_b->getPosition();
	if (len(u4)<EPSILON)
		return false;

	Vec3d v1 = norm(cross(norm(u1),norm(u2)));
	Vec3d v2 = norm(cross(norm(u3),norm(u4)));

	if (fabs(dot(v1,v2)) > angular_threshold)
		return false;

	return true;
}

bool Edge::testSkew (Edge *edge1, Edge *edge2, double angular_threshold)
{
	if (!testSkew(edge1,angular_threshold))
		return false;

	if (!testSkew(edge2,angular_threshold))
		return false;

	if (!edge1->testSkew(edge2,angular_threshold))
		return false;

	return true;
}	

bool Edge::closest_approach(Edge edge, Edge &closest, bool &skew)
{
	Vec3d p1 = _a->getPosition();
	Vec3d p2 = _b->getPosition();
	Vec3d p3 = edge._a->getPosition();
	Vec3d p4 = edge._b->getPosition();

	closest._a->setPosition(p1);
	closest._b->setPosition(p3);
	if (len(p4-p1)<len(p3-p1))
		closest._b->setPosition(p4);

	double mua = 0.0;
	double mub = 0.0;
	skew = false;
	Vec3d p13 = p1-p3;
	Vec3d p21 = p2-p1;
	Vec3d p43 = p4-p3;

	if ((len(p13)<EPSILON) || (len(p21)<EPSILON))
		return false;

	Vec3d pc1, pc2;
	closest_segment_approach( p1, p2, p3, p4, pc1, pc2 );

	closest._a->setPosition( pc1 );
	closest._b->setPosition( pc2 );

	if ((len(closest.toVector())<EPSILON) || (angle(edge)<EPSILON) || (angle(closest)<EPSILON) || (edge.angle(closest)<EPSILON))
		skew = false;
	else
		skew = true;

	return true;
}

bool Edge::skewness (Edge edge, Edge &closest, double &skewness)
{
	bool skew;
	skewness = 0.0;
	bool ok = closest_approach(edge,closest,skew);

	if (!ok) return false;

	if (!skew) {
		skewness = angle(edge);
		return true;
	}

	Vec3d n = norm(closest.toVector());
	Vec3d u = norm(toVector());
	Vec3d v = norm(edge.toVector());

	Vec3d up = norm(cross(u,n));

	double angle1 = acos(MIN(1.0,MAX(-1.0,dot(u,up))));
	double angle2 = acos(MIN(1.0,MAX(-1.0,dot(v,up))));

	skewness = fabs(angle1-angle2);
	if (skewness > M_PI/2)
		skewness = M_PI-skewness;

	return true;
}



void Edge::clear()
{
	delete _a;
	delete _b;
}

int Edge::findEdgeInList (edgeVector E) 
{
	for (int i=0;i<E.size();i++) {
		if (*this == (*E[i])) return 1;
	}
	return 0;
}

void Edge::insert (edgeVector &E) 
{
	if (!belongs(E)) {
		_id = E.size();
		E.push_back(this);
	}
}

void Edge::glVertex()
{
	glVertex3f(_a->getX(), _a->getY(), _a->getZ());
	glVertex3f(_b->getX(), _b->getY(), _b->getZ());
}

bool Edge::belongs (edgeVector E) 
{
	for (int i=0;i<E.size();i++) {
		if (equal(E[i]))
			return 1;
	}
	return 0;
}

// intersect edge <edge> with the line defined by <a,b = slope,offset>
// accepts intersection if it is embedded in <edge> otherwise, reject
// this method is used for edge clipping
bool Edge2D::intersectSegments (Vec2d a, Vec2d b, double slope, double offset, Vec2d &new_point)
{
	double xa = _a[0];
	double ya = _a[1];
	double xb = _b[0];
	double yb = _b[1];

	double d1 = yb - ya + slope * (xa - xb);

	if (fabs(d1) < EPSILON)
		return false;

	double d2 = xa*yb-xb*ya+offset*(xb-xa);

	new_point[0] = d2/d1;
	new_point[1] = slope * new_point[0] + offset;

	return ((len(new_point-_a) < length()) && (len(new_point-_b) < length()) && (len(new_point-a) < len(b-a)) && (len(new_point-b) < len(b-a)));
}

int Edge::Insert (edgeVector &E, Edge *e) 
{
	if (!belongs(E)) {
		e->_id = E.size();
		E.push_back(e);
		return 1;
	}
	return 0;
}

Vec3d Edge::centroid () 
{
	return (_a->getPosition() + _b->getPosition())/2;
}

double Edge::length() 
{
	return len(_a->getPosition() - _b->getPosition());
}

void Edge::printGeometry()
{
	LOG(LEVEL_INFO, "%d %.2f %.2f %.2f %.2f %.2f %.2f\n",_id, _a->getX(), _a->getY(), _a->getZ(), _b->getX(), _b->getY(), _b->getZ());
}

void Edge::print() 
{
	LOG(LEVEL_INFO, "%d %d %d %d\n", _id, _a->_id, _b->_id, _faces.size());
}

void Edge::print(bool verbose) 
{
	print(stdout,verbose);
}

void Edge::print(FILE *fout, bool verbose) 
{
	fprintf(fout,"%d %d %d %d\n", _id, _a->_id, _b->_id, _faces.size());
	if (verbose) {
		_a->print();
		_b->print();
	}
	for (int i=0;i<_faces.size();i++)
		fprintf(fout,"%d ",_faces[i]->_id);
	fprintf(fout,"\n");
}

void Edge::print(FILE *fout) 
{
	fprintf(fout,"%d %d %d %d ", _id, _a->_id, _b->_id, _faces.size());

	for (int i=0;i<_faces.size();i++)
		fprintf(fout,"%d ",_faces[i]->_id);
	fprintf(fout,"\n");
}

bool Edge::read (FILE *fin, vertexVector &V, faceVector &F)
{
	int id,vertexId1, vertexId2,nFaces,faceId;
	if(fscanf(fin,"%d%d%d%d",&id,&vertexId1,&vertexId2,&nFaces) != 4)
		return false;

	_id = id;
	_a = V[vertexId1];
	_b = V[vertexId2];

	for (int i=0;i<nFaces;i++) {
		if (fscanf(fin,"%d",&faceId) != 1) 
			return false;
		_faces.push_back(F[faceId]);
	}

	return true;
}

void Edge::draw(const float color[3], int linewidth)
{
	glLineWidth(linewidth);
	glColor3fv(color);
	glBegin(GL_LINES);
	_a->glVertex();
	_b->glVertex();
	glEnd();
	glLineWidth(1);
}

// ----------------------------------------------
// ------------ Faces ---------------------------

/*Face::Face()
{
	v[0] = Vertex();
	v[1] = Vertex();
	v[2] = Vertex();
	v[3] = Vertex();
	_id=0;
}

Face::Face (Vertex _a, Vertex _b, Vertex c, Vertex d)
{
	v[0] = _a;
	v[1] = _b;
	v[2] = c;
	v[3] = d;
	normal = (v[0] - v[1]).cross(v[2]-v[1]);
}

Face::Face (int _id, Vertex _a, Vertex _b, Vertex c, Vertex d)
{
	v[0] = _a;
	v[1] = _b;
	v[2] = c;
	v[3] = d;
	normal = (v[0] - v[1]).cross(v[2]-v[1]);
	_id = _id;
}

Face::Face (Vertex _v[4])
{
	v[0] = _v[0];
	v[1] = _v[1];
	v[2] = _v[2];
	v[3] = _v[3];

	normal = (v[0] - v[1]).cross(v[2]-v[1]);
}*/

Face::Face (vertexVector vector, int faceId)
{

	for (int i=0;i<vector.size();i++) {
		if (!vector[i]->belongs(_vertices))
			_vertices.push_back(vector[i]);
	}
	_id = faceId;
	_norm = normal();
	color[0] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
	color[1] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
	color[2] = 20.0 + 200.0 * (double)rand() / RAND_MAX;

}

Face::Face(int id)
{
	_id = id;

	color[0] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
	color[1] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
	color[2] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
}

Face::Face(int id, Vertex *v1, Vertex *v2, Vertex *v3, Vertex *v4)
{
	_id = id;
	_vertices.push_back(v1);
	_vertices.push_back(v2);
	_vertices.push_back(v3);
	_vertices.push_back(v4);

	_norm = normal();
	color[0] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
	color[1] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
	color[2] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
}

Face::Face(int id, Vertex *v1, Vertex *v2, Vertex *v3)
{
	_id = id;
	_vertices.clear();
	_vertices.push_back(v1);
	_vertices.push_back(v2);
	_vertices.push_back(v3);
	_norm = normal();
	color[0] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
	color[1] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
	color[2] = 20.0 + 200.0 * (double)rand() / RAND_MAX;
}

bool Face::equal (Face *f)
{
	return (this->_id == f->_id);
}

bool Face::belongs (faceVector vector)
{
	for (int i=0;i<vector.size();i++) {
		if (this->equal(vector[i])) return 1;
	}
	return 0;
}

Vec3d Face::centroid ()
{
	Vec3d v(0,0,0);
	for (int i=0;i<_vertices.size();i++) {
		v += _vertices[i]->getPosition();
	}
	return v/_vertices.size();
}

Vec3d Face::normal ()
{
	// return _a unit normal vector to the face
	assert(_vertices.size() >= 3);

	Vec3d e1 = _vertices[1]->getPosition() - _vertices[0]->getPosition();
	Vec3d e2 = _vertices[2]->getPosition() - _vertices[0]->getPosition();

	return norm(cross( norm(e1), norm(e2) ) );
}

// return the 2D barycentric coordinates of a point belonging to the face
//
bool Face::worldToBarycentric( Vec3d p, Vec2d &pbar )
{
	if ( _vertices.size() == 3 ) {
	
		Vec3d a = _vertices[0]->getPosition();
		Vec3d b = _vertices[1]->getPosition();
		Vec3d c = _vertices[2]->getPosition();

		pbar[0] = dot(p-a,b-a);
		pbar[1] = dot(p-a,c-a);
		return true;
	}

	if ( _vertices.size() == 4 ) {
	
		Vec3d a = _vertices[0]->getPosition();
		Vec3d b = _vertices[1]->getPosition();
		Vec3d c = _vertices[2]->getPosition();
		Vec3d d = _vertices[3]->getPosition();

		if ( dot(cross(p-b,d-b),cross(a-b,d-b)) > 0 ) {
			pbar[0] = dot(p-a,b-a) / (len(b-a) * len(b-a));
			pbar[1] = dot(p-a,d-a) / (len(d-a) * len(d-a));
		} else {
			pbar[0] = 1.0 - dot(p-c,d-c) / (len(d-c)*len(d-c));
			pbar[1] = 1.0 - dot(p-c,b-c) / (len(b-c)*len(b-c));
		}
		
		return true;
	}

	return false;

}

void Face::subdivide ( double resolution )
{
	Vec3d a,b,c,d;

	if ( _regular ) {

		/*a = _vertices[0]->getPosition();
		b = _vertices[1]->getPosition();
		c = _vertices[2]->getPosition();
		d = _vertices[3]->getPosition();

		Vec3d u = norm(b-a);
		Vec3d v = norm(d-a);

		int n_x = int(len(b-a)/resolution)+1;
		int n_y = int(len(d-a)/resolution)+1;

		double step_x = len(b-a) / n_x;
		double step_y = len(d-a) / n_y;

		for (int i=0;i<n_x;i++) {
			for (int j=0;j<n_y;j++) {
			
				subdivision.push_back( a + i*u + j*v );
				subdivision.push_back( a + (i+1)*u + j*v );
				subdivision.push_back( a + (i+1)*u + (j+1)*v );
				subdivision.push_back( a + i*u + (j+1)*v );
			}
		}
		*/
		//LOG(LEVEL_INFO, "face %d is regular. no subdivisions.", _id );
		return;
	}

	//LOG(LEVEL_INFO, "face %d is not regular. subdividing.", _id);

	subdivision.clear();

	if ( _vertices.size() != 3 && _vertices.size() != 4 )
		return;
	
	int i;
	
	// create the first layer
	Vec3d center = centroid();
	
	if ( _vertices.size() == 3 ) {
		
		subdivision.push_back( center );
		subdivision.push_back( ( _vertices[0]->getPosition() + _vertices[1]->getPosition() ) / 2.0 );
		subdivision.push_back( _vertices[1]->getPosition() );
		subdivision.push_back( ( _vertices[1]->getPosition() + _vertices[2]->getPosition() ) / 2.0 );
		
		subdivision.push_back( center );
		subdivision.push_back( ( _vertices[1]->getPosition() + _vertices[2]->getPosition() ) / 2.0 );
		subdivision.push_back( _vertices[2]->getPosition() );
		subdivision.push_back( ( _vertices[2]->getPosition() + _vertices[0]->getPosition() ) / 2.0 );
		
		subdivision.push_back( center );
		subdivision.push_back( ( _vertices[2]->getPosition() + _vertices[0]->getPosition() ) / 2.0 );
		subdivision.push_back( _vertices[0]->getPosition() );
		subdivision.push_back( ( _vertices[0]->getPosition() + _vertices[1]->getPosition() ) / 2.0 );
	} else {
		
		subdivision.push_back( center );
		subdivision.push_back( ( _vertices[0]->getPosition() + _vertices[3]->getPosition() ) / 2.0 );
		subdivision.push_back( _vertices[0]->getPosition() );
		subdivision.push_back( ( _vertices[0]->getPosition() + _vertices[1]->getPosition() ) / 2.0 );
		
		subdivision.push_back( center );
		subdivision.push_back( ( _vertices[0]->getPosition() + _vertices[1]->getPosition() ) / 2.0 );
		subdivision.push_back( _vertices[1]->getPosition() );
		subdivision.push_back( ( _vertices[1]->getPosition() + _vertices[2]->getPosition() ) / 2.0 );
		
		subdivision.push_back( center );
		subdivision.push_back( ( _vertices[1]->getPosition() + _vertices[2]->getPosition() ) / 2.0 );
		subdivision.push_back( _vertices[2]->getPosition() );
		subdivision.push_back( ( _vertices[2]->getPosition() + _vertices[3]->getPosition() ) / 2.0 );
		
		subdivision.push_back( center );
		subdivision.push_back( ( _vertices[2]->getPosition() + _vertices[3]->getPosition() ) / 2.0 );
		subdivision.push_back( _vertices[3]->getPosition() );
		subdivision.push_back( ( _vertices[3]->getPosition() + _vertices[0]->getPosition() ) / 2.0 );
	}
	
	a = subdivision[0];
	b = subdivision[1];
	c = subdivision[2];
	d = subdivision[3];
	
	double max_length = MAX(len(b-a), MAX(len(c-b), MAX(len(d-c), len(d-a))));
	
	bool done = max_length < resolution;
	
	while ( !done ) {
		
		std::vector<Vec3d> new_points;
		
		done = true;
		
		for (i=0;i<subdivision.size()/4;i++) {
			
			a = subdivision[4*i+0];
			b = subdivision[4*i+1];
			c = subdivision[4*i+2];
			d = subdivision[4*i+3];
			
			max_length = MAX(len(b-a), MAX(len(c-b), MAX(len(d-c), len(d-a))));
	
			if ( max_length > resolution ) {
				
				Vec3d ab = (a+b)/2;
				Vec3d bc = (b+c)/2;
				Vec3d cd = (c+d)/2;
				Vec3d da = (d+a)/2;
				
				Vec3d center = ( ab + cd ) / 2;
				
				if ( ( len(b-a) > resolution || len(c-d) > resolution ) && ( len(d-a) > resolution || len(c-b) > resolution ) ) {
					new_points.push_back( center );
					new_points.push_back( ( a + d ) / 2.0 );
					new_points.push_back( a );
					new_points.push_back( ( a + b ) / 2.0 );
					
					//printf("%f %f %f %f\n", len((a+d)/2-center), len((a+d)/2-a), len((a+b)/2-a), len((a+b)/2-center));

					new_points.push_back( center );
					new_points.push_back( ( a + b ) / 2.0 );
					new_points.push_back( b );
					new_points.push_back( ( b + c ) / 2.0 );
					
					//printf("%f %f %f %f\n", len((a+b)/2-center), len((a+b)/2-b), len((c+b)/2-b), len((c+b)/2-center));

					new_points.push_back( center );
					new_points.push_back( ( b + c ) / 2.0 );
					new_points.push_back( c );
					new_points.push_back( ( c + d ) / 2.0 );
					
					new_points.push_back( center );
					new_points.push_back( ( c + d ) / 2.0 );
					new_points.push_back( d );
					new_points.push_back( ( d + a ) / 2.0 );

				} else if ( len(b-a) < resolution && len(c-d) < resolution ) {

					new_points.push_back( bc );
					new_points.push_back( da );
					new_points.push_back( a );
					new_points.push_back( b );

					new_points.push_back( bc );
					new_points.push_back( c );
					new_points.push_back( d );
					new_points.push_back( da );

				} else if ( len(d-a) < resolution && len(c-b) < resolution ) {

					new_points.push_back( ab );
					new_points.push_back( cd );
					new_points.push_back( d );
					new_points.push_back( a );

					new_points.push_back( ab );
					new_points.push_back( b );
					new_points.push_back( c );
					new_points.push_back( cd );
				}

					done = false;
				
			} else {
				
				new_points.push_back( a );
				new_points.push_back( b );
				new_points.push_back( c );
				new_points.push_back( d );
			}
		}

		subdivision = new_points;
	}
}

// find the closest subdivision to point
//
int Face::getSubdivisionIndex( Vec3d p, double resolution ) 
{
	Vec3d a,b,c,d;

	if ( _regular ) {

		a = _vertices[0]->getPosition();
		b = _vertices[1]->getPosition();
		c = _vertices[2]->getPosition();
		d = _vertices[3]->getPosition();

		Vec3d u = norm(b-a);
		Vec3d v = norm(d-a);

		if ( dot(p-a,u) < 0 || dot(p-a,v) < 0 )
			return -1;

		if ( len(p-a) < EPS )
			return 0;

		double nu = len(p-a) * dot(norm(p-a),u);
		double nv = len(p-a) * dot(norm(p-a),v);

		if (nu > len(p-a) || nv > len(p-v))
			return -1;

		int n_x = int(nu / resolution);
		int n_y = int(nv / resolution);

		int sx = int(len(b-a)/resolution)+1;

		return n_y * sx +  n_x;
	}

	int i;

	for (i=0;i<subdivision.size()/4;i++) {
		
		a = subdivision[4*i+0];
		b = subdivision[4*i+1];
		c = subdivision[4*i+2];
		d = subdivision[4*i+3];

		Vec3d c1 = cross(p-a,b-a);
		Vec3d c2 = cross(p-b,c-b);
		Vec3d c3 = cross(p-c,d-c);
		Vec3d c4 = cross(p-d,a-d);

		if ( dot(c1,c2) > 0 && dot(c2,c3) > 0 && dot(c3,c4) > 0 && dot(c4,c1) > 0 )
			return i;
	}

	return -1;
}

bool Face::getSubdivisionVertices ( int id, double resolution, Vec3d &a, Vec3d &b, Vec3d &c, Vec3d &d )
{
	if ( _regular ) {

		a = _vertices[0]->getPosition();
		b = _vertices[1]->getPosition();
		c = _vertices[2]->getPosition();
		d = _vertices[3]->getPosition();
	
		int nx = int(len(b-a)/resolution)+1;
		int ny = int(len(d-a)/resolution)+1;

		Vec3d dx = (b-a)/nx;
		Vec3d dy = (d-a)/ny;

		int n_y = int(id/nx);
		int n_x = id - n_y * nx;

		a = a + n_x * dx + n_y * dy;
		b = a + dx;
		c = a + dx + dy;
		d = a + dy;

		return true;
	}

	if ( id >= subdivision.size()/4 )
		return false;

	a = subdivision[4*id+0];
	b = subdivision[4*id+1];
	c = subdivision[4*id+2];
	d = subdivision[4*id+3];

	return true;
}

bool Face::barycentricToWorld( Vec2d pbar, Vec3d &point )
{
	if ( _vertices.size() == 3 ) {
		Vec3d a = _vertices[0]->getPosition();
		Vec3d b = _vertices[1]->getPosition();
		Vec3d c = _vertices[2]->getPosition();

		point = a + pbar[0] * (b-a) + pbar[1] * (c-a);
		return true;
	}

	if ( _vertices.size() == 4 ) {
	
		Vec3d a = _vertices[0]->getPosition();
		Vec3d b = _vertices[1]->getPosition();
		Vec3d c = _vertices[2]->getPosition();
		Vec3d d = _vertices[3]->getPosition();

		if ( pbar[1] < 1.0 - pbar[0] ) {
			point = a + pbar[0] * (b-a) + pbar[1] * (d-a);
		} else {
			point = c + (1.0 - pbar[0]) * (d-c) + (1.0 - pbar[1]) * (b-c);
		}
		
		return true;
	}

	return false;

}

// convert a point from barycentric coordinates to index coordinates/
// given a subdivision level
//
bool Face::barycentricToIndex( Vec2d pbar, int level, int &i, int &j )
{
	// check that the coordinates are sane
	if ( pbar[0] < 0.0 || pbar[0] > 1.0 )
		return false;
	if ( pbar[1] < 0.0 || pbar[1] > 1.0 )
		return false;

	// convert the barycentric coordinates into index coordinates
	double step = 1.0;

	for (int k=0;k<level;k++)
		step /= 2.0;

	i = pbar[0] / step;
	j = pbar[1] / step;

	return true;
}

bool Face::indexToBarycentric( int level, int i, int j, Vec2d &pbar ) 
{

	double step = 1.0;

	for (int k=0;k<level;k++)
		step /= 2.0;

	pbar[0] = (double)i * step;
	pbar[1] = (double)j * step;

	if ( pbar[0] < 0.0 || pbar[0] > 1.0 )
		return false;
	if ( pbar[1] < 0.0 || pbar[1] > 1.0 )
		return false;

	return true;
}

// return true if the face occludes the <point> seen from <view_point>
bool Face::occluding( Vec3d point, Vec3d view_point )
{
	Vec3d n = point - view_point;
	if ( len(n) < EPS )
		return false;

	Vec3d a = _vertices[0]->getPosition();

	double lambda;
	Vec3d p = intersectRayPlane( view_point, n, a, _norm, lambda); 

	if ( lambda < 0 )
		return false;

	int q = _vertices.size();
	Vec3d b,c;

	for (int i=1;i<q;i++) {

		a = _vertices[i]->getPosition();
		b = _vertices[i-1]->getPosition();
		c = _vertices[(i+1)%q]->getPosition();

		if ( len(p-a) < EPS )
			return true;

		Vec3d w = norm(cross(b-a,c-a));

		double angle_bc = Acos(dot(norm(c-a),norm(b-a)));
		double angle_bp = dot(w,cross(b-a,p-a)) > 0 ? Acos(dot(norm(p-a),norm(b-a))) : 2*M_PI - Acos(dot(norm(p-a),norm(b-a)));

		if ( angle_bp > angle_bc + OCCLUSION_ANGLE_ERROR )
			return false;
	}

	return true;
}

double Face::area()
{
	if ( _vertices.size() == 3 ) {
		return 0.5 * len( cross(_vertices[1]->getPosition() - _vertices[0]->getPosition(), _vertices[2]->getPosition() - _vertices[0]->getPosition() ) );
	} 

	if ( _vertices.size() == 4 ) {
		return 0.5 * len( cross(_vertices[1]->getPosition() - _vertices[0]->getPosition(), _vertices[3]->getPosition() - _vertices[0]->getPosition() ) ) +
			0.5 * len( cross(_vertices[1]->getPosition() - _vertices[2]->getPosition(), _vertices[3]->getPosition() - _vertices[2]->getPosition() ) );
	}

	return 0.0;
}

void Face::draw(bool fill, const float color[3])
{
	if (fill)
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	else
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

	glColor3fv(color);
	glNormal3f(_norm[0],_norm[1],_norm[2]);

	glBegin(GL_POLYGON);
	for (int i=0;i<_vertices.size();i++)
		_vertices[i]->glVertex();
	glEnd();
}

void Face::regular()
{
	_regular = false;
	
	if (_vertices.size() != 4)
		return;

	double thres = toRadians(1.0);
	
	double angle = Acos(dot(norm(_vertices[1]->getPosition() - _vertices[0]->getPosition()), norm(_vertices[3]->getPosition() - _vertices[0]->getPosition())));
	if ( fabs(angle-M_PI/2) > thres )
		return;

	angle = Acos(dot(norm(_vertices[0]->getPosition() - _vertices[1]->getPosition()), norm(_vertices[2]->getPosition() - _vertices[1]->getPosition())));
	if ( fabs(angle-M_PI/2) > thres )
		return;

	angle = Acos(dot(norm(_vertices[1]->getPosition() - _vertices[2]->getPosition()), norm(_vertices[3]->getPosition() - _vertices[2]->getPosition())));
	if ( fabs(angle-M_PI/2) > thres )
		return;

	angle = Acos(dot(norm(_vertices[2]->getPosition() - _vertices[3]->getPosition()), norm(_vertices[0]->getPosition() - _vertices[3]->getPosition())));
	if ( fabs(angle-M_PI/2) > thres )
		return;

	_regular = true;
}

int Face::subdivisionSize( double resolution )
{
	if ( _regular ) {

		Vec3d a,b,c,d;

		a = _vertices[0]->getPosition();
		b = _vertices[1]->getPosition();
		c = _vertices[2]->getPosition();
		d = _vertices[3]->getPosition();

		int sx = int(len(b-a)/resolution)+1;
		int sy = int(len(d-a)/resolution)+1;

		return sx * sy;
	}

	return subdivision.size();
}

void Face::draw2D(bool fill, const float color[3])
{
	if (fill)
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	else
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

	glColor3fv(color);

	glBegin(GL_POLYGON);
	for (int i=0;i<_vertices.size();i++)
		_vertices[i]->glVertex2D();
	glEnd();
}

// return true if the input point is "inside" the face
//
bool Face::inside( Vec3d point )
{
	if ( _vertices.empty() )
		return false;

	// generate a bunch of cross products vectors
	std::vector< Vec3d > crosses;
	for (int i=0;i<_vertices.size()-1;i++) {
		crosses.push_back( cross( _vertices[i]->getPosition() - point, _vertices[i+1]->getPosition() - point ) );
	}
	crosses.push_back( cross( _vertices[_vertices.size() - 1]->getPosition() - point, _vertices[0]->getPosition() - point ) );

	// test whether all these vectors point to the same direction
	for ( i=0;i<crosses.size();i++) {
		for (int j=i+1;j<crosses.size();j++) {
			if ( dot( crosses[i], crosses[j] ) < 0.0 )
				return false;
		}
	}

	return true;
}

// return true if the current face contains the face f
//
bool Face::equivalent( Face *f )
{

	if ( _vertices.size() != f->_vertices.size() )
		return false;

	int exists = 0;

	for (int i=0;i<_vertices.size();i++) {

		int id = _vertices[i]->_id;

		for (int j=0;j<f->_vertices.size();j++) {
			if ( id == f->_vertices[j]->_id ) {
				exists++;
				break;
			}
		}
	}

	if ( exists == _vertices.size() )
		return true;
	else
		return false;
}

void Face::print() 
{
	print(stdout);
}

// get the list of edges adjacent to the current face
//
void Face::getAdjacentEdges( edgeVector &medges )
{
	medges.clear();

	for (int q=0;q<_vertices.size();q++) {
		for (int r=0;r<_vertices[q]->_edges.size();r++) {
			if (_vertices[q]->_edges[r]->_faces.empty()) {
				medges.push_back( _vertices[q]->_edges[r] );
			} else {
				for (int p=0;p<_vertices[q]->_edges[r]->_faces.size();p++) {
					if ( _vertices[q]->_edges[r]->_faces[p]->_id == _id ) {
						medges.push_back( _vertices[q]->_edges[r] );
						break;
					}
				}
			}
		}
	}

	return;
}


void Face::print(FILE *fout) 
{
	fprintf(fout,"%d %d ",_id,_vertices.size());
	for (int i=0;i<_vertices.size();i++)
		fprintf(fout,"%d ",_vertices[i]->_id);
	fprintf(fout,"\n");
}

bool Face::read (FILE *fin, vertexVector &V)
{
	int id, vertexId, nVertices;

	// read the values in the file
	if (fscanf(fin,"%d%d",&id,&nVertices) != 2)
		return false;

	_id = id;

	for (int i=0;i<nVertices;i++) {
		if (fscanf(fin,"%d",&vertexId) != 1)
			return false;
		_vertices.push_back(V[vertexId]);
	}

	// compute the normal
	_norm = normal();

	return true;
}


double ExtrinsicParameters::distance_trans( const ExtrinsicParameters &p )
{
	return len( getTranslation() - p.translation );
}

double ExtrinsicParameters::distance_rot( const ExtrinsicParameters &p )
{
	Quaternion q1 = getRotation();
	Quaternion q2 = p.quaternion;

	return fabs( q1._s - q2._s ) + (1.0 - dot( norm( q1._v ), norm( q2._v )) ) ;
}

std::string ExtrinsicParameters::get_mode_string()
{
	switch ( mode ) {
	case INIT_VERTICAL:
		return std::string( "INIT_VERTICAL" );
	case LINE_LINE:
		return std::string( "LINE_LINE" );
	case ROTATION_VOTING:
		return std::string( "ROTATION_VOTING" );
	case CORNERS:
		return std::string( "CORNERS" );
	case VANISHING_POINTS:
		return std::string( "VANISHING_POINTS" );
	case EXHAUSTIVE_SEARCH:
		return std::string( "EXHAUSTIVE_SEARCH" );
	case NONE:
		return std::string( "NONE" );
	default:
		LOG(LEVEL_ERROR, "ERROR: unknown pose mode!");
		assert(false);
	}

	return std::string("unknown");
}

bool ExtrinsicParameters::write( FILE *f )
{
	if ( f == NULL )
		return false;

	Vec3d axis = quaternion._v;
	if ( len(axis) > EPS ) {
		axis = 2.0 * Acos(quaternion._s) * norm(axis);
	}

	fprintf( f, "%f %f %f %f %f %f\n", translation[0], translation[1], translation[2], 
		axis[0], axis[1], axis[2] );

	return true;
}

bool ExtrinsicParameters::read( FILE *f )
{
	if ( f == NULL )
		return false;

	double x,y,z,q1,q2,q3;

	if ( fscanf( f, "%lf%lf%lf%lf%lf%lf", &x, &y, &z, &q1, &q2, &q3 ) != 6 )
		return false;

	translation = Vec3d( x, y, z );
	quaternion = Quaternion( Vec3d( q1, q2, q3 ), len(Vec3d( q1, q2, q3 )) );

	return true;
}

// return the translation error between two poses
double ExtrinsicParameters::translation_error( ExtrinsicParameters &pose )
{
	return len( translation - pose.getTranslation() );
}

// return the rotation error between two poses IN DEGREES
//
double ExtrinsicParameters::rotation_error( ExtrinsicParameters &pose )
{
	return toDegrees( CompareTwoRotations( quaternion, pose.getRotation() ) );
}

////////////////////////////////////////////////////////////////////////////
// 
// a correspondence between a model line and an observed edge
//

void corres::print()
{
	switch (_status) {
	case _EXPECTED:
		printf( "EXPECTED\t");
		break;
	case _CONNECTED:
		printf( "CONNECTED ");
		break;
	case _BLACKLISTED:
		printf( "BLACKLISTED\t");
		break;
	}

	printf( "[%3d] ", age );

	if (line == NULL)
		printf( "[----] ");
	else
		printf( "[%4d] ", line->_id );

	printf("\n");
}

bool corres::operator<(const corres &c)
{
	if ( !eps.empty() && eps.empty() )
		return true;
	else if ( eps.empty() && !c.eps.empty() )
		return false;
	else if ( eps.empty() && c.eps.empty() )
		return false;
	else if ( length > c.length )
		return true;
	else
		return false;
}

bool corres::operator==( const corres &c)
{
	if (line != c.line)
		return false;

	if (age != c.age)
		return false;

	return true;
}

// compute the reprojection error for a correspondence
// assuming that the correspondence edgeplane is expressed in the CCF (camera coordinate frame)
// hence the <pose> (camera pose) given as input
// 0 is bad, 1 is good.
// no frontality test is executed here since we assume that the correspondence comes from image region search
//
double corres::computeScore( ExtrinsicParameters &pose )
{
	double score = -1.0;
	max_score = -1.0;
	
	if ( line == NULL )
		return score;
	
	// transform the edgeplane
	for (int i=0;i<eps.size();i++) {
		EdgePlane tep = eps[i];
		tep.fromCameraFrameToWorldFrame( pose);
		
		// test overlap
		//if ( tep.overlap( line ) < 0.95) {
		//	scores.push_back(0.0);
		//	continue;
		//}
		
		// compute angle error
		score = tep.score_alignement( line );
		scores.push_back( score ) ;

		if ( score > max_score ) {
			best_ep = i;
			max_score = score;
		}
	}
	
	return max_score;
	//Vec3d u = norm( cross( line->getA() - pose.getTranslation(), line->getB() - pose.getTranslation() ) );
	//return fabs( dot( tep._normal, u ) );
}

// determine which edegplane among eps looks the most like ep
void corres::update_eid( EdgePlane &ep )
{
	if (!valid()) {
		eid = -1;
		return;
	}

	eid = MIN(eps.size()-1, (double)rand()/(RAND_MAX+1) * (eps.size()-1));
}

// select a new edgeplane (presumably for a new correspondence)
void corres::select_eid() 
{
	// select the longest edgeplane
	eid = -1;
	if ( eps.empty() )
		return;

	eid = 0;
	double l = eps[0].length();

	for (int i=1;i<eps.size();i++) {
		if ( eps[i].length() > eps[eid].length() ) {
			eid = i;
		}
	}
}