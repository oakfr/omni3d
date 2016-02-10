#include "basic.h"
#include "camera/camera.h"

/* print out the intrinsic parameters
 */
void Intrinsic::print()
{
	Vec2d center;
	
	switch (dist_model) {
	case SPHERICAL_DISTORTION:
		printf("%d SPHERICAL wxh: %dx%d focal: %f center: %.4f,%.4f\n",_id,width,height,f,cc[0],cc[1]);
		center = denormalize(cc);
		printf("center(pixels): %.2f,%.2f  focal(pixels): %.2f,%.2f\n",center[0],center[1],focal_x,focal_y);
	default:
		break;
	}
}

/* constructor
 */
Intrinsic::Intrinsic () 
{
	f=0.0; 
	cc=Vec2d(0,0); 
	width = height = 1;
	CC = denormalize(cc);
	kc[0] = kc[1] = kc[2] = kc[3] = kc[4] = 0.0; 
	pixel_width_mm = 0; pixel_height_mm = 0;
	value = 0.0;
	dist_model = POLYNOMIAL_DISTORTION;
	focal_x = focal_y = 0;
	pixel_angle = 0.0;
};

/* constructor
 */
Intrinsic::Intrinsic (int w, int h, double focal, double c1, double c2, double k1, double k2, double k3, double k4, double k5, double w_mm, double h_mm) 
{
	f = focal;
	width = w;
	height = h;
	cc = Vec2d(c1,c2);
	CC = denormalize(cc);
	kc[0] = k1; kc[1] = k2; kc[2] = k3; kc[3] = k4; kc[4] = k5;
	pixel_width_mm = w_mm;
	pixel_height_mm = h_mm;
	double fov = toRadians(70.0); // estimated Ladybug FOV, given the fact that 5 cameras cover 360 degrees + overlap
	pixel_angle = (double)fov / height; // radians per pixel
	dist_model = POLYNOMIAL_DISTORTION;
	value = 0.0;
	Vec2d FF = denormalize(Vec2d(f,f));
	focal_x = FF[0];
	focal_y = FF[1];

	Vec2d a = Vec2d( -0.5, -(double)h/(2.0*w) );
	Vec2d b = Vec2d(  0.5, -(double)h/(2.0*w) );
	Vec2d c = Vec2d(  0.5,  (double)h/(2.0*w) );
	Vec2d d = Vec2d( -0.5,  (double)h/(2.0*w) );

	LEFT =  Edge2D(a, d, 0);
	RIGHT = Edge2D(b, c, 0);
	TOP =   Edge2D(c, d, 0);
	BOTTOM = Edge2D(a, b, 0);
}

/* convert from pixels to radians
 */
double Intrinsic::from_pixels_to_radians( int v )
{
	return (double)v * pixel_angle;
}

/* constructor
 */
Intrinsic::Intrinsic (int id, int w, int h, double focal, double c1, double c2, double v, double w_mm, double h_mm) 
{
	_id = id;
	width = w;
	height = h;
	f = focal;
	cc = Vec2d(c1,c2);
	CC = denormalize(cc);
	value = v;
	dist_model = SPHERICAL_DISTORTION;
	kc[0] = 0.0; kc[1] = 0.0; kc[2] = 0.0; kc[3] = 0.0; kc[4] = 0.0;
	Vec2d FF = denormalize(Vec2d(f,f));
	focal_x = 345.36; //5*FF[0];
	focal_y = 345.36; //FF[1];
	pixel_width_mm = w_mm;
	pixel_height_mm = h_mm;	
	double fov = toRadians(70.0); // estimated Ladybug FOV, given the fact that 5 cameras cover 360 degrees + overlap
	pixel_angle = (double)fov / width; // radians per pixel

	Vec2d a = Vec2d( -0.5, -(double)h/(2.0*w) );
	Vec2d b = Vec2d(  0.5, -(double)h/(2.0*w) );
	Vec2d c = Vec2d(  0.5,  (double)h/(2.0*w) );
	Vec2d d = Vec2d( -0.5,  (double)h/(2.0*w) );

	LEFT =  Edge2D(a, d, 0);
	RIGHT = Edge2D(b, c, 0);
	TOP =   Edge2D(c, d, 0);
	BOTTOM = Edge2D(a, b, 0);
}

/* convert to camera matrix
 */
Mat3d Intrinsic::toKMatrix()
{
	return Mat3d(focal_x,0.0,CC[0],0.0,focal_y,CC[1],0.0,0.0,1.0);
}

/* return normalized point
 */
Vec2d Intrinsic::normalize_focal (Vec2d x)
{
	return Vec2d((x[0]-CC[0])/focal_x,(x[1]-CC[1])/focal_y);
}

/* return denormalized point
 */
Vec2d Intrinsic::denormalize_focal (Vec2d x)
{
	return Vec2d(focal_x*x[0]+CC[0],focal_y*x[1]+CC[1]);
}

/* return normalized point
 */
Vec2d Intrinsic::normalize(Vec2d x)
{
	return Vec2d (x[0]/(double)width - 0.5, x[1]/(double)width - 0.5*(double)height/(double)width);
}

/* return denormalized point
 */
Vec2d Intrinsic::denormalize (Vec2d x)
{
	return Vec2d(width*(x[0]+0.5),width*x[1]+0.5*height);
}

/* Flip horizontally
 */
Vec2d Intrinsic::flipHorizontal (Vec2d point)
{
	Vec2d p = point;
	p[0] = width - 1 - p[0];
	return p;
}

/* Return true if a 2D point is in the FOV
 */
bool Intrinsic::visible (Vec2d point)
{
	if (point[0] < 0)
		return false;
	if (point[0] >= width)
		return false;
	if (point[1] < 0)
		return false;
	if (point[1] >= height)
		return false;
	return true;
}

/* Return true if a 2D point is in the FOV
 * normalized coordinates
 */
bool Intrinsic::visible_normalized (Vec2d point)
{
	if ( point[0] < -0.5 )
		return false;
	if ( point[0] > 0.5 )
		return false;
	if ( point[1] < -(double)height/(2.0*width) )
		return false;
	if ( point[1] > (double)height/(2.0*width) )
		return false;

	return true;
}

/* clip an edge to fit in the image
 */
Edge2D Intrinsic::clip (Edge2D edge)
{
	Vec2d a = edge._a;
	Vec2d b = edge._b;
	double r = (double)height/(2.0*width);

	if (visible_normalized(a) && visible_normalized(b)) // don't clip if all inside
		return edge;

	Edge2D clipped = edge;
	Vec2d new_point;

	double dx = b[0] - a[0];
	double dy = b[1] - a[1];
	if (fabs(dx) < EPSILON) { // vertical edge
		clipped._a[1] = MAX(0,MIN(1.0,clipped._a[1]));
		clipped._b[1] = MAX(0,MIN(1.0,clipped._b[1]));
	} else if (fabs(dy) < EPSILON) { // horizontal edge
		clipped._a[0] = MAX(0,MIN(1.0,clipped._a[0]));
		clipped._b[0] = MAX(0,MIN(1.0,clipped._b[0]));
	} else { 
		double slope = dy/dx;
		double offset = a[1] - slope*a[0];

		if (!visible_normalized(a)) {
			if ((a[1] > r) && (TOP.intersectSegments(a,b,slope,offset,new_point)))
				clipped._a = new_point;
			else if ((a[1] < -r) && (BOTTOM.intersectSegments(a,b,slope,offset,new_point)))
				clipped._a = new_point;
			else if ((a[0] < -0.5) && (LEFT.intersectSegments(a,b,slope,offset,new_point)))
				clipped._a = new_point;
			else if ((a[0] > 0.5) && (RIGHT.intersectSegments(a,b,slope,offset,new_point)))
				clipped._a = new_point;
		}
		if (!visible_normalized(b)) {
			if ((b[1] > r) && (TOP.intersectSegments(a,b,slope,offset,new_point)))
				clipped._b = new_point;
			else if ((b[1] < -r) && (BOTTOM.intersectSegments(a,b,slope,offset,new_point)))
				clipped._b = new_point;
			else if ((b[0] < -0.5) && (LEFT.intersectSegments(a,b,slope,offset,new_point)))
				clipped._b = new_point;
			else if ((b[0] > 0.5) && (RIGHT.intersectSegments(a,b,slope,offset,new_point)))
				clipped._b = new_point;
		}
	}

	return clipped;
}

/* Rectify a 2D image edge
 */
Edge2D Intrinsic::rectify ( Edge2D &edge )
{
	return Edge2D( rectify( edge._a), rectify( edge._b ) );
}

/* Rectify an image point
 * input and output in normalized coordinates
 */
Vec2d Intrinsic::rectify (Vec2d xd)
{
	double r2 = len(xd);

		if (dist_model == POLYNOMIAL_DISTORTION) {

		double rd4 = r2*r2;
		double rd6 = rd4*r2;
		double rd8 = rd4*rd4;

		double beta = getKC0()*r2+getKC1()*rd4+getKC0()*getKC0()*rd4+getKC1()*getKC1()*rd8+\
			2*getKC0()*getKC1()*rd6;
		double alpha = 1 + 4*getKC0()*r2 + 6*getKC1()*rd4;

		return xd * (1 - beta/alpha);
		} else if (dist_model == SPHERICAL_DISTORTION) {

			Vec2d x = xd-cc;
			r2 = sqrlen(x);
			double r_new = value*value / sqrt(fabs(1 - value*value*value*value*r2));

			x = x * r_new+cc; //

			return x;
	}

		printf("undefined distortion model!\n");
		assert(false);
		return Vec2d (0,0);
}

/* Distort a 2D image edge 
 */
Edge2D Intrinsic::distort ( Edge2D &edge )
{
	return Edge2D( distort( edge._a), distort( edge._b ) );
}

/* Distort a 2D image point
 */
Vec2d Intrinsic::distort(Vec2d xn)
{
	double x = xn[0], y = xn[1];

	Vec2d xd;
	
	if (dist_model == POLYNOMIAL_DISTORTION) {
		double r2 = x*x + y*y;
		double c = 1+getKC0()*r2 + getKC1() *r2*r2 + getKC4()*r2*r2*r2;

		Vec2d dx = Vec2d(2*getKC2()*x*y+getKC3()*(r2+2*x*x),getKC2()*(r2+2*y*y)+2*getKC3()*x*y);

		xd = c*xn + dx;

	} else if (dist_model == SPHERICAL_DISTORTION) {
			Vec2d xp = xn - cc; ///

			double rnew2 = sqrlen(xp);
			double rnew = len(xp);

			double r_1 = rnew / (value*value*sqrt(fabs(1 + rnew2)));
			double r_2 = rnew / (value*value*sqrt(fabs(rnew2 - 1.0)));
			Vec2d xu_1 = xp * r_1/rnew + cc; // 
			Vec2d xu_2 = xp * r_2/rnew + cc; // 
		
			Vec2d xc_1 = rectify(xu_1);  

			if (len(xc_1-xn) < 0.0001) {
				xd = xu_1;
			}
			else {
				Vec2d xc_2 = rectify(xu_2); 
				if (len (xc_2 - xn) < 0.00001)
					xd = xu_2;
				else {
					xd = xu_1;
				}
			}
	}

	return xd;
}

/* return a bounding box for an edge <edge>
 * given an angle in rotation, a length in translation (in mm)
 * and the <depth> in inches of the observed line
 */
void Intrinsic::computeEdgeBoundingBox (Edge2D edge, double angle, double translation, double depth, 
										Vec2d &a, Vec2d &b, Vec2d &c, Vec2d &d, int min_mask_width, int max_mask_width)
{
	double dt = ( (double)translation / (f + inchTomm(depth)) ) / pixel_angle; // insert translation factor
	dt += fabs(angle) / pixel_angle; // insert rotation factor

	dt = MAX(dt,min_mask_width); // there is a minimum mask width
	dt = MIN(dt,max_mask_width);

	Vec2d u = norm(Vec2d(edge._b[0]-edge._a[0],edge._b[1]-edge._a[1]));
	Vec2d v = Vec2d(u[1],-u[0]); 

	a = edge._a + dt * v - dt * u;
	b = edge._b + dt * v + dt * u;
	c = edge._b - dt * v + dt * u;
	d = edge._a - dt * v - dt * u;

}

/* Debugging only. Check the sanity of rectify and distortion methods
 */
void Intrinsic::distorsionChecker()
{
	// print
	print();

	// check clipping algorithm
	Edge2D edge = Edge2D(Vec2d(-200,-100),Vec2d(10,20));
	Edge2D edge_clipped = clip(edge);

	LOG(LEVEL_DEBUG,"(%f,%f) => (%f,%f)",edge._a[0],edge._a[1],edge_clipped._a[0],edge_clipped._a[1]);
	LOG(LEVEL_DEBUG,"(%f,%f) => (%f,%f)",edge._b[0],edge._b[1],edge_clipped._b[0],edge_clipped._b[1]);

	//pick a bunch of 2D points and check their rectified value
	Vec2d point = Vec2d(2.500000000000000e+001  ,  6.950000000000000e+001);
	Vec2d point_rect = rectify(normalize(point));

	LOG(LEVEL_DEBUG,"(%f,%f) ==> (%f,%f)   should be (%f,%f)", point[0],point[1],point_rect[0],point_rect[1],
		-1.271644101537981e+000  , -6.149558565018068e-001);

	point = Vec2d(2.560000000000000e+002 ,   1.950000000000000e+001);
	point_rect = rectify(normalize(point));

	LOG(LEVEL_DEBUG,"(%f,%f) ==> (%f,%f)   should be (%f,%f)", point[0],point[1],point_rect[0],point_rect[1],
		-1.559022165084900e-002 ,  -5.935958630008766e-001);

	point = Vec2d(2.560000000000000e+002  ,  6.650000000000000e+001);
	point_rect = rectify(normalize(point));

	LOG(LEVEL_DEBUG,"(%f,%f) ==> (%f,%f)   should be (%f,%f)", point[0],point[1],point_rect[0],point_rect[1],
		-1.308190696800208e-002 ,  -3.973240271620430e-001);
	//pick a bunch of points
	// perform successive distortion and rectification and check the discrepancy

	int n_points = 10;

	for (int i=0;i<n_points;i++) {

		Vec2d point = Vec2d ((double)rand()/RAND_MAX,(double)rand()/RAND_MAX);
		Vec2d point_dist = distort(point);
		Vec2d point_dist_2 = distort(rectify(point_dist));
		LOG(LEVEL_DEBUG,"(%f,%f)  => (%f,%f)   d = %f\n",point_dist[0],point_dist[1],point_dist_2[0],point_dist_2[1],len(point_dist-point_dist_2));
	}
}



