#include "viewer.h"

void Viewer::clearWindow(bool black)
{
	if (black) {
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f); 
	} else {
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f); 
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Viewer::setWindow()
{
	assert(_valid);
	//glutSetWindow (_windowId); // fix for FLTK
}

void Viewer::set (int w, int h, int id)
{
	_width=w;
	_height=h;
	_windowId = id;
	int ww = BUF_SIZE;
	_selectBuffer = (GLuint*)malloc(BUF_SIZE*sizeof(GLuint)); // malloc the GL_SELECT buffer
	_valid = true;
}

void Viewer::init (Vec3d point, double rho, double phi, double theta, double fov, double znear, double zfar, ViewerType type)
{
	_spheric = SphericalCoord(rho,phi,theta,0);
	if (type == OUTSIDE) {
		_target = point;
	}
	else
		_eye = point;
	_anchorx = _anchory = 0;
	_znear = znear; _zfar = zfar;
	_fov = fov;
	_type = type;

	_save_eye = Vec3d(0,0,0);
	_save_spheric = _spheric;

	setup3D();

}

void Viewer::setup2D ()
{

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0,_width,0,_height);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);

}

void Viewer::setup2D (int x, int y) // to be called in GL_SELECT mode only!
{
	assert (_mode == GL_SELECT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	GLint viewport[4];
	glGetIntegerv (GL_VIEWPORT, viewport);
	gluPickMatrix ((GLdouble) x, (GLdouble) (viewport[3] - y), 10.0, 10.0, viewport);
	
	gluOrtho2D(0,_width,0,_height);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glInitNames();
}

void Viewer::setup3D ()
{
	if (_type == OUTSIDE) {
		//_spheric.print();
		_eye = fromSphericToEuler (_target,_spheric);
	} else {
		_target = fromSphericToEuler (_eye,_spheric);
	}
	_unit = norm(_target-_eye);
	Vec3d right = norm(cross(_unit,Vec3d(0,0,1)));
	_up = norm(cross(right,_unit));

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(_fov, 1, _znear, _zfar);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLookAt(_eye,_unit,_up);

	glEnable(GL_COLOR_MATERIAL);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glShadeModel (GL_SMOOTH);

}

void Viewer::drawAxis (Vec3d position, double radius)
{
	// draw a small sphere at the origin
	double small_radius = radius/10.0;
	drawSphere(position,small_radius,RED,5,5);

	// draw axis
	glLineWidth(2);
	glBegin(GL_LINES);
	glVertex(position,position+radius*Vec3d(1,0,0));
	glVertex(position,position+radius*Vec3d(0,1,0));
	glVertex(position,position+radius*Vec3d(0,0,1));
	
	glLineWidth(1);

	Vec3d a,b,c,d;

	// draw the text "X"
	a = position+radius*Vec3d(1.1,0,0);
	b = a + Vec3d(small_radius,0,0);
	c = a + Vec3d(0,small_radius,0);
	d = b + Vec3d(0,small_radius,0);
	glVertex(a,d);
	glVertex(b,c);

	// draw the "Y"
	a = position+radius*Vec3d(0,1.1,0);
	b = a + Vec3d(0,small_radius,0);
	c = a + Vec3d(small_radius/2,small_radius/2,0);
	d = a + Vec3d(small_radius,small_radius,0);
	glVertex(b,c);
	glVertex(a,d);

	// draw the "Z"
	a = position + radius*Vec3d(0,0,1.1);
	b = a + Vec3d(small_radius,0,0);
	c = a + Vec3d(0,small_radius,0);
	d = b + Vec3d(0,small_radius,0);
	glVertex(c,d);
	glVertex(d,a);
	glVertex(a,b);

	glEnd();
}

void Viewer::setup3D_topview ()
{
	assert (_type == OUTSIDE);
	_spheric.phi = M_PI;
	setup3D();
}

Vec3d Viewer::raycast (int winx, int winy)
{
	GLdouble modelMatrix[16];
	glGetDoublev (GL_MODELVIEW_MATRIX,modelMatrix);

	GLdouble projMatrix[16];
	glGetDoublev (GL_PROJECTION_MATRIX,projMatrix);
	
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	double x,y,z;

	gluUnProject(winx, winy, 1.0, modelMatrix, projMatrix, viewport, &x, &y, &z);

	Vec3d u = norm(Vec3d(x,y,z)-_eye);

	double lambda;
	return intersectRayPlane(_eye,u,_target,Vec3d(0,0,1),lambda);
}

void Viewer::setup3D (int x, int y) // to be called in GL_SELECT mode only!
{
	assert (_mode == GL_SELECT);

	if (_type == OUTSIDE) {
		_eye = fromSphericToEuler (_target,_spheric);
	} else {
		_target = fromSphericToEuler (_eye,_spheric);
	}
	_unit = norm(_target-_eye);
	Vec3d right = norm(cross(_unit,Vec3d(0,0,1)));
	_up = norm(cross(right,_unit));
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
		
	GLint viewport[4];
	glGetIntegerv (GL_VIEWPORT, viewport);
	gluPickMatrix ((GLdouble) x, (GLdouble) (viewport[3] - y), 10.0, 10.0, viewport);
	
	gluPerspective(_fov, 1, _znear, _zfar);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLookAt(_eye,_unit,_up);

	glInitNames();

	glEnable(GL_COLOR_MATERIAL);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glShadeModel (GL_SMOOTH);

}

void Viewer::rotateViewPoint (int x, int y, double sensitivity)
{
	if (_type == OUTSIDE)
		_spheric.phi += (y - _anchory)/sensitivity;
	else
		_spheric.phi -= (y - _anchory)/sensitivity;
	double error = 0.00001;
	//_spheric.phi = MIN(M_PI-error,MAX(_spheric.phi,error));
	//if (_spheric.phi <= 0.0) _spheric.phi = 0.0001;
	if (_type == OUTSIDE)
		_spheric.theta -= (x-_anchorx)/sensitivity;
	else
		_spheric.theta += (x-_anchorx)/sensitivity;
	_anchorx = x;
	_anchory = y;
}

void Viewer::scaleViewPoint (int x, int y,double sensitivity)
{
	_spheric.rho -= (x-_anchorx)*sensitivity;
	if (_spheric.rho <= 0) _spheric.rho = 0.002;
	_anchorx = x;
	_anchory = y;
}

void Viewer::translateViewPoint (int x, int y, double sensitivity)
{
	double dx = - sensitivity * (x - _anchorx);
	double dy = sensitivity * (y - _anchory);
	//_eye[0] += -dx * sin (_spheric.theta) - dy * cos (_spheric.theta);
	//_eye[1] += dx * cos (_spheric.theta) - dy * sin (_spheric.theta);
	if (_type == OUTSIDE) {
		_target[0] += -dx * sin (_spheric.theta) - dy * cos (_spheric.theta);
		_target[1] += dx * cos (_spheric.theta) - dy * sin (_spheric.theta);
	} else {
		_target[0] -= -dx * sin (_spheric.theta) - dy * cos (_spheric.theta);
		_target[1] -= dx * cos (_spheric.theta) - dy * sin (_spheric.theta);
		_eye[0] -= -dx * sin (_spheric.theta) - dy * cos (_spheric.theta);
		_eye[1] -= dx * cos (_spheric.theta) - dy * sin (_spheric.theta);
	}
	_anchorx = x;
	_anchory = y;
}

void Viewer::switchType()
{
	if (_type == OUTSIDE) {
		glEnable(GL_DEPTH_TEST);
		_eye = _target;
		_type = WALKTHROUGH;
	} else {
		glDisable(GL_DEPTH_TEST);
		_target = _eye;
		_type = OUTSIDE;
	}
	refresh();
}

GLint Viewer::setMode (GLenum mode)
{
	_mode = mode;

	if (mode == GL_SELECT) {
		glRenderMode (GL_RENDER);
		glSelectBuffer (BUF_SIZE, _selectBuffer);
		return glRenderMode (GL_SELECT);
	} else if (mode == GL_RENDER) {
		return glRenderMode (GL_RENDER);
	} else if (mode == GL_FEEDBACK) {
		return glRenderMode(GL_FEEDBACK);
	}
	assert(false);
	return 0;
}

void Viewer::stopPicking(intVector &hitVector)
{
	GLint hits;
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix ();
	
	glMatrixMode(GL_MODELVIEW);
	glFlush();
	
	hits = glRenderMode (GL_RENDER);
	_mode = GL_RENDER;

	if ( hits < 0 )
		LOG(LEVEL_ERROR,"ERROR! hits = %d", hits);
	assert(hits >= 0);

	if (hits > 0)
		processHits (hits, hitVector);	
	//else
	//	printf("no hits.\n");
}

void Viewer::stopVideoPicking(intVector &hitVector)
{
	GLint hits;
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix ();
	
	glMatrixMode(GL_MODELVIEW);
	glFlush();
	
	hits = glRenderMode (GL_RENDER);
	_mode = GL_RENDER;

	if ( hits < 0 )
		LOG(LEVEL_ERROR,"ERROR! hits = %d", hits);
	//assert(hits >= 0);

	if (hits > 0)
		processVideoHits (hits, hitVector);	
	//else
	//	printf("no hits.\n");
}

//  The hit record consists of the number of names in the name stack at
//			the time of the event, followed by the minimum and maximum depth values of all vertices that hit since the previous
//			event, followed by the name stack contents, bottom name first.
// example: 1 4288180994 4288236802 1223 1 4288102914 4288159234
// or 1.000000 0.000000 0.000000 1668.000000 1.000000

void Viewer::processHits (GLint hits, intVector &hitVector)
{
	unsigned int i;
	GLuint nitems, *ptr;
	GLfloat minZ, maxZ;

	ptr = (GLuint *) _selectBuffer;
	minZ = 0xffffffff;

	int mode = 0;

	for (i = 0; i < 4 * hits; i++) {	
	
		if ( mode == 0 ) {
			nitems = *ptr;
			ptr++;
			mode = 1;
			continue;
		}

		if ( mode == 1 ) {
			minZ = *ptr;
			ptr++;
			mode = 2;
			continue;
		}

		if ( mode == 2 ) {
			maxZ = *ptr;
			ptr++;
			mode = 3;
			continue;
		}

		if ( mode == 3 ) {
			if ( (minZ>0.0) && ( maxZ>0.0 ) )
			//if ( *ptr >= 0 )	
				hitVector.push_back( *ptr );
			//printf("%d (%f %f)\n", *ptr,minZ,maxZ);
			ptr++;
			mode = 0;
			continue;
		}
	}

	//printf("\n");
}

void Viewer::processVideoHits (GLint hits, intVector &hitVector)
{
	unsigned int i,j;
	GLuint nitems, *ptr;
	GLfloat minZ, maxZ;

	ptr = (GLuint *) _selectBuffer;
	minZ = 0xffffffff;

	int mode = 0;
	int n = *ptr;

	for (i = 0; i < hits; i++) {	
	
		nitems = *ptr;
		ptr++;
		minZ = *ptr;
		ptr++;
		maxZ = *ptr;
		ptr++;

		for (j=0;j<nitems;j++) { // items: cameraId edgeId
			hitVector.push_back( *ptr );
			ptr++;
		}
	}
}

void Viewer::drawImage (IplImage *img, int x, int y, bool color, bool flip)
{
	//glPixelZoom(1.0,-1.0);
	glRasterPos2d(x,y);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
 
	if (flip && color) {
		IplImage *flip_img;
		if (color)
			flip_img = cvCreateImage(cvSize(img->width, img->height),IPL_DEPTH_8U, 3);
		else
			flip_img = cvCreateImage(cvSize(img->width, img->height),IPL_DEPTH_8U, 1);

		cvConvertImage (img, flip_img, /*CV_CVTIMG_SWAP_RB |*/ CV_CVTIMG_FLIP);
		if (color)
			glDrawPixels(img->width, img->height, GL_RGB, GL_UNSIGNED_BYTE, flip_img->imageData);
		else
			glDrawPixels(img->width, img->height, GL_LUMINANCE, GL_UNSIGNED_BYTE, flip_img->imageData);
		cvReleaseImage(&flip_img);

	} else {
		if (color)
			glDrawPixels(img->width, img->height, GL_RGB, GL_UNSIGNED_BYTE, img->imageData);
		else
			glDrawPixels(img->width, img->height, GL_LUMINANCE, GL_UNSIGNED_BYTE, img->imageData);
	}
	glPopMatrix();

	//glPixelZoom(1.0,1.0);

}

void Viewer::drawEdge (Edge2D edge, const float color[3], int lineWidth)
{

	draw2DEdge (edge, color, lineWidth);
}

void Viewer::drawEdge (Edge edge, const float color[3], int lineWidth)
{

	edge.draw (color,lineWidth);
}

void Viewer::drawTriangle (Vec3d p1, Vec3d p2, Vec3d p3, const float color[3])
{
	glColor(color);
	glBegin(GL_TRIANGLES);
	glVertex3f(p1[0],p1[1],p1[2]);
	glVertex3f(p2[0],p2[1],p2[2]);
	glVertex3f(p3[0],p3[1],p3[2]);
	glEnd();
}

void Viewer::drawTriangle (Vec2d p1, Vec2d p2, Vec2d p3, const float color[3])
{
	glColor(color);
	glBegin(GL_TRIANGLES);
	glVertex2f(p1[0],p1[1]);
	glVertex2f(p2[0],p2[1]);
	glVertex2f(p3[0],p3[1]);
	glEnd();
}

void Viewer::drawBox (Vec2d p1, Vec2d p2, Vec2d p3, Vec2d p4, const float color[3])
{
	drawTriangle(p1,p2,p3,color);
	drawTriangle(p1,p3,p4,color);
}

void Viewer::flipEdges (edge2DVector &edges, int height)
{
	for (int i=0;i<edges.size();i++) {
		edges[i] = edges[i].flipOpenGL(height);
	}
}

// draw only edges which ID is in the list of edge planes
void Viewer::drawEdgesSelectId (edge2DVector edges, int x, int y, double zoom, const float color[3], bool flipEdge, bool flipVert, double h,\
								double w, edgePlaneVector edgeplanes, int cameraId, int linewidth)
{
	glRasterPos2d(0,0);
	glLineWidth(linewidth);

	if (_mode == GL_SELECT) {
		glPushName (cameraId);
		LOG( LEVEL_INFO, "pushing %d", cameraId );
	}

	for (int i=0;i<edgeplanes.size();i++) {

		if (edgeplanes[i]._cameraId != cameraId)
			continue;

		//if (_mode == GL_SELECT) {
		//	glPushName (i);
		//}
		
		for (int j=0;j<edgeplanes[i]._edgeIds.size();j++) {
			
			int id = edgeplanes[i]._edgeIds[j];
			if ( id >= edges.size() )
				continue;

			Edge2D edge = edges[id];
			
			if (_mode == GL_SELECT) {
				glPushName(CHAINED);
				glPushName (edgeplanes[i]._edgeIds[j]);
				glPushName (edge._edgeplaneId);
				glPushName (edge._lineId);
			}

			if (flipEdge)
				edge.flipEdgeDetector(h-1);
			if (flipVert) {
				edge.flipVertical(h-1);
				edge.flipHorizontal(w-1);
			}

			edge.scale(x,y,zoom);
			edge.draw(color,linewidth);
			
			if (_mode == GL_SELECT) {
				glPopName();
				glPopName();
				glPopName();
				glPopName();
			}
		}
		
		//if (_mode == GL_SELECT)
		//	glPopName();
	}

	if (_mode == GL_SELECT)
		glPopName();

	glLineWidth(1);
}

// draw only edges which ID is in the list of lines
void Viewer::drawEdgesSelectId (edge2DVector edges, int x, int y, double zoom, const float color[3], bool flipEdge, bool flipVert, double h,\
								double w, edgeVector &lines, int cameraId, int linewidth)
{
	glRasterPos2d(0,0);
	glLineWidth(linewidth);
	
	for (int i=0;i<lines.size();i++) {
		
		int lineId = lines[i]->_id;
		
		for (int j=0;j<edges.size();j++) {
			
			Edge2D edge = edges[j];
			
			if (edge._lineId != lineId)
				continue;
			
			if (flipEdge)
				edge.flipEdgeDetector(h-1);
			if (flipVert) {
				edge.flipVertical(h-1);
				edge.flipHorizontal(w-1);
			}
			
			edge.scale(x,y,zoom);
			edge.draw(color,linewidth);
			
		}
	}
	
	glLineWidth(1);
}


void Viewer::drawEdges (edge2DVector edges, int x, int y, double zoom, const float color[3], bool flipEdge, bool flipVert, double h, \
						double w, int cameraId)
{
	drawEdges(edges,x,y,zoom,color,flipEdge,flipVert,h,w,cameraId,2);
}

// draw a histogram
void Viewer::drawHistogram( Histogram &hist, int x, int y, int w, int h, const float color[3] )
{
	if ( !hist.valid() )
		return;

	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glColor( color );

	double xstep = (double)w / hist.getSize();

	for (int i=0;i<hist.getSize();i++) {
		glBegin( GL_POLYGON );
		glVertex2f( x + i * xstep, y );
		glVertex2f( x + (i+1) * xstep, y );
		glVertex2f( x + (i+1) * xstep, y + h * hist.getVal(i) / hist.maxValue() );
		glVertex2f( x + i * xstep, y + h * hist.getVal(i) / hist.maxValue() );
		glEnd();
	}
}


void Viewer::drawPoint( Vec3d point, int point_width, const float color[3], double blending) 
{
	glColor4f( color[0],color[1],color[2],blending);
	glEnable(GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	glPointSize( point_width );

	glBegin(GL_POINTS);

	glVertex( point );

	glEnd();
	glDisable(GL_BLEND);
}

void Viewer::drawSiftFeatures( std::vector< Vec4d > &points, int x, int y, double zoom, const float color[3], bool flipHoriz, \
						 bool flipVert, double h, double w)
{
	glEnable(GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	glPointSize(3);
	glLineWidth(2);

	for (int i=0;i<points.size();i++) {
		Vec2d point = Vec2d(points[i][0],points[i][1]);

		Vec2d point2 = point + points[i][2] * 6.0 * Vec2d(-sin(points[i][3]), -cos(points[i][3])); // shift by pi/2

		if (flipHoriz) {
			point[0] = w-1-point[0];
			point2[0] = w-1-point2[0];
		}
		if (flipVert) {
			point[0] = w-1-point[0];
			point2[0] = w-1-point2[0];
			point[1] = h-1-point[1];
			point2[1] = h-1-point2[1];
		}

		if ( point2[0] < 0.0 || point2[0] > w-1 || point2[1] < 0.0 || point2[1] > h-1 )
			continue;

		point = point / zoom;
		point2 = point2 / zoom;

		glColor4f( color[0],color[1],color[2],.5);
		glBegin(GL_LINES);
		glVertex2f(x+point[0],y+point[1]);
		glVertex2f(x+point2[0],y+point2[1]);
		glEnd();
		//glColor(ORANGE);
		glBegin(GL_POINTS);
		glVertex2f(x+point[0],y+point[1]);
		glEnd();
	}

	glEnd();
	glDisable(GL_BLEND);

}
int Viewer::drawPoints (vec2dVector points, int x, int y, double zoom, const float color[3], double blending, bool flipHoriz, \
						 bool flipVert, double h, double w)
{
	glEnable(GL_DEPTH_TEST);
	glColor4f(color[0],color[1],color[2],blending);
	glEnable(GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	glBegin(GL_POINTS);
	for (int i=0;i<points.size();i++) {
		Vec2d point = points[i]/zoom;

		if (flipHoriz)
			point[0] = w-1-point[0];
		if (flipVert)
			point[1] = h-1-point[1];

		glVertex2f(x+point[0],y+point[1]);
	}
	glEnd();
	glDisable(GL_BLEND);

	return points.size();
}

void Viewer::drawFace( Vec2d a, Vec2d b, Vec2d c, Vec2d d, int x, int y, double zoom, const float color[3], bool flipEdge, bool flipVert, double h, \
						double w )
{
	glRasterPos2d(0,0);

	if ( flipEdge ) {

		a = flipEdgeDetector( a, h );
		b = flipEdgeDetector( b, h );
		c = flipEdgeDetector( c, h );
		d = flipEdgeDetector( d, h );
	}

	if ( flipVert) {
		a[0] = w-1-a[0];
		a[1] = h-1-a[1];
		b[0] = w-1-b[0];
		b[1] = h-1-b[1];
		c[0] = w-1-c[0];
		c[1] = h-1-c[1];
		d[0] = w-1-d[0];
		d[1] = h-1-d[1];
	}

	a = Vec2d(x,y) + a / zoom;
	b = Vec2d(x,y) + b / zoom;
	c = Vec2d(x,y) + c / zoom;
	d = Vec2d(x,y) + d / zoom;

	glColor(color);

	glBegin(GL_QUADS);
	glVertex2d(a[0],a[1]);
	glVertex2d(b[0],b[1]);
	glVertex2d(c[0],c[1]);
	glVertex2d(d[0],d[1]);
	glEnd();
}

void Viewer::drawEdges (edge2DVector edges, int x, int y, double zoom, const float color[3], bool flipEdge, bool flipVert, double h, \
						double w, int cameraId, int linewidth)
{
	glRasterPos2d(0,0);
		
	for (int i=0;i<edges.size();i++) {
		
		Edge2D edge = edges[i];
				
		if (flipEdge)
			edges[i].flipEdgeDetector(h-1);
		if (flipVert) {
			edges[i].flipVertical(h-1);
			edges[i].flipHorizontal(w-1);
		}
		
		edges[i].scale(x,y,zoom);
		//color = _colors[i%NCOLORS];
		edges[i].draw(color,linewidth);
	}	
}

void Viewer::drawEdgesColorCoded (edge2DVector edges, int x, int y, double zoom, bool flipEdge, bool flipVert, double h, \
						double w, int cameraId, int linewidth)
{
	glLineWidth(linewidth);

	glRasterPos2d(0,0);
		
	for (int i=0;i<edges.size();i++) {
		
		Edge2D edge = edges[i];
				
		if (flipEdge)
			edges[i].flipEdgeDetector(h-1);
		if (flipVert) {
			edges[i].flipVertical(h-1);
			edges[i].flipHorizontal(w-1);
		}
		
		edges[i].scale(x,y,zoom);
		//color = _colors[i%NCOLORS];

		switch ( edges[i]._id ) {
		case 0 :
			edges[i].draw(GREEN,linewidth);
			break;
		case 1 :
			edges[i].draw(ORANGE,linewidth);
			break;
		case 2 :
			edges[i].draw(RED,linewidth);
			break;
		default :
			//edges[i].draw(WHITE,linewidth);
			break;
		}
	}	
}

// draw a camera frustum of size <radius> and in color <color>
//
void Viewer::drawFrustum ( ExtrinsicParameters pose, double radius, const float color[3] ) 
{
	// set the color
	glColor( color );

	// set a standard fov
	double fov_x = toRadians( 80.0 );
	double fov_y = toRadians( 50.0 );

	// specify the three axis
	Vec3d x = pose.getRotation().rotate( Vec3d(1,0,0) );
	Vec3d y = pose.getRotation().rotate ( Vec3d(0,1,0) );
	Vec3d z = pose.getRotation().rotate( Vec3d(0,0,1) );

	Vec3d cc = pose.getTranslation();

	Vec3d a = cc - radius * x + radius / (2 * cos(fov_x / 2.0)) * y + radius / (2 * cos(fov_y / 2.0)) * z;
	Vec3d b = cc - radius * x + radius / (2 * cos(fov_x / 2.0)) * y - radius / (2 * cos(fov_y / 2.0)) * z;
	Vec3d c = cc - radius * x - radius / (2 * cos(fov_x / 2.0)) * y - radius / (2 * cos(fov_y / 2.0)) * z;
	Vec3d d = cc - radius * x - radius / (2 * cos(fov_x / 2.0)) * y + radius / (2 * cos(fov_y / 2.0)) * z;

	glBegin(GL_LINES);
	glVertex(a); 	glVertex(b);
	glVertex(b); 	glVertex(c);
	glVertex(c); 	glVertex(d);
	glVertex(d); 	glVertex(a);

	glVertex(cc);	glVertex(a);
	glVertex(cc); 	glVertex(b); 
	glVertex(cc); 	glVertex(c);
	glVertex(cc); 	glVertex(d);

	glEnd();

	glEnd();



}


void Viewer::drawEdges (edgeVVector &edges, const float color[3], int width, bool wireframe)
{

	glLineWidth(width);

	for (int i=0;i<edges.size();i++) {
		//if (!wireframe && !edges[i].sharp())
		//	continue;

		if (_mode == GL_SELECT)
			glPushName(i);

		edges[i].draw(color,width);

		if (_mode == GL_SELECT)
			glPopName();
	}

	glLineWidth(1);

}

void Viewer::drawEdges (edgeVector &edges, const float color[3], int width, bool wireframe)
{

	glLineWidth(width);

	for (int i=0;i<edges.size();i++) {
		if (!wireframe && edges[i]->_flat)
			continue;
		if (wireframe && edges[i]->_flat) {
			edges[i]->draw(RED,width);
			continue;
		}

		if (_mode == GL_SELECT && edges[i]->_flat)
			continue;

		if (_mode == GL_SELECT )
			glPushName(i);

		edges[i]->draw(color,width);

		if (_mode == GL_SELECT)
			glPopName();
	}

	glLineWidth(1);

}

void Viewer::drawEdgeBoxes (edgeVector &edges, double width)
{
	//LOG(LEVEL_INFO, "***************************************");

	for (int i=0;i<edges.size();i++) {
		///if (!edges[i]->sharp())
		//	continue;

		if (_mode == GL_SELECT && edges[i]->_flat)
			continue;

		if (_mode == GL_SELECT) {
			//LOG(LEVEL_INFO, "pushing name %d", i );
			glPushName(i);
		}

		Edge *edge = edges[i];
		double diam = width*_spheric.rho;
		Vec3d u = norm(cross(edge->dir,norm(_target-_eye)));
		Vec3d a = edge->_a->getPosition() + diam*u;
		Vec3d b = edge->_a->getPosition() -diam*u;
		Vec3d c = edge->_b->getPosition() + diam*u;
		Vec3d d = edge->_b->getPosition() -diam*u;
		//glBegin(GL_POLYGON);
		//glVertex(a,b);
		//glVertex(d,c);
		//glEnd();

		glLineWidth(5);
		glBegin(GL_LINES);
		glVertex(edge->getA());
		glVertex(edge->getB());
		glEnd();

		if (_mode == GL_SELECT)
			glPopName();
	}

}

void Viewer::drawEdgesFeedback (edgeVector &edges, double step)
{
	
	GLint mode;
	glGetIntegerv(GL_RENDER_MODE,&mode);
	//assert(mode == GL_FEEDBACK);
	
	for (int i=0;i<edges.size();i++) {
		Edge *edge = edges[i];
		//printf("%d\n",edge->_id);
		glPassThrough (edge->_id);
		glBegin(GL_POINTS);
		glNormal3f (0.0, 0.0, 1.0);

		for (int j=0;j<edge->length()/step;j++) {
			Vec3d a = edge->_a->getPosition();
			Vec3d b = edge->_b->getPosition();
			Vec3d u = norm(b-a);
			glVertex(a+j*step*u);
		}
		glEnd();
	}
}

void Viewer::drawFaces (faceVector &faces, const float color[3], bool fill)
{

	for (int i=0;i<faces.size();i++)
		faces[i]->draw(fill,color);
}

void Viewer::drawFace( Vec3d a, Vec3d b, Vec3d c, Vec3d d, Vec3d n, double thickness, const float color[3] ) 
{

	glColor(color);

	glBegin(GL_QUADS);
	glVertex(a+n*thickness);
	glVertex(b+n*thickness);
	glVertex(c+n*thickness);
	glVertex(d+n*thickness);
	glEnd();

	glBegin(GL_QUADS);
	glVertex(a-n*thickness);
	glVertex(b-n*thickness);
	glVertex(c-n*thickness);
	glVertex(d-n*thickness);
	glEnd();
}

void Viewer::drawEdgePlanes (edgePlaneVector edgeplanes, Vec3d center, Quaternion rotation, double radius, int cameraId, int color_code, int linewidth)
{
	//color_code: 0 = by edgeId, 1 = by uid, 2 = by cameraId, 3 = by chaineId

	int counter=0;

	//if (_mode == GL_SELECT)
	//		glPushName (cameraId);
		
	for (edgePlaneVector::iterator iter = edgeplanes.begin(); iter != edgeplanes.end(); iter++) {

	//	if (_mode == GL_SELECT)
	//		glPushName (counter);
		
		drawEdgePlane(*iter,center,rotation,radius,color_code,linewidth);

	//	if (_mode == GL_SELECT)
	//		glPopName();

		counter++;
	}

	//if (_mode == GL_SELECT)
	//		glPopName();

}

void Viewer::drawEdgePlane (EdgePlane edgeplane, Vec3d center, Quaternion rotation, double radius, int color_code, int linewidth)
{
	//color_code: 0 = by edgeId, 1 = by uid, 2 = by cameraId, 3 = by chaineId	
	int counter=0;
	
	switch (color_code) {
	case 0:
		glColor(_colors[edgeplane._edgeIds[0]%10]);
		break;
	case 1:
		glColor(_colors[edgeplane._uid%10]);
		break;
	case 2:
		glColor(_colors[edgeplane._cameraId%10]);
		break;
	case 3:
		glColor(_colors[edgeplane._chaineId%10]);
		break;
	default:
		glColor(_colors[edgeplane._edgeIds[0]%10]);
	}
	
	edgeplane.fromCameraFrameToWorldFrame(center,rotation);
	Vec3d a = edgeplane._s+radius*edgeplane._a;
	Vec3d b = edgeplane._s+radius*edgeplane._b;
		
	//drawLine(a,b,linewidth);
	drawArcLine(edgeplane._s,a,b,10,linewidth);
}

void Viewer::drawEdgePlanesDetails (edgePlaneVector edgeplanes, Vec3d center, Quaternion rotation, double radius, int cameraId, int color_code, int linewidth)
{
	//color_code: 0 = by edgeId, 1 = by uid, 2 = by cameraId, 3 = by chaineId
		
	for (edgePlaneVector::iterator iter = edgeplanes.begin(); iter != edgeplanes.end(); iter++) {
		
		drawEdgePlaneDetails(*iter,center,rotation,radius,color_code,linewidth);
	}

}

void Viewer::drawEdgePlaneDetails (EdgePlane edgeplane, Vec3d center, Quaternion rotation, double radius, int color_code, int linewidth)
{
	//color_code: 0 = by edgeId, 1 = by uid, 2 = by cameraId, 3 = by chaineId	
	int counter=0;
	
	switch (color_code) {
	case 0:
		glColor(_colors[edgeplane._edgeIds[0]%10]);
		break;
	case 1:
		glColor(_colors[edgeplane._uid%10]);
		break;
	case 2:
		glColor(_colors[edgeplane._cameraId%10]);
		break;
	case 3:
		glColor(_colors[edgeplane._chaineId%10]);
		break;
	default:
		glColor(_colors[edgeplane._edgeIds[0]%10]);
	}
	
	edgeplane.fromCameraFrameToWorldFrame(center,rotation);

	Vec3d a = edgeplane._s+radius*edgeplane._a;
	Vec3d b = edgeplane._s+radius*edgeplane._b;
		
	drawArcLine(edgeplane._s,a,b,10,linewidth);
	drawLine(edgeplane._s,a,1);
	drawLine(edgeplane._s,b,1);

	//draw the normal vector
	//drawLine(edgeplane._s+radius*(edgeplane._a+edgeplane._b)/3.0,edgeplane._s+radius*(edgeplane._a+edgeplane._b)/3.0+radius*edgeplane._normal/3.0,1);
}

void Viewer::screenshot (const char *filename)
{

	// allocate memory
	GLfloat *pixels;
	unsigned char *pixels_char;
	pixels = (GLfloat*)malloc(3*_width*_height*sizeof(GLfloat));
	pixels_char = (unsigned char*)malloc(3*_width*_height*sizeof(unsigned char));

	// read pixels from the screen
	glReadPixels(0,0,_width,_height,GL_RGB,GL_FLOAT,pixels);

	// save the pixels in an image
	static IplImage *ip;
	static int myfirstTime = 1;
	if (myfirstTime) {
		ip = cvCreateImage(cvSize(_width,_height), IPL_DEPTH_8U, 3);
		myfirstTime = 0;
	}
	fromGLfloatToChar(pixels,pixels_char,_width,_height);
	cvSetImageData(ip,pixels_char,_width*3);
	cvSaveImage(filename,ip);

	// release memory
	//cvReleaseImage(&ip);
	delete pixels;
	delete pixels_char;
}

void Viewer::screenshot (const char *filename, int x1, int y1, int dx, int dy)
{

	//glRasterPos2d( 0, 0 );

	// allocate memory
	GLfloat *pixels;
	unsigned char *pixels_char;
	pixels = (GLfloat*)malloc(3*(dx)*(dy)*sizeof(GLfloat));
	pixels_char = (unsigned char*)malloc(3*(dx)*(dy)*sizeof(unsigned char));

	// read pixels from the screen
	glReadPixels(x1,y1,dx,dy,GL_RGB,GL_FLOAT,pixels);

	// save the pixels in an image
	static IplImage *ip;
	static int myfirstTime = 1;
	if (myfirstTime) {
		ip = cvCreateImage(cvSize(dx,dy), IPL_DEPTH_8U, 3);
		myfirstTime = 0;
	}
	fromGLfloatToChar(pixels,pixels_char,dx,dy);
	cvSetImageData(ip,pixels_char,(dx)*3);
	cvSaveImage(filename,ip);

	// release memory
	//cvReleaseImage(&ip);
	delete pixels;
	delete pixels_char;
}

void Viewer::screenshot (int timestamp, int x1, int y1, int dx, int dy)
{
	char name[256];
	sprintf(name, "snaps/%3d.bmp",timestamp);
	
	screenshot( name, x1, y1, dx, dy );
}


void Viewer::mouse (int button, int state, int x, int y, int glutModifier)
{
	_anchorx = x;
	_anchory = y;

	if (state != GLUT_DOWN) {
		refresh();
		return;
	}
	
	if (glutModifier == GLUT_ACTIVE_CTRL) {
		_istate = none;
		return;
	}

	if (glutModifier == GLUT_ACTIVE_SHIFT) {
		_istate = none;
		return;
	}

	// middle button
	if (button == 2) {
		_istate = translating;
		return;
	}
	
	// left button
	if (button == 1) {
		_istate = rotating;
	}
	// right button
	else if (button == 3) {
		_istate = scaling;
	} else {
		_istate = none;
	}

}

void Viewer::motion (int x, int y, double sens_trans, double sens_rot, double sens_scale)
{
	switch (_istate) {
	case translating:
		//printf("translation\n");
		translateViewPoint(x,y,sens_trans);
		break;
	case rotating:
		//printf("rotation\n");
		rotateViewPoint(x,y,sens_rot);
		break;
	case scaling:
		//printf("scaling\n");
		scaleViewPoint(x,y,sens_scale);
		break;
	case pitching:
	case climbing:
	case none:
		break;
	default:
		//assert(false);
		break;
	}
	setup3D();
	//glutPostRedisplay();
}

void Viewer::glRasterPos2D(int x, int y)
{
	glRasterPos2d(x,y);
}

void Viewer::setFill(bool fill)
{
	if (fill)
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	else
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
}

void Viewer::drawBoxes (std::vector<Vec2d> &vector, const float color[3], int size)
{
	for (int i=0;i<vector.size();i++) {
		drawBox(vector[i][0]-size,vector[i][1]-size,vector[i][0]+size,vector[i][1]+size,1,color);
	}
}

void Viewer::drawBox (int x1, int y1, int x2, int y2, int line_width, const float color[3])
{
	glColor(color);

	glLineWidth(line_width);

	glBegin(GL_LINES);
	glVertex2f(x1,y1);
	glVertex2f(x2,y1);
	glVertex2f(x2,y1);
	glVertex2f(x2,y2);

	glVertex2f(x2,y2);
	glVertex2f(x1,y2);
	glVertex2f(x1,y2);
	glVertex2f(x1,y1);
	glEnd();

	glLineWidth(1);

}

void Viewer::drawBox (Vec2d p1, Vec2d p2, double transparency, const float color[3])
{
	glColor4f(color[0],color[1],color[2],transparency);
	glEnable(GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	int x1 = p1[0], y1 = p1[1], x2 = p2[0], y2 = p2[1];

	glBegin(GL_QUADS);
	glVertex2f(x1,y1);
	glVertex2f(x2,y1);
	glVertex2f(x2,y2);
	glVertex2f(x1,y2);
	glEnd();

	glDisable(GL_BLEND);
}

void Viewer::drawCone( Vec3d center, Vec3d dir, double base_radius, const float color[3], int slices, int stacks ) 
{
	GLUquadricObj* pQuadric = gluNewQuadric();
	glPushMatrix();
	glTranslatef(center[0],center[1],center[2]);
	Vec3d rotation = cross(Vec3d(0,0,1), norm(dir));
	if ( len(rotation) > EPS ) {
		glRotatef(toDegrees(Asin(len(rotation))), rotation[0], rotation[1], rotation[2]);
	}
	glColor(color);
	gluCylinder(pQuadric, base_radius, 0, len(dir), slices, stacks);
	glPopMatrix();
}

void Viewer::drawSphere (Vec3d center, double radius, const float color[3], int slices, int stacks)
{
	GLUquadricObj* pQuadric = gluNewQuadric();
	glPushMatrix();
	glTranslatef(center[0],center[1],center[2]);
	glColor(color);
	gluSphere(pQuadric,radius,slices,stacks);
	glPopMatrix();
}

void Viewer::drawCircle( Vec3d center, Vec3d normal, double radius, const float color[3], int line_width, int slices )
{
	Vec3d u = cross(normal, Vec3d(1,0,0));
	if ( len(u) < EPS )
		u = cross( normal, Vec3d(0,1,0) );
	u = norm(u);
	Vec3d v = u;
	Quaternion q = Quaternion(normal,2*M_PI/slices);

	glColor(color);
	glLineWidth(line_width);

	glBegin(GL_LINES);
	for (int i=0;i<slices+1;i++) {
		v = q.rotate(u);
		glVertex(center + radius * u);
		glVertex(center + radius * v);
		u = v;
	}
	glEnd();

}

void Viewer::drawEllipse(int x, int y, float xradius, float yradius, const float color[3])
{
	glColor(color);
   glBegin(GL_LINE_LOOP);
 
   for (int i=0; i < 360; i++)
   {
      //convert degrees into radians
      float degInRad = toRadians(i);
      glVertex2f(x + cos(degInRad)*xradius, y + sin(degInRad)*yradius);
   }
 
   glEnd();
}

void Viewer::drawBox (Vec3d position, double size, const float color[3], bool fill)
{
	setFill(fill);

	Vec3d a = position + size*Vec3d(1,-1,0);
	Vec3d b = position + size*Vec3d(1,+1,0);
	Vec3d c = position + size*Vec3d(-1,1,0);
	Vec3d d = position + size*Vec3d(-1,-1,0);

	drawTriangle(a,b,c,color);
	drawTriangle(c,d,a,color);
}
	

void Viewer::drawLine (Vec3d a, Vec3d b, const float color[3], int width)
{
	glLineWidth(width);
	glColor(color);

	glBegin(GL_LINES);
	glVertex3f(a[0],a[1],a[2]);
	glVertex3f(b[0],b[1],b[2]);
	glEnd();
}

void Viewer::drawLine (Vec3d a, Vec3d b, int width)
{
	glLineWidth(width);

	glBegin(GL_LINES);
	glVertex3f(a[0],a[1],a[2]);
	glVertex3f(b[0],b[1],b[2]);
	glEnd();
}

void Viewer::drawLine (Vec2d a, Vec2d b, const float color[3], int width)
{
	glLineWidth(width);
	glColor(color);

	glBegin(GL_LINES);
	glVertex2f(a[0],a[1]);
	glVertex2f(b[0],b[1]);
	glEnd();
}

void Viewer::glLighting( Vec3d position )
{
	glEnable(GL_COLOR_MATERIAL);
	//glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	//glMateriali(GL_FRONT, GL_SHININESS, 96);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glShadeModel (GL_SMOOTH);
	// Somewhere in the initialization part of your program…
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	
	// Create light components
	//GLfloat ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	//GLfloat diffuseLight[] = { 0.8f, 0.8f, 0.8, 1.0f };
	//GLfloat specularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };

	glFrontFace(GL_CCW);

	// Assign created components to GL_LIGHT0
	//glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);

	//position[2] = position[2] * 4;
	GLfloat light_position[] = { position[0], position[1], position[2], 0.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}

void Viewer::displayMsg (double x, double y, const float colors[3], int font, const char *string, ...) 
{
	const int MAX_TRACE_STR = 1024;

	if (strlen(string) < MAX_TRACE_STR) {

		char buf[MAX_TRACE_STR*2];

		va_list arg_ptr;

		va_start (arg_ptr,string);

		vsprintf(buf,string,arg_ptr);

		va_end (arg_ptr);

		displayMsg(x,y,buf, colors, font);
	}
}

void Viewer::displayMsg (double x, double y, char *string, \
				   const float colors[3], int font) 
{
	int i;
   
	// Switch to the projection matrix. We want to draw the text "right
	// on to the screen" and we *don't* want the text transforming around
	// with the model.
	glMatrixMode(GL_PROJECTION);
	
	// We need to save the matrix that gluPerspective shoved on there
	// for us... or else we'll lose our lovely view of the model.
	glPushMatrix();
	//
	// Now we set up a new projection for the text. 
	glLoadIdentity();
	glOrtho(0,_width,0,_height,-1.0,1.0);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	// No lighting or depth-testing for text. 
	//glDisable(GL_LIGHTING);
	//glDisable(GL_DEPTH_TEST); 
	
	// Green text looks horrible. Perfect.
	glColor3fv(colors);
	
	// Tell GL where we want to start drawing text.
	glRasterPos2f(x, y);
	// For each character in the string... draw it!
	for (i = 0; i < ((int) strlen(string)); i++) {
		// Let GLUT do the hard part for us
		//glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, string[i]);
		//glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, string[i]);
		glutBitmapCharacter(fonts[font], string[i]);
	}
	
	// We're done with our special "text matrix", so we can get rid
	// of it, and restore the proper perspective matrix for the 3D model.
	glPopMatrix();
	//glEnable(GL_LIGHTING);
	//glEnable(GL_DEPTH_TEST);
	//glReset(_width,_height);
}

void Viewer::restoreView()
{
	Vec3d eye_temp = _save_eye;
	_save_eye = _eye;
	_eye = eye_temp;

	SphericalCoord spheric_temp = _save_spheric;
	_save_spheric = _spheric;
	_spheric = spheric_temp;
}

Viewer::~Viewer()
{
}

void Viewer::drawCorrespondence( corres &c, ExtrinsicParameters pose, const float color[3] )
{
	for (int i=0;i<c.eps.size();i++) {
		EdgePlane e = c.eps[i];
		e.fromCameraFrameToWorldFrame( pose );
		double radius = 50.0;
		drawEdgePlaneDetails(e,Vec3d(0,0,0),Quaternion(),radius,2,6); // color by camera ID
	}
	if ( c.line != NULL )
		drawLine(c.line->getA(),c.line->getB(),color,6); // color by camera ID
}

void Viewer::drawCorrespondenceInfo( corres &c, ExtrinsicParameters &pose, int x, int y, const float color[3] )
{
	// display the line ID
	if (c.line == NULL) {
		displayMsg(x,y,color,1,"Line: NULL");
		return;
	}
	else {
		displayMsg(x,y,color,1,"Line: %d (%f)",c.line->_id, c.line->_weight);
	}
	y -= 15;

	// display the number of edgeplanes
	displayMsg( x, y, color, 1, "size: %d", c.eps.size() );
	y -= 15;

	// display the age
	displayMsg(x,y,color,0,"Age: %d",c.age);

	y -= 15;
	
	// display the status
	switch ( c.line->status ) {
	case ACCEPTED:
		displayMsg(x,y,GREEN,1,"ACCEPTED");
		break;
	case PENDING:
		displayMsg(x,y,ORANGE,1,"PENDING");
		break;
	case UNKNOWN:
		displayMsg(x,y,RED,1,"UNKNOWN");
		break;
	default:
		assert( false );
	}
	y -= 15;

	// display the length
	displayMsg(x,y,color,0,"length: %.4f deg.", toDegrees( c.length ) );
	y -= 15;

	// display the angles
	for (int i=0;i<c.eps.size();i++) {
		EdgePlane ep = c.eps[i];
		ep.fromCameraFrameToWorldFrame( pose );
		displayMsg(x,y,color,0,"[%d][%d] angle : %.4f deg.", i, ep._uid, toDegrees( ep.angle( c.line ) ) );
		y -= 15;
	}

	// display the overlap
	for ( i=0;i<c.eps.size();i++) {
		EdgePlane ep = c.eps[i];
		ep.fromCameraFrameToWorldFrame( pose );
		displayMsg(x,y,color,0,"overlap (%d): %.4f ", i, ep.overlap( c.line ) );
		y -= 15;
	}
}

void Viewer::drawUserMask( int x, int y, double zoom, std::vector< Vec2d > pts, bool flipVertical, int h )
{
	glColor( BLACK );
	glPolygonMode( true );
	glBegin( GL_POLYGON );

	for (int i=0; i<pts.size(); i++ ) {

		double a = x + pts[i][1]/zoom ;
		double b = !flipVertical? y + ( h - 1 - pts[i][0] )/zoom  : y + pts[i][0]/zoom;

		glVertex2f( a, b );
	}

	glEnd();
}

