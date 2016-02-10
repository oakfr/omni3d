#include "util/util.h"
#include "math/linear/linear.h"

Vecd toHomg (Vec3d v)
{
	return Vecd (4,v[0],v[1],v[2],1.0);
}

Vec3d toNonHomg (Vecd v)
{
	return Vec3d(v[0],v[1],v[2]) / v[3];
}

std::string util_toFilename (char *root, std::string str, std::string str2)
{
	return (std::string(root) + "/" + str + "/" + str2);
}

std::string util_toFilename (char *root, char *str, char *str2)
{
	return (std::string(root) + "\\" + std::string(str) + "\\" + std::string(str2));
}

bool existFile (const char *filename) 
{
	std::fstream check;
	check.open(filename, std::ios::in);
	if (!check.is_open())
		return false;
	else
		check.close();
		return true;
}

// return a single value angle comparing two rotations
// a simple metric for the error between a reference orientation
// R_ref and a test orientation R_test is to compute the composite matrix:
// R = R_ref * inv(R_test)
// Then, find the minimum angle of rotation of R by decomposing it into
// axis/angle representation. (matthew antone)
//
double CompareTwoRotations( Quaternion q1, Quaternion q2 )
{

	Quaternion q = q2.bar() * q1;

	double angle = 2.0 * MIN( Acos( q._s ), Acos( -q._s ) );

	return angle;

}

void Free (edgeVVector vector)
{
	for (int i=0;i<vector.size();i++) {
		delete vector[i]._a;
		delete vector[i]._b;
	}
}

void Free (edgeVector vector)
{
	for (int i=0;i<vector.size();i++) {
		delete vector[i]->_a;
		delete vector[i]->_b;
	}
}

void Free (Edge edge)
{
	delete edge._a;
	delete edge._b;
}

std::string findNextAvailableFilename () {
	std::string output;
	char *out;
	out = new char[10];
	std::fstream check;
	for (int i=0;i<100000;i++) {
		sprintf(out,"captures/%d",i);
		sprintf(out,"%s.tif",out);
		check.open(out, std::ios::in);
		if (!check.is_open())
			break;
		check.close();
	}
	check.close();
	output = out;
	return output;
}

// map an index i between 0 and n-1 onto an RGB color (coded 0 < 1) 
// p^3 = n
//
bool indexToColor( int n, int index, float *color )
{
	int p = int( powf( (double)n, (double)1.0/3.0 ) ) + 1;

	int color_step = int( 255.0 / (p+1) );

	int level = int( index / (p*p) );

	int rest = index - p*p*level;

	int x_level = int( rest / p );

	int y_level = rest - x_level * p;

	color[0] = (level+1) * color_step / 255.0;
	color[1] = (x_level+1) * color_step / 255.0;
	color[2] = (y_level+1) * color_step / 255.0;

	return true;
}

// map an RGB color to an index between 0 and n-1
//
bool colorToIndex( int n, int &index, float color[3] )
{
	int p = int( powf( (double)n, (double)1.0/3.0 ) ) + 1;

	int color_step = int( 255.0 / (p+1) );

	int level = int ( color[0] );
	if ( level < color_step )
		return false;
	if ( level % color_step != 0 )
		return false;
	level = level / color_step - 1;

	int x_level = int ( color[1] );
	if ( x_level < color_step )
		return false;
	if ( x_level % color_step != 0 )
		return false;
	x_level = x_level / color_step - 1;

	int y_level = int ( color[2] );
	if ( y_level < color_step )
		return false;
	if ( y_level % color_step != 0 )
		return false;
	y_level = y_level / color_step - 1;

	index = p*p * level + x_level * p + y_level;

	return true;
}

Vec2d flipEdgeDetector( Vec2d a, double h )
{
	double temp = a[0];
	a[0] = a[1];
	a[1] = h-temp;

	return a;
}

Vec2d intersectLines (Vec2d a, Vec2d b, Vec2d c, Vec2d d)
{
	Vec2d cd = d-c;
	Vec2d ab = b-a;
	Vec2d ca = a-c;

	double d1 = ca[0]*ab[1]-ca[1]*ab[0];
	double d2 = cd[0]*ab[1]-cd[1]*ab[0];

	return c + d1/d2 * cd;
}

int fltk2gl_mousebutton (int button) 
{
	switch (button) {
	case FL_LEFT_MOUSE:
		return GLUT_LEFT_BUTTON;
	case FL_RIGHT_MOUSE:
		return GLUT_RIGHT_BUTTON;
	default:
		return GLUT_LEFT_BUTTON;
	}
}

int fltk2gl_mousemodifier ()
{
	if (Fl::event_state(FL_SHIFT))
		return GLUT_ACTIVE_SHIFT;
	if (Fl::event_state(FL_ALT))
		return GLUT_ACTIVE_ALT;
	if (Fl::event_state(FL_CTRL))
		return GLUT_ACTIVE_CTRL;
	return -1;
}

void glDisplayMsg (double x, double y, char *string, int width, int height, \
				   const float colors[3], int font) 
{
	int i;
   
	// Switch to the projection matrix. We want to draw the text "right
	// on to the screen" and we *don't* want the text transforming around
	// with the model.
	glMatrixMode(GL_PROJECTION);
	
	// We need to save the matrix that gluPerspective shoved on there
	// for us... or else we'll lose our lovely view of the model.
	//glPushMatrix();
	//
	// Now we set up a new projection for the text. 
	glLoadIdentity();
	glOrtho(0,width,0,height,-1.0,1.0);
	
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
	//glPopMatrix();
	//glEnable(GL_LIGHTING);
	//glEnable(GL_DEPTH_TEST);
	glReset(width,height);
}

void glDisplayMsg (double x, double y, double num, int width, int height, \
				   const float color[3], int font) 
{
	char str[20];
	if (int(num) == num)
		sprintf(str,"%d",int(num));
	else
		sprintf(str,"%f",num);
	glDisplayMsg(x,y,str,width,height,color,font);
}

void sleep_fps (double fps)
{
	Sleep((double)(1000)/fps);
}

void glReset(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,w,0,h,-1.0,1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRasterPos2f(0,0);
}

void glRectangle (int x, int y, int w, int h, const float color[3], bool full)
{
	if (full)
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	else
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

	glColor3fv(color);
	glBegin(GL_POLYGON);
	glVertex2f(x,y);
	glVertex2f(x+w,y);
	glVertex2f(x+w,y+h);
	glVertex2f(x,y+h);
	glEnd();
}

void glRectangle (int x, int y, int z, int w, int h, const float color[3], bool full)
{
	if (full)
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	else
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

	glColor3fv(color);
	glBegin(GL_POLYGON);
	glVertex3f(x,y,z);
	glVertex3f(x+w,y,z);
	glVertex3f(x+w,y+h,z);
	glVertex3f(x,y+h,z);
	glEnd();
}

void glPolygon (int n, int x, int y, int w, int h, const float color[3], bool full)
{
	if (full)
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	else
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

	double angle = 2*M_PI/n;
	double angle0 = M_PI/n;

	int xc = x+w/2;
	int yc = y+h/2;

	glColor3fv(color);
	glBegin(GL_POLYGON);
	for (int i=0;i<n;i++) {
		glVertex2f(xc+w/2*cos(i*angle+angle0),yc+h/2*sin(i*angle+angle0));
	}
	glEnd();
}

void readVertices (vertexVector &vector)
{
	FILE *fin;
	fin = fopen ("c:/omni3d/data/vertices.txt","r");
	if (fin == NULL) {
		fprintf(stderr,"failed to open vertices file.\n");
		return;
	}

	int id;
	double x,y,z;

	while (fscanf(fin,"%d%lf%lf%lf",&id,&x,&y,&z) == 4) {	
		Vertex *v = new Vertex(id,x,y,z);
		vector.push_back(v);
	}

	fclose(fin);
	printf("%d vertices\n",vector.size());

}

void readEdges (edgeVector &vector, vertexVector &vertex_vector)
{
	FILE *fin;
	fin = fopen ("c:/omni3d/data/edges.txt","r");
	if (fin == NULL) {
		fprintf(stderr,"failed to open vertices file.\n");
		return;
	}

	int id,id1,id2;
	while (fscanf(fin,"%d%d%d",&id,&id1,&id2) == 3) {
		Edge *edge = new Edge(id,vertex_vector[id1], vertex_vector[id2]);
		vector.push_back(edge);
	}

	fclose(fin);
	printf("%d edges\n",vector.size());
}

void readFaces (faceVector &vector, vertexVector &vertex_vector)
{
	FILE *fin;
	fin = fopen ("c:/omni3d/data/faces.txt","r");
	if (fin == NULL) {
		fprintf(stderr,"failed to open vertices file.\n");
		return;
	}

	int id,id1,id2,id3,id4;
	while (fscanf(fin,"%d%d%d%d%d",&id,&id1,&id2,&id3,&id4) == 5) {
		vector.push_back(new Face(id,vertex_vector[id1],vertex_vector[id2],vertex_vector[id3],\
		vertex_vector[id4]));
	}

	fclose(fin);
	printf("%d faces\n",vector.size());
}

void readFaceFile (faceVector &vec, std::string filename)
{
	FILE *fin;
	fin = fopen(filename.c_str(),"r");
	if (fin == NULL) {
		fprintf(stderr,"failed to open file %s\n",filename);
		return;
	}

	vec.clear();
	int n=0;
	double x,y,z;
	vertexVector v;
	
	for (int k=0;k<4;k++)
		v.push_back(new Vertex());

	while (fscanf(fin,"%lf%lf%lf",&x,&y,&z) == 3) {
		v[n] = new Vertex(x,y,z);
		n = (n+1)%4;
		if (n == 0)
			vec.push_back(new Face(v,0));
	}

	if (n != 0)
		fprintf(stderr, "wrong number of lines in file %s. Should be multiple of 4.\n",filename);

	fclose(fin);
}

bool isInside (int x, int y, int x0, int y0, int w, int h)
{
	if (x < x0) return false;
	if (x > x0+w) return false;
	if (y < y0) return false;
	if (y > y0+h) return false;
	return true;
}


// corner_x = MAP_DOWN_LEFT_CORNER_X
// corner_y = MAP_DOWN_LEFT_CORNER_Y
// height = HEIGHT
// map_width = MAP_WIDTH
void mouseMove (int x, int y, int &anchorx, int &anchory, double &zoomMap, \
				SphericalCoord &orientation_eye, int height, int map_width, \
				int corner_x, int corner_y)
{
	if ((height-y > corner_y) && (x < corner_x + map_width)) {
		// 2D map
		// mouse modify the zoom factor
		zoomMap += 4*(x-anchorx);
		orientation_eye.rho = zoomMap;
		anchorx = x;
		anchory = y;
	} else if ((height-y < corner_y) && (x < corner_x + map_width)) {
		// 3D map
		// mouse modify the orientation view
		mouseRotate(x,y,anchorx,anchory,orientation_eye, map_width);
	}
}

void mouseRotate ( int x, int y, int &anchorx, int &anchory, SphericalCoord &orientation_eye, \
				  int width) 
{	
	orientation_eye.phi += pow((y - anchory)/(double)(width),1) * 2.0;
	if (orientation_eye.phi <= 0.0) orientation_eye.phi = 0.0001;
	orientation_eye.theta -= pow((x-anchorx)/(double)(width),1) * 4.0;
	anchorx = x;
	anchory = y;
}

void fromGLfloatToChar (GLfloat *floats, unsigned char *chars, int w, int h)
{
	for (int j=0;j<h;j++) {
		for (int i=0;i<w;i++) {
			chars[3*(j*w+i)+2] = (char)int(255*floats[3*((h-j-1)*w+i)]);
			chars[3*(j*w+i)+1] = (char)int(255*floats[3*((h-j-1)*w+i)+1]);
			chars[3*(j*w+i)] = (char)int(255*floats[3*((h-j-1)*w+i)+2]);
		}
	}
}

void screenSnapshot (int window, int wstart, int hstart, int w, int h, int &timeStamp)
{
	char *filename;
	filename = (char*)malloc(256*sizeof(char));
	sprintf(filename,"snaps/%06d.bmp",timeStamp);
	timeStamp++;

	GLfloat *pixels;

	pixels = (GLfloat*)malloc(3*w*h*sizeof(GLfloat));
	
	unsigned int i=0,j=0;
	
	for (i=0;i<3*w*h;i++)
		pixels[i]=1;

	//glutSetWindow(window);
	glReadPixels(wstart,hstart,w,h,GL_RGB,GL_FLOAT,pixels);

	unsigned char *pixels_c;
	pixels_c = (unsigned char*)malloc(3*w*h*sizeof(char));
	for (j=0;j<h;j++) {
		for (i=0;i<w;i++) {
			pixels_c[3*(j*w+i)+2] = (char)int(255*pixels[3*((h-j-1)*w+i)]);
			pixels_c[3*(j*w+i)+1] = (char)int(255*pixels[3*((h-j-1)*w+i)+1]);
			pixels_c[3*(j*w+i)] = (char)int(255*pixels[3*((h-j-1)*w+i)+2]);
		}
	}

	static int myfirstTime = 1;
	static IplImage *image;
	if (myfirstTime) {
		image = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3); // depth = 8, channels = 3
		myfirstTime = 0;
	}
	
	//LOG(LEVEL_INFO, "saved %d x %d image in %s.", w, h, filename );

	cvSetImageData(image,pixels_c,w*3);
	cvSaveImage(filename,image);
	delete filename;
	delete pixels;
	delete pixels_c;
}

void insertPosition (std::vector< Vertex > &vector, int maxsize, Vertex v, Vertex w, Vertex z)
{
	vector.push_back(v);
	vector.push_back(w);
	vector.push_back(z);
	

	if (vector.size()/3 > maxsize) {
		std::vector<Vertex>::iterator iter;
		iter = vector.begin();
		vector.erase(iter);
		iter = vector.begin();
		vector.erase(iter);
		iter = vector.begin();
		vector.erase(iter);
	}
}

// compute the intersection of a bunch of edgeplanes into a model line
// edgeplanes are expressed in the world coordinate frame
//
bool intersectEdgeplanesToLine( edgePlaneVector edgeplanes, Edge *line )
{
	int i,j;

	if ( edgeplanes.size() < 3 )
		return false;

	Vec3d pa, pb;
	int n = 0;

	for (i=0;i<edgeplanes.size();i++) {

		EdgePlane ei = edgeplanes[i];

		for (j=i+1;j<edgeplanes.size();j++) {

			EdgePlane ej = edgeplanes[j];

			if ( Acos ( dot( ei._normal, ej._normal) ) < toRadians( 5.0 ) ) {
				continue;
			}

			Vec3d a, b, c, d;
			double lambda;

			a = intersectRayPlane( ei._s, ei._a, ej._s, ej._normal, lambda );
			if ( lambda < 0.0 )
				continue;

			b = intersectRayPlane( ei._s, ei._b, ej._s, ej._normal, lambda );
			if ( lambda < 0.0 )
				continue;

			c = intersectRayPlane( ej._s, ej._a, ei._s, ei._normal, lambda );
			if ( lambda < 0.0 )
				continue;

			d = intersectRayPlane( ej._s, ej._b, ei._s, ei._normal, lambda );
			if ( lambda < 0.0 )
				continue;

			// find the longest segment
			Vec3d za = a, zb = b;

			if ( len(c-a) > len(zb-za) ) {
				za = a; zb = c; }
			if ( len(d-a) > len(zb-za) ) {
				za = a; zb = d; }
			if ( len(c-b) > len(zb-za) ) {
				za = b; zb = c; }
			if ( len(d-b) > len(zb-za) ) {
				za = b; zb = d; }
			if ( len(c-d) > len(zb-za) ) {
				za = d; zb = c; }

			// the longest segment is now [za,zb]
			if ( n == 0 ) {
				pa = za; pb = zb; n++;
			} else {
				if ( len( za-pa ) < len(za-pb) && len( zb-pb ) < len(zb-pa) ) {
					pa = 1.0 / ( n+1 ) * za + (double)n / (n+1) * pa;
					pb = 1.0 / ( n+1 ) * zb + (double)n / (n+1) * pb;
					n++;
				} else if ( len( za-pb ) < len(za-pa) && len( zb-pa ) < len(zb-pb) ) {
					pa = 1.0 / ( n+1 ) * zb + (double)n / (n+1) * pa;
					pb = 1.0 / ( n+1 ) * za + (double)n / (n+1) * pb;
					n++;
				}
			}
		}
	}

	if ( n > 0 ) {
		line->_a->setPosition ( pa );
		line->_b->setPosition ( pb );
		line->_id = 0;
		line->_visible = true;
		line->_correspondenceId = 0;
		line->_flat = false;
		line->_horizontal = false;
		line->_weight = 0.0;

		return true;
	}

	return false;

}

void drawFrustum(Eye_pose pose, const float color[3], double focalLength, double size) 
{
	drawFrustum (pose.eye, pose.target, pose.up, color, focalLength, size);
}

void drawFrustum(Vec3d position, Vec3d target, Vec3d up, const float color[3], double focalLength, double size) 
{
	glDisable(GL_LIGHTING);

	Vec3d v = norm(target);
	
	Vec3d u = norm(up);
	
	Vec3d w = norm(cross(v,u));
	
	Vec3d a = position + v * focalLength + (u-w)*size;
	Vec3d b = position + v * focalLength + (u+w)*size;
	Vec3d c = position + v * focalLength - (u-w)*size;
	Vec3d d = position + v * focalLength - (u+w)*size;
	
	glColor3fv(color);
	glLineWidth(3);
	glBegin(GL_LINES);
	glVertex(a,b);
	glVertex(b,c);
	glVertex(c,d);
	glVertex(d,a);
	glVertex(position,a);
	glVertex(position,b);
	glVertex(position,c);
	glVertex(position,d);
	glEnd();

	glEnable(GL_LIGHTING);
	
}

void glCircle (int x, int y, double r1, double r2, int width, int height) 
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,width,0,width,-1.0,1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef (x,y,0);
	GLUquadricObj *myobject;
	myobject = gluNewQuadric();
	gluDisk(myobject,r1,r2,50,50);
}

void subdivideEdge (Edge edge, int id1, int id2, Vertex &aa, Vertex &bb, int SUBDIVISION_RESOLUTION)
{
	int nb=int(edge.length()/SUBDIVISION_RESOLUTION)+1;
	double f11 = (double)id1/nb;
	double f12 = (double)(nb-id1)/nb;
	double f21 = (double)id2/nb;
	double f22 = (double)(nb-id2)/nb;
	aa._position = edge._a->_position * f11 + edge._b->_position * f12;
	bb._position = edge._a->_position * f21 + edge._b->_position * f22;
}

void scaleEdges (edgeVector &edges, int X, int Y, double scale)
{
	Vec3d C(X,Y,0);
	//Vertex C2(WIDTH_IMAGE/2,HEIGHT_IMAGE/2,0);
	Vec3d C2(0,0,0);

	for (int i=0;i<edges.size();i++) {
		Edge *e = edges[i];
		//e.a = Vertex((WIDTH-offset)/2,(HEIGHT-offy)/2,0) + C + ( (e.a - C) * scale);
		//e.b = Vertex((WIDTH-offset)/2,(HEIGHT-offy)/2,0) + C + ( (e.b - C) * scale);
		e->_a->_position = C + C2 + (e->_a->_position - C2) * scale;
		e->_b->_position = C + C2 + (e->_b->_position - C2) * scale;
		//edges[i] = e;
	}
}

void processMousePressed (int x, int y, bool &allEdges, bool &visibleEdges)
{
	if (isInside(x,y,640,415,60,15)) {
		allEdges = !allEdges;
		if (visibleEdges)
			visibleEdges = false;
	}

	if (isInside(x,y,710,415,60,15)) {
		printf("youpi2!\n");
		visibleEdges = !visibleEdges;
		if (allEdges)
			allEdges = false;
	}

	printf("%d %d\n",allEdges,visibleEdges);

}

int readVisibilityDiff (int id1, int id2)
{
	FILE *fin;
	fin = fopen("data/nodes_delta_summary.txt","r");
	if (fin == NULL) {
		fprintf(stderr,"failed to open diff file!\n");
		return 0;
	}

	int _id1, _id2, d1, d2;

	while (fscanf(fin,"%d%d%d%d",&_id1,&_id2,&d1,&d2) == 4) {
		if (((_id1 == id1) && (_id2 == id2)) || \
			((_id1 == id2) && (_id2 == id1))) {
			fclose(fin);
			return (d1+d2);
		}
	}

	//fprintf(stderr,"failed to find diff values for id = %d and %d\n",id1,id2);
	fclose(fin);
	return 600;
}

Vec3d getNormal (Vec3d p1, Vec3d p2, Vec3d p3)
{
	return norm(cross(p1-p2,p1-p3));
}

Mat3d fromEulerTo3x3Matrix (double rx, double ry, double rz)
{
	return EulerXRotation(rx) * EulerYRotation(ry) * EulerZRotation(rz);
}

Mat3d skew (Vec3d w)
{
	Mat3d R;

	R[0][0] = R[1][1] = R[2][2] = 0.0;

	R[0][1] = -w[2];
	R[0][2] = w[1];

	R[1][0] = w[2];
	R[1][2] = -w[0];

	R[2][0] = -w[1];
	R[2][1] = w[0];

	return R;
}

bool removeElement (EdgePlane *edgeplane, edgePlaneVector &edgeplanes)
{

	edgePlaneVectorIter iter = edgeplanes.begin();
	
	for (;iter!=edgeplanes.end();iter++) {
		if (iter->_uid == edgeplane->_uid) {
			edgeplanes.erase(iter);
			return true;
		}
	}
	
	return false;
}

	

Mat3d Identity ()
{
	Mat3d R;
	R[0][0] = R[1][1] = R[2][2] = 1.0;

	R[0][1] = R[0][2] = R[1][0] = R[1][2] = R[2][0] = R[2][1] = 0.0;

	return R;
}

Mat3d rotationFromQuaternion (Vec3d w, double theta)
{
	w = norm(w);
	Mat3d Q = skew(w);

	Mat3d R = Identity() + sin(theta)*Q + (1.0-cos(theta))*Q*Q;

	assert(len(w-R*w)<EPSILON);
	return R;
}

Mat3d EulerXRotation (double theta)
{
	Mat3d M;
	M[0][0] = 1.0;
	M[0][1] = 0.0;
	M[0][2] = 0.0;

	M[1][0] = 0.0;
	M[1][1] = cos(theta);
	M[1][2] = sin(theta);
	
	M[2][0] = 0.0;
	M[2][1] = -sin(theta);
	M[2][2] = cos(theta);

	return M;
}

Mat3d EulerYRotation (double theta)
{
	Mat3d M;
	M[0][0] = cos(theta);
	M[0][1] = 0.0;
	M[0][2] = -sin(theta);

	M[1][0] = 0.0;
	M[1][1] = 1.0;
	M[1][2] = 0.0;
	
	M[2][0] = sin(theta);
	M[2][1] = 0.0;
	M[2][2] = cos(theta);

	return M;
}

Mat3d EulerZRotation (double theta)
{
	Mat3d M;
	M[0][0] = cos(theta);
	M[0][1] = sin(theta);
	M[0][2] = 0.0;

	M[1][0] = -sin(theta);
	M[1][1] = cos(theta);
	M[1][2] = 0.0;
	
	M[2][0] = 0.0;
	M[2][1] = 0.0;
	M[2][2] = 1.0;

	return M;
}

Mat3d toMat3d (Matd P)
{
	Mat3d M(P[0][0],P[0][1],P[0][2],P[1][0],P[1][1],P[1][2],P[2][0],P[2][1],P[2][2]);
	return M;
}

bool readCameraMatrix (FILE *fp, Matd &A)
{
	assert(fp != NULL);
	
	for (int i=0;i<3;i++) {
		for (int j=0;j<4;j++) {
			double val;
			if (fscanf(fp,"%lf",&val) != 1)
				return false;
			A[i][j] = val;
		}
	}

	return true;
}


bool readCameraRotationMatrix (FILE *fp, Mat3d &A)
{
	assert(fp != NULL);
	
	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			double val;
			if (fscanf(fp,"%lf",&val) != 1)
				return false;
			A[i][j] = val;
		}
	}

	return true;
}

bool Exist (int a, std::vector<int> A)
{
	for (int i=0;i<A.size();i++) {
		if (a == A[i])
			return true;
	}

	return false;
}

bool readHomogeneousPoint (FILE *fp, Vec3d &point)
{
	double x,y,z,t;
	if (fscanf(fp,"%lf%lf%lf%lf",&x,&y,&z,&t) != 4)
		return false;

	if (fabs(t) < EPSILON)
		printf("Warning: homogeneous point at infininity (%.4f, %.4f, %.4f, %.4f)!\n",x,y,z,t);
	point = Vec3d (x/t,y/t,z/t);
	return true;
}

bool readNonHomogeneousPoint (FILE *fp, Vec3d &point)
{
	double x,y,z;
	if (fscanf(fp,"%lf%lf%lf",&x,&y,&z) != 3)
		return false;

	point = Vec3d (x,y,z);
	return true;
}

// merge a set of 2D edges into a single edge
bool merge2DEdges ( edge2DVector &edges, Edge2D &edge )
{
	if ( edges.empty() )
		return false;

	//LOG(LEVEL_INFO, "init");
	edge = edges[0];
	//edge.print();

	for (int i=1;i<edges.size();i++) {

		//edges[i].print();

		// for each end point, find the closest end point on <edge> and update it
		Vec2d p = edges[i]._a;
		if ( len( p - edge._a ) < len( p - edge._b ) ) {
			if ( len( edge._b - p ) > len( edge._b - edge._a ) )
				edge._a = p;
		} else {
			if ( len( edge._a - p ) > len( edge._b - edge._a ) )
				edge._b = p;
		}

		p = edges[i]._b;
		if ( len( p - edge._a ) < len( p - edge._b ) ) {
			if ( len( edge._b - p ) > len( edge._b - edge._a ) )
				edge._a = p;
		} else {
			if ( len( edge._a - p ) > len( edge._b - edge._a ) )
				edge._b = p;
		}

		//LOG( LEVEL_INFO, "result");
		//edge.print();
	}

	return true;
}

void detectHoughLines (IplImage *img, IplImage *hough_img, IplImage *temp_img, edge2DVector &edges)
{
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	cvCvtColor( img, temp_img, CV_BGR2GRAY );
	cvCanny( temp_img, hough_img, 50, 200, 3 );
	lines = cvHoughLines2( hough_img, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 50, 30, 10 );
	
	edges.clear();

	for( int i = 0; i < lines->total; i++ )
	{
		CvPoint* lline = (CvPoint*)cvGetSeqElem(lines,i);
		edges.push_back(Edge2D(Vec2d(lline[0].x,hough_img->height-lline[0].y),Vec2d(lline[1].x,hough_img->height-lline[1].y),i));
	}

	cvClearMemStorage(storage);
	cvRelease((void**)&lines);

}

// check that Q belongs to the sub-plane defined by P1, P11 and P12
bool checkPlaneIntersection (Vec3d P1, Vec3d P11, Vec3d P12, Vec3d Q) 
{
	double angle_error = 0.01;

	double angle1 = fabs( dot (norm(P11-P1),norm(Q-P1)));
	double angle2 = fabs( dot (norm(P12-P1),norm(Q-P1)));
	double angle3 = fabs( dot (norm(P12-P1),norm(P11-P1)));
	
	return ( (angle1 > angle3 - angle_error) && (angle2 > angle3 - angle_error));
}

// compute the intersection between a ray defined by (center, ray) and a set of planes
// returns false if no intersection is found, true otherwise
//
int intersectRayFaces( Vec3d center, Vec3d ray, faceVector &faces, Vec3d &point )
{
	// initialize solution to null
	//int face_id = -1;
	int face_id = -1;

	double best_lambda = 1E10;
	double lambda = 0.0;

//	printf("center: %f %f %f\n", center[0], center[1], center[2]);
//	printf("ray: %f %f %f\n", ray[0], ray[1], ray[2]);

	// for each face, compute the intersection
	for (int j=0; j<faces.size();j++ ) {
		Face *f = faces[j];

		if ( f == NULL ) 
			continue;

	//	printf("test\n");
	//	printf("doing face %d (%f %f %f)\n", f->_id, f->_norm[0], f->_norm[1], f->_norm[2]);

		if ( len(ray) < EPS || fabs(dot(ray,f->_norm)) < 1E-4 || len(f->_norm) < EPS)
			continue;

		if ( f->_vertices.empty() ) {
			continue;
		}

		if (f->_vertices[0] == NULL)
			continue;

		//printf("face point: %f %f %f\n", f->_vertices[0]->getPosition()[0], f->_vertices[0]->getPosition()[1], f->_vertices[0]->getPosition()[2]);
		
		Vec3d p = intersectRayPlane( center, ray, f->_vertices[0]->getPosition(), f->_norm, lambda );

	//	printf("lambda: %f   point = %f %f %f\n", lambda, p[0], p[1], p[2]);

		if ( lambda < 0.0 ) // if the face is not in front, skip
			continue;

		if ( lambda > best_lambda ) // if we found a closer face before, skip
			continue;

	//	printf("inside in\n");
		if ( !f->inside( p ) ) // if the point does not belong to the face, skip
			continue;
	//	printf("inside out\n");
//
		best_lambda = lambda;
		
		face_id = j;

		point = p;

	}

	//printf("returning %d\n", face_id);

	return face_id;
}

// compute the min and max dihedral angle of lines l1 and l2 seen from point <p> with a spherical error of diameter <d>
// instead of looking for a complex closed-form solution, i am implementing a brute-force approach
// consisting in sampling the sphere and computing the dihedral angle at each sample point
// stores the sampling results in a pdf (samples)
// also computes the max polar angle. A pair of lines defines two poles. For each pole, we compute the max polar angle, then we take
// the min of the two and store it into max_polar_angles
//
void minMaxDihedralAngle( Edge *l1, Edge *l2, Vec3d p, double diam, double &min_angle, double &max_angle, double &max_polar_angle/*, int n_samples, double *samples*/ )
{
	assert( l1 != NULL);
	assert( l2 != NULL);

	//doubleVector hist;

	int n = 10;

	min_angle = M_PI;
	max_angle = 0.0;

	max_polar_angle = 0.0;

	// sample the sphere by sampling the cube containing the sphere
	for (int i=0;i<n;i++) {
		double x = (double)(i-n/2)/n * diam;
		for (int j=0;j<n;j++) {
			double y = (double)(j-n/2)/n * diam;
			for (int k=0;k<n;k++) {
				double z = (double)(k-n/2)/n * diam;
				
				Vec3d dp = Vec3d(x,y,z);
				if (2.0*len(dp) > diam)
					continue;
				
				Vec3d c = p + dp; // compute corresponding point in the world coord frame
				
				Vec3d l1a = l1->getA() - c;
				Vec3d l1b = l1->getB() - c;
				Vec3d l2a = l2->getA() - c;
				Vec3d l2b = l2->getB() - c;

				Vec3d u = cross( l1a, l1b ); // compute the normal vectors
				Vec3d v = cross( l2a, l2b );

				if ( len(u) < EPS || len(v) < EPS ) // skip if normal vectors are undefined
					continue;
				
				u = norm(u);
				v = norm(v);
				
				double d = dot( u, v );						
				double angle = MIN( Acos(d), Acos(-d) ); // compute the dihedral angle
				
				max_angle = MAX( max_angle, angle );
				min_angle = MIN( min_angle, angle );
				
				//hist.push_back( angle ); // store the angle in a histogram

				max_polar_angle = MAX(max_polar_angle, maxPolarAngle( l1a, l1b, l2a, l2b ));
			}
		}
	}

	// compute the pdf
	/*
	for (int k=0;k<n_samples;k++)
		samples[k] = 0.0;

	for (k=0;k<hist.size();k++) {
		int index = int( n_samples * (hist[k] - min_angle) / (max_angle - min_angle) );
		index = MIN( n_samples-1, MAX(0, index) );
		samples[index]++;
	}

	for (k=0;k<n_samples;k++)
		samples[k] = samples[k] / hist.size();
	*/
}

// compute the max polar angle of two edge planes <l1a,l1b> and <l2a,l2b>
//
double maxPolarAngle( Vec3d l1a, Vec3d l1b, Vec3d l2a, Vec3d l2b )
{
	l1a = norm(l1a); // normalize the vectors
	l1b = norm(l1b);
	l2a = norm(l2a);
	l2b = norm(l2b);

	Vec3d u = cross( l1a, l1b ); // compute the normal vectors
	Vec3d v = cross( l2a, l2b );

	Vec3d pole_1 = norm(cross(u,v)); // compute the poles
	Vec3d pole_2 = - pole_1;

	double max_polar_1 = MAX(Acos(dot(pole_1,l1a)), Acos(dot(pole_1,l1b))); // compute the max polar angle from pole 1
	max_polar_1 = MAX(max_polar_1, Acos(dot(pole_1,l2a)));
	max_polar_1 = MAX(max_polar_1, Acos(dot(pole_1,l2b)));
		
	double max_polar_2 = MAX(Acos(dot(pole_2,l1a)), Acos(dot(pole_2,l1b))); // compute the max polar angle from pole 2
	max_polar_2 = MAX(max_polar_2, Acos(dot(pole_2,l2a)));
	max_polar_2 = MAX(max_polar_2, Acos(dot(pole_2,l2b)));
				
	return MIN( max_polar_1, max_polar_2 ); // keep the min of the two
}

// compute the min and max subtended angle of a model lines given a camera position and a spherical uncertainty (diam)
// use monte carlo sampling
void minMaxSubtendedAngle( Edge *line, Vec3d position, double diam, double &min_angle, double &max_angle )
{
	min_angle = M_PI;
	max_angle = 0.0;
	
	int n = 10;
	
	for (int i=0;i<n;i++) {
		double x = (double)(i-n/2)/n * diam;
		for (int j=0;j<n;j++) {
			double y = (double)(j-n/2)/n * diam;
			for (int k=0;k<n;k++) {
				double z = (double)(k-n/2)/n * diam;
				
				Vec3d dp = Vec3d(x,y,z);
				if (2.0*len(dp) > diam)
					continue;
				
				Vec3d c = position + dp; // compute corresponding point in the world coord frame
				
				Vec3d u = line->getA() - c;
				Vec3d v = line->getB() - c;
				
				if ( len(u) < EPS || len(v) < EPS )
					continue;
				
				double angle = Acos(dot(norm(u), norm(v)));
				
				max_angle = MAX( max_angle, angle );
				min_angle = MIN( min_angle, angle );
			}
		}
	}
}

// which_bag returns the bag ID of the bag containing the feature (x,y)
// booleans up and left are updated according to whether the feature is on the left/right and up/down side of the bag 
// (this is useful to find the neighboring bags later on)
// bags are indexed starting at the lower left corner: 0, 1, ...
//
int which_bag( int x, int y, int window_size, int size_x, int size_y, bool &up, bool &left )
{
	double xr = (double)x / window_size;
	double yr = (double)y / window_size;

	// decides whether the feature is on the left/right and up/down side of the bag 
	up = ( (yr - int(yr)) > 0.5 );
	left = ( (xr - int(xr)) > 0.5 );

	int bag_i = int( yr ) > size_y - 1 ? size_y - 1 : int( yr );
	int bag_j = int( xr ) > size_x - 1 ? size_x - 1 : int( xr );

	// find the bag ID from bag_i and bag_j
	int bag_id = bag_i * size_x + bag_j;

	return bag_id;
}

// finds the neighbor bags of bag ID <bag_id> given the booleans <up> and <left>
// reminder: the bags grid size is size_x x size_y
//
void which_bags_neighbor( int bag_id, std::vector< int > &bag_ids, int size_x, int size_y, bool up, bool left )
{

	int bag_i = bag_id / size_x;
	int bag_j = bag_id - size_x * bag_i;

	if ( left && up ) {

		if ( bag_j < size_x-1 )
			bag_ids.push_back( (bag_i) * size_x + (bag_j+1) );

		if ( bag_i < size_y-1 )
			bag_ids.push_back( (bag_i+1) * size_x + (bag_j) );

		if ( bag_j < size_x-1 && bag_i < size_y-1 )
			bag_ids.push_back( (bag_i+1) * size_x + (bag_j+1) );
	}

	if ( left && !up ) {

		if ( bag_j < size_x-1 )
			bag_ids.push_back( (bag_i) * size_x + (bag_j+1) );

		if ( bag_i > 0 )
			bag_ids.push_back( (bag_i-1) * size_x + (bag_j) );

		if ( bag_j < size_x-1 && bag_i > 0 )
			bag_ids.push_back( (bag_i-1) * size_x + (bag_j+1) );
	}

	if ( !left && up ) {

		if ( bag_j > 0 )
			bag_ids.push_back( (bag_i) * size_x + (bag_j-1) );

		if ( bag_i < size_y-1 )
			bag_ids.push_back( (bag_i+1) * size_x + (bag_j) );

		if ( bag_j > 0 && bag_i < size_y-1 )
			bag_ids.push_back( (bag_i+1) * size_x + (bag_j-1) );
	}

	if ( !left && !up ) {

		if ( bag_j > 0 )
			bag_ids.push_back( (bag_i) * size_x + (bag_j-1) );

		if ( bag_i > 0 )
			bag_ids.push_back( (bag_i-1) * size_x + (bag_j) );

		if ( bag_j > 0 && bag_i > 0 )
			bag_ids.push_back( (bag_i-1) * size_x + (bag_j-1) );
	}

}


// compute the vanishing points of a set of edgeplanes
// the vanishing points can be computed very easily by simply taking the dot product of the normal vectors
// returns a set of unit 3D vectors
//
void computeVanishingPoints( edgePlaneVector &edgeplanes, vp_vector &vps )
{
	// clear the vanishing points
	vps.clear();

	// compute the vanishing points for each pair of edgeplanes
	for (int i=0;i<edgeplanes.size();i++) {
		EdgePlane ep1 = edgeplanes[i];

		for (int j=i+1;j<edgeplanes.size();j++) {
			EdgePlane ep2 = edgeplanes[j];

			Vec3d a = cross( ep1._normal, ep2._normal );

			if ( len(a) > EPS ) {
				vp v1;			// add two symmetric vanishing points
				v1.first = norm(a);
				v1.second = -1;
				vp v2;			
				v2.first = -norm(a);
				v2.second = -1;
			
				// check that the generate vanishing point does not lie inside the edgeplanes
				// (otherwise, it can't be a vanishing point)
				// the test consists in verifying that the cross product with edgeplane end points point to the same direction
				double d1 = dot( cross(a,ep1._a), cross(a,ep1._b));
				double d2 = dot( cross(a,ep2._a), cross(a,ep2._b));

				if ( d1 < 0 || d2 < 0 )
					continue;

				vps.push_back( v2 );
			}
		}
	}
}

// cluster the vanishing points using the k-mean method
// <k> is the number of expected clusters
// <vps> is the input
// <vpc> is the output
//
void clusterVanishingPoints( vp_vector &vps, vp_vector &vpc, int k ) 
{
	int i,j;

	// clear the output
	vpc.clear();

	// initialize the k clusters
	for (i=0;i<k;i++) {
		vpc.push_back( vps[ i * vps.size() / (k+2) ] );
	}

	// loop until done
	while ( 1 ) {

		bool done = true;

		// assign each element to its cluster (closest neighbor)
		for (i=0;i<vps.size();i++) {
			double best_distance = M_PI;
			int id = vps[i].second;
			for (j=0;j<k;j++) {
				if ( len(vpc[j].first) < EPS ) // if the cluster is empty, skip it
					continue;
				double distance = Acos(dot(vpc[j].first,vps[i].first));
				if ( distance < best_distance ) {
					best_distance = distance;
					vps[i].second = j;
				}
			}
			if ( id != vps[i].second ) // if at least one of the vp has changed cluster, we are not done
				done = false;
		}

		// compute the centroids
		for (i=0;i<k;i++) {
			Vec3d centroid = Vec3d(0,0,0);
			int n = 0;

			for (j=0;j<vps.size();j++) {
				if ( vps[j].second == i ) {
					centroid += vps[j].first;
					n++;
				}
			}

			if ( n != 0 ) {
				vpc[i].first = norm(centroid / n);
				vpc[i].second = i;
			} else {
				vpc[i].first = Vec3d(0,0,0);
				vpc[i].second = -1;
			}
		}

		if ( done )
			break;
	}


	// remove duplicates
	vp_vector res;
	for (i=0;i<vpc.size();i++) {
		if ( len(vpc[i].first) > EPS ) { // if cluster non empty
			bool found = false;
			for (j=0;j<res.size();j++) {
				if (fabs(1.0 - dot(res[j].first,vpc[i].first)) < EPS) {
					found = true;
					break;
				}
			}

			if ( !found )
				res.push_back( vpc[i] );
		}
	}

	vpc.clear();
	vpc = res;
}

// score a set of vanishing points against another set of vanishing points
// each vp in set 1 is compared to its closest neighbor in set 2
// the score is the square angle between the two
//
double scoreSetsOfVanishingPoints( vp_vector &set1, vp_vector &set2, double angle_threshold )
{
	double score = 0.0;

	// for each vp in set 1, find the closest neighbor in set 2
	for (int i=0;i<set1.size();i++) {
		vp v1 = set1[i];
		double min_angle = M_PI;

		for (int j=0;j<set2.size();j++) {
			vp v2 = set2[j];
			double angle = Acos( dot( v1.first, v2.first ) );

			if ( angle < min_angle ) {
				min_angle = angle;
			}
		}

		score += min_angle * min_angle;
	}

	return score;
}

// given a set of vanishing points <set1> (expressed in the global coordinate frame)
// and a set of vanishing points <set2> (expressed in the camera coord frame)
// find the best camera rotation that brings <set2> in alignment with <set1>
// using a RANSAC algorithm (two pairs of VPs are sufficient for a "realization")
//
void alignSetsOfVanishingPoints( vp_vector &set1, vp_vector &set2, std::vector< ExtrinsicParameters > &solutions )
{
	int n_trials = 400;
	double angle_threshold = toRadians(10.0);
	double min_score = 1E10;

	solutions.clear();
	ExtrinsicParameters pose, best_pose;

	for (int trial = 0; trial < n_trials; trial++) {

		// select two vps in <set1> randomly
		intVector indx1;
		selectNRandomInt( 2, set1.size(), indx1 );

		// select two vps in <set2> randomly
		intVector indx2;
		selectNRandomInt( 2, set2.size(), indx2 );

		// get corresponding vps
		vp vp11 = set1[indx1[0]];
		vp vp12 = set1[indx1[1]];
		vp vp21 = set1[indx2[0]];
		vp vp22 = set1[indx2[1]];

		// sanity check
		Vec3d a1 = cross(vp11.first,vp12.first);
		Vec3d a2 = cross(vp21.first,vp22.first);

		if ( len(a1) < EPS ) 
			continue;
		if ( len(a2) < EPS ) 
			continue;

		a1 = norm(a1);
		a2 = norm(a2);
		// find the rotation that brings a2 onto a1
		Vec3d a = cross(a1,a2);

		if ( len(a) < EPS ) 
			continue;

		double angle = -Acos(dot(a1,a2));
		Quaternion q1 = Quaternion( a, angle );

		double check = Acos(dot(a1,q1.rotate(a2))); // should be zero

		// apply rotation to the vectors
		vp11.first = q1.rotate( vp11.first );
		vp12.first = q1.rotate( vp12.first );
		vp21.first = q1.rotate( vp21.first );
		vp22.first = q1.rotate( vp22.first );

		// find the rotation along a1 that brings vp21 onto vp11
		a = cross( vp11.first, vp21.first );

		if ( len(a) < EPS )
			continue;

		angle = -Acos(dot(vp11.first, vp21.first));

		Quaternion q2 = Quaternion(a, angle );

		// compose rotations
		Quaternion q = q2 * q1;

		// create a set of rotated vps
		vp_vector rset2;
		for (int i=0;i<set2.size();i++) {
			vp v = set2[i];
			v.first = q.rotate( v.first );
			rset2.push_back( v );
		}

		// score the set
		double score = scoreSetsOfVanishingPoints( set1, rset2, angle_threshold );

		// keep the solution
		ExtrinsicParameters pose;
		pose.setRotation( q );
		pose.set_score( score );
		solutions.push_back( pose );

		// if score is lower, keep it
		if ( score < min_score ) {
			min_score = score;
		}
	}

	// sort the solutions by increasing score
	std::sort( solutions.begin(), solutions.end() );

	for (int i=0;i<solutions.size();i++) {
		LOG(LEVEL_INFO, "score: %f", solutions[i].get_score());
	}
}

// compute the min and max polar angles of lines l1 and l2 seen from point <p> with a spherical error of diameter <d>
// instead of looking for a complex closed-form solution, i am implementing a brute-force approach
// consisting in sampling the sphere and computing the dihedral angle at each sample point
// see edgeplanes.ppt for more info
//
bool minMaxPolarAngles( Edge *l1, Edge *l2, Vec3d p, double diam, double &min1, double &max1, double &min2, double &max2 )
{
	assert (l1 != NULL);
	assert (l2 != NULL);

	int n = 6;
	min1 = min2 = M_PI;
	max1 = max2 = 0.0;
	bool accept = false; // boolean to determine whether the edgeplanes where skew or not

	// sample the sphere
	for (int i=0;i<n;i++) {
		double phi = (double)i / n * M_PI;

		for (int j=0;j<n;j++) {
			double theta = (double)j / n * 2 * M_PI;

			Vec3d c = p + diam / 2 * Vec3d( cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi) ); // sample point on the sphere

			Vec3d l11 = norm(l1->getA() - c); // unit end point directions
			Vec3d l12 = norm(l1->getB() - c);
			Vec3d l21 = norm(l2->getA() - c);
			Vec3d l22 = norm(l2->getB() - c);

			Vec3d n1 = norm(cross(l11,l12)); // unit normal vectors
			Vec3d n2 = norm(cross(l21,l22));

			Vec3d u = cross(n1,n2); // intersection of the two edgeplanes

			if ( len(u) < EPS )  // if the two edgeplanes are not skew, skip it
				continue;

			accept = true;
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
		}
	}

	return accept;
}

// convert a score between 0 and 1 into a color between blue and red
void score_to_color( double score, float color[3] )
{
	color[1] = 4 * ( 1 - score ) * score; // green channel
	color[0] = score; // red channel
	color[2] = 1.0 - score; // blue channel
}


// given a row of n integers, find the p top-k values
// the output is sorted in decreasing order
// lazy boy... use stl implementation!
void find_topk ( int *row, int n, int *values, int p )
{
	intVector v;
	int i;

	for (i=0;i<n;i++)
		v.push_back( row[i] );

	std::sort(v.begin(),v.end());

	std::reverse( v.begin(), v.end() );

	for (i=0;i<MIN(p,n);i++)
		values[i] = v[i];
}

void find_topk ( double *row, int n, double *values, int p )
{
	doubleVector v;
	int i;

	for (i=0;i<n;i++)
		v.push_back( row[i] );

	std::sort(v.begin(),v.end());

	std::reverse( v.begin(), v.end() );

	for (i=0;i<MIN(p,n);i++)
		values[i] = v[i];
}

// intersect two planes (P1,P11,P12) and (P2,P21,P22) and write the output in points (A,B)
bool intersectPlanes (Vec3d P1, Vec3d P11, Vec3d P12, Vec3d P2, Vec3d P21, Vec3d P22, Vec3d &A1, Vec3d &A2, Vec3d &A3, Vec3d &A4)
{
	Vec3d n1 = cross(P11-P1,P12-P1);
	Vec3d n2 = cross(P21-P2,P22-P2);

	double lambda[4];
	bool check[4];
	Vec3d AA[4];
	AA[0] = intersectRayPlane(P1,P11-P1,P2,n2,lambda[0]);
	check[0] = checkPlaneIntersection(P2,P21,P22,AA[0]);
	AA[1] = intersectRayPlane(P1,P12-P1,P2,n2,lambda[1]);
	check[1] = checkPlaneIntersection(P2,P21,P22,AA[1]);
	AA[2] = intersectRayPlane(P2,P21-P2,P1,n1,lambda[2]);
	check[2] = checkPlaneIntersection(P1,P11,P12,AA[2]);
	AA[3] = intersectRayPlane(P2,P22-P2,P1,n1,lambda[3]);
	check[3] = checkPlaneIntersection(P1,P11,P12,AA[3]);

	//printf("check angles: %d %d %d %d\n",check[0],check[1],check[2],check[3]);

	int ok = 0;
	int id[4];

	bool result = true;
	
	for (int i=0;i<4;i++) {
		if ((lambda[i] > 0) && check[i]) {
			id[ok] = i;
			ok++;
		}
	}

	if (ok < 2)
		return false;
	
	//printf("intersection: %d: lambda = %f    %d: lambda = %f\n",id[0],lambda[id[0]],id[1],lambda[id[1]]);

	A1 = AA[id[0]];
	A2 = AA[id[1]];
	if (ok > 2)
		A3 = AA[id[2]];
	else 
		A3 = A1;
	if (ok > 3)
		A4 = AA[id[3]];
	else
		A4 = A2;


	// find the longest edge

	Vec3d A,B;

	A = A1;
	B = A2;

	if (len(A3-A1) > len(B-A))
		B = A3;

	if (len(A4-A1) > len(B-A)) 
		B = A4;

	if (len(A4-A2) > len(B-A)) {
		A = A2;
		B = A4;
	}

	if (len(A3-A2) > len(B-A)) {
		A = A2;
		B = A3;
	}

	if (len(A4-A3) > len(B-A)) {
		A = A3;
		B = A4;
	}

	A1 = A;
	A2 = B;

	return true;
}

// return the shortest distance between a point and an edge
double Distance (Vec3d a, Edge edge)
{
	return Distance (a, edge._a->getPosition(), edge._b->getPosition());
}

double Distance (Edge edge1, Edge edge2)
{
	return Distance (edge1._a->getPosition(),edge2) + Distance(edge1._b->getPosition(),edge2);
}

Edge Average (std::vector <Edge> edges)
{
	Edge edge;

	assert (!edges.empty());

	for (int i=0;i<edges.size();i++) {
		edge._a->_position += edges[i]._a->getPosition();
		edge._b->_position += edges[i]._b->getPosition();
	}

	edge._a->_position /= edges.size();
	edge._b->_position /= edges.size();

	return edge;
}


void insertDouble (doubleVector &vector, double a, int max_size)
{
	vector.push_back(a);
	if (vector.size() > max_size) {
		doubleVector::iterator iter;
		iter = vector.begin();
		vector.erase(iter);
	}
}

void writeVertices (vertexVector &V, const char *filename)
{
	FILE *fin = fileOpen (filename,"w");
	for (int i=0;i<V.size();i++) {
		V[i]->print(fin);
		fprintf(fin,"\n");
	}

	fileClose(fin);
}

void readVertices (vertexVector &V, const char *filename)
{
	FILE *fin = fileOpen (filename,"r");
	Vertex *v = new Vertex();

	/*int id;
	double a,b,c;
	fscanf(fin,"%d%lf%lf%lf",&id,&a,&b,&c);
	printf("%d %f %f %f\n",id,a,b,c);*/

	while (v->read(fin)) {
		V.push_back(v);
		v = new Vertex();

	}

	delete v;
	fileClose(fin);
	printf("%d vertices read.\n",V.size());
}

void writeEdges (edgeVector &E, const char *filename)
{
	FILE *fin = fileOpen (filename,"w");
	for (int i=0;i<E.size();i++)
		E[i]->print(fin);

	fileClose(fin);
}

void readEdges (edgeVector &E, vertexVector &V, faceVector &F, const char *filename)
{
	FILE *fin = fileOpen (filename,"r");
	Edge *edge = new Edge();

	while (edge->read(fin,V,F)) {	
		E.push_back(edge);
		edge = new Edge();
	}

	delete edge;
	fileClose(fin);
}

void writeFaces (faceVector &F, const char *filename)
{
	FILE *fin = fileOpen (filename,"w");
	for (int i=0;i<F.size();i++) {
		fprintf(fin,"%d %d ",F[i]->_id,F[i]->_vertices.size());
		for (int j=0;j<F[i]->_vertices.size();j++) 
			fprintf(fin,"%d ",F[i]->_vertices[j]->_id);
		fprintf(fin,"\n");
	}

	fileClose(fin);
}

void readFaces (faceVector &F, vertexVector &V, const char *filename)
{
	FILE *fin = fileOpen (filename,"r");
	Face *f = new Face();

	while (f->read(fin,V)) {
		F.push_back(f);
		f = new Face();
	}

	delete f;
	fileClose(fin);
}

///////////////////////////////////////////////////////////////
// roundUp
// round any double to xxx.getY()y
double roundUp (double _a) 
{
	int _b = int(100*_a);
	double c = (double)_b/100;
	return c;
}

////////////////////////////////////////////////////////////////
// round
// round any double to the closer integer
int round (double val) {
	int _a = int(val);
	if (val - _a > 0.5) return _a+1;
	return _a;
}

void extractXYZ (const std::string line, double *table) 
{
	std::string token9 = "X";
	std::string token10 = "Y";
	std::string token11 = "Z";
	std::string::size_type pos1 = line.find(token9,0)+2;
	std::string::size_type pos2 = line.find(token10,0)+2;
	std::string::size_type pos3 = line.find(token11,0)+2;
	std::string extract;

	//std::cout << line << std::endl;
	extract = line.substr(pos1, pos2-pos1);
	table[0] = roundUp(atof(extract.c_str()));
	extract = line.substr(pos2, pos3-pos2);
	table[1] = roundUp(atof(extract.c_str()));
	extract = line.substr(pos3, line.length()-1);
	table[2] = roundUp(atof(extract.c_str()));
	//std::cout << "Extract: " << table[0] << ", " << table[1] << ", " << table[2] << std::endl;
}

void extractXYZ (const std::string line, Vec3d &v) 
{
	std::string token9 = "X";
	std::string token10 = "Y";
	std::string token11 = "Z";
	std::string::size_type pos1 = line.find(token9,0)+2;
	std::string::size_type pos2 = line.find(token10,0)+2;
	std::string::size_type pos3 = line.find(token11,0)+2;
	std::string extract;

	//std::cout << line << std::endl;
	extract = line.substr(pos1, pos2-pos1);
	v[0] = roundUp(atof(extract.c_str()));
	extract = line.substr(pos2, pos3-pos2);
	v[1] = roundUp(atof(extract.c_str()));
	extract = line.substr(pos3, line.length()-1);
	v[2] = roundUp(atof(extract.c_str()));
	//std::cout << "Extract: " << table[0] << ", " << table[1] << ", " << table[2] << std::endl;
}

void extractXYZ (const std::string line, Vertex *table) 
{
	std::string token9 = "X";
	std::string token10 = "Y";
	std::string token11 = "Z";
	std::string::size_type pos1 = line.find(token9,0)+2;
	std::string::size_type pos2 = line.find(token10,0)+2;
	std::string::size_type pos3 = line.find(token11,0)+2;
	std::string extract;

	//std::cout << line << std::endl;
	extract = line.substr(pos1, pos2-pos1);
	table->setX(roundUp(atof(extract.c_str())));
	extract = line.substr(pos2, pos3-pos2);
	table->setY(roundUp(atof(extract.c_str())));
	extract = line.substr(pos3, line.length()-1);
	table->setZ(roundUp(atof(extract.c_str())));
	//std::cout << "Extract: " << table[0] << ", " << table[1] << ", " << table[2] << std::endl;
}

///////////////////////////////////////////////////////////////
// display
// display _a vertex vector (for debug purpose)
void printVertexVector (vertexVector V) 
{
	for (int i=0;i<V.size();i++) {
		V[i]->print();
	}
	printf ("\n");
}

///////////////////////////////////////////////////////////////
// display
// display an edge vector (for debug purpose)
void printEdgeVector (edgeVector E) 
{
	for (int i=0;i<E.size();i++) {
		E[i]->print();
		if ((E[i]->_a->_id <0) || (E[i]->_b->_id <0)) {
			printf("ERROR FOUND\n");
		}
	}
	printf("\n");
}

void readEdgesVector (std::vector< std::vector<Edge2D> > &edges, const char *filename)
{
	edges.clear();

	double x1,x2,y1,y2;
	int index;

	FILE *fp = fopen (filename,"r");
	assert (fp != NULL);

	std::vector<Edge2D> s_edges;

	char input[200];
	fgets (input, 200, fp);
	if (input[0] != '#' || input[1] != '^') {
    printf ("Bad edge file \n");
	fclose(fp);
	return;
	}
  
	while (fgets (input, 200, fp)) {
		if (sscanf (input, "%d%lf%lf%lf%lf", &index, &x1, &y1, &x2, &y2)==5) {
			Edge2D edge (Vec2d(x1,y1), Vec2d(x2,y2), index);
			//printf("%d %.4f %.4f %.4f %.4f\n",index,x1,y1,x2,y2);
			s_edges.push_back(edge);
		}

		if (input[0] == '#' && input[1] == '!') {
			edges.push_back(s_edges);
			s_edges.clear();
			//printf("#\n");
		}
	}

	fclose(fp);
}



void writeEdgesVector (std::vector< std::vector<Edge2D> > edges, const char *filename, int nframes)
{

	if (edges.empty())
		return;
	
	int i,j,k;
	Edge2D edge;
	FILE *fp = fopen (filename,"w");
	assert(fp != NULL);
	
	for (i=0; i<nframes; i++) {

		// write header 
		fprintf(fp,"#^\n");
		fprintf(fp,"# Automatically generated edge file\n");
		//fprintf(fp,"# frameId = %d\n",i);
		fprintf(fp,"#\n");
		fprintf(fp,"# format :\n");
		fprintf(fp,"# index    x1    y1    x2    y2 \n");
		fprintf(fp,"#\n\n");

		// write the edges
		edge2DVector frameEdges;

		for (j=0;j<edges.size();j++) {

			for (k=0;k<edges[j].size();k++) {
				
				assert(k<nframes);

				if (edges[j][k]._id == i) {
					edge = edges[j][k];
					edge._id = j;
					
					frameEdges.push_back(edge);
					break;
				}
			}
		}

		for (j=0;j<frameEdges.size();j++) {
			edge = frameEdges[j];
			fprintf(fp,"%d\t%.7f\t%.7f\t%.7f\t%.7f\n",edge._id,edge._a[0],edge._a[1],edge._b[0],edge._b[1]);
		}
		fprintf(fp,"#!\n\n");
	}

	fclose(fp);
}

void rotateCW (IplImage *src, IplImage *dst)
{
	assert ((src->width == dst->height) && (src->height == dst->width));

	int w = src->width;
	int h = src->height;

	char *data_src = src->imageData;
	char *data_dst = dst->imageData;

	for (int x=0;x<w;x++) {
		for (int y=0;y<h;y++) {

			data_src = src->imageData + ((h-y-1)*w+x)*src->nChannels;
			memcpy(data_dst,data_src,dst->nChannels);
			data_dst += dst->nChannels;
		}
	}
}

void rotateCCW (IplImage *src, IplImage *dst)
{
	assert ((src->width == dst->height) && (src->height == dst->width));

	int w = src->width;
	int h = src->height;

	char *data_src = src->imageData;
	char *data_dst = dst->imageData;

	for (int x=0;x<w;x++) {
		for (int y=0;y<h;y++) {

			data_src = src->imageData + (y*w+(w-1-x))*src->nChannels;
			memcpy(data_dst,data_src,dst->nChannels);
			data_dst += dst->nChannels;
		}
	}
}

void flipHorizontal (IplImage *src, IplImage *dst)
{
	assert ((src->width == dst->width) && (src->height == dst->height));

	int w = src->width;
	int h = src->height;

	char *data_src = src->imageData;
	char *data_dst = dst->imageData;

	for (int y=0;y<h;y++) {
		for (int x=0;x<w;x++) {

			data_src = src->imageData + (y*w+(w-1-x))*src->nChannels;
			memcpy(data_dst,data_src,dst->nChannels);
			data_dst += dst->nChannels;
		}
	}
}
///////////////////////////////////////////////////////////////
// subdivideEdge
//
// subdivide an edge into _vertices and put it in edgeVertices
///////////////////////////////////////////////////////////////

void subdivideEdge (vertexVector &edgeVertices, Edge *edge) 
{
	int j=0;
	Vec3d current;
	Vec3d dir = norm(edge->_b->sub(edge->_a));
	
	for (j=0;j<edge->length();j++) {
		Vertex *v;
		current = edge->_a->getPosition() + dir*j;
		v = new Vertex (current[0], current[1], current[2], j);
		edgeVertices.push_back(v);
	}
}

///////////////////////////////////////////////////////////////
// getEdgeFromId
// return an edge given its _id in vector E
Edge* getEdgePtrFromId (int _id, edgeVector &E)
{
	for (int i=0;i<E.size();i++) {
		if (E[i]->_id == _id) return E[i];
	}
	assert(false);
	printf("ERROR: did not find edge for ID = %d\n",_id);
	return NULL;
}

///////////////////////////////////////////////////////////////
// getEdgeFromId
// return an edge given its _id in vector E
Edge getEdgeFromId (int _id, edgeVector &E)
{
	for (int i=0;i<E.size();i++) {
		if (E[i]->_id == _id) return *(E[i]);
	}
	assert(false);
	printf("ERROR: did not find edge for ID = %d\n",_id);
	return Edge();
}

///////////////////////////////////////////////////////////////
// getVertexFromId
// return _a vertex given its _id in vector V
Vertex* getVertexPtrFromId (int _id, vertexVector &V)
{
	for (int i=0;i<V.size();i++) {
		if (V[i]->_id == _id) 
			return V[i];
	}
	printf("ERROR: did not find vertex for ID = %d\n",_id);
	assert(false);
	return NULL;
}

///////////////////////////////////////////////////////////////
// getVertexFromId
// return _a vertex given its _id in vector V
Vertex getVertexFromId (int _id, vertexVector &V)
{
	Vertex w;
	for (int i=0;i<V.size();i++) {
		w = *(V[i]);
		if (w._id == _id) 
			return w;
	}
	printf("ERROR: did not find vertex for ID = %d\n",_id);
	assert(false);
	return Vertex();
}

///////////////////////////////////////////////////////////////
// getSubVertex 
//
// return the position in the 3D world of a subdivided edge vertex
///////////////////////////////////////////////////////////////

Vertex getSubVertex (Edge *edge, int pos, int SUBDIVISION_RESOLUTION)
{
  unsigned int nb=int(edge->length()/SUBDIVISION_RESOLUTION+1);
  Vertex v;
  v._position = (edge->_b->getPosition())*(double)pos/nb + (edge->_a->getPosition())*(double)(nb-pos)/nb;
  v._id = pos;
  v._weight = 0;
  return v;
}

void printTime(const char *text) 
{
  time_t timer = time(NULL);
  printf("TIMER :: %s : %s\n",text,asctime(localtime(&timer)));
}

/* useful function for reading in ppm/pgm headers */
void getline(char *buf,FILE *fp)
{  //read until end of line from fp
	char *p=buf;
	char c;
	while ((c=fgetc(fp))!='\n')
	{	   *p=c;
	p++;
	}
	*p='\0';//terminate as string
}

// 1D DFT
void FFT (int *input, int N, doubleVector &real_vector, doubleVector &im_vector)
{
	real_vector.clear();
	im_vector.clear();

	for (int k=0;k<N;k++) {

		double real = 0.0, im = 0.0;

		for (int j=0;j<N;j++) {

			real += input[j]*cos(-2*M_PI*j*k/N)/N;
			im += input[j]*sin(-2*M_PI*j*k/N)/N;
		}

		real_vector.push_back(real);
		im_vector.push_back(im);
	}

	assert(real_vector.size() == N);
	assert(im_vector.size() == N);
}

void Rotate (Vec3d center, Vec3d axis, double angle, Vec3d &point)
{
	Quaternion q(angle,axis);
	point = center + q.rotate(point-center);
}

// select N random ints between 0 and M-1, each different from each other
// use std::random_shuffle for small lists
//
void selectNRandomInt (int N, int M, intVector &vector) // select N random ints between 0 and M-1, each different from each other
{
	vector.clear();

	if ( 10*N > M ) { // for small lists

		for (int i=0;i<M;i++)
			vector.push_back( i );

		std::random_shuffle( vector.begin(), vector.end() );

		if ( N < M )
			vector.resize( N );
	} else {

		while ( vector.size() < N ) {
			int p = MIN(M-1,(double)rand()/(RAND_MAX+1) * (M-1));

			bool exist = false;

			for (int j=0;j<vector.size();j++) {
				if ( vector[j] == p ) {
					exist = true;
					break;
				}
			}

			if ( !exist )
				vector.push_back( p );
		}
	}
}

// cluser a set of positions into K clusters
// and return the centroid of each cluster
// return true if one of the centroid gets more than twice votes than each other centroid
//
bool cluster_poses( std::vector<ExtrinsicParameters> &poses, std::vector<ExtrinsicParameters> &centroids, int K)
{

	int i,j;

	// if too few samples, exit
	if ( poses.size() <= K ) {

		centroids = poses;
		return false;
	}

	// initialize the clusters
	for (i=0;i<poses.size();i++) {
		poses[i].id = -1;
	//	poses[i].print();
	}

	centroids.clear();
	for (i=0;i<K;i++) {
		poses[i].id = i;
		centroids.push_back( poses[i] );
	//	poses[i].print();
	}

	bool done = false;
	intVector counter;

	while ( !done ) {

		done = true;

		// distribute the poses
		for (i=0;i<poses.size();i++) {

			int best_j = 0;
			
			// find the closest cluster
			for (j=1;j<centroids.size();j++) {
				if ( len(centroids[j].getTranslation() - poses[i].getTranslation()) < len(centroids[best_j].getTranslation() - poses[i].getTranslation()) )
					best_j = j;
			}

			// assign id
			if ( poses[i].id != best_j )
				done = false;

			poses[i].id = best_j;
		}

		// compute the new centroids
		counter.clear();
		for (i=0;i<K;i++) {
			counter.push_back( 0 );
			centroids[i].setTranslation( Vec3d(0,0,0) );
		}

		for (i=0;i<poses.size();i++) {
			centroids[poses[i].id].setTranslation( centroids[poses[i].id].getTranslation() + poses[i].getTranslation());
			counter[poses[i].id]++;
		}

		for (i=0;i<K;i++) {
			if ( counter[i] > 0 ) {
				centroids[i].setTranslation( centroids[i].getTranslation() / counter[i] );
			}
		}
	}

	// compute the rotation of each centroid
	std::vector<Quaternion> quaternions;
	for (i=0;i<K;i++)
		quaternions.push_back(Quaternion(0,Vec3d(0,0,0)));

	for (i=0;i<poses.size();i++)
		quaternions[poses[i].id] = quaternions[poses[i].id] + poses[i].getRotation();

	for (i=0;i<K;i++)
		centroids[i].setRotation( quaternions[i].normalize() );

	// print out counter
	for (j=0;j<counter.size();j++) 
		LOG(LEVEL_INFO, "counter(%d) = %d", j, counter[j]);

	// check whether one of the centroids gets more than twice votes than each other centroid
	int best_j = 0;
	for (j=0;j<centroids.size();j++) {
		if ( counter[j] > counter[best_j] )
			best_j = j;
	}

	bool high_vote = true;
	for (j=0;j<centroids.size();j++) {
		if ( j == best_j ) 
			continue;
		if ( counter[j] * 2 >= counter[best_j] ) {
			high_vote = false;
			break;
		}
	}
	
	// put the best centroid first
	if ( best_j >= 1 ) {
		ExtrinsicParameters p = centroids[0];
		centroids[0] = centroids[best_j];
		centroids[best_j] = p;
	}
	
	// if it is the case, return the best centroid first, otherwise return all centroids as is
	return high_vote;
}

// conversion from RGB to HSV
// r,g,b values are from 0 to 1
// h = [0,1], s = [0,1], v = [0,1]
//		if s == 0, then h = -1 (undefined)
//
// source: http://en.wikipedia.org/wiki/HSV_color_space
//
void RGBtoHSV( double r, double g, double b, double *h, double *s, double *v )
{
	double min, max, delta;
	min = MIN( r, MIN( g, b) );
	max = MAX( r, MAX( g, b) );
	*v = max;				// v
	delta = max - min;
	if( max != 0 )
		*s = delta / max;		// s
	else {
		// r = g = b = 0
		*s = 0;
		*h = 0.0; // undefined
		return;
	}
	if ( delta == 0 ) {
		*h = 0.0; // undefined
		return;
	} else if ( r == max ) {
		if ( g >= b ) {
			*h = 60 * (g-b)/delta;
		} else {
			*h = 60 * (g-b)/delta + 360.0;
		}
	} else if ( g == max ) {
		*h = 60 * (b-r)/delta + 120.0;
	} else if ( b == max ) {
		*h = 60 * (r-g)/delta + 240.0;
	} else {
		*h = 0;
	}
	if( *h < 0 )
		*h += 360.0;

	*h /= 360.0;
}

// conversion from RGB to HSV
// r,g,b values are from 0 to 1
void HSVtoRGB( double h, double s, double v, double *r, double *g, double *b )
{
	int i;
	double f, p, q, t;

	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}

	h /= 60;			// sector 0 to 5
	i = floor( h );
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );

	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
		default:		// case 5:
			*r = v;
			*g = p;
			*b = q;
			break;
	}

}

// return the average of translations
//
Vec3d average_translations( std::vector<ExtrinsicParameters> &poses )
{
	Vec3d average = Vec3d(0,0,0);

	for (int i=0;i<poses.size();i++) {
		average = average + poses[i].getTranslation();
	}

	if ( !poses.empty() )
		average = average / poses.size();

	return average;
}

// return the average of rotations
//
Quaternion average_rotations( std::vector<ExtrinsicParameters> &poses )
{
	if (poses.empty()) 
		return Quaternion();

	return poses[0].getRotation();
}
