#include "util/util.h"

void glLookAt (Vec3d position, Vec3d unit_target, Vec3d unit_up)
{
	gluLookAt (position[0], position[1], position[2], position[0]+unit_target[0], \
		position[1]+unit_target[1], position[2]+unit_target[2], unit_up[0], unit_up[1], unit_up[2]);
}

void drawCorners(CvPoint2D32f* corners, int corner_count, int window, const float color[3]) 
{
	
	glutSetWindow(window);
	glColor3f(1,1,0);
	glBegin(GL_LINES);
	for (int i=0;i<corner_count;i++) {
		glBox2D(corners[i],3,3,true,color);
	}
	glEnd();
}

void drawCorners(std::vector<Vertex> corners, int window, int window_width, int window_height, const float color[3]) 
{
	glutSetWindow(window);
	glBegin(GL_LINES);
	glColor3f(1,0,0);
	for (int i=0;i<corners.size();i++) {
		// draw selected corners along with its ID on the window (for your eyes only)
		glBox2D(corners[i].getPosition(),3,3,true,color);
	}
	glEnd();

	for (i=0;i<corners.size();i++) {
		glDisplayMsg(corners[i].getX(),corners[i].getY()-15,i,window_width, window_height, _colors[1], 0);
	}
}

void drawBox (int x, int y, int r, const float colors[3]) 
{
	
	glColor3fv(colors);
	glBegin(GL_LINES);
	glVertex2f(x-r,y+r);
	glVertex2f(x+r,y+r);
	glVertex2f(x+r,y+r);
	glVertex2f(x+r,y-r);
	glVertex2f(x+r,y-r);
	glVertex2f(x-r,y-r);
	glVertex2f(x-r,y-r);
	glVertex2f(x-r,y+r);
	glEnd();
}

void draw2DEdge (Edge2D edge, const float color[3], int lineWidth)
{
	glColor(color);
	glLineWidth(lineWidth);
	glBegin(GL_LINES);

	edge.glVertex();
	glEnd();
	glLineWidth(1);
}

void draw2DEdges(edge2DVector edges, const float color[3])
{
	glColor(color);
	glLineWidth(2);
	glBegin(GL_LINES);

	int i=0;
	for (i=0;i<edges.size();i++) {
		edges[i].glVertex();
	}
	glEnd();
	glLineWidth(1);
	
	// draw the end points of edges
	glPointSize(4);
	glBegin(GL_POINTS);
	
	for (i=0;i<edges.size();i++) {
		edges[i].glVertex();
	}
	glEnd();
	glPointSize(1);
}

void draw3DEdges (edgeVVector edges, const float color[3], int width)
{
	glColor(color);
	glLineWidth(width);
	glBegin(GL_LINES);
	int i=0;
	for (i=0;i<edges.size();i++) {
		Edge edge = edges[i];
		edge.glVertex();
	}
	glEnd();
	glLineWidth(1);
	
	// draw the end points of edges
	glPointSize(width);
	glBegin(GL_POINTS);
	
	for (i=0;i<edges.size();i++) {
		Edge edge = edges[i];
		edge.glVertex();
	}
	glEnd();
	glPointSize(1);
}

void removeEdge (edge2DVector &edges, int id)
{
	assert(id < edges.size());

	edge2DVector::iterator iter;
	int i=0;
	for (iter=edges.begin();iter!=edges.end();iter++) {
		if (i == id) {
			edges.erase(iter);
			return;
		}
		i++;
	}
}

void glPyramid(Vec3d s, Vec3d a, Vec3d b, Vec3d c, Vec3d d, bool fill)
{
	
	if (fill) {
		glBegin(GL_TRIANGLES);
		glVertex(s); glVertex(a); glVertex(b);
		glVertex(s); glVertex(b); glVertex(c);
		glVertex(s); glVertex(c); glVertex(d);
		glVertex(s); glVertex(d); glVertex(a);
		glVertex(a); glVertex(b); glVertex(c);
		glVertex(a); glVertex(c); glVertex(d);
		glEnd();
	}

	if (fill)
		glColor3f(0.0,0.0,0.0);
	
	glBegin(GL_LINES);
	glVertex(s); glVertex(a);
	glVertex(s); glVertex(b);
	glVertex(s); glVertex(c);
	glVertex(s); glVertex(d);
	glVertex(a); glVertex(b);
	glVertex(b); glVertex(c);
	glVertex(c); glVertex(d);
	glVertex(d); glVertex(a);
	glEnd();
	
}

void glVertex (Vec3d v)
{
	glVertex3f(v[0],v[1],v[2]);
}

void glVertex (Vec3d v, Vec3d w)
{
	glVertex(v);
	glVertex(w);
}

void glVertex (Vec3d v1, Vec3d v2, Vec3d v3, Vec3d v4)
{
	glVertex(v1);
	glVertex(v2);
	glVertex(v3);
	glVertex(v4);
}

void glVertex (Vertex v, Vertex w) 
{
	v.glVertex();
	w.glVertex();
}

void glPolygonMode (bool fill)
{
	if (fill)
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	else
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
}

void glBox (Vec3d v, double r_x, double r_y, double r_z, bool fill, const float color[3]) 
{
	glColor3fv(color);
	glPolygonMode(fill);
	Vec3d v1 = v + Vec3d(r_x,r_y,r_z);
	Vec3d v2 = v + Vec3d(r_x,r_y,-r_z);
	Vec3d v3 = v + Vec3d(r_x,-r_y,-r_z);
	Vec3d v4 = v + Vec3d(r_x,-r_y,r_z);

	Vec3d v5 = v + Vec3d(-r_x,r_y,r_z);
	Vec3d v6 = v + Vec3d(-r_x,r_y,-r_z);
	Vec3d v7 = v + Vec3d(-r_x,-r_y,-r_z);
	Vec3d v8 = v + Vec3d(-r_x,-r_y,r_z);

	glBegin(GL_QUADS);
	glVertex(v1,v2,v3,v4);
	glVertex(v5,v6,v7,v8);
	glVertex(v1,v5,v6,v2);
	glVertex(v4,v8,v7,v3);
	glVertex(v1,v5,v8,v4);
	glVertex(v2,v3,v7,v6);
	glEnd();
}

void glBox (Vec3d v, double radius, bool fill, const float color[3]) 
{
	glBox(v,radius,radius,radius,fill,color);
}

void glBox (Vertex v, double r_x, double r_y, double r_z, bool fill, const float color[3]) 
{
	glBox (v.getPosition(),r_x,r_y,r_z,fill,color);
}

void glBox (Vertex *v, double r_x, double r_y, double r_z, bool fill, const float color[3]) 
{
	glBox (v->getPosition(),r_x,r_y,r_z,fill,color);
}

void glBox (Vertex v, double radius, bool fill, const float color[3]) 
{
	glBox(v,radius,radius,radius,fill,color);
}

void glBox (Vertex *v, double radius, bool fill, const float color[3]) 
{
	glBox(v,radius,radius,radius,fill,color);
}

void glColor(const float colors[3])
{
	glColor3f(colors[0],colors[1],colors[2]);
}

void glBox2D (Vec3d v, double r_x, double r_y, bool fill, const float color[3])
{
	glColor3fv(color);
	glPolygonMode(fill);
	
	Vec3d v1 = v + Vec3d(r_x,r_y,0);
	Vec3d v2 = v + Vec3d(r_x,-r_y,0);
	Vec3d v3 = v + Vec3d(-r_x,-r_y,0);
	Vec3d v4 = v + Vec3d(-r_x,r_y,0);
	
	glBegin(GL_QUADS);
	glVertex(v1,v2,v3,v4);
	glEnd();
}

void glBox2D (CvPoint2D32f v, double r_x, double r_y, bool fill, const float color[3])
{
	glBox2D (Vec3d(v.x,v.y,0),r_x,r_y,fill,color);
}

void draw_simple (vertexVector &V, GLenum mode, const float color[3])
{
	Vertex *v;
	if (mode == GL_FEEDBACK) {
		for (int i=0;i<V.size();i++) {
			v = V[i];
			glPassThrough (i);
			glBegin(GL_POINTS);
			v->glVertex();
			glEnd();
		}	
		return;
	}

	glColor3fv(color);

	glBegin(GL_POINTS);
	for (int i=0;i<V.size();i++) {
		V[i]->glVertex();
	}
	glEnd();
}

void drawArcLine (Vec3d s, Vec3d a, Vec3d b, int n, int linewidth)
{
	double alpha = 0.0;
	Vec3d u = norm(a-s);
	Vec3d v = norm(b-s);
	double r = len(a-s);

	glLineWidth(linewidth);
	glBegin(GL_LINES);
	for (int i=0;i<n;i++) {

		double beta = alpha + 1.0/(double)n;
		Vec3d d = s+r*norm(alpha*u+(1.0-alpha)*v);
		Vec3d e = s+r*norm(beta*u+(1.0-beta)*v);
		glVertex(d);
		glVertex(e);
		alpha = beta;
	}
	glEnd();
	glLineWidth(1);
}

void draw (vertexVector &V, int vertexId, int edgeId, GLenum mode, bool pickingMode, double scale, bool redraw, const float color[3])
{
	Vertex *v;

	if (mode == GL_SELECT) {
		glPushName (VERTEX);
		for (int i=0;i<V.size();i++) {
			v = V[i];
			glPushName (i);
			glBox(V[i],scale,true,YELLOW);
			glPopName();
		}	
		glPopName();
		return;
	}

	glPointSize(3);
	glColor3fv(color);

	if (pickingMode) {
		for (int i=0;i<V.size();i++) {
			v = V[i];
			glBox(V[i],scale,true,YELLOW);
		}	
	}

	for (int i=0;i<V.size();i++) {
		
		glPointSize(10);
		
		if (i==vertexId) {
			
			glColor3fv(YELLOW);

			glBox(V[i],scale,true,_colors[0]);

			glLineWidth(5);
			for (int j=0;j<V[i]->_edges.size();j++) {
				
				V[i]->_edges[j]->draw(color,1);
			}
			glLineWidth(1);
			glColor3fv(color);
		}
		if (i==vertexId) glColor3fv(YELLOW);
		glPointSize(1);
		
	}
}

void draw_simple (edgeVector &E, GLenum mode, const float color[3])
{
	if (mode == GL_FEEDBACK) {
		for (int i=0;i<E.size();i++) {
			glPassThrough(i);
			E[i]->draw(color,1);
		}
		return;
	}

	glBegin(GL_LINES);
	for (int i=0;i<E.size();i++) {
			E[i]->draw(color,1);
		}
	glEnd();
}

void draw (edgeVector &E, int vertexId, int edgeId, GLenum mode, bool pickingMode, double scale, bool redraw, const float color[3])
{
	if (mode == GL_FEEDBACK) {
		for (int i=0;i<E.size();i++) {
			glPassThrough(i);
			E[i]->draw(color);
		}
		return;
	}

	glColor3f(0,1,0);
	Edge *e;

	if (mode == GL_SELECT) {
		glPushName(EDGE);
		for (int i=0;i<E.size();i++) {
			e = E[i];
			glPushName (i);
			glBox(e->centroid(),scale,true,YELLOW);
			glPopName();
		}	
		glPopName();
		return;
	}
	
	if (pickingMode) {
		for (int i=0;i<E.size();i++) {
			e = E[i];
			glBox(e->centroid(),scale,true,YELLOW);
		}	
	}

	glColor3fv(color);

	for (int i=0;i<E.size();i++) {
		if (i == edgeId) {
			// draw the wings of the edge

			for (int j=0;j<E[i]->_faces.size();j++) {
				E[i]->_faces[j]->draw(true,color);
			}
			glLineWidth(5);
		}

		E[i]->draw(color);

		if (i == edgeId) {
			glLineWidth(1);
		}
	}
}

void draw_simple (faceVector &F, GLenum mode, const float color[3])
{
	if (mode == GL_FEEDBACK) {
		for (int i=0;i<F.size();i++) {
			glPassThrough(i);
			F[i]->draw(true,color);
		}
		return;
	}

	for (int i=0;i<F.size();i++) {
			F[i]->draw(true,color);
	}
}

void draw (faceVector &F, int faceId, int edgeId, GLenum mode, bool pickingMode, bool redraw, const float color[3])
{
	Face *f;

	if (mode == GL_FEEDBACK) {
		for (int i=0;i<F.size();i++) {
			glPassThrough(i);
			F[i]->draw(true,color);
		}
		return;
	}

	if (mode == GL_SELECT) {
		glPushName(FACE);
		for (int i=0;i<F.size();i++) {
			f = F[i];
			glPushName (i);
			f->draw(true,color);
			glPopName();
		}	
		glPopName();
		return;
	}
	
	glColor3f(1,1,1);

	for (int i=0;i<F.size();i++) {
		if (i == faceId) {
		}
		F[i]->draw(true,color);
		if (i == faceId) {
		}
		
	}
}

void draw (vertVector &V, edgeVVector &E, double scale, Vec3d landmark, const float color_vertex[3], \
		   const float color_landmark[3], const float color_edges[3]) 
{
	unsigned int i=0;

	for (i=0;i<V.size();i++) {
		glBox(&V[i],scale,true,color_vertex);
	}

	glBox(landmark,scale,true,color_landmark);

	glLineWidth(5);
	glBegin(GL_LINES);
	for (i=0;i<E.size();i++) {
		E[i].draw(color_edges);
	}
	glEnd();
	glLineWidth(1);
}	

void draw (vertVector &V, edgeVVector &E, double scale, const float colors1[3], const float colors2[3]) 
{
	unsigned int i=0;

	glColor(colors1);
	for (i=0;i<V.size();i++) {
		glBox(&V[i],scale,true,colors1);
	}

	glLineWidth(5);
	for (i=0;i<E.size();i++) {
		E[i].draw(colors2);
	}
	glLineWidth(1);
}	


void lighting (Vec3d lightsource)
{
	GLfloat mat_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_diffuse[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat light_position[] = { lightsource[0], lightsource[1], lightsource[2], 0.0 };
	GLfloat lm_ambient[] = { 0.1, 0.1, 0.1, 0.5 };
	
	glEnable( GL_DEPTH_TEST );
	glEnable(GL_LIGHTING); // lighting is ON by default
	glEnable( GL_COLOR_MATERIAL );

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glShadeModel (GL_SMOOTH);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_LIGHT0);
}

void lighting (bool on)
{
	if (on) {
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	} else {
		glDisable(GL_LIGHTING);
	}
}
