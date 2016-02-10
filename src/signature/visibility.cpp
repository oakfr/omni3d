#include "visibility.h"


GLfloat getVal (GLfloat *pixels, int x, int y, int w, int h) 
{
	x = MIN(x,w-1);
	x = MAX(x,0);
	y = MIN(y,h-1);
	y = MAX(y,0);
	return pixels[x+y*w];
}

bool checkValue (GLfloat *pixels, double x, double y, GLfloat feedbackZ, int w, int h, int error) 
{
	int _x = roundi(x), _y = roundi(y);
	GLfloat val = getVal(pixels,_x,_y,w,h);

	//printf("comparing %f against: %f\n",feedbackZ,val);

	if (error == 0)
		//return (fabs(val-feedbackZ)<0.001);
		return (feedbackZ<val);
	else {
		for (int i=0;i<3;i++) {
			for (int j=0;j<3;j++) {
				val = getVal(pixels,_x+i-1,_y+j-1,w,h);
				//if (fabs(val-feedbackZ)<0.0004)
					if (feedbackZ<val)
					return 1;
			}
		}
		return 0;
	}

	return 0;
}

// return true if an edge is visible
// if an edge is smaller than MIN_SUBTENDED_ANGLE viewed from position, it is not added
//
bool processHits (GLint hits, GLfloat *buffer, GLfloat *pixels, Model *model, Vec3d position, 
				  int w, int h,Edge *edge, double STEP)
{
	unsigned int i;
	GLfloat x,y,z;
	GLfloat *ptr;
	GLint code, id;
	ptr = (GLfloat *) buffer;
	int error = 1;
	Vec3d a = edge->_a->getPosition();
	Vec3d b = edge->_b->getPosition();
	Vec3d u = norm(b-a);
	int counter = 0;
	int min=0,max=0;

	// assume edge is not visible
	if ( !edge->_matched )
		edge->_visible = false;
	
	//LOG(LEVEL_INFO, "buffer ( %d )", hits);

	//for (i=0;i<hits;i++) {
	//	printf("%f ", (double)*(ptr+i));
	//}

	for (i=0;i<hits;i++) {
		code = *ptr;

		if (code == GL_PASS_THROUGH_TOKEN) {
			
			id = *(ptr+1);
			ptr += 2;
			i += 1;
			continue;
		}

		if (code == GL_POINT_TOKEN) {
			x = *(ptr+1);
			y = *(ptr+2);
			z = *(ptr+3);

			if ((x>0) && checkValue(pixels,x,y,z,w,h,error)) {
				counter++;
				if ( counter > 1 ) { // if (counter / int(edge->length()/STEP) > .2) {  // more than 20% of the edge is visible
				  edge->_visible = true;
				  edge->_matched = true;
				  return true;
				}

				if (counter == 1)
					min = max = id;
				else {
					min = MIN(min,id);
					max = MAX(max,id);
				}
			}
			ptr += 4;
			i += 3;
			continue;
		}

		ptr++;
	}

	//if (min != max) {
	//	Edge *new_edge = new Edge(edge->_id,a+min*STEP*u,a+max*STEP*u);
		//double cosangle = dot( norm(edge->getA() - position), norm( edge->getB() - position) );
		//if ( cosangle > MIN_SUBTENDED_ANGLE )
		//	return;
		//double ratio = new_edge->length() / edge->length();
		//if ( ratio < 0.25 )
	//		return;
	///	new_edge->_keep = true;
	//	edges.push_back(new_edge);
	//}

//	if ( edge->_id == 1460 )
//		printf("counter: %d\n", counter);
	return edge->_visible;
}


// return true if a point is visible
bool processHits (GLint hits, GLfloat *buffer, GLfloat *pixels, Vec3d position, int w, int h)
{
	unsigned int i;
	GLfloat x,y,z;
	GLfloat *ptr;
	GLint code, id;
	ptr = (GLfloat *) buffer;
	int error = 1;

	for (i=0;i<hits;i++) {
		code = *ptr;

		if (code == GL_PASS_THROUGH_TOKEN) {
			
			id = *(ptr+1);
			ptr += 2;
			i += 1;
			continue;
		}

		if (code == GL_POINT_TOKEN) {
			x = *(ptr+1);
			y = *(ptr+2);
			z = *(ptr+3);
			printf("point: %f %f %f\n", x, y, z);
			if ((x>0) && checkValue(pixels,x,y,z,w,h,error)) {
				return true;
			}
			ptr += 4;
			i += 3;
			continue;
		}

		ptr++;
	}

	return false;
}

// clean-up the visible lines based on dihedral angle criteria and subtended angle criteria
//
void cleanupVisibleLines( Vec3d position, edgeVector &lines, double min_subtended_angle, double min_dihedral_angle, int max_lines )
{
	edgeVector blines;

	// set lines to not visible
	for (int i=0;i<lines.size();i++)
		lines[i]->_visible = false;

	//LOG(LEVEL_INFO, "min subtended angle: %f  min_dihedral_angle: %f max_lines: %d", min_subtended_angle, min_dihedral_angle, max_lines);

	// filter lines with too small subtended angle
	for (edgeVector::iterator iter = lines.begin(); iter != lines.end(); iter++) {
		Edge *line = *iter;
		double subtended_angle = Acos( dot( norm(line->getA() - position), norm(line->getB() - position) ) );
		if ( subtended_angle > min_subtended_angle )
			blines.push_back( line );
	}

	lines.clear();
	lines = blines;

	// filter lines on dihedral angles
	if ( min_dihedral_angle > EPS ) {
		lines.clear();

		// if lines are too similar, filter them
		for ( iter = blines.begin(); iter != blines.end(); iter++ ) {
			Edge *line = *iter;
			bool similar = false;

			for ( edgeVector::iterator iter2 = lines.begin(); iter2 != lines.end(); iter2++ ) {
				Edge *line2 = *iter2;

				// if lines are similar (from the <position> point of view), skip
				Vec3d u = norm( cross( line->getA() - position, line->getB() - position ) );
				Vec3d v = norm( cross( line2->getA() - position, line2->getB() - position ) );
				
				// if <position> is between the two lines, don't test
				if ( dot( ( line->getA() + line->getB() ) / 2.0 - position, ( line2->getA() + line2->getB() ) / 2.0 - position ) < 0 )
					continue;

				// test similarity
				if ( Acos( fabs(dot( u, v )) ) < min_dihedral_angle ) {
					similar = true;
					break;
				}
				if ( M_PI - Acos( fabs(dot( u, v )) ) < min_dihedral_angle ) {
					similar = true;
					break;
				}
			}

			if ( !similar )
				lines.push_back( line );
		}
	}

	// filter on the max number of lines
	if ( max_lines > 0 && max_lines < lines.size() ) {
		for (iter = lines.begin();iter != lines.end(); iter++) {
			(*iter)->computeWeight( position );
		}

		std::sort( lines.begin(), lines.end(),std::greater<Edge*>() );

		lines.resize( max_lines );
	}

	// set lines to visible
	for (i=0;i<lines.size();i++)
		lines[i]->_visible = true;
}

void writeVisibility(Vec3d position, Model *model, edgeVector &edges, const char *file, const char *file_sub, const char *file_list)
{
	// write visible edges
	FILE *f = fopen(file,"a");
	if (f == NULL)
		f = fopen(file,"w");
	assert(f!=NULL);

	fprintf(f,"%f %f %f\n",position[0],position[1],position[2]);
	for (int i=0;i<model->_edges.size();i++) {
		Edge *edge = model->_edges[i];
		if (edge->_visible)
			fprintf(f,"%d ",edge->_id);
	}

	fprintf(f,"\n$\n");

	fclose(f);

	// write visible sub-edges
	f = fopen(file_sub,"a");
	if (f == NULL)
		f = fopen(file_sub,"w");
	assert(f!=NULL);

	fprintf(f,"%f %f %f\n",position[0],position[1],position[2]);
	fflush(f);
	long fpos = ftell(f);

	for (i=0;i<edges.size();i++) {
		Edge *edge = edges[i];
		Vec3d a = edge->_a->getPosition();
		Vec3d b = edge->_b->getPosition();
		fprintf(f,"%d %f %f %f %f %f %f\n",edge->_id,a[0],a[1],a[2],b[0],b[1],b[2]);
	}
	fprintf(f,"\n$\n");
	fclose(f);

	// write position in the table
	f = fopen(file_list,"a");
	if (f == NULL)
		f = fopen(file_list,"w");
	assert(f!=NULL);
	fprintf(f,"%f %f %f %d\n",position[0],position[1],position[2],fpos);
	fclose(f);

}

bool readVisibility (Vec3d position, Model *model, edgeVector &edges, const char *file,const char *file_sub, const char *file_list)
{
	FILE *f = fopen(file_list,"r");
	if (f == NULL)
		return false;

	double x,y,z;
	double x1,y1,z1,x2,y2,z2;
	int id;
	long int fpos;

	while (fscanf(f,"%lf%lf%lf%d",&x,&y,&z,&fpos) == 4) {
		if (len(Vec3d(x,y,z)-position) < EPSILON) {
			
			// read the visible edges
			FILE *f2 = fopen(file_sub,"r");
			assert(f2 != NULL);
			
			fseek(f2,fpos,SEEK_SET);
			
			while (fscanf(f2,"%d%lf%lf%lf%lf%lf%lf",&id,&x1,&y1,&z1,&x2,&y2,&z2) == 7) {
				Edge *edge = model->getEdge(id); //new Edge(id,Vec3d(x1,y1,z1),Vec3d(x2,y2,z2),true);
				//edge->_keep = true;
				edge->computeWeight(position);
				edges.push_back(edge);
				edge->_visible = true;
				//model->getEdge(id)->_visible = true;
			}
			
			fclose(f2);
			fclose(f);
			return true;
		}
	}
	
	fclose(f);
	return false;

}

bool testVisibility (Vec3d position, const char *file_list)
{
	FILE *f = fopen(file_list,"r");
	if (f == NULL)
		return false;

	double x,y,z;
	long int fpos;

	while (fscanf(f,"%lf%lf%lf%d",&x,&y,&z,&fpos) == 4) {
		if (len(Vec3d(x,y,z)-position) < EPSILON)
			return true;
	}
	
	fclose(f);
	return false;

}
