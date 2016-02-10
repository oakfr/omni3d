#include "model.h"

/* destructor
 */
Model::~Model ()
{
	for (int i=0;i<_maxEdgeId;i++)
		delete _edge_table[i];
}

/* Insert a vertex in the model
 */
Vertex* Model::Insert(Vec3d a)
{
	Vertex *v = Find (a,_vertices);
	
	if (v == NULL) {
		v = new Vertex(a[0],a[1],a[2],_vertices.size());
		_vertices.push_back(v);
	}
	
	return v;
}

/* Get vertex from the model
 */
Vertex* Model::getVertex (int id)
{
	if ( id < _vertices.size() && _vertices[id]->_id == id ) 
		return _vertices[id];

	vertexVector::iterator iter;
	
	for (iter=_vertices.begin();iter!=_vertices.end();iter++) {
		if ((*iter)->_id == id)
			return *iter;
	}
	
	return NULL;
}

/* Get face from the model
 */
Face* Model::getFace (int id)
{
	if ( id >= _faces.size() || id < 0) {
		LOG(LEVEL_ERROR, "error accessing face %d out of %d faces.", id, _faces.size());
		return NULL;
	}

	return _faces[ id ];
}

/* Get edge from the model
 */
Edge* Model::getEdge (int id)
{
	return _edge_table[id];
}

/* Find a vertex in a list of vertices
 */
Vertex* Model::Find(Vec3d v, vertexVector V)
{
	vertexVector::iterator iter;
	
	for (iter=V.begin();iter!=V.end();iter++) {
		if (len(v - (*iter)->getPosition()) < MODEL_RESOLUTION)
			return *iter;
	}
	
	return NULL;
}

/* Insert an edge in the model
 */
Edge* Model::Insert(Vec3d a, Vec3d b)
{
	
	Edge *edge = Find (a,b,_edges);
	
	if ((edge == NULL) && (len(b-a) > MODEL_RESOLUTION)) {
		Vertex *v1 = Insert(a);
		Vertex *v2 = Insert(b);
		edge = new Edge(_edges.size(),v1,v2);
		edge->_keep = true;
		_edges.push_back(edge);
	}
	
	return edge;
}

/* Insert an edge in the model
 */
Edge* Model::Insert(Vec3d a, Vec3d b, Face *face)
{
	
	Edge *edge = Find (a,b,_edges);
	
	if ((edge == NULL) && (len(b-a) > MODEL_RESOLUTION)) {
		Vertex *v1 = Insert(a);
		Vertex *v2 = Insert(b);
		edge = new Edge(_edges.size(),v1,v2);
		edge->_keep = true;
		_edges.push_back(edge);
	}
	
	if (edge != NULL) {
		bool found = false;
		for (int j=0;j<edge->_faces.size();j++) {
			if ( edge->_faces[j]->_id == face->_id ) {
				found = true;
				break;
			}
		}
		if ( !found )
			edge->_faces.push_back(face);
	}

	return edge;
}

/* Insert edge in the model
 */
Edge* Model::Insert(Vertex *a, Vertex *b, Face *f)
{
	Edge *edge = NULL;

	bool exist = false;
	for (int i=0;i<_edges.size();i++) {
		if ( (_edges[i]->_a->_id == a->_id && _edges[i]->_b->_id == b->_id) || 
			(_edges[i]->_a->_id == a->_id && _edges[i]->_b->_id == b->_id)) {
			exist = true;
			edge = _edges[i];
			break;
		}
	}

	if ( !exist ) {
		edge = new Edge(_edges.size(),a,b);
		edge->_keep = true;
	}

	exist = false;
	for (int j=0;j<edge->_faces.size();j++) {
		if ( edge->_faces[j]->_id == f->_id ) {
			exist = true;
			break;
		}
	}
	if ( !exist )
		edge->_faces.push_back(f);

	if ( !exist ) 
		_edges.push_back(edge);

	return edge;
}

/* Insert edge in the model
 */
Edge* Model::Insert(Vec3d a, Vec3d b, int id)
{
	
	Edge *edge = getEdge (id);
	
	if (edge == NULL) {
		if (len(b-a) > MODEL_RESOLUTION) {
			Vertex *v1 = Insert(a);
			Vertex *v2 = Insert(b);
			edge = new Edge(id,v1,v2);
			edge->_keep = true;
			_edges.push_back(edge);
		}
	} else {
		edge->_id = id;
		edge->_a = Insert(a);
		edge->_b = Insert(b);
	}
	
	return edge;
}

/* Find an edge in a list of edges
 */
Edge* Model::Find (Vec3d a, Vec3d b, edgeVector V)
{
	edgeVector::iterator iter;
	
	for (iter=V.begin();iter!=V.end();iter++) {

		if ((len(a - (*iter)->_a->getPosition()) < MODEL_RESOLUTION) && (len(b - (*iter)->_b->getPosition()) < MODEL_RESOLUTION))
			return *iter;
		if ((len(a - (*iter)->_b->getPosition()) < MODEL_RESOLUTION) && (len(b - (*iter)->_a->getPosition()) < MODEL_RESOLUTION))
			return *iter;
	}
	
	return NULL;
}

/* Insert a face in the model
 */
Face* Model::Insert (Vec3d a, Vec3d b, Vec3d c, Vec3d d)
{
	int id = _faces.size();
	
	if ( len(b-a) < MODEL_RESOLUTION )
		return Insert(a,c,d);
	if ( len(b-c) < MODEL_RESOLUTION )
		return Insert(a,c,d);
	if ( len(c-a) < MODEL_RESOLUTION )
		return Insert(a,b,d);
	if ( len(d-a) < MODEL_RESOLUTION )
		return Insert(a,c,b);
	if ( len(d-c) < MODEL_RESOLUTION )
		return Insert(a,b,d);
	if ( len(d-b) < MODEL_RESOLUTION )
		return Insert(a,b,c);

	Face *face = new Face (id, Insert(a), Insert(b), Insert(c), Insert(d));
	
	Insert(a,b,face);
	Insert(b,c,face);
	Insert(c,d,face);
	Insert(d,a,face);
	
	_faces.push_back(face);
	
	return face;
}

/* Insert a face in the model
 */
Face* Model::Insert( Vec3d a, Vec3d b, Vec3d c )
{
	int id = _faces.size();

	if ( len(b-a) < MODEL_RESOLUTION )
		return NULL;
	if ( len(b-c) < MODEL_RESOLUTION )
		return NULL;
	if ( len(c-a) < MODEL_RESOLUTION )
		return NULL;

	Face *face = new Face( id, Insert(a), Insert(b), Insert(c) );

	Insert(a,b, face);
	Insert(b,c, face);
	Insert(c,a, face);

	_faces.push_back( face );

	return face;
}

/* Insert a face in the model
 */
Face* Model::Insert (Vertex *a, Vertex *b, Vertex *c, Vertex *d)
{
	int id = _faces.size();
	
	assert( (a!=NULL) && (b!=NULL) && (c!=NULL) && (d!=NULL));

	if ( len(b->getPosition()-a->getPosition()) < MODEL_RESOLUTION )
		return Insert(a,c,d);
	if ( len(b->getPosition()-c->getPosition()) < MODEL_RESOLUTION )
		return Insert(a,c,d);
	if ( len(c->getPosition()-a->getPosition()) < MODEL_RESOLUTION )
		return Insert(a,b,d);
	if ( len(d->getPosition()-a->getPosition()) < MODEL_RESOLUTION )
		return Insert(a,c,b);
	if ( len(d->getPosition()-c->getPosition()) < MODEL_RESOLUTION )
		return Insert(a,b,d);
	if ( len(d->getPosition()-b->getPosition()) < MODEL_RESOLUTION )
		return Insert(a,b,c);

	Face *face = new Face (id, a,b,c,d);

	Insert(a,b,face);
	Insert(b,c,face);
	Insert(c,d,face);
	Insert(d,a,face);

	_faces.push_back(face);
	
	return face;
}

/* Insert a face in the model
 */
Face* Model::Insert (Vertex *a, Vertex *b, Vertex *c)
{
	int id = _faces.size();
	
	assert( (a!=NULL) && (b!=NULL) && (c!=NULL));

	if ( len(b->getPosition()-a->getPosition()) < MODEL_RESOLUTION )
		return NULL;
	if ( len(b->getPosition()-c->getPosition()) < MODEL_RESOLUTION )
		return NULL;
	if ( len(c->getPosition()-a->getPosition()) < MODEL_RESOLUTION )
		return NULL;

	Face *face = new Face (id, a,b,c);

	Insert(a,b,face);
	Insert(b,c,face);
	Insert(c,a,face);

	_faces.push_back(face);
	
	return face;
}

/* Insert a cube in the model (debugging)
 */
void Model::InsertCube (std::vector<Vec3d> points, int index)
{
	if (points.size() < index + 8) {
		printf("WARNING: less than 8 points in the cube (%d)!\n",points.size());
		return;
	}
	
	Vec3d a,b,c,d,e,f,g,h;
	
	a = points[index+0];
	b = points[index+1];
	c = points[index+2];
	d = points[index+3];
	
	e = points[index+4];
	f = points[index+5];
	g = points[index+6];
	h = points[index+7];
	
	Insert(a,b);
	Insert(b,c);
	Insert(c,d);
	Insert(d,a);
	
	Insert(e,f);
	Insert(f,g);
	Insert(g,h);
	Insert(h,e);
	
	Insert(a,e);
	Insert(b,f);
	Insert(c,g);
	Insert(d,h);
}

/* Insert a cube in the model
 */
void Model::InsertCube (Vec3d origin, Vec3d dir1, Vec3d dir2, double l1, double l2, double l3)
{
	Vec3d a,b,c,d,e,f,g,h;
	Vec3d u = norm(dir1), v = norm(dir2), w = norm(cross(u,v));
	
	a = origin;
	b = a + l1 * u;
	c = a + l1 * u + l2 * v;
	d = a + l2 * v;
	
	e = a + w * l3;
	f = b + w * l3;
	g = c + w * l3;
	h = d + w * l3;
	
	Insert(a,b);
	Insert(b,c);
	Insert(c,d);
	Insert(d,a);
	
	Insert(e,f);
	Insert(f,g);
	Insert(g,h);
	Insert(h,e);
	
	Insert(a,e);
	Insert(b,f);
	Insert(c,g);
	Insert(d,h);
}

/* Print out a model to std output
 */
void Model::print()
{
	printf("**** PRINTING MODEL ****\n");
	printf("**** VERTICES [%d]  ****\n",_vertices.size());
	
	vertexVector::iterator iterv;
	for (iterv=_vertices.begin();iterv!=_vertices.end();iterv++) {
		(*iterv)->print();
	}
	
	printf("**** EDGES [%d]  ****\n",_edges.size());
	
	edgeVector::iterator itere;
	for (itere=_edges.begin();itere!=_edges.end();itere++) {
		(*itere)->print();
	}
	
	printf("**** FACES [%d]  ****\n",_faces.size());
	printf("we don't print faces.\n");
}

/* Read a model from directory
 */
bool Model::read (const char *dirname, ModelType type)
{
	_dirname = std::string(dirname);
	_type = type;

	clear();

	switch (type) {
	case AUTOCAD:
		MODEL_FILE = std::string("model.log");
		break;
	case INVENTOR:
		MODEL_FILE = std::string("model.iv");
		break;
	default:
		assert(false);
		break;
	}

	_homography_translation = Vec3d(0,0,0);
	_homography_rotation = Quaternion();

	// first look if the geometry files are older than the .log file
	std::string s1 = toFilename(MODEL_FILE.c_str());
	char* file_log = (char*)s1.c_str();
	std::string s2 = toFilename(MODEL_VERTEX_FILE);
	char *file_geom = (char*)s2.c_str();
	std::string s3 = toFilename(MODEL_LIST_FILE);
	char *file_list = (char*)s3.c_str();
	std::string s4 = toFilename(MODEL_CONFIG_FILE).c_str();
	char *file_config = (char*)s4.c_str();

	_global_offset = 0;

	if (!existFile(file_geom) || isFileOlder(file_log,file_geom) || isFileOlder(file_list,file_geom) || isFileOlder(file_config, file_geom)) {

		if ( existFile(file_list) ) { // we are dealing with a list of models
			std::ifstream inFile (file_list);
			std::string line;
			while (std::getline(inFile,line,'\n')) {
				std::string s4 = toFilename(line.c_str());
				file_log = (char*)s4.c_str();
				if ( !existFile(file_log) )
					continue;

				// read homography to apply to the model, if any
				_homography_translation = Vec3d(0,0,0);
				_homography_rotation = Quaternion();

				if ( existFile( toFilename("homography.dat").c_str() ) ) {
					std::ifstream inFileHomography (toFilename("homography.dat").c_str() );
					std::string lineHomography;
					while (std::getline(inFileHomography,lineHomography,'\n')) {
						char mname[120];
						char str[256];
						sprintf(str,"%s",lineHomography.c_str());
						double rx,ry,rz,tx,ty,tz;
						
						if ( sscanf(str, "%s%lf%lf%lf%lf%lf%lf", mname, &rx,&ry,&rz,&tx,&ty,&tz) == 7 ) {
							if ( strcmp(mname, line.c_str()) == 0 ) {
								LOG(LEVEL_INFO, "homography on model %s: %f %f %f %f %f %f", mname, rx, ry, rz, tx, ty, tz );
								_homography_translation = Vec3d(tx,ty,tz);
								Vec3d axis = Vec3d(rx,ry,rz);
								_homography_rotation = Quaternion( axis, len(axis) );
							}
						}
					}

					inFileHomography.close();
				}

				readFile(file_log, dirname);
			}
			inFile.close();
			buildConnectivity();
			cleanupModel();

		} else	if (!existFile(file_log)) {
			printf("warning: model files %s and %s do not exist. Generate with Autocad first.\n",file_log, file_list);
			assert(false);
			return false;
		} else { // we are dealing with a single model
			LOG(LEVEL_INFO, "reading geometry from file %s", file_log);
			readFile(file_log,dirname);
			buildConnectivity();
			cleanupModel();
			
		}
		updateModel();
		writeGeometry(dirname);
		//readTextures();
		return true;
	}
	
	LOG(LEVEL_INFO,"reading geometry from geom/");
	readGeometry(dirname);
	updateModel();
	//readTextures();
	return false;

}

/* Compute model centroid and diameter
 */
void Model::computeCentroidAndDiameter()
{
	// centroid
	_centroid = Vec3d(0,0,0);
	assert(_vertices.size() > 0);
	
	for (int i=0;i<_vertices.size();i++)
		_centroid += _vertices[i]->getPosition();
	
	_centroid /= _vertices.size();
	
	// diameter
	_diameter = 0.0;
	
	for (i=0;i<_vertices.size();i++)
		_diameter = MAX(_diameter,len(_vertices[i]->getPosition()-_centroid));
}

/* Read geometry from a file
 */
int Model::readFile (const char *filename, const char *dirname)
{
	switch(_type) {
	case AUTOCAD:
		readAutocadFile(filename,dirname);
		break;
	case INVENTOR:
		readInventorFile(filename,dirname);
		break;
	default:
		printf("Unrecognized model type.\n");
		assert(false);
	}

	return 0;
}

/* Cleanup a model files
 */
void Model::cleanup(bool geometry, bool nodes)
{
	if (geometry) {
		fileRemove(toFilename(MODEL_VERTEX_FILE).c_str());
		fileRemove(toFilename(MODEL_FACE_FILE).c_str());
	}

	if (nodes) {
		fileRemove(toFilename(MODEL_VISI_FILE).c_str());
		fileRemove(toFilename(MODEL_NODE_FILE).c_str());
	}
}

/* Cleanup a model (duplicates, etc.)
 */
void Model::cleanupModel()
{
	removeVertexDuplicates();
	removeEdgeDuplicates();
	removeFaceDuplicates();
	mergeFaces();
	mergeEdges();

	updateModel();
}

/* Remove edges duplicates
 */
void Model::removeEdgeDuplicates()
{
	int i;	
	int counter = 0;

	// remove all duplicates in the edge list
	bool done = false;

	for (i=0;i<_edges.size();i++)
		_edges[i]->_visible = false;

	while (!done ) {
		done = true;
		for (edgeVector::iterator iter1 = _edges.begin(); iter1 != _edges.end(); iter1++) {

			if ( (*iter1)->_visible )
				continue;

			edgeVector::iterator iter2 = iter1;
			++iter2;

			for (; iter2 != _edges.end(); iter2++) {

				if ( (*iter1) == (*iter2) || (*iter1)->equal(*iter2) ) {

					// replace in all vertices
					for (int j=0;j<(*iter2)->_a->_edges.size();j++) {
						if ( (*iter2)->_a->_edges[j]->_id == (*iter2)->_id ) {
							(*iter2)->_a->_edges[j] = *iter1;
						}
					}
					for (j=0;j<(*iter2)->_b->_edges.size();j++) {
						if ( (*iter2)->_b->_edges[j]->_id == (*iter2)->_id ) {
							(*iter2)->_b->_edges[j] = *iter1;
						}
					}

					// update the faces
					for ( j=0;j<(*iter2)->_faces.size();j++) {
						bool found=false;
						for (int k=0;k<(*iter1)->_faces.size();k++) {
							if ( (*iter1)->_faces[k]->_id == (*iter2)->_faces[j]->_id ) {
								found=true;
								break;
							}
						}
						if (!found)
							(*iter1)->_faces.push_back( (*iter2)->_faces[j] );
					}

					_edges.erase( iter2 );
					counter++;
					done = false;
					break;
				}
			}

			if ( !done )
				break;
			else
				(*iter1)->_visible = true;
		}
	}

	// remove duplicate in each vertex list
	for (i=0;i<_vertices.size();i++) {
		edgeVector es;

		for (int j=0;j<_vertices[i]->_edges.size();j++) {
			int id = _vertices[i]->_edges[j]->_id;
			bool exist = false;
			for (int k=0;k<es.size();k++) {
				if ( es[k]->_id == id ) {
					exist = true;
					break;
				}
			}
			if ( !exist ) {
				es.push_back( _vertices[i]->_edges[j] );
			}
		}

		_vertices[i]->_edges = es;
	}

	LOG(LEVEL_INFO, "removed %d duplicate edges", counter);
}

/* Remove faces duplicates
 */
void Model::removeFaceDuplicates()
{
	int i;
	int counter = 0;

	// for each pair of face, check whether they are equivalent
	for (i=0;i<_faces.size();i++)
		_faces[i]->_visible = false;

	bool done = false;
	
	while (!done ) {
		done = true;
		for (faceVector::iterator iter1 = _faces.begin(); iter1 != _faces.end(); iter1++) {

			if ( (*iter1)->_visible )
				continue;

			faceVector::iterator iter2 = iter1;
			++iter2;

			for (; iter2 != _faces.end(); iter2++) {

				if ( (*iter1) == (*iter2) || (*iter1)->equivalent(*iter2) ) {

					// replace in all edges
					for (int j=0;j<(*iter2)->_vertices.size();j++) {
						for (int k=0;k<(*iter2)->_vertices[j]->_edges.size();k++) {
							for (int m=0;m<(*iter2)->_vertices[j]->_edges[k]->_faces.size();m++) {
								if ( (*iter2)->_vertices[j]->_edges[k]->_faces[m]->_id == (*iter2)->_id ) {
									(*iter2)->_vertices[j]->_edges[k]->_faces[m] = *iter1;
								}
							}
						}
					}

					_faces.erase( iter2 );
					counter++;
					done = false;
					break;
				}
			}

			if ( !done )
				break;
			else
				(*iter1)->_visible = true;
		}
	}

	// remove duplicates in each edge list
	for (i=0;i<_edges.size();i++) {
		faceVector fs;

		for (int j=0;j<_edges[i]->_faces.size();j++) {
			int id = _edges[i]->_faces[j]->_id;
			bool exist = false;
			for (int k=0;k<fs.size();k++) {
				if ( fs[k]->_id == id ) {
					exist = true;
					break;
				}
			}
			if ( !exist )
				fs.push_back( _edges[i]->_faces[j] );
		}

		_edges[i]->_faces = fs;
	}
	
	LOG(LEVEL_INFO, "removed %d duplicate faces", counter);
}

/* Remove duplicates and flat edges
 */
void Model::removeVertexDuplicates() 
{

	int i,j,k,m,c=0;

	bool done = false;

	for (i=0;i<_vertices.size();i++)
		_vertices[i]->_visible = false;

	while (!done) {

		done =true;

		for (vertexVector::iterator iter1 = _vertices.begin(); iter1 != _vertices.end(); iter1++) {
		
			if ( (*iter1)->_visible )
				continue;

			vertexVector::iterator iter2 = iter1;
			++iter2;

			for ( ; iter2 != _vertices.end(); iter2++) {
				
				if ( len((*iter1)->getPosition() - (*iter2)->getPosition()) < MODEL_RESOLUTION ) {
					
					// replace iterator 2 by iterator 1 everywhere
					
					// replace reference in faces
					for (int p=0;p<(*iter2)->_edges.size();p++) {
						for (k=0;k<(*iter2)->_edges[p]->_faces.size();k++) {
							
							for (m=0;m<(*iter2)->_edges[p]->_faces[k]->_vertices.size();m++) {
								if ( (*iter2)->_edges[p]->_faces[k]->_vertices[m]->_id == (*iter2)->_id ) 
									(*iter2)->_edges[p]->_faces[k]->_vertices[m] = *iter1;
							}
						}
					}
					
					// replace reference in edges
					for (k=0;k<(*iter2)->_edges.size();k++) {
						
						if ( (*iter2)->_edges[k]->_a->_id == (*iter2)->_id ) 
							(*iter2)->_edges[k]->_a = *iter1;
						if ( (*iter2)->_edges[k]->_b->_id == (*iter2)->_id ) 
							(*iter2)->_edges[k]->_b = *iter1;
					}
					
					// actually remove element
					iter2 = _vertices.erase( iter2 );
					done = false;
					c++;
					//printf("remaining %d elements\n", _vertices.size());
					break;
				} 
			}

			if ( !done )
				break;
			else
				(*iter1)->_visible = true;
		}
	}

	// remove duplicate vertices in each face
	for (i=0;i<_faces.size();i++) {
		vertexVector fss;

		for (j=0;j<_faces[i]->_vertices.size();j++) {
			int id = _faces[i]->_vertices[j]->_id;
			bool exist = false;
			for (int k=0;k<fss.size();k++) {
				if ( fss[k]->_id == id ) {
					exist = true;
					break;
				}
			}
			if ( !exist )
				fss.push_back( _faces[i]->_vertices[j] );
		}

		_faces[i]->_vertices = fss;
	}

}

/* Remove flat edges
 */
void Model::mergeFaces()
{
	int i,j;
	edgeVector::iterator iter;
	edgeVector remove_list;
	int counter = 0;

	for (iter = _edges.begin(); iter != _edges.end(); iter++) {

		Edge *e = *iter;
		double alpha = 0.0;
		double beta = 0.0;

		if ( e->_faces.empty() ) {
			e->_flat = true;
			continue;
		}

		if ( e->_faces.size() == 1 ) {
			e->_flat = false;
			continue;
		}

		for ( i=0;i<e->_faces.size();i++) {
			for ( j=i+1;j<e->_faces.size();j++) {

				Face *f1 = e->_faces[i];
				Face *f2 = e->_faces[j];
				
				if ( f1->_vertices.size() < 3 || f2->_vertices.size() < 3 )
					continue;

				Vertex *c = e->_a->next( f1 );
				Vertex *d = e->_a->next( f2 );
				
				if ( c == NULL || d == NULL )
					continue;

				Vec3d n1 = cross(c->getPosition()-e->_a->getPosition(),e->_b->getPosition() - e->_a->getPosition());
				int cc=0;
				while ( len(n1) < EPS ) {
					c = c->next(f1);
					n1 = cross(c->getPosition()-e->_a->getPosition(),e->_b->getPosition() - e->_a->getPosition());
					if ( cc > f1->_vertices.size() )
						break;
					cc++;
				}
				if ( len(n1) < EPS )
					continue;
				n1 = norm(n1);

				cc=0;
				Vec3d n2 = cross(d->getPosition()-e->_a->getPosition(),e->_b->getPosition() - e->_a->getPosition());
				while ( len(n2) < EPS ) {
					d = d->next(f2);
					n2 = cross(d->getPosition()-e->_a->getPosition(),e->_b->getPosition() - e->_a->getPosition());
					if ( cc > f1->_vertices.size() )
						break;
					cc++;
				}
				if ( len(n2) < EPS )
					continue;
				n2 = norm(n2);
								
				double angle = Acos(dot(n1,n2));
				alpha = MAX(alpha, angle);

				beta = MAX(beta,MIN(angle,M_PI-angle));

			}
		}
	
		if ( beta > MODEL_MIN_EDGE_ANGLE ) {
			e->_flat = false;
		} else {
			if ( alpha > MODEL_MIN_EDGE_ANGLE ) {
				e->_flat = true;
				counter++;
			}
			else {
				e->_flat = false;
			}
		}
	}

	LOG(LEVEL_INFO, "merging faces: removed %d edges", counter);
}

/* Remove a face from the model
 */
void Model::removeFace( Face *f )
{
	// remove references to this face in edges
	for (int i=0;i<_edges.size();i++) {
		for (faceVector::iterator iter = _edges[i]->_faces.begin(); iter != _edges[i]->_faces.end(); iter++) {
			if ( (*iter)->_id == f->_id ) {
				_edges[i]->_faces.erase( iter );
				break;
			}
		}
	}

	// remove face from faces list
	for (faceVector::iterator iter = _faces.begin(); iter != _faces.end(); iter++) {
		if ( (*iter)->_id == f->_id ) {
			_faces.erase( iter );
			break;
		}
	}

	delete f;
}

/* remove an edge from the model
 */
void Model::removeEdge(Edge *e)
{
	int i;
	edgeVector::iterator iter;

	// remove from vertices links
	for (i=0;i<_vertices.size();i++) {

		for (iter = _vertices[i]->_edges.begin();iter != _vertices[i]->_edges.end(); iter++) {
			if ( (*iter)->_id == e->_id ) {
				_vertices[i]->_edges.erase ( iter );
				break;
			}
		}
	}
	
	// remove the edge from the edge list
	for (iter = _edges.begin(); iter != _edges.end(); iter++ ) {
		if ( (*iter)->_id == e->_id ) {
			_edges.erase( iter );
			break;
		}
	}

	delete e;
}

/* Remove a vertex from the model
 */
void Model::removeVertex( Vertex *v )
{
	if ( !v->_edges.empty() )
		return;

	// remove the vertex from all faces
	for (int i=0;i<_faces.size();i++) {

		for (vertexVector::iterator iter = _faces[i]->_vertices.begin(); iter != _faces[i]->_vertices.end(); iter++) {

			if ( (*iter)->_id == v->_id ) {
				_faces[i]->_vertices.erase( iter );
				break;
			}
		}
	}

	// remove vertex from vertex list
	for (vertexVector::iterator iter = _vertices.begin(); iter != _vertices.end(); iter++) {
		if ((*iter)->_id == v->_id) {
			_vertices.erase( iter );
			break;
		}
	}

	delete v;
}

/* Remove aligned edges
 */
void Model::mergeEdges()
{
	int i,j,counter=0;

	bool done = false;

	while (!done) {

		done = true;

		// find the first non-flat edge and try to merge it
		for (i=0;i<_edges.size();i++) {
			
			if ( _edges[i]->_flat )
				continue;

			for (j=0;j<_edges[i]->_a->_edges.size();j++) {

				if ( _edges[i]->_a->_edges[j]->_id == _edges[i]->_id )
					continue;

				if ( _edges[i]->_a->_edges[j]->_flat )
					continue;

				Vertex *b = _edges[i]->_a->sibling( _edges[i]->_a->_edges[j] );

				Vec3d n1 = norm(_edges[i]->_b->getPosition() - _edges[i]->_a->getPosition());
				Vec3d n2 = norm(b->getPosition() - _edges[i]->_a->getPosition());
				if ( fabs(M_PI - Acos(dot(n1,n2))) < MODEL_MIN_EDGE_ANGLE ) {
					_edges[i]->_flat = true;
					_edges[i]->_a->_edges[j]->_flat = true;
					Edge *edge = new Edge( _edges.size(), b, _edges[i]->_b );
					edge->_flat = false;
					b->_edges.push_back( edge );
					_edges[i]->_b->_edges.push_back( edge );
					_edges.push_back( edge );
					done = false;
					counter++;
					break;
				} else if ( Acos(dot(n1,n2)) < MODEL_MIN_EDGE_ANGLE ) {
					if ( _edges[i]->length() > _edges[i]->_a->_edges[j]->length() ) {
						_edges[i]->_a->_edges[j]->_flat = true;
					} else {
						 _edges[i]->_flat = true;
					}
					done = false;
					counter++;
					break;
				}
			}

			if ( !done )
				break;

			for (j=0;j<_edges[i]->_b->_edges.size();j++) {

				if ( _edges[i]->_b->_edges[j]->_id == _edges[i]->_id )
					continue;

				if ( _edges[i]->_b->_edges[j]->_flat )
					continue;

				Vertex *b = _edges[i]->_b->sibling( _edges[i]->_b->_edges[j] );

				Vec3d n1 = norm(_edges[i]->_a->getPosition() - _edges[i]->_b->getPosition());
				Vec3d n2 = norm(b->getPosition() - _edges[i]->_b->getPosition());
				if ( fabs(M_PI - Acos(dot(n1,n2))) < MODEL_MIN_EDGE_ANGLE ) {
					_edges[i]->_flat = true;
					_edges[i]->_b->_edges[j]->_flat = true;
					Edge *edge = new Edge( _edges.size(), b, _edges[i]->_a );
					edge->_flat = false;
					b->_edges.push_back( edge );
					_edges[i]->_a->_edges.push_back( edge );
					_edges.push_back( edge );
					done = false;
					counter++;
					break;
				} else if ( Acos(dot(n1,n2)) < MODEL_MIN_EDGE_ANGLE ) {
					if ( _edges[i]->length() > _edges[i]->_b->_edges[j]->length() ) {
						_edges[i]->_b->_edges[j]->_flat = true;
					} else {
						 _edges[i]->_flat = true;
					}
					done = false;
					counter++;
					break;
				}
			}

			if ( !done )
				break;
		}
	}

	LOG(LEVEL_INFO, "merged %d edges", counter);
}

/* Read geometry from an Inventory file
 */
int Model::readInventorFile (const char *filename, const char *dirname)
{
	std::string token_header = "#Inventor V2.0 ascii";
	std::string token_points = "Coordinate3 {";
	std::string token_table = "[";
	std::string token_coma = "[";
	std::string token_face = "IndexedFaceSet {";
	std::string line;

	printf("reading inventor file %s\n",filename);

	FILE *f = fopen(filename,"r");
	
	int type = 0;
	char *ch;
	double x,y,z;
	int id1,id2,id3,id4,id5;
	ch = (char*)malloc(256*sizeof(char));
	ch[1] = '\0';

	if (f == NULL) {
		printf("ERROR: %s does not exist.\n",filename);
		return 1;
	}

	// check header
	fscanf(f,"%s",ch);
	assert (strncmp(ch,"#Inventor",9) == 0);
	fscanf(f,"%s",ch); 
	assert (strncmp(ch,"V2.0",4) == 0);
	fscanf(f,"%s",ch); 
	assert (strncmp(ch,"ascii",5) == 0);

	// read the file
	bool ok = false;
	int offset = _global_offset;
	int counter = 0;

	while (!feof(f)) {

		//scan until "Coordinate3 {" then until "["
		if (!fileScan(f,"Coordinate3",11))
			break;
		printf("Coordinate3\n");
		if (!fileScan(f,"point",5))
			break;
		fileOffset(f,2);

		// scan 3D points
		while (fscanf(f,"%lf%lf%lf",&x,&y,&z) == 3) {

			// apply the homography
			Vec3d p = _homography_rotation.rotate( Vec3d(x,y,z) ) + _homography_translation;

			_vertices.push_back(new Vertex(p[0],p[1],p[2],_vertices.size()));
			counter++;
			fileOffset(f,1);
		}

		if (!fileScan(f,"IndexedFaceSet",14))
			break;
		if (!fileScan(f,"coordIndex ",10))
			break;
		fileOffset(f,2);

		// scan faces
		while (fscanf(f,"%d",&id1) == 1) {
			fileOffset(f,1);
			if (fscanf(f,"%d",&id2) != 1)
				break;
			fileOffset(f,1);
			if ( fscanf(f,"%d",&id3) != 1)
				break;
			fileOffset(f,1);
			if ( fscanf(f,"%d",&id4) != 1 )
				break;
			fileOffset(f,1);
			if (id4 == -1 && id1 != -1 && id2 != -1 && id3 != -1 ) {
				if ( id1+offset >= _vertices.size() ) 
					printf("error: accessing element %d out of %d elements.\n", id1+offset, _vertices.size() );
				if ( id2+offset >= _vertices.size() ) 
					printf("error: accessing element %d out of %d elements.\n", id2+offset, _vertices.size() );
				if ( id3+offset >= _vertices.size() ) 
					printf("error: accessing element %d out of %d elements.\n", id3+offset, _vertices.size() );
				Insert(getVertex(id1+offset),getVertex(id2+offset),getVertex(id3+offset));
				continue;
			}
			if ( fscanf(f,"%d",&id5) != 1 )
				break;
			fileOffset(f,1);
			
			if ( id1 != -1 && id2 != -1 && id3 != -1 && id4 != -1 )
				if ( id1+offset >= _vertices.size() ) 
					printf("error: accessing element %d out of %d elements. (offset = %d)\n", id1+offset, _vertices.size(), offset );
				if ( id2+offset >= _vertices.size() ) 
					printf("error: accessing element %d out of %d elements. (offset = %d)\n", id2+offset, _vertices.size(), offset  );
				if ( id3+offset >= _vertices.size() ) 
					printf("error: accessing element %d out of %d elements. (offset = %d)\n", id3+offset, _vertices.size() , offset );
				if ( id4+offset >= _vertices.size() ) 
					printf("error: accessing element %d out of %d elements. (offset = %d)\n", id4+offset, _vertices.size(), offset  );
				Insert(getVertex(id1+offset),getVertex(id2+offset),getVertex(id3+offset),getVertex(id4+offset));
		}

		offset = _vertices.size();
		break; // we want only the first chunks of vertices
	}

	_global_offset += counter;

	fclose(f);
	
	return 0;
}

/* Read geometry from an Autocad log file
 */
int Model::readAutocadFile(const char *filename, const char *dirname)
{
	std::string line;
	std::string token1 = "from point";
	std::string token2 = "to point";
	std::string token3 = " LINE";
	std::string token4 = "first point";
	std::string token5 = "second point";
	std::string token6 = "third point";
	std::string token7 = "fourth point";
	std::string token8 = "3D FACE";
	std::string token9 = "LWPOLYLINE";
	std::string token10 = "at point";
	std::string token11 = "BLOCK REFERENCE";
	std::string token12 = "END SEQUENCE";
	
	Vec3d a,b,c,d;
	int firstPointInPolyline = 1;
	int type;
	printf("Model reading file %s\n",filename);
	
	std::ifstream inFile (filename);
	
	if (inFile == NULL) {
		printf("ERROR: %s does not exist.\n",filename);
		return 1;
	}

	while (std::getline(inFile,line,'\n'))
	{
		if ((line.find (token3, 0) != std::string::npos)) {
			type = 1; // LINE
		}
		else if (line.find (token8, 0) != std::string::npos) {
			type = 2; // 3D FACE - in autocad, _faces always have 4 _edges.
		}
		else if ((line.find(token9, 0) != std::string::npos)) {
			type = 3; // LWPOLYLINE
			firstPointInPolyline = 1;
		}
		else if (line.find(token11, 0) != std::string::npos) {
			type = 0; // BLOCK REFERENCE
		}
		else if (line.find(token12, 0) != std::string::npos) {
			type = 0; // END SEQUENCE
		}
		
		
		// LINE
		if (type == 1) {
			if (line.find(token1, 0) != std::string::npos) {
				extractXYZ(line, a);
			}
			
			if (line.find(token2,0) != std::string::npos) {
				extractXYZ(line, b);
				Insert(a,b);
			}
		}
		
		// 3D FACE
		if (type == 2) {
			if (line.find(token4, 0) != std::string::npos) {
				extractXYZ(line, a);
			}
			
			if (line.find(token5,0) != std::string::npos) {
				extractXYZ(line, b);	
			}
			
			if (line.find(token6,0) != std::string::npos) {
				extractXYZ(line, c);
			}
			
			if (line.find(token7,0) != std::string::npos) {
				type = 0;
				extractXYZ(line,d);
				Insert(a,b,c,d);
			}
			
		}
		
		
		// POLYLINE
		if (type == 3) {
			if (line.find(token10, 0) != std::string::npos) {
				if (firstPointInPolyline == 1) {
					extractXYZ(line,a);
					firstPointInPolyline = 0;
				} else {
					extractXYZ(line,b);
					
					Insert(a,b);
					a = b;
				}
			}
		}
		
	}
	
	printf("read %d vertices, %d edges and %d faces.\n",_vertices.size(),_edges.size(),_faces.size());
	
	inFile.close();
		
	return 0;
	
	
}

/* Sanity check a model
 */
void Model::updateModel()
{
	// compute the bounding box of the model
	assert(_vertices.size()>2);

	_bounding_box[0] = _vertices[0]->getPosition();
	_bounding_box[1] = _vertices[0]->getPosition();

	vertexVectorIter iter=_vertices.begin();
	for (;iter!=_vertices.end();iter++) {
		_bounding_box[0][0] = MIN(_bounding_box[0][0],(*iter)->getPosition()[0]);
		_bounding_box[0][1] = MIN(_bounding_box[0][1],(*iter)->getPosition()[1]);
		_bounding_box[0][2] = MIN(_bounding_box[0][2],(*iter)->getPosition()[2]);

		_bounding_box[1][0] = MAX(_bounding_box[1][0],(*iter)->getPosition()[0]);
		_bounding_box[1][1] = MAX(_bounding_box[1][1],(*iter)->getPosition()[1]);
		_bounding_box[1][2] = MAX(_bounding_box[1][2],(*iter)->getPosition()[2]);
	}

	LOG(LEVEL_INFO, "model size: %d x %d", (int)(_bounding_box[1][0] - _bounding_box[0][0]), (int)(_bounding_box[1][1] - _bounding_box[0][1]));

	// renumber all elements
	int i;
	for (i=0;i<_vertices.size();i++) 
		_vertices[i]->_id = i;
	for (i=0;i<_edges.size();i++)
		_edges[i]->_id = i;
	int counter = 0;
	for (i=0;i<_faces.size();i++) {
		if ( _faces[i]->_vertices.size() < 3 )
			_faces[i]->_id = -1;
		else {
		_faces[i]->_id = counter;
		counter++;
		}
	}

	// fill in the edge table for quick search
	_maxEdgeId = 0;
	edgeVectorIterator iter_edge;
	for (iter_edge = _edges.begin(); iter_edge != _edges.end(); iter_edge++)
		_maxEdgeId = MAX(_maxEdgeId,(*iter_edge)->_id);

	_edge_table = (Edge**)malloc(_maxEdgeId*sizeof(Edge*));

	for (i=0;i<_maxEdgeId;i++)
		_edge_table[i] = NULL;

	for  (iter_edge = _edges.begin(); iter_edge != _edges.end(); iter_edge++) {
		_edge_table[(*iter_edge)->_id] = *iter_edge;
	}

	printf("read %d vertices, %d edges and %d faces.\n",_vertices.size(),_edges.size(),_faces.size());
	
	computeCentroidAndDiameter();

	// update regular faces
	for (i=0;i<_faces.size();i++) {
		_faces[i]->regular();
	}
}

/* For each edge, retrieve the adjacent faces
 * for each vertex, retrieve the edges
 * this function is very time consuming and should be run in offline mode only
 * for models which do not include connectivity (such as Inventor for example)
 */
void Model::buildConnectivity()
{
	// counter for progress info since this function is long!
	int counter = 0, max = _edges.size() * _faces.size();
	int ratio = 0;
	int c=0;

	LOG(LEVEL_INFO, "building connectivity...");

	// for each edge...
	for (edgeVector::iterator iter = _edges.begin(); iter != _edges.end(); iter++) {
		int id_a = (*iter)->_a->_id;
		int id_b = (*iter)->_b->_id;

		// update adjacency of vertices
		bool exist = false;
		for (int j=0;j<(*iter)->_a->_edges.size();j++) {
			if ( (*iter)->_a->_edges[j]->_id == (*iter)->_id ) {
				exist = true;
				break;
			}
		}
		if ( !exist ) {
			c++;
			(*iter)->_a->_edges.push_back( *iter );
		}

		exist = false;
		for (j=0;j<(*iter)->_b->_edges.size();j++) {
			if ( (*iter)->_b->_edges[j]->_id == (*iter)->_id ) {
				exist = true;
				break;
			}
		}
		if ( !exist ) {
			c++;
			(*iter)->_b->_edges.push_back( *iter );
		}

		// try each face to see whether it is adjacent
		for (faceVector::iterator fiter = _faces.begin(); fiter != _faces.end(); fiter++) {
			Face *face = *fiter;

			// check whether the face is already there
			for (int j=0;j<(*iter)->_faces.size();j++) {
				if ( (*iter)->_faces[j]->_id == face->_id )
					continue;
			}

			bool contain_a =false, contain_b = false;

			for (int k=0;k<face->_vertices.size();k++) {
				if ( face->_vertices[k]->_id == id_a )
					contain_a = true;
				if ( face->_vertices[k]->_id == id_b ) 
					contain_b = true;
			}

			if ( contain_a && contain_b ) {
				(*iter)->_faces.push_back( face );
				c++;
			}

			counter++;

			// print out the counter
			if ( (double)counter * 100.0 / max > ratio ) {
				printf("|");
				ratio++;
				if ( ratio % 10 == 0 ) 
					printf( " %d ", ratio );
				fflush( stdout );
			}
		}

		// check that each face has end points as vertices
		for (fiter = (*iter)->_faces.begin(); fiter != (*iter)->_faces.end(); fiter++) {
			bool exist_a = false;
			bool exist_b = false;
			for (int j=0;j<(*fiter)->_vertices.size();j++) {
				if ( (*fiter)->_vertices[j]->_id == id_a )
					exist_a = true;
				if ( (*fiter)->_vertices[j]->_id == id_b ) 
					exist_b = true;
			}

			if ( !exist_a ) {
				c++;
				(*fiter)->_vertices.push_back( (*iter)->_a );
			}
			if ( !exist_b ) {
				c++;
				(*fiter)->_vertices.push_back( (*iter)->_b );
			}
		}

	}

	LOG(LEVEL_INFO, "built connectivity. %d face repairs.", c);
}

/* Returns true if a 3D point is outside of the bounding box of the model
 */
bool Model::isOutside (Vec3d point)
{
	if (point[0] > _bounding_box[1][0])
		return true;
	if (point[1] > _bounding_box[1][1])
		return true;
	if (point[2] > _bounding_box[1][2])
		return true;

	if (point[0] < _bounding_box[0][0])
		return true;
	if (point[1] < _bounding_box[0][1])
		return true;
	if (point[2] < _bounding_box[0][2])
		return true;

	return false;
}

/* Write model geometry to files
 */
int Model::writeGeometry(const char *dirname)
{
	int i,j;
	
	printf("writing model [%d,%d,%d]\n",_vertices.size(),_edges.size(),_faces.size());
	
	// write vertices
	FILE *f = fopen(toFilename(MODEL_VERTEX_FILE).c_str(),"w");
	if (f == NULL) {
		printf("error writing into %s\n",MODEL_VERTEX_FILE);
		return 1;
	}
	
	
	for (i=0;i<_vertices.size();i++) {
		Vec3d a = _vertices[i]->getPosition();
		int id = _vertices[i]->_id;
		fprintf(f,"%d %lf %lf %lf\n",id,a[0],a[1],a[2]);
	}
	
	fclose(f);
	
	// write faces
	f = fopen(toFilename(MODEL_FACE_FILE).c_str(),"w");
	
	if (f == NULL) {
		printf("error writing into %s\n",MODEL_FACE_FILE);
		return 1;
	}
	
	for (i=0;i<_faces.size();i++) {
		if ( _faces[i]->_id == -1)
			continue;
		int id = _faces[i]->_id;
		fprintf(f,"%d ",id);
		for (int j=0;j<_faces[i]->_vertices.size();j++) {
			fprintf(f,"%d ",_faces[i]->_vertices[j]->_id);
		}
		fprintf(f,"$\n");
	}
	
	fclose(f);
	
	// write centroid and diameter
	f = fopen(toFilename(MODEL_INFO_FILE).c_str(),"w");
	
	if (f == NULL) {
		printf("error writing into %s\n",MODEL_INFO_FILE);
		return 1;
	}
	
	fprintf(f,"%f %f %f\n%f\n",_centroid[0],_centroid[1],_centroid[2],_diameter);
	fclose(f);
	
	return 0;
}

/* Read geometry from files
 */
int Model::readGeometry(const char *dirname)
{
	printf("Reading geometry files...\n");

	// read vertices
	FILE *f = fopen(toFilename(MODEL_VERTEX_FILE).c_str(),"r");
	
	if (f == NULL) {
		printf("error reading file %s\n",MODEL_VERTEX_FILE);
		return 1;
	}
	
	int id;
	double a,b,c;
	
	while (fscanf(f,"%d%lf%lf%lf", &id, &a, &b, &c) == 4) {
		Vertex *v = new Vertex (a,b,c,id);
		_vertices.push_back(v);
	}
	
	fclose(f);
	LOG(LEVEL_INFO, "read %d vertices", _vertices.size());

	// read faces
	f = fopen(toFilename(MODEL_FACE_FILE).c_str(),"r");
	
	if (f == NULL) {
		printf("error reading file %s\n",MODEL_FACE_FILE);
		return 1;
	}
	
	while (!feof(f)) {

		if ( fscanf(f,"%d", &id) != 1 )
			break;

		Face *face = new Face(id);

		while (fscanf(f,"%d",&id) == 1 ) {
			face->_vertices.push_back( getVertex(id) );
		}

		face->_norm = face->normal();

		char c;
		fscanf(f,"%c",&c);
		
		if ( face->_vertices.size() > 2 )
			_faces.push_back(face);
	}
	
	fclose(f);

	LOG(LEVEL_INFO, "read %d faces", _faces.size());

	// populate edges
	faceVector::iterator iter_face;
	for (iter_face = _faces.begin();iter_face!=_faces.end();iter_face++) {
		int i;
		for (i=0;i<(*iter_face)->_vertices.size();i++) {
			Vertex *a = (*iter_face)->_vertices[i];
			Vertex *b;
			if ( i == (*iter_face)->_vertices.size()-1 ) {
				b = (*iter_face)->_vertices[0];
			} else {
				b = (*iter_face)->_vertices[i+1];
			}

			Insert( a, b, *iter_face );
		}
	}

	LOG(LEVEL_INFO, "read %d edges", _edges.size());

	// read info -- and create it if does not exist
	f = fopen(toFilename(MODEL_INFO_FILE).c_str(),"r");
	
	if (f == NULL) {
		LOG(LEVEL_INFO,"error reading file %s. creating it...\n",MODEL_INFO_FILE);
		computeCentroidAndDiameter();
		f = fopen(toFilename(MODEL_INFO_FILE).c_str(),"w");
		
		if (f == NULL) {
			printf("error writing into %s\n",MODEL_INFO_FILE);
			return 1;
		}
		
		fprintf(f,"%f %f %f\n%f\n",_centroid[0],_centroid[1],_centroid[2],_diameter);
		fclose(f);
		return 1;
	}
	
	fclose(f);

	double x,y,z,diam;
	
	fscanf(f,"%lf%lf%lf%lf",&x,&y,&z,&diam);
	_centroid = Vec3d(x,y,z);
	_diameter = diam;
	
	printf("read %d vertices, %d edges, %d faces.\n",_vertices.size(),_edges.size(),_faces.size());

	return 0;
}

/* Reset a model
 */
void Model::reset()
{
	for (int i=0;i<_edges.size();i++)
		_edges[i]->_visible = false;

	for (i=0;i<_vertices.size();i++)
		_vertices[i]->_visible = false;
}

/* Clear a model
 */
void Model::clear()
{
	_edges.clear();
	_vertices.clear();
	_faces.clear();
}

/* Convert name to full path name
 */
std::string Model::toFilename (const char *name)
{
	return (_dirname + "/" + std::string(name));
}

//////////////////////////////////////////////////////////////
//	class Clique
//	
//  a clique of 3 3D lines (used for localization)
/*
Clique::Clique()
{
	a = b = c = NULL;
	_valid = false;
	_ok = false;
}

// create a clique and test flag OK
// a clique is OK iff the lines are skew pair-wise and if each image edge belongs to a different camera
// a clique is valid iff it can be used successfully for localization (POSE_TEST = OK && RANSAC_TEST = OK)
Clique::Clique( Edge *e1, Edge *e2, Edge *e3, EdgePlane ep1, EdgePlane ep2, EdgePlane ep3, Vec3d &position, double angular_threshold )
{
	//LOG( LEVEL_DEBUG, "new clique: %d %d %d", ep1._cameraId, ep2._cameraId, ep3._cameraId );

	a = e1; b = e2; c = e3;

	ea = ep1; eb = ep2; ec = ep3;

	assert (a != NULL);
	assert (b != NULL);
	assert (c != NULL);

	// clique is invalid a priori
	_valid = false;

	_ok = checkOK( angular_threshold );
	
}

Clique::Clique( corres ca, corres cb, corres cc, Vec3d &position, double angular_threshold )
{
	//LOG( LEVEL_DEBUG, "new clique: %d %d %d", ep1._cameraId, ep2._cameraId, ep3._cameraId );

	a = ca.line; b = cb.line; c = cc.line;

	ea = ca.head(); eb = cb.head(); ec = cc.head();

	assert (a != NULL);
	assert (b != NULL);
	assert (c != NULL);

	// clique is invalid a priori
	_valid = false;

	_ok = checkOK( angular_threshold );
	
}

bool Clique::checkOK( double angular_threshold )
{
	// if not skew, stop here
	if (!a->testSkew(b,c,angular_threshold))
		return false;

	// check distribution of 3D lines in space
	// simple test: edgeplanes on different cameras!

	if (ea._cameraId == eb._cameraId)
		return false;
	if (ea._cameraId == ec._cameraId)
		return false;
	if (ec._cameraId == eb._cameraId)
		return false;

	return true;
}

void Clique::draw( Viewer &viewer, ExtrinsicParameters pose, const float color[3] )
{
	drawEdges( viewer, pose, color );
	drawLines( viewer, color );
}

void Clique::drawEdges( Viewer &viewer, ExtrinsicParameters pose, const float color[3] )
{
		// draw the edge plane
	EdgePlane eea,eeb,eec;
	eea = ea;
	eeb = eb;
	eec = ec;
	eea.fromCameraFrameToWorldFrame( pose );
	eeb.fromCameraFrameToWorldFrame( pose );
	eec.fromCameraFrameToWorldFrame( pose );
	double radius = 50.0;
	viewer.drawEdgePlaneDetails(eea,Vec3d(0,0,0),Quaternion(),radius,2,6); // color by camera ID
	viewer.drawEdgePlaneDetails(eeb,Vec3d(0,0,0),Quaternion(),radius,2,6); // color by camera ID
	viewer.drawEdgePlaneDetails(eec,Vec3d(0,0,0),Quaternion(),radius,2,6); // color by camera ID
}

void Clique::drawLines( Viewer &viewer, const float color[3] )
{
	viewer.drawLine(a->getA(),a->getB(),color,6); // color by camera ID
	viewer.drawLine(b->getA(),b->getB(),color,6); // color by camera ID
	viewer.drawLine(c->getA(),c->getB(),color,6); // color by camera ID
}

// return true if the clique shares two lines or more with the input clique
bool Clique::shareTwoLines( Clique &clique )
{
	int counter = 0;

	if ( (a == clique.a) || (a == clique.b) || (a == clique.c) )
		counter++;

	if ( (b == clique.a) || (b == clique.b) || (b == clique.c) )
		counter++;
	if (counter == 2)
		return true;

	if ( (c == clique.a) || (c == clique.b) || (c == clique.c) )
		counter++;
	if (counter >= 2)
		return true;

	return false;
}

// a clique is accepted iff it shares less than 2 lines with each clique of the set
// this is a pretty tough test so don't run it on the BACKUP pool but rather on the VALIDATION pool
bool Clique::accepted( cliqueVector &cliques )
{
	for ( cliqueVector::iterator iter = cliques.begin(); iter != cliques.end(); iter++ ) {
		if ( shareTwoLines( *iter ) )
			return false;
	}

	return true;
}

// compute the reprojection error for a clique
double Clique::reprojectionError( ExtrinsicParameters &pose )
{
	double error = 0.0;
	
	EdgePlane epa = ea, epb = eb, epc = ec;
	epa.rotate( pose.getRotation() );
	epb.rotate( pose.getRotation() );
	epc.rotate( pose.getRotation() );
	Vec3d t = pose.getTranslation();

	error += fabs( 1.0 - fabs( dot(epa._normal, norm( cross( a->getA() -  t, a->getB() - t ) ) ) ) );
	error += fabs( 1.0 - fabs( dot(epb._normal, norm( cross( b->getA() -  t, b->getB() - t ) ) ) ) );
	error += fabs( 1.0 - fabs( dot(epc._normal, norm( cross( c->getA() -  t, c->getB() - t ) ) ) ) );

	return error;
}
*/
///////////////////////////////////////////////////////////////////////////////////
//
// icosahedron sphere subdivision
//

/* Generate a unit icosahedron
 */
void Model::make_icosahedron ( std::vector< Vec3d > &points )
{
	// poles
	points.push_back( Vec3d(0,0,1) );
	points.push_back( Vec3d(0,0,-1) );

	// the other vertices are on two pentagons
	// of latitude theta = 26.565 degrees
	double sint = sin( toRadians( 26.565 ) );
	double cost = cos( toRadians( 26.565 ) );

	double psi = 0.0;

	for (int i=0;i<5;i++) {

		points.push_back( Vec3d(cost*cos(psi), cost*sin(psi), sint) );
		psi += 2 * M_PI / 5.0;
	}

	psi = 2*M_PI/10.0;

	for (i=0;i<5;i++) {

		points.push_back( Vec3d(cost*cos(psi), cost*sin(psi), -sint) );
		psi += 2 * M_PI / 5.0;
	}

}

/* generate a triangle subdivision of initial points with triangle of size smaller than <resolution>
 */
void Model::tessellate ( double resolution, std::vector< Vec3d > &points )
{
	int i,j,k;

	for (i=0;i<points.size();i++) {
		printf("%f %f %f\n", points[i][0], points[i][1], points[i][2] );
	}

	// compute the min distance between two points
	double lgth = 2.0;
	
	for (i=0;i<points.size();i++) {
		for (j=i+1;j<points.size();j++) {
			lgth = MIN(lgth,len(points[i]-points[j]));
		}
	}
	
	// create the new points
	while ( lgth > resolution ) {
		
		std::vector< Vec3d > new_points;
		
		for (i=0;i<points.size();i++) {
			for (j=i+1;j<points.size();j++) {
				if ( len(points[i] - points[j]) < 1.05 * lgth )
					new_points.push_back( (points[i] + points[j])/2 );
			}
		}
		
		lgth /= 2.0;
		
		for (i=0;i<new_points.size();i++) 
			points.push_back( new_points[i] );
	}
	
	printf("%d points\n", points.size() );
		
	// compute vertices
	for (i=0;i<points.size();i++) {
		Vertex *v = new Vertex(points[i][0],points[i][1],points[i][2],_vertices.size());
		_vertices.push_back( v );
	}
	
	// compute edges
	for (i=0;i<_vertices.size();i++) {
		for ( j=i+1;j<_vertices.size();j++) {
			if ( len(_vertices[i]->getPosition() - _vertices[j]->getPosition()) < 1.005 * lgth ) {
				Edge *edge = new Edge( _edges.size(), _vertices[i], _vertices[j] );
				_edges.push_back( edge );

			}
		}
	}

	printf("%d edges\n", _edges.size() );
	
	// compute faces
	for (i=0;i<_vertices.size();i++) {
		
		Vertex *a = _vertices[i];
		
		for (j=0;j<a->_edges.size();j++) {
			
			Edge *edge_a = a->_edges[j];
			
			assert( edge_a != NULL );

			Vertex *b = a->sibling( edge_a );
			
			if ( b->_id <= a->_id )
				continue;

			for ( k = 0; k < b->_edges.size(); k++) {
				
				Edge *edge_b = b->_edges[k];
				
				assert( edge_b != NULL );
				
				Vertex *c = b->sibling( edge_b );
				
				if ( c->_id <= b->_id )
					continue;
				
				for (int m=0;m<a->_edges.size();m++) {
					
					Edge *edge_c = a->_edges[m];
					
					assert( edge_c != NULL );
					
					Vertex *d = a->sibling( edge_c );
					
					if ( d->_id == c->_id ) {
						
						Face *face = new Face( _faces.size(), a, b, c );
						face->_norm = norm( ( a->getPosition() + b->getPosition() + c->getPosition() ) / 3.0 );

						_faces.push_back( face );
						
						edge_a->_faces.push_back( face );
						edge_b->_faces.push_back( face );
						edge_c->_faces.push_back( face );
												
						break;
						
					}
				}
				
			}
		}
	}

	printf("%d faces.\n", _faces.size() );

	// normalize the tessellation
	for (i=0;i<_vertices.size();i++) 
		_vertices[i]->setPosition( norm( _vertices[i]->getPosition() ) );

	// normal vectors must point toward the outside of the sphere
	Vertex *temp;
	for (i=0;i<_faces.size();i++) {
		if ( dot( _faces[i]->_norm, _faces[i]->_vertices[0]->getPosition() ) < 0 ) {
			_faces[i]->_norm = - _faces[i]->_norm;
			temp = _faces[i]->_vertices[1];
			_faces[i]->_vertices[1] = _faces[i]->_vertices[2];
			_faces[i]->_vertices[2] = temp;
		}
	}
}

/* Compute the closest point to input point in the tessellation
 * find the solution by initializing the answer anywhere on the sphere
 * and recursively looking for the closest point in the neighborhood
 */
Vertex* Model::closest_vertex( Vec3d p )
{
	assert( !_vertices.empty() );

	p = norm(p);

	// this is the solution point
	Vertex *r = _vertices[0];

	Vertex *b = r;

	double lgth = dot( p,r->getPosition() ); // we want the dot to be close to 1

	bool done = false;

	while ( !done ) {

		for (int j=0;j<r->_edges.size();j++) {

			Vertex *n = r->sibling( r->_edges[j] );

			double l = dot(n->getPosition(), p);
			if ( l > lgth ) {

				b = n;

				lgth = l;
			}
		}

		done = (r->_id == b->_id); // done if didn't find better

		r = b; // update the solution
	}

	return r;
}

/* Find the closest cells to a given point
 * first find the closest vertex and then list the adjacent faces for each adjacent edge
 */
void Model::closest_cells( Vec3d p, intVector &cells )
{
	cells.clear();

	// find the closest vertex
	Vertex *c = closest_vertex( p );

	// for each adjacent edge...
	for (int i=0;i<c->_edges.size();i++ ) {

		Edge *edge = c->_edges[i];

		for (int j=0;j<edge->_faces.size();j++) { // list adjacent faces

			int cell_id = edge->_faces[j]->_id;

			bool exist = false;

			for (int k=0;k<cells.size();k++) { // did we add it already?

				if ( cell_id == cells[k] ) {
					
					exist = true;
					break;
				}
			}

			// if not, add it
			if ( !exist )
				cells.push_back( cell_id );
			
		}
	}
}

/* Pick a random position in the model at a height between the bounds
 */
Vec3d Model::random_position( double min_height, double max_height )
{
	double x = (ran1(&idum) - 0.5 ) * _diameter;
	double y = (ran1(&idum) - 0.5 ) * _diameter;
	double h = min_height + ran1(&idum) * (max_height - min_height - 1.0);

	return Vec3d(_centroid[0], _centroid[1], 0.0) + Vec3d(x,y,h);
}

/* Find the closest cell to a given point
 * first find the closest cells and then pick the best one
 */
int Model::closest_cell( Vec3d p )
{
	// find the closest cells
	intVector cells;
	closest_cells( p, cells );

	// score each cell and keep the best one
	double d = 1E10;
	int best_cell = -1;

	for (int i=0;i<cells.size();i++) {

		Face *f = _faces[cells[i]];

		double s = 0.0;

		for (int j=0;j<f->_vertices.size();j++) {

			s += len( p - f->_vertices[j]->getPosition() );
		}

		if ( s < d ) {

			d = s;
			best_cell = cells[i];
		}
	}

	assert( best_cell != -1 );

	return best_cell;

}

/* Draw model faces with textures if flag is ON
 */
void Model::drawFaces ( bool texture )
{
	int n = _faces.size();

	int i = 0;
	
	if ( texture  ) {

		glDisable(GL_LIGHTING);

		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

		drawColorHits();

	} else {

		glEnable(GL_LIGHTING);

		/* no texture, simple drawing of faces */
		for (faceVector::iterator iter = _faces.begin(); iter != _faces.end(); iter++) {

			glLineWidth(1);
			// draw the face

			glColor (WHITE );

			Vec3d center = Vec3d(0,0,0);

			//if ( dot( (*iter)->_norm, norm(Vec3d(-1,-1,-1)) ) < -0.8 )
			//	continue;

			glNormal3f((*iter)->_norm[0],(*iter)->_norm[1],(*iter)->_norm[2]);
			glBegin(GL_POLYGON);
			for (i=0;i<(*iter)->_vertices.size();i++) {
				glVertex3d((*iter)->_vertices[i]->getPosition()[0],(*iter)->_vertices[i]->getPosition()[1],(*iter)->_vertices[i]->getPosition()[2]);
				center += Vec3d((*iter)->_vertices[i]->getPosition()[0],(*iter)->_vertices[i]->getPosition()[1],(*iter)->_vertices[i]->getPosition()[2]) / (*iter)->_vertices.size();
			}
			glEnd();

			glLineWidth(1);
		}
	}
}

/* Write the color hits to files
 */
void Model::writeColorHits()
{
	LOG(LEVEL_INFO, "writing color hits for level %d", _lod );

	char filename[256];
	sprintf(filename,"%s/color/%d/hits.txt", _dirname.c_str(), _lod);

	FILE *f = fopen (filename, "w" );
	if ( f == NULL ) {
		LOG(LEVEL_INFO, "failed to open file %s.", filename);
		return;
	}

	for (int i=0;i<_color_hits.size();i++) {

			color_hit c = _color_hits[i];

			fprintf(f, "%d %d %d %d %d %d\n", c.face_id, c.i, c.r, c.g, c.b, c.hits);
	}

	fclose( f );
}

/* Read the color hits from file
 */
void Model::readColorHits()
{
	_color_hits.clear();
	_color_offset.clear();

	_color_resolution = 50.0;

	int i,k;

	for (k=0;k<_lod;k++) 
		_color_resolution /= 2.0;

	_color_resolution = MAX(_color_resolution, 1.0);

	// subdividing faces
	LOG(LEVEL_INFO, "subdividing faces...");
	for (i=0;i<_faces.size();i++)
		_faces[i]->subdivide( _color_resolution );
	LOG(LEVEL_INFO, "done.");

	char filename[256];
	sprintf(filename,"%s/color/%d/hits.txt", _dirname.c_str(), _lod);
	
	FILE *f = fopen (filename, "r" );
	if ( f == NULL ) {
		LOG(LEVEL_INFO, "failed to open file %s. Creating empty color patches", filename);
		
		for (k=0;k<_faces.size();k++) {
			_color_offset.push_back( _color_hits.size() );
			
			int steps = _faces[k]->subdivisionSize( _color_resolution );
			
			for (int li=0;li<steps;li++) {
				color_hit hit;
				hit.face_id = _faces[k]->_id;
				hit.i = li;
				hit.r = 0;//(li%3 == 0) ? 255 : 0; // for fun debugging
				hit.g = 0;//(li%3 == 1) ? 255 : 0; // for fun debugging
				hit.b = 0;//(li%3 == 2) ? 255 : 0; // for fun debugging
				hit.hits = 0; // 1 for fun debugging
				_color_hits.push_back( hit );
				
			}
		}
		
		_color_offset.push_back( _color_hits.size() );
		return;
	}
	
	_color_hits.clear();

	int prev_face_id = -1;

	while ( !feof(f) ) {

		int face_id, r, g, b, hits;

		if (fscanf( f, "%d%d%d%d%d%d", &face_id, &i, &r, &g, &b, &hits ) == 6 ) {

			color_hit c;
			c.face_id = face_id;
			c.i = i;
			c.r = r;
			c.g = g;
			c.b = b;
			c.hits = hits;

			if ( prev_face_id != face_id ) {
				while ( prev_face_id != face_id) {
					_color_offset.push_back( _color_hits.size() );
					prev_face_id++;
				}
			}

			_color_hits.push_back( c );

		}
	}

	_color_offset.push_back( _color_hits.size() );

	fclose ( f );

}

/* Insert a color hit
 */
void Model::insertColorHit ( color_hit hit )
{
	int step = 1;
	for (int k=0;k<_lod;k++)
		step *= 2;

	int start = _color_offset[hit.face_id];
	int end = _color_offset[hit.face_id+1];

	for (int i= start; i < end; i++) {

		color_hit c = _color_hits[i];

		if ( hit.face_id == c.face_id && hit.i == c.i ) {

			if ( c.hits == 0 ) {

				c.r = hit.r;
				c.g = hit.g;
				c.b = hit.b;
				c.hits = 1;

				_color_hits[i] = c;
				return;
			}

			c.hits++;

			c.r = int ( 1.0 / c.hits * hit.r + (double)(c.hits - 1) / c.hits * c.r );
			c.g = int ( 1.0 / c.hits * hit.g + (double)(c.hits - 1) / c.hits * c.g );
			c.b = int ( 1.0 / c.hits * hit.b + (double)(c.hits - 1) / c.hits * c.b );
			_color_hits[i] = c;
			return;
		}
	}
}

/* Clear the color hits
 */
void Model::clearColorHits()
{
	_color_hits.clear();
}

/* Draw the color hits
 */
void Model::drawColorHits()
{
	for (int k=0;k<_color_hits.size();k++) {

		color_hit hit = _color_hits[k];

		Face *f = _faces[hit.face_id];

		Vec3d a,b,c,d;
		if ( f->getSubdivisionVertices( hit.i, _color_resolution, a, b, c, d ) ) {
			
			if ( hit.hits == 0 ) {
				glColor(WHITE);
			} else {
				glColor3f( (double)hit.r/255.0, (double)hit.g/255.0, (double)hit.b/255.0);
			}
			
			glNormal3f(f->_norm[0], f->_norm[1], f->_norm[2]);

			glBegin(GL_QUADS);
			glVertex(a);
			glVertex(b);
			glVertex(c);
			glVertex(d);
			glEnd();
		}
	}
}

/*void Model::processColorHits()
{

	int i,j;

	printf("starting %d x %d ...\n", TEXTURE_WIDTH, TEXTURE_HEIGHT);

	int** counters = new int*[TEXTURE_WIDTH];
	for (i=0;i<TEXTURE_WIDTH;i++) {
		counters[i] = new int[TEXTURE_HEIGHT];
	}


	unsigned char *pixels = new unsigned char[TEXTURE_WIDTH * TEXTURE_HEIGHT * 3];

	int hit_id = 0;

	// create a new texture image for each face
	for (faceVector::iterator fiter = _faces.begin(); fiter != _faces.end(); fiter++ ) {

		hit_id = (*fiter)->_id;

		// reset the counters and pixels
		for (i=0;i<TEXTURE_WIDTH;i++) {
			for (j=0;j<TEXTURE_HEIGHT;j++) {
				counters[i][j] = 0;
			}
		}

		memset(pixels,(unsigned char)0,TEXTURE_WIDTH * TEXTURE_HEIGHT * 3);

		// process each hit
		if ( hit_id >= _color_hits.size())
			continue;

		for (std::vector<color_hit>::iterator iter = _color_hits[hit_id].begin();iter != _color_hits[hit_id].end(); iter++) {

			double r = (double)iter->r;
			double g = (double)iter->g;
			double b = (double)iter->b;

			int x = int(iter->x * TEXTURE_WIDTH);
			int y = int(iter->y * TEXTURE_HEIGHT);

			int wx = iter->size;

			for (i=MAX(0,MIN(TEXTURE_WIDTH-1,x-wx/2));i<MAX(0,MIN(TEXTURE_WIDTH-1,x+wx/2));i++) {
				for (j=MAX(0,MIN(TEXTURE_HEIGHT-1,y-wx/2));j<MAX(0,MIN(TEXTURE_HEIGHT-1,y+wx/2));j++) {

					counters[i][j]++;

					pixels[3*(j*TEXTURE_WIDTH+i)+0] = (unsigned char)((double)pixels[3*(j*TEXTURE_WIDTH+i)+0] + r);
					pixels[3*(j*TEXTURE_WIDTH+i)+1] = (unsigned char)((double)pixels[3*(j*TEXTURE_WIDTH+i)+1] + g);
					pixels[3*(j*TEXTURE_WIDTH+i)+2] = (unsigned char)((double)pixels[3*(j*TEXTURE_WIDTH+i)+2] + b);
				}
			}
		}

		// take the average at each pixel
		for (i=0;i<TEXTURE_WIDTH;i++) {
			for (j=0;j<TEXTURE_HEIGHT;j++) {

				int c = counters[i][j];

				if ( c != 0 ) {
					pixels[3*(j*TEXTURE_WIDTH+i)+0] = (unsigned char)((double)pixels[3*(j*TEXTURE_WIDTH+i)+0] / counters[i][j]);
					pixels[3*(j*TEXTURE_WIDTH+i)+1] = (unsigned char)((double)pixels[3*(j*TEXTURE_WIDTH+i)+1] / counters[i][j]);
					pixels[3*(j*TEXTURE_WIDTH+i)+2] = (unsigned char)((double)pixels[3*(j*TEXTURE_WIDTH+i)+2] / counters[i][j]);
				}
			}
		}

		memcpy(_textureImage->imageData, pixels, TEXTURE_WIDTH * TEXTURE_HEIGHT * 3);

		// save in texture image file 
		char filename[256];
		sprintf(filename,"%s/color/%d.tif", _dirname.c_str(), (*fiter)->_id);
		cvSaveImage( filename, _textureImage ); 
	}
	
	for (i=0;i<TEXTURE_WIDTH;i++)
		delete [] counters[i];
	delete [] counters;

	delete [] pixels;

}
*/

/* read the texture for each face
 */
void Model::readTextures()
{
	if ( _read_texture )
		return;

	/* read the color hits */

	//glEnable(GL_TEXTURE_2D);

	int i=0;

	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL /*GL_MODULATE*/);

	int n = _faces.size();

	GLint texSize; 
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &texSize); 

	LOG(LEVEL_INFO, "generating %d textures (%d maximum textures)", n, texSize );

	// reserve texture IDs
	//glutSetWindow(0);

	_idtextures = new GLuint[n];

	glGenTextures( n, _idtextures );

	// create texture image
	_textureImage = cvCreateImage(cvSize(int(TEXTURE_WIDTH), int(TEXTURE_HEIGHT)),  IPL_DEPTH_8U, 3);

	// link each texture to an image, if it exists
	for (faceVector::iterator iter = _faces.begin(); iter != _faces.end(); iter++ ) {
		readTextures( i, *iter );
		i++;
	}

	_read_texture = true;

}

void Model::readTextures( int id, Face *f )
{
	return;

	if ( f == NULL )
		return;

	/* generate the filename */
	char id_string[10];
	sprintf(id_string, "%d", f->_id );

	std::string filename = toFilename("color/") + std::string(id_string) + std::string(".tif");

	if ( !fileExist(filename.c_str()) )
		filename = toFilename("color/") + std::string("blank") + std::string(".tif");

	/* load the texture image */
	_textureImage = cvLoadImage( filename.c_str() );

	if ( _textureImage == NULL ) {
		LOG(LEVEL_ERROR,"error loading image file %s",filename.c_str());
		return;
	}

	/* check the image size */
	if ( _textureImage->width != TEXTURE_WIDTH || _textureImage->height != TEXTURE_HEIGHT ) {
		LOG(LEVEL_ERROR,"texture image size is wrong: %d x %d instead of %d x %d (%s)",\
		_textureImage->width, _textureImage->height, TEXTURE_WIDTH, TEXTURE_HEIGHT, filename.c_str());
		return;
	}

	/* apply the image to the texture */
	glBindTexture(GL_TEXTURE_2D,id);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,TEXTURE_WIDTH,TEXTURE_HEIGHT,0,
		GL_RGB,GL_UNSIGNED_BYTE,_textureImage->imageData);
}
