#include "signatureviewer.h"

/* constructor
 */
LUT::LUT()
{
	 _valid = false; 
	 _grid_computed = false; 
	 _draw_grid = false; 
	 _draw_pixels = false; 
	 _pixels = NULL;
	 _model = new Model();
	 _node = NULL;
}

/* Reset LUT files
 */
void LUT::resetFiles(bool new_geometry)
{
	if (!existFile(_model->toFilename(MODEL_VISI_FILE).c_str())) {
		fileTouch(_model->toFilename(MODEL_VISI_FILE).c_str());
	//	fileTouch(_model->toFilename(MODEL_VISI_LIST_FILE).c_str());
	}

	if (new_geometry || isFileOlder(_model->toFilename(MODEL_CONFIG_FILE).c_str(),_model->toFilename(MODEL_VISI_FILE).c_str())) {
		fileTouch(_model->toFilename(MODEL_VISI_FILE).c_str());
	//	fileTouch(_model->toFilename(MODEL_VISI_LIST_FILE).c_str());
	}
}

/* return true if the position has changed
 * false otherwise
 */
bool LUT::lookup ( Vec3d position, double min_subtended_angle, double min_dihedral_angle, int max_lines, int depth = 0)
{
	if ( nodes.empty() )
		return false;

	// find the closest node
	Node *node;
	
	if ( _node == NULL ) {
		node = find_closest_node( position );
	}
	else {
		node = find_closest_node( position );
	}

	if ( ( node == _node ) && ( depth == _depth ) ) { // if same node and same depth, don't do anything except filtering
		if ( min_subtended_angle > 0 || min_dihedral_angle > 0 || max_lines > 0 )
			cleanupVisibleLines( node->_pos, _lines, min_subtended_angle, min_dihedral_angle, max_lines );
		return false;
	}

	return lookup( node, min_subtended_angle, min_dihedral_angle, max_lines, depth );
}

/* lookup visibility set in LUT table
 */
bool LUT::lookup( Node *node, double min_subtended_angle, double min_dihedral_angle, int max_lines, int depth = 0 ) 
{
	if ( node == NULL )
	return false;

	_node = node;
	_depth = depth;

	// set lines to not visible
	for (int i=0;i<_lines.size();i++)
		_lines[i]->_visible = false;

	// clear the lines
	_lines.clear();
	_faces.clear();

	// find the neighbors (avoiding duplicates)
	nodeVector neighbors;
	findNodeNeighbors( node, depth, neighbors, true );

	// read the visible lines
	for (  nodeVectorIterator iter = neighbors.begin(); iter != neighbors.end(); iter++ ) {
		edgeVector lines;
		faceVector faces;
		(*iter)->ReadVisibilitySet( _node_histogram, lines, faces, _model );

		InsertLines( lines, _lines ); // insert lines
		InsertFaces( faces, _faces ); // insert faces
	}


	LOG(LEVEL_INFO, "before filter: %d lines", _lines.size());

	if ( min_subtended_angle > 0 || min_dihedral_angle > 0 || max_lines > 0 )
		cleanupVisibleLines( node->_pos, _lines, min_subtended_angle, min_dihedral_angle, max_lines);

	// set lines to visible
	for (i=0;i<_lines.size();i++)
		_lines[i]->_visible = true;

	// find vertices having at least two visible edges
	_vertices.clear();
	for (edgeVector::iterator iter_edge = _lines.begin(); iter_edge != _lines.end(); iter_edge++) {
		vertexVector::iterator iter2;

		if ( (*iter_edge)->_a->n_visible_edges() > 1 ) {
			bool exist = false;
			for (iter2 = _vertices.begin(); iter2 != _vertices.end(); iter2++) {
				if ( (*iter2)->_id == (*iter_edge)->_a->_id ) {
					exist = true;
					break;
				}
			}

			if ( !exist )
				_vertices.push_back( (*iter_edge)->_a);
		}

		if ( (*iter_edge)->_b->n_visible_edges() > 1 ) {
			bool exist = false;
			for (iter2 = _vertices.begin(); iter2 != _vertices.end(); iter2++) {
				if ( (*iter2)->_id == (*iter_edge)->_b->_id ) {
					exist = true;
					break;
				}
			}

			if ( !exist )
				_vertices.push_back( (*iter_edge)->_b);
		}
	}


	return true;
}

/* return true if the pose if valid, ie if the closest node to the input pose
 * is actually the current node
 * this method is used during initialization
 */
bool LUT::valid_pose( ExtrinsicParameters &pose )
{
	// find the closest nodes
	Node *closest_node = find_closest_node( pose.getTranslation() );

	// does that node belong to the neighborhood of the current node?
	nodeVector neighbors;
	findNodeNeighbors( _node, 1, neighbors, false );

	for (int i=0;i<neighbors.size();i++) {

		if ( neighbors[i]->_id == closest_node->_id )
			return true;
	}

	return false;
}

/* Finds the closest node to a 3D position
 */
Node* LUT::find_closest_node( Vec3d position ) 
{
	nodeVector::iterator iter = nodes.begin();
	if ( *iter == NULL )
		return NULL;
	Node *node = *iter;

	double d = len( (*iter)->_pos - position );

	for ( ; iter != nodes.end(); iter++ ) {

		if ( len( (*iter)->_pos - position ) < d ) {

			d = len( (*iter)->_pos - position );
			node = *iter;
		}
	}

	return node;
}

/* Compute the vanishing points corresponding to the set of visible lines
 */
void LUT::computeVanishingPoints()
{
	if ( _node == NULL )
		return;

	int i,j;

	// clear the current vps
	_vps.clear();

	// compute the new vps
	for (i=0;i<_lines.size();i++) {
		Edge *line1 = _lines[i];

		for (int j=i+1;j<_lines.size();j++) {
			Edge *line2 = _lines[j];

			if ( fabs(1.0 - fabs(dot(line1->dir,line2->dir))) > EPS )
				continue;

			Vec3d n1 = norm( cross( line1->getA() - _node->_pos, line1->getB() - _node->_pos ) );
			Vec3d n2 = norm( cross( line2->getA() - _node->_pos, line2->getB() - _node->_pos ) );
			
			Vec3d a = cross( n1, n2 );

			if ( len(a) > EPS ) {
				vp v1;			// add two symmetric vanishing points
				v1.first = norm(a);
				v1.second = -1;
				_vps.push_back ( v1 );

				vp v2;
				v2.first = -norm(a);
				v2.second = -1;
				_vps.push_back ( v2 );
			}
		}
	}

	// remove duplicates
	vp_vector vpc;
	for (i=0;i<_vps.size();i++) {
		bool found = false;
		for (j=0;j<vpc.size();j++) {
			if ( fabs(1.0 - dot(vpc[j].first, _vps[i].first)) < EPS ) {
				found = true;
				break;
			}
		}

		if ( !found ) 
			vpc.push_back( _vps[i] );
	}

	_vps = vpc;

	LOG(LEVEL_INFO,"LUT: %d vanishing points", _vps.size());

}

/* compute the min and max dihedral angle of two edges given a camera pose estimate encoded in the current node
 * and assuming that the camera lies within a cylinder of radius GRID_STEP / 2 and of height between heights h1 and h2
 */
void LUT::min_max_dihedral_angle( Edge *a, Edge *b, double h1, double h2, double &min_dihedral_angle, double &max_dihedral_angle, double &max_polar_angle )
{
	min_dihedral_angle = M_PI;
	max_dihedral_angle = 0.0;

	// search in the book first
	std::map<std::pair<int,int>, double>::iterator iter;
	iter = book_min_dihedral.find( std::pair<int,int>( a->_id, b->_id ));
	if ( iter != book_min_dihedral.end() ) {
		min_dihedral_angle = iter->second;
	}
	iter = book_max_dihedral.find( std::pair<int,int>( a->_id, b->_id ));
	if ( iter != book_max_dihedral.end() ) {
		max_dihedral_angle = iter->second;
		return;
	}

	max_polar_angle = 0.0;

	// sample every 5 inches and 10 degrees
	for (double angle = 0.0; angle < 2*M_PI; angle += M_PI / 18.0) {

		for (double d = 0.0; d < GRID_STEP / 2.0; d += 5.0) {

			for ( double h = h1; h <= h2; h+= (h2-h1)/5.0 ) {
	
				Vec3d t = _node->_pos + Vec3d( d * cos(angle), d * sin(angle), 0.0 );
				t[2] = h;

				Vec3d l1a = a->getA() - t;
				Vec3d l1b = a->getB() - t;
				Vec3d l2a = b->getA() - t;
				Vec3d l2b = b->getB() - t;

				Vec3d n1 = cross( l1a, l1b );
				Vec3d n2 = cross( l2a, l2b );

				if ( len( n1 ) < EPS || len( n2 ) < EPS )
					continue;

				n1 = norm(n1);
				n2 = norm(n2);

				double dih_angle = MIN( Acos(dot(n1,n2)), Acos(dot(n1,-n2)) );

				min_dihedral_angle = MIN( min_dihedral_angle, dih_angle );
				max_dihedral_angle = MAX( max_dihedral_angle, dih_angle );

				max_polar_angle = MAX(max_polar_angle, maxPolarAngle( l1a, l1b, l2a, l2b ));
			}
		}
	}

	// store the min and max in a book (careful: this is for the current node only)
	std::pair<int,int> p;
	std::pair<std::pair<int,int>,double> element;
	p.first = a->_id;
	p.second = b->_id;
	element.first = p;
	element.second = min_dihedral_angle;
	book_min_dihedral.insert( element );

	p.first = b->_id;
	p.second = a->_id;
	element.first = p;
	element.second = min_dihedral_angle;
	book_min_dihedral.insert( element );

	p.first = a->_id;
	p.second = b->_id;
	element.first = p;
	element.second = max_dihedral_angle;
	book_max_dihedral.insert( element );

	p.first = b->_id;
	p.second = a->_id;
	element.first = p;
	element.second = max_dihedral_angle;
	book_max_dihedral.insert( element );
}

/* compute the max subtended angle of a given line for a given camera position estimate
 * encoded in the current node
 * and assuming that the camera lies within a cylinder of radius GRID_STEP / 2 and of height between heights h1 and h2
 */
void LUT::max_subtended_angle( Edge *a, double h1, double h2, double &max_subtended_angle )
{

	max_subtended_angle = 0.0;

	// first tries to read from book
	std::map<int, double>::iterator iter;
	iter = book_max_subtended.find( a->_id );
	if ( iter != book_max_subtended.end() ) {
		max_subtended_angle = iter->second;
		return;
	}

	// sample every 5 inches and 10 degrees
	for (double angle = 0.0; angle < 2*M_PI; angle += M_PI / 18.0) {

		for (double d = 0.0; d < GRID_STEP  / 2.0 ; d += 5.0) {

			for ( double h = h1; h <= h2; h+= (h2-h1)/5.0 ) {
	
				Vec3d t = _node->_pos + Vec3d( d * cos(angle), d * sin(angle), 0.0 );
				t[2] = h;

				Vec3d aa = a->getA() - t;
				Vec3d bb = a->getB() - t;

				if ( len(aa) < EPS || len(bb) < EPS )
					continue;
				
				max_subtended_angle = MAX( max_subtended_angle, Acos(dot(norm(aa),norm(bb))) );

			}
		}
	}

	// insert the result in the book
	std::pair<int,double> p;
	p.first = a->_id;
	p.second = max_subtended_angle;
	book_max_subtended.insert( p );
}

/* Insert a set of lines into another one (avoiding duplicates)
 */
void LUT::InsertLines( edgeVector &src, edgeVector &dest )
{
	for ( edgeVector::iterator iter = src.begin(); iter != src.end(); iter++ ) {
		dest.push_back( *iter );
	}
}

void LUT::InsertFaces( faceVector &src, faceVector &dest )
{
	for ( faceVector::iterator iter = src.begin(); iter != src.end(); iter++ ) {
		dest.push_back( *iter );
	}
}

/* histogram the angles
 */
void LUT::computeAngleHistogram( Vec3d position, edgeVector &lines, Histogram &histogram )
{
	histogram.reset();
	
	int n = histogram.getSize();
	// fill in histogram
	for (int i=0;i<lines.size();i++) {
		Edge *l1 = lines[i];
		EdgePlane p1 = EdgePlane( l1->getA() - position, l1->getB() - position, position, 0, 0, 0 );

		for (int j=i+1;j<lines.size();j++) {
			Edge *l2 = lines[j];
			EdgePlane p2 = EdgePlane( l2->getA() - position, l2->getB() - position, position, 0, 0, 0 );

			double val = MAX( 0.0, MIN( 1.0, p1.angle(p2) / M_PI ) );  // 0 < val < 1
			
			int index = int( val * n );

			if ( index == n )
				index--;

			histogram.Increment( index );
		}
	}

	histogram.normalize();
}

/* Compute angle histogram
*/
void LUT::computeAngleHistogram( Node *node, Histogram &histogram )
{
	// lookup the visibility set
	if ( node == NULL )
		return;

	edgeVector lines;
	faceVector faces;

	node->ReadVisibilitySet( _node_histogram, lines, faces, _model );

	int n = histogram.getSize();

	if ( n == 0 )
		return;
	
	computeAngleHistogram( node->_pos, lines, histogram );
}
	
/* find the neighbors of a given node avoiding duplicates
 */
void LUT::findNodeNeighbors( Node *node, int depth, nodeVector &neighbors, bool active )
{
	if ( node == NULL )
		return;
	
	neighbors.push_back( node );

	for ( int i=0; i<depth; i++ ) {
		
		int p = neighbors.size();

		for ( int j=0;j<p; j++) {

			int n = active ? neighbors[j]->_rns.size() : neighbors[j]->_ns.size();

			for ( int k=0;k<n; k++ ) {

				// insert the node in the list if it's not there
				bool found = false;
				int id = active ? nodes[neighbors[j]->_rns[k]]->_id : nodes[neighbors[j]->_ns[k]]->_id;

				for ( int m=0; m<neighbors.size();m++ ) {

					if ( neighbors[m]->_id == id ) {
						found = true;
						break;
					}
				}

				if ( !found ) {
					Node *addnode = active ? nodes[neighbors[j]->_rns[k]] : nodes[neighbors[j]->_ns[k]];
					neighbors.push_back( addnode );
				}
			}
		}
	}
}
	

/* Draw camera pose history
 */
void LUT::drawHistory( Viewer &viewer, const float color[3], int size )
{
	for ( std::vector< Vec3d >::iterator iter = history.begin(); iter != history.end(); iter++ )
	{
		viewer.drawSphere( *iter, size, color, 10, 10 );
	}

}

/* the flag <read_positions_only> is set to true if we only want to read the node positions
 * as opposed to the whole visibitility set. This feature is used at startup when we want
 * to make a list of the node positions computed so far.
 */
bool LUT::compute( Node *node )
{
	if ( node == NULL )
		return false;

	//return true;

	_model->reset();

	edgeVector lines;
	faceVector faces;

	if ( node->_file_pos != -1 ) {
		return node->ReadVisibilitySet( _node_histogram, lines, faces, _model );
	}

	// if not found, compute visibility in all six directions
	_viewer._eye = node->_pos;
	_viewer._spheric.phi = M_PI/2;
	_viewer._spheric.theta = 0;

	lines.clear();

	// reset match flags
	for (edgeVector::iterator iter = _model->_edges.begin(); iter != _model->_edges.end(); iter++ )
		(*iter)->_matched = false;
	for (faceVector::iterator fiter = _model->_faces.begin(); fiter != _model->_faces.end(); fiter++ )
		(*fiter)->_matched = false;

	nodeVector neighbors;
	findNodeNeighbors( node, 1, neighbors, false );

	computeVisibleSet ( node, neighbors, 0);
	_viewer._spheric.phi = M_PI/2;
	_viewer._spheric.theta = M_PI/2;
	computeVisibleSet ( node, neighbors, 1);
	_viewer._spheric.phi = M_PI/2;
	_viewer._spheric.theta = 2*M_PI/2;
	computeVisibleSet ( node, neighbors, 2);
	_viewer._spheric.phi = M_PI/2;
	_viewer._spheric.theta = 3*M_PI/2;
	computeVisibleSet ( node, neighbors, 3);
	_viewer._spheric.phi = EPS;
	_viewer._spheric.theta = 0;
	computeVisibleSet ( node, neighbors, 4);
	_viewer._spheric.phi = M_PI-EPS;
	_viewer._spheric.theta = 0;
	computeVisibleSet ( node, neighbors, 5);

	// update the edges and the faces
	for ( iter = _model->_edges.begin();iter!= _model->_edges.end();iter++) {
		if ( !(*iter)->_visible )
			continue;

		(*iter)->computeWeight(node->_pos);
		lines.push_back( *iter );
	}

	for ( fiter = _model->_faces.begin(); fiter != _model->_faces.end(); fiter++ ) {
		if ( (*fiter)->_visible ) {
			faces.push_back( *fiter );
		//	printf("%d ", (*fiter)->_id);
		}
	}

	// sort the lines by weight
	std::sort( lines.begin(), lines.end(), std::greater<Edge*>());

	// cleanup inconsistencies ( if node1 sees node2 but node2 does not see node1, remove node2 from neighborhood of node1)
	bool done = false;
	while ( !done ) {

		done = true;
		for ( intVector::iterator iter = node->_rns.begin(); iter != node->_rns.end(); iter++ ) {
			Node *neighbor = nodes[*iter];
			if ( neighbor->_file_pos == -1 )
				continue;
			bool found = false;
			neighbor->print();
			for (int k=0;k<neighbor->_rns.size();k++) {
				if ( neighbor->_rns[k] == node->_id ) {
					found =true;
					break;
				}
			}

			if ( !found ) {
				node->_rns.erase( iter );
				done = false;
				break;
			}
		}
	}

	node->_signature_pos = -1;

	// write into file
	node->WriteVisibilitySet( lines, faces, _model );
		
	return true;
}

/* sort nodes by affinity to the signature given as input
 */
void LUT::sortNodes( Histogram &hist, nodeVector &out_nodes )
{
	out_nodes.clear();

	Histogram node_hist;

	// for each node, compute affinity
	for ( nodeVector::iterator iter = nodes.begin(); iter != nodes.end(); iter++ ) {
		Node *node = *iter;
		edgeVector lines;
		faceVector faces;
		node->ReadVisibilitySet( node_hist, lines, faces, _model );
		node->affinity = hist.Diff( node_hist );
		out_nodes.push_back( node );
	}

	// sort the nodes by decreasing affinity
	std::sort( out_nodes.begin(), out_nodes.end(), std::greater<Node*>() );

	// print out scores
	for (int i=0;i<out_nodes.size();i++)
		printf( "%f ", out_nodes[i]->affinity );
	printf("\n");
}

/* Setup a region
 */
void LUT::setupRegions()
{

	// reset regions
	regions.clear();

	// reset visited flags
	for ( nodeVector::iterator iter = nodes.begin(); iter != nodes.end(); iter++ ) {
		(*iter)->visited = false;
	}

	// while there is a node to visit...
	while ( 1 ) {

		Node *node = NULL;

		// find a candidate node
		for (int i=0;i<nodes.size();i++) {
			if ( !nodes[i]->visited ) {
				node = nodes[i];
				break;
			}
		}

		if ( node == NULL )
			break;

		Region region;

		// extend the region to neighborhood
		nodeVector neighbors;
		findNodeNeighbors( node, REGION_SIZE, neighbors, true );

		// setup region
		region._nodes = neighbors;
		for ( i=0;i<neighbors.size();i++ )
			neighbors[i]->visited = true;

		regions.push_back( region );
	}

	LOG( LEVEL_INFO, "# regions: %d", regions.size() );
}

/* given a node position, compute the list of visible model lines
 * and determine whether the neighbor nodes are visible or not
 *
 */
void LUT::computeVisibleSet ( Node *node, nodeVector &neighbors, int runid )
{
	if ( node == NULL )
		return;

	//PerfTimer timer;

	_pixels = (GLfloat*)malloc(3*WIDTH*HEIGHT*sizeof(GLfloat)); //_pixels used to render visibility set
	_cpixels = (GLubyte*)malloc(3*WIDTH*HEIGHT*sizeof(GLubyte)); //_pixels used to render visibility set

	//timer.print("alloc");

	Vec3d position = node->_pos;

	// setup viewer
	_viewer.clearWindow(true);
	_viewer.setup3D();

	// draw the faces
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glRenderMode (GL_RENDER);
	_viewer.clearWindow(true);
	
	// draw each face with a different color
	// and reset the visible flag to false
	int n = _model->_faces.size() + neighbors.size();

	float *color;
	color = (float*)malloc(3*sizeof(float));
	for (int i=0;i<n; i++) {

		indexToColor( n, i, color );

		if ( i < _model->_faces.size() ) {
			_model->getFace( i )->draw( true, color );
			if ( !_model->getFace( i )->_matched)
				_model->getFace( i )->_visible = false;
		} else {

			int idx = i - _model->_faces.size();

		// draw the neighbor nodes are sphere
			if ( len(neighbors[idx]->_pos - node->_pos ) > 10.0 ) {
				_viewer.drawSphere( neighbors[idx]->_pos, 15.0,color,10,10);
			} else {
				//printf("skipping node %d\n", idx);
			}
		}
	}

	// read pixels
	glReadPixels(0,0,WIDTH, HEIGHT, GL_RGB,GL_UNSIGNED_BYTE,(void*)_cpixels);
	glReadPixels(0,0,WIDTH, HEIGHT, GL_DEPTH_COMPONENT,GL_FLOAT,(void*)_pixels);

	// for each pixel, determine the visible face using its color

	for (int p=0;p<HEIGHT;p+=2) {
		for (int q=0;q<WIDTH;q+=2) {
			int index = p * WIDTH + q;

			color[0] = (float)_cpixels[ 3*index ];
			color[1] = (float)_cpixels[ 3*index+1 ];
			color[2] = (float)_cpixels[ 3*index+2 ];
			
			if ( !colorToIndex( n, index, color ) ) {
				continue;
			}

			if ( index < _model->_faces.size() ) {
				_model->getFace( index )->_visible = true;
				_model->getFace( index )->_matched = true;
			} else {
				
				if ( index >= _model->_faces.size() &&  index < n ) {

					// if node already seen, skip it
					bool seen = false;
					
					for (int k=0;k<node->_rns.size();k++) {
						if ( node->_rns[k] == neighbors[index-_model->_faces.size()]->_id )
							seen = true;
					}
					
					if ( seen )
						continue;

					node->_rns.push_back( neighbors[index-_model->_faces.size()]->_id );
					
				}
				
			}
		}
	}

	// now, every line adjacent to a visible face is a candidate for visibility
	// while the others are not even considered

	// reset the visible flag for all edges
	for (edgeVector::iterator iter = _model->_edges.begin(); iter != _model->_edges.end(); iter++ ) {
		if ( !(*iter)->_matched )
			(*iter)->_visible = false;
	}

	// each edge adjacent to a visible face is considered as a visible candidate
	for (faceVector::iterator fiter = _model->_faces.begin(); fiter != _model->_faces.end(); fiter++ ) {

		if ( !(*fiter)->_visible )
			continue;

		for (int q=0;q<(*fiter)->_vertices.size();q++) {
			for (int r=0;r<(*fiter)->_vertices[q]->_edges.size();r++) {
				if ((*fiter)->_vertices[q]->_edges[r]->_faces.empty()) {
					(*fiter)->_vertices[q]->_edges[r]->_visible = true;
				} else {
					for (int m=0;m<(*fiter)->_vertices[q]->_edges[r]->_faces.size();m++) {
						if ( (*fiter)->_vertices[q]->_edges[r]->_faces[m]->_id == (*fiter)->_id ) {
							(*fiter)->_vertices[q]->_edges[r]->_visible = true;
							break;
						}
					}
				}
			}
		}
	}

	// for each candidate edge, determine whether it is really visible
	// draw edges in feedback mode
	_viewer.setup3D();

	for ( i=0;i<_model->_edges.size();i++) {
		Edge *edge = _model->getEdge( i );

		if ( !edge->_visible )
			continue;

		if ( edge->_flat ) {
			edge->_visible = false;
			continue;
		}

		// initialize the feedback buffer
		_viewer.setMode(GL_RENDER);
		GLfloat *feedbackBuf = (GLfloat *)malloc(BUFSIZE_FEEDBACK*sizeof(GLfloat));
		glFeedbackBuffer(BUFSIZE_FEEDBACK,GL_3D,feedbackBuf);

		(void) _viewer.setMode (GL_FEEDBACK);
		_viewer.clearWindow(true);
		
		// render the edge as a set of points
		if ( node->_id == 253 && edge->_id == 1460 )
			LOG(LEVEL_INFO, "length: %f  steps: %d", edge->length(), int(edge->length() / EDGE_STEP )) ;

		for (int j=0;j<int(edge->length()/EDGE_STEP);j++) {
			Vec3d a = edge->_a->getPosition();
			Vec3d b = edge->_b->getPosition();
			Vec3d u = norm(b-a);
			glPassThrough (j);
			glBegin(GL_POINTS);
			Vec3d p = a+j*EDGE_STEP*u;
			glVertex(p - 1.0 * norm(p-position) );
			glEnd();
		}
		
		GLint hits = _viewer.setMode (GL_RENDER);

		// if hits, process them
		if (hits>0) {
			bool res = processHits(hits,feedbackBuf,_pixels,_model,position,_viewer._width,_viewer._height,edge, EDGE_STEP );
		} 

		delete 	feedbackBuf;	
	}

	//delete structures
	delete color;
	delete _pixels;
	delete _cpixels;
}


/* read the nodes from the file
 */
int LUT::readNodes()
{
	// clear the nodes
	nodes.clear();

	FILE *f = fopen( _model->toFilename( MODEL_NODE_FILE ).c_str(), "r" );

	if (f == NULL ) {
		initNodes();
		return nodes.size();
	}

	while ( !feof( f ) ) {
		Node *node = new Node();
		if ( node->Read( f ) )
			nodes.push_back( node );
	}

	fclose( f );
	return nodes.size();
}

/* Sanity check nodes
 */
int LUT::checknodes ()
{
	// if two nodes are less than 1.0 apart, there is a problem
	for (int i=0;i<nodes.size();i++) {
		for (int j=i+1;j<nodes.size();j++) {
			if ( len(nodes[i]->_pos - nodes[j]->_pos) < 1.0 ) {
				printf("Error: nodes %d and %d are very close to each other.\n", i, j);
				return -1;
			}
		}
	}

	return 0;
}

/* builds a 2D grid map of the model
 */
int LUT::initNodes()
{
	printf("Building nodes...\n");

	bool done = false;
	
	// create start node
	Vec3d c = _model->_centroid;
	c[2] = inchToModelUnit ( 70.0 );

	Node *node = new Node( c, 0 );

	nodes.push_back( node );

	double b = (double)GRID_STEP / sqrt(3.0);
	double t = (double)GRID_STEP * sqrt(2.0/3.0);

	while ( !done ) {

		done = true;

		// try to add a new neighbor to a node
		for ( int m = 0; m<nodes.size(); m++ ) {

			Node *cnode = nodes[m];

			// if complete, continue
			if ( cnode->_ns.size() == 6 ) 
				continue;

			// first add neighbors
			double alpha = 0.0;

			for (int i=0; i<6;i++) {

				Vec3d pos = cnode->_pos;

				if ( i >= 12 ) {
					pos[0] += cos(alpha)*GRID_STEP;
					pos[1] += sin(alpha)*GRID_STEP;
					pos[2] += GRID_STEP;
				} else if ( i >= 6 ) {
					pos[0] += cos(alpha)*GRID_STEP;
					pos[1] += sin(alpha)*GRID_STEP;
					pos[2] -= GRID_STEP;
				} else {
					pos[0] += cos(alpha)*GRID_STEP;
					pos[1] += sin(alpha)*GRID_STEP;
				}

				if ( pos[2] < -150 || pos[2] > 130) {
					alpha += M_PI/3.0;
					continue;
				}

				if ( _model->isOutside( pos ) ) {
					alpha += M_PI/3.0;
					continue;
				}

				if ( !cnode->HasNeighbor( nodes, pos ) ) {
					Node *node = new Node( pos, nodes.size() );
					node->InsertNeighbor( cnode->_id );
					cnode->InsertNeighbor( node->_id );
					nodes.push_back( node );

					// update neighborhood
					for (int j=0;j<cnode->_ns.size();j++) {
						if ( len( node->_pos - nodes[cnode->_ns[j]]->_pos ) < sqrt(2.0)*GRID_STEP + 1.0 ) {
							node->InsertNeighbor( cnode->_ns[j] );
							nodes[cnode->_ns[j]]->InsertNeighbor( node->_id );
						}

						Node *n1 = nodes[cnode->_ns[j]];

						for (int k=0;k<n1->_ns.size();k++) {
							if ( len( node->_pos - nodes[n1->_ns[k]]->_pos ) < sqrt(2.0)*GRID_STEP + 1.0 ) {
								node->InsertNeighbor( n1->_ns[k] );
								nodes[n1->_ns[k]]->InsertNeighbor( node->_id );
							}

							Node *n2 = nodes[n1->_ns[k]];

							for (int p=0;p<n2->_ns.size();p++) {
								if ( len( node->_pos - nodes[n2->_ns[p]]->_pos ) < sqrt(2.0)*GRID_STEP + 1.0 ) {
									node->InsertNeighbor( n2->_ns[p] );
									nodes[n2->_ns[p]]->InsertNeighbor( node->_id );
								}
							}
						}
					}
					
					done = false;

					LOG( LEVEL_INFO, "adding node %d at position %.2f %.2f %.2f", node->_id, node->_pos[0], node->_pos[1], node->_pos[2] );
				}	

				alpha += M_PI/3.0;

			}

			if ( !done )
				break;
		}
	}

	int n = nodes.size();

	for (int p=0;p<n;p++) {
		Vec3d pos = nodes[p]->_pos;

		pos[2] -= 150.0;

		Node *node = new Node( pos, nodes.size() );

		node->InsertNeighbor( nodes[p]->_id );
		nodes[p]->InsertNeighbor( node->_id );
		nodes.push_back( node );
	}

	return nodes.size();
}

/* Draw a list of nodes
 */
void LUT::drawNodes( Viewer &viewer, const float color1[3], const float color2[3] )
{
	// draw all the nodes
	for ( nodeVectorIterator iter = nodes.begin(); iter != nodes.end(); iter++ ) {
		drawNode( viewer, color1, color2, *iter, false );
	}

	// draw the active node
	drawNode( viewer, color1, color2, _node, true );
}

/* Draw a node
 */
void LUT::drawNode( Viewer &viewer, const float color1[3], const float color2[3], Node *node, bool active )
{
	if ( node == NULL )
		return;

	// draw a sphere at the node position
	if ( active )
		viewer.drawSphere( node->_pos, 5.1, color1, 10, 10 );
	else
		viewer.drawSphere( node->_pos, 5.0, color2, 10, 10 );

	// draw line connections to the neighbors
	nodeVector neighbors;
	findNodeNeighbors( node, _depth, neighbors, true );

	for ( nodeVectorIterator iter = neighbors.begin(); iter != neighbors.end(); iter++)  {
		Node *cnode = *iter;

		assert( cnode != NULL );

		if ( active ) {
			for ( int i=0; i<cnode->_rns.size(); i++ ) {
				assert( cnode->_rns[i] < nodes.size() );
				viewer.drawLine( cnode->_pos, nodes[cnode->_rns[i]]->_pos, color1, 2 );
			}
		} else {
			for ( int i=0; i<cnode->_rns.size(); i++ ) {
				assert( cnode->_rns[i] < nodes.size() );
				viewer.drawLine( cnode->_pos, nodes[cnode->_rns[i]]->_pos, color2, 1 );
			}
		}
	}
}

/* print out nodes to std output
 */
void LUT::printNodes()
{
	LOG( LEVEL_DEBUG, "**** nodes (%d) *****", nodes.size() );
	for ( nodeVectorIterator iter = nodes.begin(); iter != nodes.end(); iter++ )
		(*iter)->print();
}

/* Process a list of nodes
 */
void LUT::computeNodes()
{
	for ( nodeVectorIterator iter = nodes.begin(); iter != nodes.end(); iter++ ) {
		if ( (*iter)->_file_pos != -1 )
			continue;

		compute( *iter );

	}
}

/* Draw the scene
 */
void LUT::drawScene ()
{
	compute(false);
	glutSwapBuffers();
}

/* Init the LUT
 */
void LUT::init (int w, int h, std::string dirname)
{
	// first read the type of model
	if ( dirname.length() < 1 )
		return;

	_model->_dirname = std::string(dirname);
	
	rude::Config config;
	config.load(_model->toFilename(MODEL_CONFIG_FILE).c_str());

	std::string type = std::string(config.getStringValue("TYPE"));

	if (type == "AUTOCAD")
		TYPE = AUTOCAD;
	if (type == "INVENTOR")
		TYPE = INVENTOR;

	std::string length_unit = std::string(config.getStringValue("UNIT"));
	LOG(LEVEL_INFO, "Model unit: %s", length_unit.c_str());

	if ( length_unit == "FOOT" ) {
		LENGTH_UNIT = UNIT_FOOT; 
	} else if ( length_unit == "INCH" ) {
		LENGTH_UNIT = UNIT_INCH;
	} else {
		LENGTH_UNIT = UNIT_NONE;
	}

	_model->MODEL_RESOLUTION = config.getDoubleValue("MODEL_RESOLUTION");
	_model->MODEL_MIN_EDGE_ANGLE = toRadians(config.getDoubleValue("MODEL_MIN_EDGE_ANGLE"));

	// then read the model
	bool new_geometry = _model->read(dirname.c_str(),TYPE);

	// read config info
	_position = _model->_centroid;
	_fft = true;

	WIDTH = config.getIntValue("WIDTH");
	HEIGHT = config.getIntValue("HEIGHT");
	

	DEPTH_OF_FIELD  = config.getDoubleValue("DEPTH_OF_FIELD");

	GRID_STEP = config.getDoubleValue("GRID_STEP");
	EDGE_STEP = config.getDoubleValue("EDGE_STEP");
	REGION_SIZE = config.getIntValue("REGION_SIZE");
	_model->NORTH_ANGLE = toRadians( config.getDoubleValue( "NORTH_ANGLE" ) );

	_viewer.init(_position,1.0,0,0,92,1.0,10000.0,WALKTHROUGH);
	_viewer_3d.init(_position,_model->_diameter,3*M_PI/4,M_PI/2,45,1.0,10000.0,OUTSIDE);
	_viewer_3d_topview.init(_position,_model->_diameter,3*M_PI/4,M_PI,45,1.0,10000.0,OUTSIDE);
	_viewer_3d.set(w,h,0);
	_viewer_3d_topview.set(w,h,0);
	_viewer.set(WIDTH,HEIGHT, 0);
	_viewer_3dsubmap.init(_position,_model->_diameter,3*M_PI/4,M_PI/2,45,1.0,10000.0,OUTSIDE);
	_currentScore = 0.0; // this variable is for display only
	_scaleFactor = 23.0; // this is a fake scale factor

	//reset files
	resetFiles(new_geometry);

	_valid = true;

	// this is the node jump counter mechanism
	nframes = 0;

	// init the nodes
	readNodes();
}

/* convert a length in inches into the model length unit (foot,inch,etc.)
 */
double LUT::inchToModelUnit( double length )
{
	switch ( LENGTH_UNIT ) {
	case UNIT_INCH:
		return length;
	case UNIT_FOOT:
		return length * 0.083333;
	case UNIT_NONE:
	default:
		LOG(LEVEL_ERROR, "unknown model unit!");
		assert(false);
		return 0.0;
	}

	return 0.0;
}

/* picking method for GUI
 */
void LUT::model_picking_3d (int x, int y, bool wireframe, bool depth_test)
{
	_viewer_3d.setMode(GL_SELECT);

	_viewer_3d.setup3D(x,y);

	glInitNames();

	drawScene_3d(wireframe,depth_test,true,false,false,0);

	intVector hits;
	_viewer_3d.stopPicking(hits);

	if (hits.size() > 0) {
		if (hits[0] < _model->_edges.size()) {
			Edge *edge = _model->getEdge(hits[0]);
			_selected_lines.push_back(edge);
		}
	}
}

/* Draw 3D map and stuff
 */
void LUT::drawScene_3d(bool wireframe, bool depth_test, bool draw_visible_lines, bool texture, bool line_color_coding, int cameraId)
{
	if (!_valid)
		return;

	int i;

	if (depth_test) {
		glEnable(GL_DEPTH_TEST);
	}
	else
		glDisable(GL_DEPTH_TEST);
	
	if (_viewer_3d._mode == GL_SELECT) {
			_viewer_3d.drawEdgeBoxes(_model->_edges,0.005);
		return;
	}

	// draw image if needed
	if (_draw_pixels) {
		_viewer._eye = _position;
		switch (cameraId) {
		case 0:
			_viewer._spheric.phi = M_PI/2;
			_viewer._spheric.theta = 0;
			break;
		case 1:
			_viewer._spheric.phi = M_PI/2;
			_viewer._spheric.theta = M_PI/2;
			break;
		case 2:
			_viewer._spheric.phi = M_PI/2;
			_viewer._spheric.theta = 2*M_PI/2;
			break;
		case 3:
			_viewer._spheric.phi = M_PI/2;
			_viewer._spheric.theta = 3*M_PI/2;
			break;
		case 4:
			_viewer._spheric.phi = 0;
			_viewer._spheric.theta = 0;
			break;
		case 5:
			_viewer._spheric.phi = M_PI;
			_viewer._spheric.theta = 0;
			break;
		default:
			assert(false);
		}
		
		_viewer.setup3D();

		_model->drawFaces( texture) ;
		//_viewer.drawFaces(_faces,BLACK,true);

		// draw the visible edges
		for (i=0;i<_lines.size();i++) {
			_viewer.drawEdge(_lines[i],GREEN,3);		
		}
		return;
	}
	
	// draw the 3D model
	_model->drawFaces( texture) ;
	//_viewer_3d.drawFaces(_model->_faces,WHITE,true);
	
	glDisable(GL_LIGHTING);

	// draw the edges
	_viewer_3d.drawEdges(_model->_edges,BLACK,1,wireframe);

	// draw visible faces
	for (i=0;i<_faces.size();i++) {
		Face *f = _faces[i];
	}

	// draw the selected lines
	if (!draw_visible_lines) {
		_viewer_3d.drawEdges(_selected_lines,RED,5,true);
	} else {
		if ( !line_color_coding) {
			for (i=0;i<_lines.size();i++) {
				_viewer_3d.drawEdge(_lines[i],GREEN,3);		
			}
		} else {
			for (i=0;i<_lines.size();i++) {
				switch ( _lines[i]->status ) {
				case ACCEPTED:
					_viewer_3d.drawEdge(_lines[i],GREEN,3);
					break;
				case UNKNOWN:
					_viewer_3d.drawEdge(_lines[i],RED,3);
					break;
				case PENDING:
					_viewer_3d.drawEdge(_lines[i],ORANGE,3);
					break;
				default:
					_viewer_3d.drawEdge(_lines[i],GREEN,3);
					break;
				}
			}
		}
	}

	// draw the nodes
	if ( _draw_grid ) {
		drawNodes( _viewer_3d, RED, BLUE );
	}


}

void LUT::drawScene_3d_topview()
{
	if (!_valid)
		return;

	glEnable(GL_DEPTH_TEST);

	_viewer_3d_topview.clearWindow(true);
	_viewer_3d_topview.setup3D_topview();

	// draw the 3D model
	glColor(BLUE);
	glBegin(GL_LINES);
	for (int i=0;i<_model->_edges.size();i++) {
		Vec3d a = _model->_edges[i]->getA();
		Vec3d b = _model->_edges[i]->getB();
		a[2] = 0.0;
		b[2] = 0.0;
		glVertex(a);
		glVertex(b);
	}
	glEnd();
}

void LUT::drawScene_3dsubmap()
{

}

void LUT::keyboard ( unsigned char key, int x, int y )
{
	switch ( key ) {
	case 'q':
	case 'Q':
	case 27: // exit
		exit(0);
		break;
	case 'w':
		_viewer_3d.switchType();
		break;
	case 's':
		break;
	case 'f':
		break;
	case 'g':
		_signature.findScalingFactor(_signature0);
		break;

	}

	_viewer_3d.refresh();
	_viewer_sig.refresh();
	_viewer_3dsubmap.refresh();
	_viewer_match.refresh();
}

void LUT::mySpecialKeyFunc( int key, int x, int y  )
{	
	switch ( key ) {
	case GLUT_KEY_UP:
		_position += GRID_STEP * Vec3d(1,0,0);
		break;
	case GLUT_KEY_DOWN:
		_position += GRID_STEP * Vec3d(-1,0,0);
		break;
	case GLUT_KEY_RIGHT:
		_position += GRID_STEP * Vec3d(0,1,0);
		break;
	case GLUT_KEY_LEFT:			
		_position += GRID_STEP * Vec3d(0,-1,0);
		break;
	}

	_viewer.refresh();
	_viewer_3d.refresh();
	
}

void LUT::mouse_3d (int button, int state, int x, int y, int glutModifier)
{
	_viewer_3d.mouse (button, state, x, y,glutModifier);
}

void LUT::mouse_3d_topview (int button, int state, int x, int y)
{
	_viewer_3d_topview.mouse (button, state, x, y,glutGetModifiers());
}

void LUT::motion_3d (int x, int y)
{
	double sensitivity = 360.0;
	_viewer_3d.motion(x,y,4.0,400.0,4.0);
}

void LUT::motion_3d_topview (int x, int y)
{
	double sensitivity = 360.0;
	_viewer_3d_topview.motion(x,y,4.0,400.0,4.0);
}

/* Clear the LUT
 */
void LUT::clear ()
{
	// clear the model
	_model->clear();

	// clear the nodes
	_node = NULL;

	for (int i=0;i<nodes.size();i++) {
		delete nodes[i];
	}

	nodes.clear();
}

/* automatically process the LUT
 */
void LUT::automate(Fl_Progress *progress)
{

	edgeVector lines;
	int counter = 0, ratio = 0;
	progress->maximum( nodes.size() );
	printf( "100: ");

	for ( nodeVectorIterator iter = nodes.begin(); iter != nodes.end(); iter++ ) {

		if ( (*iter)->_file_pos == -1 ) {
			compute( *iter );
		}
		
		progress->value(counter);
		//Fl::check();
		//progress->parent()->redraw();
		counter++;
		if ( 100.0*counter/nodes.size() > ratio ) {
			printf("|");
			fflush( stdout );
			ratio++;
			if ( ratio%10 == 0 ) {
				printf(" %d ", ratio );
				fflush( stdout );
			}
		}
	}
}

void LUT::selectRandomLines (int N, edgeVector &select_lines, bool testSkew)
{
	int n = select_lines.size(), i, j;

	intVector indexes;
	edgeVector lines;

	selectNRandomInt(N,n,indexes);

	for (i=0;i<N;i++) {
		lines.push_back(select_lines[indexes[i]]);
	}

	if (!testSkew) {
		for (i=0;i<N;i++)
			_selected_lines.push_back(lines[i]);
		return;
	}


	bool good = false;
	
	while (!good) {

		good = true;

		for (i=0;i<N;i++) {
			for (j=0;j<N;j++) {
				if (i == j)
					continue;

				if (lines[i]->testSkew(lines[j],1-EPSILON)) {
					good = false;
				}
			}
		}

		if (!good) {
			lines.clear();
			selectNRandomInt(N,n,indexes);
			for (i=0;i<N;i++) {
				lines.push_back(select_lines[indexes[i]]);
			}
		}
	}


	for (i=0;i<N;i++)
		_selected_lines.push_back(lines[i]);
}

void LUT::resetLineWeights()
{
	for (edgeVector::iterator iter = _lines.begin(); iter != _lines.end(); iter++) {
		(*iter)->_weight = 0.0;
	}
}

void LUT::setLines( Status status )
{
	for (edgeVector::iterator iter = _lines.begin(); iter != _lines.end(); iter++) {
	//	printf( "%d ", (*iter)->_id);
		(*iter)->status = status;
	}
	//printf("\n");
}

void Region::draw( Viewer &viewer, const float color[3], double blending )
{
	// draw the region in blended color
	glEnable(GL_DEPTH_TEST);
	glColor4f(color[0],color[1],color[2],blending);
	glEnable(GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	for (int i=0;i<_nodes.size();i++) {
		for (int j=i+1;j<_nodes.size();j++) {
			for (int k=j+1;k<_nodes.size();k++) {
				glBegin(GL_TRIANGLES);
				glVertex( _nodes[i]->_pos );
				glVertex( _nodes[j]->_pos );				
				glVertex( _nodes[k]->_pos );
				glEnd();
			}
		}
	}

	glDisable( GL_BLEND );
}
