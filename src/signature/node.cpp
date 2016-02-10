#include "signatureviewer.h"

bool Node::Read( FILE *f )
{
	if ( f == NULL )
		return  false;

	int id;
	double x,y,z;
	long int file_pos, signature_pos;

	if ( fscanf( f, "%d%lf%lf%lf%d%d", &id, &x, &y, &z, &file_pos, &signature_pos ) != 6 ) 
		return false;

	_id = id;
	_pos = Vec3d( x, y, z);
	_file_pos = file_pos;
	_signature_pos = signature_pos;

	while ( fscanf( f, "%d", &id ) == 1 ) {
		_rns.push_back( id );
	}

	char str[5];
	fscanf( f, "%s", str); // read the string '$$'

	while ( fscanf( f, "%d", &id ) == 1 ) {
		_ns.push_back( id );
	}

	fscanf( f, "%s", str); // read the string '$$'

	return true;
}

void Node::InsertNeighbor( int id )
{
	if ( id == _id )
		return;

	for (int i=0;i<_ns.size();i++) {
		if ( id == _ns[i] )
			return;
	}

	_ns.push_back( id );
}

bool Node::Write( FILE *f )
{
	if ( f == NULL )
		return false;

	fprintf( f, "%d %f %f %f %d %d ", _id, _pos[0], _pos[1], _pos[2], _file_pos, _signature_pos );

	for (int i=0;i<_rns.size();i++)
		fprintf( f, "%d ", _rns[i] );

	fprintf( f, "$$ " ); // write the string '$$'

	for ( i=0;i<_ns.size();i++)
		fprintf( f, "%d ", _ns[i] );

	fprintf( f, "$$\n" ); // write the string '$$'

	return true;
}

bool Node::WriteVisibilitySet( edgeVector &lines, faceVector &faces, Model *model )
{
	FILE *f = fopen( model->toFilename( MODEL_VISI_FILE ).c_str(), "a" );

	if ( f == NULL )
		return false;

	setbuf(f,NULL);
	fprintf(f," ");
	fflush( f );
	_file_pos = ftell ( f );

	// print the node ID
	fprintf( f, "%d\n", _id );
	//printf( "\nnode %d:\n", _id );
	//printf( "filename: %s\n", model->toFilename( MODEL_VISI_FILE ).c_str());

	// print the lines ID
	for ( edgeVector::iterator iter = lines.begin(); iter != lines.end(); iter++ ) {
		fprintf( f, "%d ", (*iter)->_id );
		//printf( "%d ", (*iter)->_id );
	}

	fprintf( f, "$$ ");

	// print the face IDs
	for (faceVector::iterator fiter = faces.begin(); fiter != faces.end(); fiter++ ) {
		fprintf( f, "%d ", (*fiter)->_id );
		//printf( "%d ", (*fiter)->_id );
	}

	fprintf( f, "$$\n");

	fclose ( f );

	// write the node in the node list file
	f = fopen( model->toFilename( MODEL_NODE_FILE ).c_str(), "a" );
	Write ( f );
	fclose( f );

	return true;
}

// return true if the node has <position> as a neighbor
bool Node::HasNeighbor( nodeVector &nodes, Vec3d position )
{
	for ( int i=0; i<_ns.size(); i++ ) {
		Node *n1 = nodes[_ns[i]];
		if ( len( n1->_pos - position ) < 1.0 )
			return true;
		//for (int j=0;j<n1->_ns.size();j++) {
		//	Node *n2 = nodes[n1->_ns[j]];
		//	if ( len( n2->_pos - position ) < 1.0 )
		//		return true;
		//}
	}

	return false;
}

void Node::print()
{
	char str[256];

	sprintf( str, "[ " );
	for (int i=0;i<_ns.size();i++) {
		sprintf( str, "%s %d ", str, _ns[i] );
	}
	sprintf( str, "%s ]", str );

	sprintf( str, "%s [ ", str );
	for ( i=0;i<_rns.size();i++) {
		sprintf( str, "%s %d ", str, _rns[i] );
	}
	sprintf( str, "%s ] ", str );

	//LOG( LEVEL_DEBUG, "%d %s %d %.2f %.2f %.2f", _id, str, _file_pos, _pos[0], _pos[1], _pos[2] );
}

bool Node::ReadVisibilitySet( Histogram &hist, edgeVector &lines, faceVector &faces, Model *model )
{
	if ( _file_pos == -1 )
		return false;

	FILE *f = fopen( model->toFilename( MODEL_VISI_FILE ).c_str(), "r" );

	if ( fseek( f, _file_pos-1, SEEK_SET ) != 0 ) {
		fclose( f );
		return false;
	}

	// read the node ID
	int id;
	fscanf( f, "%d", &id );

	if ( id != _id ) {
		fclose( f );
		return false;
	}

	// read the lines IDs
	while (fscanf( f, "%d", &id ) == 1 ) {
		Edge *line = model->getEdge( id );
		if ( line != NULL )
			lines.push_back( line );
	}

	char str[5];
	fscanf( f, "%s", str); // read the string '$$'

	// read the faces IDs
	while (fscanf( f, "%d", &id ) == 1 ) {
		Face *face = model->getFace( id );
		if ( face != NULL )
			faces.push_back( face );
	}

	fclose( f );

	// read the histogram
	hist.Read( model->toFilename( MODEL_SIGNATURE_FILE ).c_str(), _signature_pos );

	return true;
}

/*
// the flag <read_positions_only> is set to true if we only want to read the node positions
// as opposed to the whole visibitility set. This feature is used at startup when we want
// to make a list of the node positions computed so far.
bool LUT::compute( Node *node )
{
	if ( node == NULL )
		return false;

	if (read_positions_only && testVisibility(_position,_model->toFilename(MODEL_VISI_LIST_FILE).c_str())) {
		_computed_positions.push_back(_position);
		return true;
	}

	_model->reset();

	// insert position in list if needed
	for (int i=0;i<_computed_positions.size();i++) {
		if (len(_computed_positions[i]-_position) < EPSILON)
			break;
	}

	if (i == _computed_positions.size())
		_computed_positions.push_back(_position);

	
	// try to read in the database
	if (readVisibility(_position,_model,_lines,_model->toFilename(MODEL_VISI_FILE).c_str(),\
		_model->toFilename(MODEL_VISI_SUB_FILE).c_str(),_model->toFilename(MODEL_VISI_LIST_FILE).c_str())) {

		//_signature.set(_position,_lines);
		//if (!_signature.read(_position,_model->toFilename(MODEL_SIGN_FILE).c_str(),_model->toFilename(MODEL_SIGN_LIST_FILE).c_str())) {
		//	_signature.compute();
		//	_signature.write(_model->toFilename(MODEL_SIGN_FILE).c_str(),_model->toFilename(MODEL_SIGN_LIST_FILE).c_str());
		//}
			
		return true;
	}

	// if not found, compute visibility in all six directions
	_viewer._eye = _position;
	_viewer._spheric.phi = M_PI/2;
	_viewer._spheric.theta = 0;

	edgeVector v[6];

	_lines.clear();

	computeVisibilityField (_viewer, _position, _model, _lines, EDGE_STEP, DEPTH_OF_FIELD, MIN_SUBTENDED_ANGLE, _pixels);
	_viewer._spheric.phi = M_PI/2;
	_viewer._spheric.theta = M_PI/2;
	computeVisibilityField (_viewer, _position, _model, _lines/, EDGE_STEP, DEPTH_OF_FIELD, MIN_SUBTENDED_ANGLE, _pixels);
	_viewer._spheric.phi = M_PI/2;
	_viewer._spheric.theta = 2*M_PI/2;
	computeVisibilityField (_viewer, _position, _model, _lines, EDGE_STEP, DEPTH_OF_FIELD, MIN_SUBTENDED_ANGLE, _pixels);
	_viewer._spheric.phi = M_PI/2;
	_viewer._spheric.theta = 3*M_PI/2;
	computeVisibilityField (_viewer, _position, _model, _lines, EDGE_STEP, DEPTH_OF_FIELD, MIN_SUBTENDED_ANGLE, _pixels);
	_viewer._spheric.phi = 0;
	_viewer._spheric.theta = 0;
	computeVisibilityField (_viewer, _position, _model, _lines, EDGE_STEP, DEPTH_OF_FIELD, MIN_SUBTENDED_ANGLE, _pixels);
	_viewer._spheric.phi = M_PI;
	_viewer._spheric.theta = 0;
	computeVisibilityField (_viewer, _position, _model, _lines, EDGE_STEP, DEPTH_OF_FIELD, MIN_SUBTENDED_ANGLE, _pixels);

	// sort the lines by weight
	std::sort( _lines.begin(), _lines.end(), std::greater<Edge*>());

	Node *node = new Node( _position, nodes.size() );

	node->writeVisibilitySet( _lines );

	//writeVisibility(_position,_model,_lines,_model->toFilename(MODEL_VISI_FILE).c_str(),\
	//	_model->toFilename(MODEL_VISI_SUB_FILE).c_str(),_model->toFilename(MODEL_VISI_LIST_FILE).c_str());
		
	return true;
	//_signature.set(_position,_lines);
	//_signature.compute();
	//_signature.print();
	//_signature.write(_model->toFilename(MODEL_SIGN_FILE).c_str(),_model->toFilename(MODEL_SIGN_LIST_FILE).c_str());
}
*/