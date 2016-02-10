#include "main.h"

// init the edge tracker by creating a tracker for each image edge
//
void MyGlWindow::initEdgeTracker ()
{
	int i,j;

	int counter = 0;

	// for each image edge, create a new tracker
	for (i=0;i<_camera->_frame._edgeplanes_chained.size();i++) {
		for (j=0;j<_camera->_frame._edgeplanes_chained[i].size();j++) {

			if ( _camera->_frame._edgeplanes_chained[i][j].length() < MIN_EDGE_SIZE_TRACKING )
				continue;

			std::vector< std::pair< int, EdgePlane > > vector;

			vector.push_back( std::pair< int, EdgePlane > ( frameId, _camera->_frame._edgeplanes_chained[i][j] ) );

			_tracked_edges.push_back( std::pair< int, std::vector< std::pair< int, EdgePlane > > > ( 1, vector ) );

			counter++;
		}
	}

	// init with a keyframe
	_keyframes.clear();

	_keyframes.push_back( std::pair< int, ExtrinsicParameters > ( frameId, _camera->getPose() ) );

	LOG( LEVEL_INFO, "tracker init with %d tracks", counter );
}

// update the edge tracker
//
void MyGlWindow::updateEdgeTracker () 
{
	// if the tracker is empty, initialize it
	if ( _tracked_edges.empty() ) {
		initEdgeTracker();
		return;
	}

	// memorize empty slots
	intVector tracked_edges;

	// for each tracked edge, update
	int i,j,trackid;

	for (trackid=0;trackid<_tracked_edges.size();trackid++) {

		// if the track is empty, skip it
		if ( _tracked_edges[trackid].second.empty() ) {
			continue;
		}

		// otherwise, update the tracker
		EdgePlane ref_edge = _tracked_edges[trackid].second.back().second;

		// find the bucket ID of the edge
		intVector bucket_ids;
		get_edgeplane_buckets(ref_edge, bucket_ids);

		// store the correct matches in a book
		std::vector< std::pair< double, int> > book;

		edgePlaneVector edgeplanes;

		// find the corresponding matches on the image
		for (i=0;i<bucket_ids.size();i++) {

			int bucket_id = bucket_ids[i];

			for (j=0;j<_edges_buckets[bucket_id].size();j++) {

				int edge_id = _edges_buckets[bucket_id][j];

				EdgePlane edge;
				if (!_camera->_frame.get_edgeplane_chained( edge_id, edge ) ) {
					LOG(LEVEL_ERROR, "error accessing edgeplane %d out of %d edgeplanes.", edge_id, _camera->_frame.n_edgeplanes_chained );
					continue;
				}

				if ( edge.length() < MIN_EDGE_SIZE_TRACKING )
					continue;

				if ( edge.overlap( ref_edge ) < .1 && ref_edge.overlap( edge ) < .1 )
					continue;

				if ( edge.colorDistance( ref_edge ) > 0.03 )
					continue;
				
				std::pair< double, int > element;
				element.first = fabs(edge.angle(ref_edge)) /* fabs( edge.length() - ref_edge.length() ) / ref_edge.length()*/;
				element.second = edge_id;
				
				edgeplanes.push_back ( edge );

				book.push_back( element );
			}
		}

		// if nothing in the book, continue
		if ( book.empty() )
			continue;

		// otherwise, keep the best match in the book
		std::sort( book.begin(), book.end() );

		// update the track
		EdgePlane new_edge;
		_camera->_frame.get_edgeplane_chained( book[0].second, new_edge );
		_tracked_edges[trackid].second.push_back( std::pair< int, EdgePlane > ( frameId, new_edge ) );

		// remember that this edge is being tracked
		tracked_edges.push_back( book[0].second );
	}
				
	// for each non-tracked edge, insert a new track in the tracker
	for (i=0;i<_camera->_frame.n_edgeplanes_chained;i++) {

		bool tracked = false;

		for (j=0;j<tracked_edges.size();j++) {
			if ( i == tracked_edges[j] ) {
				tracked = true;
				break;
			}
		}

		if ( tracked )
			continue;
		
		std::vector< std::pair< int, EdgePlane > > vector;
		
		EdgePlane edge;
		_camera->_frame.get_edgeplane_chained( i, edge );

		if ( edge.length() < MIN_EDGE_SIZE_TRACKING )
			continue;

		vector.push_back( std::pair< int, EdgePlane > ( frameId, edge ) );
		
		_tracked_edges.push_back( std::pair< int, std::vector< std::pair< int, EdgePlane > > > ( 1, vector ) );
	}

	// update the keyframes and the tracker counters
	if ( updateKeyFrames() )
		updateTrackerCounters();

	// cleanup the tracker
	cleanupEdgeTracker();

	LOG(LEVEL_INFO, "edge tracker: %d tracks", _tracked_edges.size() );
}

// update the keyframes : if the current camera pose is far enough from other keyframes
// then create a new keyframe
//
bool MyGlWindow::updateKeyFrames ()
{
	// determine whether the current camera position corresponds to a new keyframe
	int i;

	if ( _keyframes.empty() ) {

		LOG(LEVEL_INFO, "new keyframe at frame %d", frameId);

		_keyframes.push_back( std::pair< int, ExtrinsicParameters > ( frameId, _camera->getPose() ) );

		return true;

	}

	
	double min_dist = len( _camera->getTranslation() - _keyframes[0].second.getTranslation() );

	if ( min_dist < KEYFRAME_DISTANCE )
		return false;

	
	for (i=1;i<_keyframes.size();i++) {

		min_dist = len( _camera->getTranslation() - _keyframes[i].second.getTranslation() );

		if ( min_dist < KEYFRAME_DISTANCE )
			return false;
	}

	
	LOG(LEVEL_INFO, "new keyframe at frame %d", frameId);
	
	_keyframes.push_back( std::pair< int, ExtrinsicParameters > ( frameId, _camera->getPose() ) );
	
	return true;
}

// update the counter of each track. The counter is the number of keyframes within the track
// this function should only be run if updateKeyFrames returned true
//
void MyGlWindow::updateTrackerCounters ()
{
	int i;

	if ( _keyframes.empty() || _keyframes.back().first != frameId )
		return;

	// for each track, update the counter if the current frame ID is in the track
	for (i=0;i<_tracked_edges.size();i++) {

		if ( _tracked_edges[i].second.empty() )
			continue;

		if ( _tracked_edges[i].second.back().first == frameId ) 
			_tracked_edges[i].first++;
	}
}

// cleanup the tracker: when a track has less than 3 hits and is too old, trash it
//
void MyGlWindow::cleanupEdgeTracker()
{

	int counter = 0; // count the number of tracks removed

	bool done = _tracked_edges.empty();

	while ( !done ) {

		done = true;

		// remove empty tracks
		std::vector< std::pair< int, std::vector< std::pair< int, EdgePlane > > > >::iterator iter;

		for (iter = _tracked_edges.begin(); iter != _tracked_edges.end(); iter++) {

			if ( iter->second.empty() ) {

				_tracked_edges.erase( iter );

				counter++;
				done = false;
				break;
			}
		}

		if ( !done )
			continue;

		// remove track with less than some number of hits and older than some number of frames
		for (iter = _tracked_edges.begin(); iter != _tracked_edges.end(); iter++) {

			if ( iter->first < MIN_KEYFRAMES_PER_TRACK ) {

				if ( frameId - iter->second.back().first > MAX_TRACK_AGE ) {

					_tracked_edges.erase( iter );

					counter++;
					done = false;
					break;
				}
			}
		}
	}


	LOG(LEVEL_INFO, "edge tracker: removed %d tracks", counter);
}
	
// process tracked edges by refining both the camera poses and the model lines
//
void MyGlWindow::processTrackedEdges ()
{
	int i,j,k,m;

	ExtrinsicParameters save_pose = _camera->getPose();

	double residual = 0.0;

	// for each track, compute the model line by intersection (if hits > 3)
	std::vector< std::pair< int, Edge* > > lines;

	for (i=0;i<_tracked_edges.size();i++) {

		// if less than three hits, skip it
		if ( _tracked_edges[i].first < 3 )
			continue;

		// compute the intersection of the image edges to form a 3D model line
		edgePlaneVector edgeplanes;
		for (j=0;j<_keyframes.size();j++) {
			ExtrinsicParameters pose = _keyframes[j].second;
			int frameid = _keyframes[j].first;

			for (k=0;k<_tracked_edges[i].second.size();k++) {
				EdgePlane ep = _tracked_edges[i].second[k].second;
				if ( _tracked_edges[i].second[k].first == frameid ) {
					ep.fromCameraFrameToWorldFrame( pose );
					edgeplanes.push_back( ep );
				}
			}
		}

		Edge *line = new Edge();
		if ( !intersectEdgeplanesToLine( edgeplanes, line ) ) {
			delete line;
			continue;
		}

		// compute the residual
		residual = 0.0;
		for (j=0;j<edgeplanes.size();j++) {
			EdgePlane line_plane = EdgePlane( line->getA() - edgeplanes[j]._s,  line->getB() - edgeplanes[j]._s, edgeplanes[j]._s, 0, 0, 0 );
			residual += fabs( line_plane.angle( edgeplanes[j] ) ) / edgeplanes.size();
		}

		LOG(LEVEL_INFO, "residual: %f", residual);

		lines.push_back( std::pair< int, Edge* > (i, line) );
	}

	LOG(LEVEL_INFO, "created %d new lines", lines.size());

	// for each keyframe, refine the camera pose based on the previously calculated model lines
	for (i=0;i<_keyframes.size();i++) {

		CorrespondenceVector correspondences;

		for (j=0;j<_tracked_edges.size();j++) {

			bool found = false;

			for (k=0;k<lines.size();k++) {

				if ( lines[k].first == j ) {

					Edge *line = lines[k].second;
					
					for (m=0;m<_tracked_edges[j].second.size();m++) {

						if ( _tracked_edges[j].second[m].first == _keyframes[i].first ) {

							EdgePlane ep = _tracked_edges[j].second[m].second;

							correspondences.push_back( Correspondence ( line, ep ) );

							found = true;

							break;
						}
					}
				}

				if ( found )
					break;
			}
		}

		// refine the camera pose
		
		if ( correspondences.size() < 5 )
			continue;

		LOG(LEVEL_INFO, "refining camera pose [%d] at frame ID = %d from %d correspondences", i, _keyframes[i].first, correspondences.size());

		_camera->setPose( _keyframes[i].second );

		refineCameraPoseFromNCorrespondences( correspondences );

		_keyframes[i].second = _camera->getPose();
	}

	// add the lines in the model
	for (i=0;i<lines.size();i++) {
		
		Edge *line = lines[i].second;
	
		line->_keep = true;
		line->_id = _LUT._model->_edges.size();

		_LUT._model->_edges.push_back( line );
	}

	LOG( LEVEL_INFO, "added %d lines", lines.size() );

}

// write tracker data into a file
//
void MyGlWindow::WriteEdgeTracker ()
{
	int i,j;

	if ( _database == NULL ) 
		return;

	std::string filename = _database->_dirname + "/tracker.dat";

	FILE *fp = fopen( filename.c_str(), "w" );

	if ( fp == NULL )
		return;

	// write out the tracker edges

	for (i=0;i<_tracked_edges.size();i++) {

		fprintf( fp, "%d ", _tracked_edges[i].first ); // number of hits

		for (j=0;j<_tracked_edges[i].second.size();j++) {

			int frameid = _tracked_edges[i].second[j].first;
			EdgePlane ep = _tracked_edges[i].second[j].second;

			fprintf( fp, "%d %f %f %f %f %f %f %f %f %f %f %f %f %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f ", 
				frameid, ep._s[0], ep._s[1], ep._s[2], ep._a[0], ep._a[1], ep._a[2], ep._b[0], ep._b[1], ep._b[2], ep._normal[0], ep._normal[1], ep._normal[2], \
				ep._cameraId, ep._uid, ep._length, ep.l_avg[0], ep.l_avg[1], ep.l_avg[2], ep.r_avg[0], ep.r_avg[1], ep.r_avg[2], \
				ep.l_dev[0], ep.l_dev[1], ep.l_dev[2], ep.r_dev[0], ep.r_dev[1], ep.r_dev[2] );

		}

		fprintf( fp, "$\n" );
	}

	fclose( fp );

	LOG( LEVEL_INFO, "wrote %d tracks", _tracked_edges.size() );

	// write out the keyframes

	filename = _database->_dirname + "/keyframes.dat";

	fp = fopen( filename.c_str(), "w" );

	for (i=0;i<_keyframes.size();i++) {

		int frameid = _keyframes[i].first;
		Vec3d t = _keyframes[i].second.getTranslation();
		Quaternion r = _keyframes[i].second.getRotation();

		fprintf( fp, "%d %f %f %f %f %f %f %f\n", frameid, t[0], t[1], t[2], r._s, r._v[0], r._v[1], r._v[2] );
	}

	fclose( fp );

	LOG( LEVEL_INFO, "wrote %d keyframes", _keyframes.size() );
}

// read tracker data from file
//
void MyGlWindow::ReadEdgeTracker ()
{
	if ( _database == NULL ) 
		return;

	// clear the current tracker

	_tracked_edges.clear();

	int max_frame_id = 0;

	// read the tracker edges

	std::string filename = _database->_dirname + "/tracker.dat";

	FILE *fp = fopen( filename.c_str(), "r" );

	if ( fp == NULL )
		return;

	bool done = false;

	while ( !done ) {

		done = true;

		int hits = 0;

		if ( fscanf( fp, "%d", &hits ) == 1 ) {

			done = false;

			int frameid;
			Vec3d s,a,b,n;
			int cameraid, uid;
			double length;
			Vec3d l_avg, r_avg, l_dev, r_dev;

			std::vector< std::pair< int, EdgePlane > > vs;

			while ( fscanf( fp, "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf", &frameid, &s[0], &s[1], &s[2], &a[0], &a[1], &a[2], \
				&b[0], &b[1], &b[2], &n[0], &n[1], &n[2], &cameraid, &uid, &length, &l_avg[0], &l_avg[1], &l_avg[2], &r_avg[0], &r_avg[1], &r_avg[2], \
				&l_dev[0], &l_dev[1], &l_dev[2], &r_dev[0], &r_dev[1], &r_dev[2]) == 28 ) {

				EdgePlane ep;
				ep._a = a;
				ep._b = b;
				ep._s = s;
				ep._normal = n;
				ep._cameraId = cameraid;
				ep._uid = uid;
				ep._length = length;
				ep.l_avg = l_avg;
				ep.r_avg = r_avg;
				ep.l_dev = l_dev;
				ep.r_dev = r_dev;

				max_frame_id = MAX( max_frame_id, frameid );

				vs.push_back( std::pair< int, EdgePlane > ( frameid, ep ) );
			}

			_tracked_edges.push_back( std::pair< int, std::vector< std::pair< int, EdgePlane > > > ( hits, vs ) );

			char str[5];
			fscanf( fp, "%s", str); // read the string '$'
		}
	}

	LOG( LEVEL_INFO, "read %d tracks", _tracked_edges.size() );

	fclose( fp );

	// read the keyframes

	_keyframes.clear();

	filename = _database->_dirname + "/keyframes.dat";

	fp = fopen( filename.c_str(), "r" );
	
	if ( fp == NULL )
		return;

	int frameid;
	Vec3d t;
	Quaternion r;

	while ( fscanf( fp, "%d%lf%lf%lf%lf%lf%lf%lf", &frameid, &t[0], &t[1], &t[2], &r._s, &r._v[0], &r._v[1], &r._v[2] ) == 8 ) {
		
		ExtrinsicParameters pose;
		pose.setTranslation( t );
		pose.setRotation( r );

		_keyframes.push_back( std::pair < int, ExtrinsicParameters > ( frameid, pose ) );
	}

	LOG( LEVEL_INFO, "read %d keyframes (max frame ID = %d)", _keyframes.size(), max_frame_id );

	fclose( fp );

	// set the frame ID to the maximum frame ID found in the tracker
	frame_slider->value( max_frame_id, 3, 1, _database->_nImages-1 );

}
