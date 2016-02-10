#include "main.h"


// relock
void MyGlWindow::relock ( double max_dihedral_angle, int nruns )
{
	// lookup LUT
	_LUT.lookup( _camera->came.getTranslation(), MIN_SUBTENDED_ANGLE, 0.0, 100, 0);

	int i;

	printf("detecting edges\n");

	// detect edges
	detect_edges();

	// distribute visible edges into buckets
	std::vector< intVector > edges_buckets;
	distribute_edgeplanes_into_buckets( edges_buckets );

	// compute the region size on the tessellatio
	printf("computing region\n");

	int level = 1 + max_dihedral_angle / SPHERE_TESSELLATION_RESOLUTION;
	LOG(LEVEL_INFO, "region size: %d", level);

	int N = _LUT._lines.size();

	printf("converting lines\n");
	// convert the model lines into edgeplanes in the camera coordinate frame
	edgePlaneVector lines_edgeplanes;
	intVector line_ids;
	for (i=0;i<N;i++) {
		Vec3d a = _camera->fromWorldFrameToCameraFrame( _LUT._lines[i]->getA() );
		Vec3d b = _camera->fromWorldFrameToCameraFrame( _LUT._lines[i]->getB() );
		Vec3d s = Vec3d(0,0,0);
		lines_edgeplanes.push_back( EdgePlane( a, b, s, -1, _LUT._lines[i]->_id, i ) );
		line_ids.push_back( _LUT._lines[i]->_id );
	}

	corresVector correspondences;

	// clear the state machine
	SM_Clear();

	// for each model line, search the possible matches
	for (i=0;i<N;i++) {

		EdgePlane line = lines_edgeplanes[i];

		intVector cells;
		get_edgeplane_buckets( line, cells );

		// create a correspondence
		corres c ( _LUT._lines[i] );

		for (int m=0;m<cells.size();m++) {

			int bucket_id = cells[m];

			for (int j=0;j<edges_buckets[bucket_id].size();j++) {

				int edge_id = edges_buckets[bucket_id][j];

				EdgePlane edge;
				if (!_camera->_frame.get_edgeplane_chained( edge_id, edge ) ) {
					LOG(LEVEL_ERROR, "error accessing edgeplane %d out of %d edgeplanes.", edge_id, _camera->_frame.n_edgeplanes_chained );
					continue;
				}

				double angle = line.angle( edge );

				if ( angle > max_dihedral_angle ) 
					continue;

				c.eps.push_back( edge );
			}

			if ( c.valid() ) {
				correspondences.push_back( c );
				SM_InsertItem( c );
			}
		}
	}

	ExtrinsicParameters original_pose = _camera->getPose();
	ExtrinsicParameters best_pose = original_pose;
	double best_score = score_camera_pose( original_pose, edges_buckets );
	
	LOG(LEVEL_INFO, "start score: %f", best_score );
	
	// find the best camera pose possible
	for (int run=0; run < nruns; run++) {

		// reset camera pose
		_camera->setPose( original_pose );

		// draw random correspondences
		CorrespondenceVector cs;

		intVector indices;
		selectNRandomInt( 5/*INIT_MIN_CORRESPONDENCES*/, SM_Size(), indices);

		// for each correspondence, pick a choice randomly
		for (i=0;i<indices.size();i++) {
			Correspondence d;
			d.first = SM_Get(indices[i]).line;
			int p = MIN( SM_Get(indices[i]).eps.size()-1, (double)rand() / (RAND_MAX+1) * (SM_Get(indices[i]).eps.size()-1));
			d.second = SM_Get(indices[i]).eps[p];
			cs.push_back( d );
		}

		refineCameraPoseFromNCorrespondences( cs );

		double score = score_camera_pose( _camera->getPose(), edges_buckets );

		if ( score > best_score ) {

			best_score = score;
			best_pose = _camera->getPose();
		}	
	}

	// keep the best camera pose found so far
	_camera->setPose( best_pose );

	LOG(LEVEL_INFO, "new score: %f", best_score );

	// clear the state machine
	SM_Clear();

	// populate correspondences again
	correspondences.clear();
	init_correspondences( correspondences, edges_buckets, MAINTENANCE_DIHEDRAL_ANGLE, MAINTENANCE_MIN_OVERLAP, true );
}

// populate a set of correspondences
// correspondences are saved in the state machine if <store> is set to true
//
void MyGlWindow::init_correspondences( corresVector &correspondences, std::vector< intVector > &edges_buckets, double max_dihedral_angle, double min_overlap, bool store )
{
	int i,k, counter=0;

	// lookup LUT
	_LUT.lookup( _camera->came.getTranslation(), MIN_SUBTENDED_ANGLE, 0.0, 100, 0);

	int N = MIN(MAINTENANCE_MAX_CORRESPONDENCES ,_LUT._lines.size());
	int M = _camera->_frame.nedgeplanes_chained();

	LOG(LEVEL_INFO, "init correspondences: %d model lines and %d images edges", N, M);

	if ( store ) {
		SM_Clear();
		SM_ClearHistory();
	}

	correspondences.clear();

	// convert the model lines into edgeplanes in the camera coordinate frame
	edgePlaneVector lines_edgeplanes;
	intVector line_ids;
	for (i=0;i<N;i++) {
		Vec3d a = _camera->fromWorldFrameToCameraFrame( _LUT._lines[i]->getA() );
		Vec3d b = _camera->fromWorldFrameToCameraFrame( _LUT._lines[i]->getB() );
		Vec3d s = Vec3d(0,0,0);
		lines_edgeplanes.push_back( EdgePlane( a, b, s, -1, _LUT._lines[i]->_id, i ) );
		line_ids.push_back( _LUT._lines[i]->_id );
	}

	std::vector< intVector > matches; // each element is a list of edge IDs matching the corresponding line
	for (i=0;i<N;i++) {					// the first element is the local line ID
		intVector v;
		v.push_back( i );
		matches.push_back( v );
	}

	// for each model line, search the possible matches
	for (i=0;i<N;i++) {
		
		EdgePlane line = lines_edgeplanes[i];
		
		
		intVector bucket_ids;
		get_edgeplane_buckets( line, bucket_ids );
				
		// create a correspondence
		corres c ( _LUT._lines[i] );
		c.age = 11;
		c.line->status = UNKNOWN; // status of line is unknown
		
		double best_angle = M_PI;
		bool found = false;
		EdgePlane best_edge;
		
		for (k=0;k<bucket_ids.size();k++) {
			
			int bucket_id = bucket_ids[k];
			
			//LOG(LEVEL_INFO, "processing line %d (bucket: %d, has %d elements)", _LUT._lines[i]->_id, bucket_id, edges_buckets[bucket_id].size() );

			for (int j=0;j<edges_buckets[bucket_id].size();j++) {
				
				int edge_id = edges_buckets[bucket_id][j];
				
				EdgePlane edge;
				if (!_camera->_frame.get_edgeplane_chained( edge_id, edge ) ) {
					LOG(LEVEL_ERROR, "error accessing edgeplane %d out of %d edgeplanes.", edge_id, _camera->_frame.n_edgeplanes_chained );
					continue;
				}
				
				double angle = line.angle( edge );
				
				if ( edge.overlap( line ) < EPS && line.overlap( edge ) < EPS )
					continue;
				
				if ( edge.length() < toRadians(15.0) )
					continue;
				
				if ( angle < best_angle && angle < toRadians( 5.0 ) ) {
					
					best_angle = angle;
					
					best_edge = edge;
					
					found = true;
				}
				
				//c.eps.push_back( edge );
			}
		}

		if ( found ) {
			//LOG(LEVEL_INFO, "found a match");
			_camera->updateColor( best_edge, false );
			c.eps.push_back( best_edge );
			c.eid = 0;
			c.length = best_edge.length();
			c.line->status = ACCEPTED;
			counter++;
		} else {
			//LOG(LEVEL_INFO, "found no match");
		}

		correspondences.push_back( c );

		if ( store )
			SM_InsertItem( c );
	}

	// contraint geometry on correspondences
	//SM_VerifyCorrespondences( _camera->getPose() );

	ExtrinsicParameters pose = _camera->getPose();
	pose.id = frameId;

	LOG(LEVEL_INFO, "%d valid correspondences.", counter);

	pose_history.push_back( pose );
	pose_history.push_back( pose );
	pose_history.push_back( pose );
	pose_history.push_back( pose );
	pose_history.push_back( pose );
	pose_history.push_back( pose );
	pose_history.push_back( pose );
	pose_history.push_back( pose );
	pose_history.push_back( pose );
	pose_history.push_back( pose );
	pose_history.push_back( pose );

}


// populate the set of correspondences and ransac-refine the camera pose
//
double MyGlWindow::refine_camera_pose( std::vector< intVector > &edges_buckets )
{
	corresVector correspondences, correspondences_filtered;
	init_correspondences( correspondences, edges_buckets, SCORE_MAX_DIHEDRAL_ANGLE, INIT_REFINEMENT_OVERLAP, false );

	for (int i=0;i<correspondences.size();i++) {
		if ( correspondences[i].valid() )
			correspondences_filtered.push_back( correspondences[i] );
	}

	return refine_camera_pose( correspondences_filtered, edges_buckets );
}

// ransac-based refine camera pose based on set of correspondences
//
double MyGlWindow::refine_camera_pose( corresVector &correspondences, std::vector< intVector > &edges_buckets )
{
	ExtrinsicParameters best_pose = _camera->getPose();

	double best_score = score_camera_pose( best_pose, edges_buckets );

	int n = correspondences.size();

	int i,j,k;

	_line_matching_history.clear();
	_line_matching_pose_history.clear();

	if ( n < INIT_MIN_CORRESPONDENCES )
		return best_score;

	// for each run, draw a random subset of correspondences and compute the camera pose based on it
	for (int run = 0; run < 1000; run++) {

		CorrespondenceVector cs;

		intVector indices;

		selectNRandomInt( INIT_MIN_CORRESPONDENCES, n-1, indices ); // select random correspondences

		for (i=0;i<indices.size();i++) {

			corres c = correspondences[indices[i]];

			int index = MIN(c.eps.size()-1, int( (double)rand()/(RAND_MAX+1) * (c.eps.size()-1) )); // pick a random edge in each correspondence

			Correspondence d;
			d.first = c.line;
			d.second = c.eps[index];
			cs.push_back( d );
		}

		if ( cs.size() < INIT_MIN_CORRESPONDENCES )
			LOG(LEVEL_ERROR, "warning: number of correspondences mismatch: %d instead of %d", cs.size(), INIT_MIN_CORRESPONDENCES);

		// compute the coarse camera pose from three lines
		bool success = false;

		for (i=0;i<cs.size();i++) {
			for (j=i+1;j<cs.size();j++) {
				for (k=j+1;k<cs.size();k++) {

					ExtrinsicParameters pose = _camera->getPose();
					success = cameraPoseFromThreeCorrespondences( cs[i].second, cs[j].second, cs[k].second, cs[i].first, cs[j].first, cs[k].first, pose );
					if ( success ) {
						_camera->setPose( pose );
						break;
					}
				}

				if ( success ) break;
			}

			if ( success ) break;
		}

		// update the history (for debugging only)
		intVector lines_id_history;
		edgePlaneVector edgeplanes_history;

		for (k=0;k<cs.size();k++) {
			lines_id_history.push_back( cs[k].first->_id );
			edgeplanes_history.push_back( cs[k].second );
		}

		_line_matching_history.push_back( std::pair< intVector, edgePlaneVector > (lines_id_history, edgeplanes_history) );
		// end debugging

		find_inliers_and_refine_camera_pose( correspondences );

		_line_matching_pose_history.push_back( _camera->getPose() );

		double score = score_camera_pose( _camera->getPose(), edges_buckets );

		if ( score > best_score  ) {

			if ( _camera->getTranslation()[2] > 0.0 && _camera->getTranslation()[2] < INIT_MAX_CAMERA_HEIGHT && _LUT.valid_pose( _camera->getPose() ) ) {
				best_pose = _camera->getPose();
				best_score = score;
			}
		}
	}

	// refine the translation
	//refine_camera_translation( best_pose, _camera->getHeight()-_LUT.inchToModelUnit(20.0), _camera->getHeight()+_LUT.inchToModelUnit(20.0), edges_buckets);

	// set camera pose to best solution found so far
	_camera->setPose( best_pose );

	return best_score;

}

// find inliers given the current camera pose
// and refine camera pose based on it
//
void MyGlWindow::find_inliers_and_refine_camera_pose( corresVector &correspondences )
{

	CorrespondenceVector inliers;

	ExtrinsicParameters pose = _camera->getPose();

	// for each correspondence, determine whether it is an inlier
	for (int i=0;i<correspondences.size();i++) {

		corres c = correspondences[i];

		Edge *line = c.line;

		for (int j=0;j<c.eps.size();j++) {
	
			EdgePlane ep = c.eps[j];

			ep.fromCameraFrameToWorldFrame( pose );

			if ( ep.overlap( line ) > INIT_REFINEMENT_OVERLAP  && ep.angle( line ) < INIT_REFINEMENT_ANGLE ) {
				Correspondence d;
				d.first = line;
				d.second = c.eps[j];
				inliers.push_back( d );
				break;
			}
		}
	}

	// refine pose on inliers
	refineCameraPoseFromNCorrespondences( inliers );
}

bool MyGlWindow::update_correspondences()
{
	ExtrinsicParameters original_pose = _camera->getPose();

	Vec3d eps = get_expected_position(8);
	Quaternion epr = get_expected_rotation();

	//_camera->setTranslation( eps );

	// update correspondences
	SM_UpdateCorrespondences( );

	printf("correspondences updated [%d]\n", SM_Size());

	// enforce geometric constraints
	SM_VerifyCorrespondences( _camera->getPose() );

	int i,j,run;

	// quick hack for video -- remove asap
	/*for (i=0;i<pose_history.size();i++) {
		if ( pose_history[i].id == frameId ) {
			_camera->setPose( pose_history[i] );
			break;
		}
	}

	return true;*/
	// end quick hack

	//detect_edges();

	//std::vector< intVector > edges_buckets;
	//distribute_edgeplanes_into_buckets( edges_buckets );

	//ExtrinsicParameters pose = original_pose;
	write_correspondences( frameId );
	
	ExtrinsicParameters best_pose = original_pose;

	// keep only valid correspondences
	CorrespondenceVector ds;
	for (i=0;i<SM_Size();i++) {
		if ( SM_Get(i).valid() && SM_Get(i).age >= 4 &&  SM_Get(i).eid != -1) {
			Correspondence d;
			d.first = SM_Get(i).line;
			d.second = SM_Get(i).eps[SM_Get(i).eid];
			ds.push_back( d );
		}
	}

	int n = ds.size();

	if ( n < 5 ) {
		LOG(LEVEL_INFO, "too few correspondences to localize (%d).", n );

		return false;
	}

	_n_correspondences = n;

	LOG(LEVEL_INFO, "using %d correspondences for localization", ds.size());
	
	double max_distance = 2 * _LUT.inchToModelUnit( mmToInch(_camera->_max_translation_speed) );

	// RANSAC
	_point_cloud.clear();

	double best_penalty = 1E10;

	int size = INIT_MIN_CORRESPONDENCES;

	std::vector< ExtrinsicParameters > poses;
	double wweight = 0.0;
	//double nweight = 0.0;
	Vec3d average_position = Vec3d(0,0,0);

	for (run=0;run<6000;run++) {

		_camera->setPose( original_pose );

		CorrespondenceVector set;

		intVector indices;

		selectNRandomInt( size, ds.size(), indices );

		for (i=0;i<indices.size();i++) {
			Correspondence d = ds[indices[i]];

			set.push_back( d );
		}

		refineCameraPoseFromNCorrespondences( set );

		if ( len(_camera->getTranslation() - original_pose.getTranslation()) > max_distance ) 
			continue;

		_point_cloud.push_back( _camera->getTranslation() );
		
		poses.push_back( _camera->getPose() );

		// keep pose that scores the best
		double penalty = 0.0;
		int counter = 0;

		Vec3d center = _camera->getTranslation();

		for (i=0;i<n;i++) {

			bool used = false;
			for (j=0;j<indices.size();j++) {
				if ( indices[j] == i ) {
					used = true;
					break;
				}
			}

			if ( used )
				continue;

			counter++;

			Edge *line = ds[i].first;
			Vec3d a = line->getA()-center;
			Vec3d b = line->getB()-center;
			EdgePlane edgeplane = EdgePlane( a, b, center, 0, 0, 0 );
			edgeplane.fromWorldFrameToCameraFrame( _camera->getPose() );
		
			penalty += fabs( edgeplane.angle( ds[i].second ) );
		}

		if ( counter > 0 ) {
			penalty /= counter;
		} else {
			continue;
		}

		if ( penalty > EPS ) {
			// update the average
			average_position += 1.0 / penalty * _camera->getTranslation();
			wweight += 1.0 / penalty;
			//printf("weight = %f\n", wweight);
		}

		if ( penalty < best_penalty ) {

			best_penalty = penalty;

			best_pose = _camera->getPose();
		}

		//_camera->getPose().print();
	}

	_camera->setPose( best_pose );
	
	//if ( wweight > EPS ) {
	//	printf("weight = %f\n", wweight);
	//	_camera->setTranslation( average_position / wweight );
	//}

	double dd = len(_camera->getTranslation() - original_pose.getTranslation());

	if ( dd > max_distance ) {
		_camera->setPose( original_pose );
		return true;
	}
			
	return true;
}

void MyGlWindow::refine_from_accepted_correspondences ()
{
	ExtrinsicParameters original_pose = _camera->getPose();

	double max_distance = _LUT.inchToModelUnit( mmToInch(_camera->_max_translation_speed) );

	CorrespondenceVector ds;
	for (int i=0;i<SM_Size();i++) {
		if ( SM_Get(i).valid() && SM_Get(i).line->status == ACCEPTED &&  SM_Get(i).eid != -1) {
			Correspondence d;
			d.first = SM_Get(i).line;
			d.second = SM_Get(i).eps[SM_Get(i).eid];
			ds.push_back( d );
		}
	}

	int n = ds.size();

	if ( n < 15 ) {
		return;
	} else {

		LOG(LEVEL_INFO, "refining from accepted correspondences (%d).", n );
	}

	refineCameraPoseFromNCorrespondences( ds );

	if ( len( _camera->getTranslation() - original_pose.getTranslation()) > max_distance )
		_camera->setPose( original_pose );
}

// init the camera pose
// find the combination of sm correspondences that maximizes the score
//
void MyGlWindow::update_init_pose( std::vector< intVector > &edges_buckets )
{
	ExtrinsicParameters original_pose = _camera->getPose();

	ExtrinsicParameters best_pose = original_pose;

	double best_score = -1.0;

	int n = SM_Size();

	intVector best_eids;

	for (int run=0;run<1000;run++) {

		_camera->setPose( original_pose );

		// pick a guess randomly for each correspondence
		CorrespondenceVector set;

		intVector eids;

		for (int i=0;i<n;i++) {
			corres cc = SM_Get(i);
			Correspondence c;
			c.first = cc.line;
			int p = MIN(cc.eps.size()-1, int((double)rand()/(RAND_MAX+1) * (cc.eps.size()-1)));
			c.second = cc.eps[p];
			eids.push_back( p );

			set.push_back( c );
		}

		refineCameraPoseFromNCorrespondences( set );

		double score = score_camera_pose( _camera->getPose(), edges_buckets );

		if ( score > best_score ) {

			best_score = score;

			best_pose = _camera->getPose();

			best_eids = eids;

		}
	}

	if ( !best_eids.empty() ) {
		for (int i=0;i<n;i++)
			SM_SetId( i, best_eids[i] );
	}

	_camera->setPose( best_pose );
}

// define adjacency rules
//
void MyGlWindow::define_adjacency_rules( std::vector< std::pair< std::pair<int,int>, std::pair<int,int> > > &rules )
{
	rules.clear();

	for (int i=0;i<SM_Size();i++) {

		corres c_i = SM_Get(i);

		EdgePlane line_i = _camera->fromWorldFrameToCameraFrame( c_i.line );

		for (int j=0;j<SM_Size();j++) {

			if ( i == j )
				continue;

			corres c_j = SM_Get(j);
		
			EdgePlane line_j = _camera->fromWorldFrameToCameraFrame( c_j.line );

			if ( line_i.angle( line_j ) > SPHERE_TESSELLATION_RESOLUTION )
				continue;

			if ( line_i.overlap( line_j ) < EPS && line_j.overlap( line_i ) < EPS )
				continue;

			Vec3d n1 = line_i._normal;
			Vec3d n2 = line_j._normal;

			Vec3d n = cross(n1,n2);
			if ( len(n) < EPS )
				continue;

			n = norm(n);

			double angle_i_j = Acos(dot(n1,n2));
			
			if ( M_PI - angle_i_j < angle_i_j ) {
				n2 = -n2;
				angle_i_j = M_PI - angle_i_j;
			}

			if ( dot(n,cross(n1,n2)) < 0)
				angle_i_j = - angle_i_j;

			for (int ii = 0; ii < c_i.eps.size(); ii++) {

				Vec3d n_i = c_i.eps[ii]._normal;

				double best_score = M_PI;

				int best_jj = -1;

				for (int jj=0;jj < c_j.eps.size(); jj++) {

					Vec3d n_j = c_j.eps[jj]._normal;

					double angle_ii_jj = Acos(dot(n_i,n_j));
					
					if ( M_PI - angle_ii_jj < angle_ii_jj ) {
						n_j = -n_j;
						angle_ii_jj = M_PI - angle_ii_jj;
					}

					if ( dot(n,cross(n_i,n_j)) < 0)
						angle_ii_jj = - angle_ii_jj;
					
					double score = angle_ii_jj*angle_i_j > 0 ? fabs(angle_ii_jj - angle_i_j) : M_PI;

					if ( score < best_score ) {

						best_score = score;
						best_jj = jj;
					}
				}

				if ( best_jj == -1 )
					continue;

				std::pair<int, int> p1;
				p1.first = i;
				p1.second = ii;
				std::pair<int, int> p2;
				p2.first = j;
				p2.second = best_jj;

				std::pair< std::pair<int,int>, std::pair<int,int> > element;
				element.first = p1;
				element.second = p2;
	
				rules.push_back( element );
			}
		}
	}

	// mutual consistency check
	std::vector< std::pair< std::pair<int,int>, std::pair<int,int> > > consistent_rules;

	for (std::vector< std::pair< std::pair<int,int>, std::pair<int,int> > >::iterator iter = rules.begin(); iter != rules.end(); iter++) {

		bool ok = false;

		for (std::vector< std::pair< std::pair<int,int>, std::pair<int,int> > >::iterator iter2 = rules.begin(); iter2 != rules.end(); iter2++) {

			if ( iter2->first.first == iter->second.first && iter2->first.second == iter->second.second && 
				iter2->second.first == iter->first.first && iter2->second.second == iter->first.second ) {
				ok = true;
				break;
			}
		}

		if ( ok ) {

			std::pair< std::pair<int,int>, std::pair<int,int> > element = *iter;

			std::pair< std::pair<int,int>, std::pair<int,int> > transpose;

			transpose.first.first = element.second.first;
			transpose.first.second = element.second.second;
			transpose.second.first = element.first.first;
			transpose.second.second = element.first.second;

			consistent_rules.push_back( element );
		
			LOG(LEVEL_INFO, "introducing pair (%d,%d) (%d,%d)", element.first.first, element.first.second, element.second.first, element.second.second );
		}
	}

	rules.clear();

	rules = consistent_rules;

	LOG(LEVEL_INFO, "rules done");
}

// return the average camera position over the last n frames
//
Vec3d MyGlWindow::get_average_position( int n )
{

	Vec3d t = Vec3d(0,0,0);

	int counter = 0;
	for (int i=pose_history.size()-1;i>=MAX(0,pose_history.size()-n);i--) {
		t = t + pose_history[i].getTranslation();
		counter++;
	}

	if ( counter > 0 ) 
		t = t / counter;

	return t;
}


Vec3d MyGlWindow::get_expected_position ( int n )
{
	if ( pose_history.size() < n )
		return _camera->getTranslation();

	// compute the average direction vector
	Vec3d direction_vector = pose_history[pose_history.size()-5].getTranslation() - pose_history[pose_history.size()-n].getTranslation();

	// return the expected position
	return pose_history[pose_history.size()-1].getTranslation() + direction_vector / (n-5);
}

Quaternion MyGlWindow::get_expected_rotation( ) 
{
	if ( pose_history.empty() )
		exit(0);

	if ( pose_history.size() == 1)
		return pose_history[0].getRotation();

	int p = pose_history.size();

	Quaternion q1 = pose_history[p-2].getRotation();
	Quaternion q2 = pose_history[p-1].getRotation();

	return q2 * q1.bar() * q2;
}

// write out the correspondences for the current frame
void
MyGlWindow::write_correspondences ( int frameid ) 
{
	char filename[256];
	sprintf(filename,"correspondences/%d.dat", frameid);

	FILE *f = fopen( filename, "a" );
	if ( !f )
		return;
	
	int i;
	for (i=0;i<SM_Size();i++) {
		if ( SM_Get(i).valid() &&  SM_Get(i).eid != -1) {
			Edge *line = SM_Get(i).line;
			EdgePlane ep = SM_Get(i).eps[SM_Get(i).eid];
			
			fprintf(f, "%d %d %d %f %f %f %f %f %f %f %f %f ", line->_id, (int)line->status, ep._cameraId, 
				ep._s[0], ep._s[1], ep._s[2], ep._a[0], ep._a[1], ep._a[2], ep._b[0], ep._b[1], ep._b[2]);
		}
	}	
	
	fclose (f);
}

// read a correspondence in history
bool
MyGlWindow::read_correspondence ( int frameid, int lineid, EdgePlane *ep )
{
	char filename[256];
	sprintf(filename, "correspondences/%d.dat", frameid);
	
	FILE *f = fopen( filename, "r" );
	
	if ( !f )
		return false;
	
	double s0,s1,s2,a0,a1,a2,b0,b1,b2;
	int camera_id, line_status,line_id;
	
	while ( fscanf(f, "%d%d%d%lf%lf%lf%lf%lf%lf%lf%lf%lf", &line_id, &line_status, &camera_id,
		&s0, &s1, &s2, &a0, &a1, &a2, &b0, &b1, &b2 ) == 12 ) {
		
		if ( line_id == lineid ) {
			ep->_a = Vec3d(a0,a1,a2);
			ep->_b = Vec3d(b0,b1,b2);
			ep->_s = Vec3d(s0,s1,s2);
			ep->_cameraId = camera_id;		
			
			ep->_normal = norm(cross(norm(ep->_a),norm(ep->_b)));
			ep->_uid = 0;
			ep->_length = ep->length();
			
			fclose(f);
			
			return true;
		}
	}
	
	fclose (f);
	return false;
}


// update the history of line states for the current frame id
//
void MyGlWindow::updateLineHistory()
{
	std::vector< std::pair< int, int > > states;

	for (int i=0;i<_LUT._lines.size();i++) {

		std::pair<int, int> element;
		element.first = _LUT._lines[i]->_id;

		switch ( _LUT._lines[i]->status ) {
		case ACCEPTED:
			element.second = 0;	
			break;
		case PENDING:
			element.second = 1;	
			break;
		case UNKNOWN:
			element.second = 2;	
			break;
		default:
			LOG(LEVEL_ERROR, "inconsistency in history of line [%d]. status = %d", _LUT._lines[i]->_id, _LUT._lines[i]->status);
			element.second = 2;
			break;
		}

		states.push_back( element );
	}

	bool done = false;

	for (i=0;i<_lines_history.size();i++) {

		if ( _lines_history[i].first == frameId ) {

			_lines_history[i].second = states;
			done = true;
			break;
		}
	}

	if ( !done )
		_lines_history.push_back( std::pair< int, std::vector< std::pair< int, int > > > (frameId, states) );
}

// update the visible lines along with their status according to the history
//
bool MyGlWindow::getLineHistory()
{

	_LUT._lines.clear();

	for (int i=0;i< _lines_history.size(); i++) {

		if ( _lines_history[i].first != frameId )
			continue;

		for (int j=0;j<_lines_history[i].second.size();j++) {
	
			Edge *line = _LUT._model->getEdge( _lines_history[i].second[j].first );

			if ( line != NULL ) {

				switch ( _lines_history[i].second[j].second ) {
				case 0:
					line->status = ACCEPTED;
					break;
				case 1:
					line->status = PENDING;
					break;
				case 2:
					line->status = UNKNOWN;
					break;
				default:
					line->status = UNKNOWN;
					break;
				}

				if ( line->_id == 2333 ) {

					FILE *f1 = fopen( "line_2333.dat", "a");
					fprintf(f1,"%d\n",line->status);
					fclose(f1);
				} else 
				if ( line->_id == 2286 ) {

					FILE *f1 = fopen( "line_2286.dat", "a");
					fprintf(f1,"%d\n",line->status + 3);
					fclose(f1);
				}

				_LUT._lines.push_back( line );
			}
		}

		return true;
	}

	return false;
}

// write the line states history in a local file
//
void MyGlWindow::writeLineHistory()
{
	FILE *f = fopen("lines.dat", "w");

	if ( f == NULL )
		return;

	for (int i=0;i<_lines_history.size();i++) {
	
			fprintf(f, "%d $ ", _lines_history[i].first );

			for (int j=0;j<_lines_history[i].second.size();j++) {

				fprintf(f, "%d %d ", _lines_history[i].second[j].first, _lines_history[i].second[j].second);
			}

			fprintf(f, "$\n");

	}

	fclose( f );
}

// read the line states history file from the database
//
void MyGlWindow::readLineHistory ()
{
	if ( _database == NULL )
		return;

	std::string filename = _database->_dirname + "/lines.dat";

	FILE *f = fopen( filename.c_str(), "r" );

	if ( f == NULL )
		return;

	while ( !feof( f ) ) {

		int frameid;
		char str[5];

		if ( fscanf( f, "%d", &frameid ) == 1 ) {

			std::vector< std::pair< int, int > > states;

			
			fscanf( f, "%s", str); // read the string '$'

			int id, status;

			while ( fscanf( f, "%d%d", &id, &status ) == 2 ) {

				states.push_back( std::pair< int, int > (id, status ) );
			}

			fscanf( f, "%s", str); // read the string '$'

			_lines_history.push_back( std::pair< int, std::vector< std::pair< int, int > > > (frameid, states) );
		}		
	}

	LOG(LEVEL_INFO, "read %d line history", _lines_history.size());

	fclose( f );
}


// clear the line history for the specified frame
void MyGlWindow::clearLineHistory( int frameid)
{
	for (int i=0;i< _lines_history.size(); i++) {

		if ( _lines_history[i].first != frameid )
			continue;

		 _lines_history[i].second.clear();

		break;
	}
}



