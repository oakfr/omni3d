#include "state.h"

////////////////////////////////////////////////////
// correspondence state machine
//

void SM_Init( MyGlWindow *ptr, int max_n_correspondences, int max_n_frames)
{
	LOG(LEVEL_INFO, "init history: max items [%d] time frame [%d]", max_n_correspondences, max_n_frames );

	w = ptr;
	index = 0;
	_max_n_correspondences = max_n_correspondences;
	_max_n_frames = max_n_frames;

	// init the history
	for (int i=0;i<max_n_correspondences;i++) {
		std::vector< corres > vv;
		history.push_back( vv );
	}

	for (i=0;i<max_n_correspondences;i++)
		empty_seats.push_back( i );
}

void SM_ProcessItems( )
{
	int counter = 0;
	for ( corresVector::iterator iter = cq.begin(); iter != cq.end(); iter++, counter++ ) {
		corres c = *iter;
		SM_ProcessItem( c );
		*iter = c;
	}
}

// process a correspondence - state machine
// rule 1: if EXPECTED for too long, move to BLACKLISTED
// rule 2: if BLACKLISTED for some period of time, move to EXPECTED again
// rule 3: if EXPECTED and valid, move to CONNECTED
// rule 4: if CONNECTED and invalid, move to EXPECTED
//
void SM_ProcessItem( corres &c )
{
	// rule 1: if EXPECTED for too long, move to BLACKLISTED
	if ( ( c.status() == _EXPECTED ) && (c.age > EXPECTED_TIMEOUT) ) {
		c.status( _BLACKLISTED );
		c.clear();
		return;
	}

	// rule 2: if BLACKLISTED for some period of time, move to EXPECTED again
	if ( ( c.status() == _BLACKLISTED ) && (c.age > BLACKLISTED_TIMEOUT) ) {
		c.status( _EXPECTED );
		c.clear();
		return;
	} 

	// rule 3: if EXPECTED and valid, move to CONNECTED
	if ( ( c.status() == _BLACKLISTED ) && (c.valid()) ) {
		c.status( _CONNECTED );
		return;
	} 

	// rule 4: if CONNECTED and invalid, move to EXPECTED
	if ( ( c.status() == _CONNECTED ) && (!c.valid()) ) {
		c.status( _EXPECTED );
		c.clear();
		return;
	} 

	c.age++;
}

void SM_RemoveItem ( corres &c ) 
{
	for ( corresVector::iterator iter = cq.begin(); iter != cq.end(); iter++ ) {
		if (c._id == iter->_id) {
			cq.erase( iter );
			return;
		}
	}

	LOG( LEVEL_ERROR, "failed to remove item in the queue." );
	assert( false );
}

void SM_RemoveItem( Edge *line )
{
	for ( corresVector::iterator iter = cq.begin(); iter != cq.end(); iter++ ) {
		if (iter->line == line) {
			cq.erase( iter );
			return;
		}
	}
}

// brute-force insert a correspondence in the queue
void SM_InsertItem ( corres &c ) 
{
	if ( empty_seats.empty() )
		return;

	int id = empty_seats.back();

	empty_seats.pop_back();

	c._id = id;
	c.notseen = 0;

	cq.push_back( c );

	history[id].push_back( c );
}

// brute-force insert a correspondence in the queue
void SM_InsertItem( Correspondence &c )
{
	SM_InsertItem( _EXPECTED, c.second, c.first, 0 );
}

// insert a correspondence in the queue
void SM_InsertItem( corresStatus status, EdgePlane ep, Edge *line, int age )
{
	corres c= corres( line, ep, status );
	c._id = index;
	c.age = age;
	index++;
	cq.push_back( c );
}

// insert a correspondence in the queue (with line == NULL)
void SM_InsertItem( corresStatus status, EdgePlane &ep )
{
	corres c = corres( NULL, ep, status );
	c._id = index;
	index++;
	cq.push_back( c );
}

// insert a correspondence in the queue
// avoid duplicates (each model line has at most one correspondence)
void SM_InsertItem( corresStatus status, Edge *line )
{
	assert( line != NULL );

	for (int i=0;i<cq.size();i++) {
		if ( cq[i].line->_id == line->_id ) {
			LOG(LEVEL_INFO, "trying to insert line that already exists %d", line->_id);
			assert(false);
		}
	}

	cq.push_back( corres( line, EdgePlane(), status ) );
}
bool SM_get_correspondence( int p, Correspondence &c )
{ 
	if ( p < cq.size() ) { c.first = cq[p].line; c.second = cq[p].eps[0]; return true; } else return false;
}

int SM_Size()
{
	return cq.size();
}

corres SM_Get( int p )
{
	assert( p < SM_Size() );
	return cq[p];
}

void SM_Print()
{
	LOG( LEVEL_DEBUG, "******  Queue: %d items *******", cq.size() );
	for (corresVector::iterator iter = cq.begin(); iter != cq.end(); iter++ )
		iter->print();
	LOG( LEVEL_DEBUG, "************* %d ******************", cq.size() );
}

void SM_Clear()
{
	cq.clear();
}

void SM_SetId( int i, int eid )
{
	if ( i >= cq.size() ) 
		LOG(LEVEL_ERROR, "access error %d out of %d", i, cq.size());
	else
		cq[i].eid = eid;
}

// for each correspondence, update the correspondence
//
void SM_UpdateCorrespondences( )
{
	for (int i=0;i<cq.size();i++) {
		corres c = SM_Get(i);
		corres c_copy = SM_Get(i);

		bool test = w->PROJ_TEST( c, true, false, CLUSTER_MAX_ANGLE_THRESHOLD );

		//printf("e6\n");
	
		cq[i] = c;

		//printf("e7\n");
	}

	// sort correspondences
	std::sort( cq.begin(), cq.end() );
}

void SM_VerifyCorrespondences( ExtrinsicParameters &pose )
{
	// 1. if two model lines get assigned to the same image edge, one correspondence must go
	int i,j;
	Vec3d center = pose.getTranslation();

	// for each pair of valid correspondences
	bool done = false;

	while (!done) {
		
		done = true;

		for (i=0;i<cq.size();i++) {
			
			if ( !cq[i].valid() )
				continue;
			
			for (j=i+1;j<cq.size();j++) {
				
				if ( !cq[j].valid() )
					continue;
				
				// if the two correspondences have the same image edge...
				if ( cq[i].eps[cq[i].eid]._uid == cq[j].eps[cq[j].eid]._uid ) {
					
					done = false;

					EdgePlane edge = cq[i].eps[cq[i].eid];
					
					EdgePlane line_i, line_j;
					Vec3d a = cq[i].line->getA()-center;
					Vec3d b = cq[i].line->getB()-center;
					
					// make a synthetic edge plane for model line 1
					line_i = EdgePlane( a, b, center, 0, 0, 0 );
					line_i.fromWorldFrameToCameraFrame( pose );
					
					a = cq[j].line->getA()-center;
					b = cq[j].line->getB()-center;
					
					// make a synthetic edge plane for model line 2
					line_j = EdgePlane( a, b, center, 0, 0, 0 );
					line_j.fromWorldFrameToCameraFrame( pose );
					
					// cancel one of the two correspondences
					if ( edge.angle( line_i ) < edge.angle( line_j ) ) {
						cq[j].eps.clear();
						cq[j].eid = -1;
						cq[j].length = 0.0;
					} else {
						cq[i].eps.clear();
						cq[i].eid = -1;
						cq[i].length = 0.0;
					}

					break;
				}
			}

			if ( !done )
				break;
		}
	}
	
	return;

	// 2. if two close lines are assigned to two close edges, enforce ordering
	done = false;

	while (!done) {
		
		done = true;

		for (i=0;i<cq.size();i++) {
			
			if ( !cq[i].valid() )
				continue;
			
			for (j=i+1;j<cq.size();j++) {
				
				if ( !cq[j].valid() )
					continue;

				EdgePlane ei = cq[i].eps[cq[i].eid];
				EdgePlane ej = cq[j].eps[cq[j].eid];

				if ( ei.angle( ej ) > toRadians( 10.0 ) ) 
					continue;

				if ( ei.overlap( ej ) < EPS || ej.overlap( ei ) < EPS )
					continue;

				// flip normals if needed
				if ( dot(ei._normal, ej._normal) < 0 ) 
					ej._normal = -ej._normal;

				EdgePlane line_i, line_j;
				Vec3d a = cq[i].line->getA()-center;
				Vec3d b = cq[i].line->getB()-center;
				
				// make a synthetic edge plane for model line 1
				line_i = EdgePlane( a, b, center, 0, 0, 0 );
				line_i.fromWorldFrameToCameraFrame( pose );
				
				a = cq[j].line->getA()-center;
				b = cq[j].line->getB()-center;
				
				// make a synthetic edge plane for model line 2
				line_j = EdgePlane( a, b, center, 0, 0, 0 );
				line_j.fromWorldFrameToCameraFrame( pose );

				// check that the swap is possible
				if ( ej.overlap( line_i ) < 0.5 || ei.overlap( line_j ) < 0.5 )
					continue;

				// flip normals if needed
				if ( dot(line_i._normal, ei._normal) < 0 ) 
					line_i._normal = -line_i._normal;

				if ( dot(line_j._normal, ei._normal) < 0 ) 
					line_j._normal = -line_j._normal;

				// swap correspondences
				if ( dot(cross(ei._normal,ej._normal), cross(line_i._normal, line_j._normal)) < 0.0 ) {
					LOG(LEVEL_INFO, "flipping correspondences %d and %d", i, j);
					cq[i].eps[cq[i].eid] = ej;
					cq[j].eps[cq[j].eid] = ei;
				}
			}
		}
	}

}

void SM_UpdateHistory()
{
	for (int i=0;i<cq.size();i++) {
		// update the history
		corres c = SM_Get(i);
		history[c._id].push_back( c );
		if ( history[c._id].size() > _max_n_frames )
			history[c._id].erase( history[c._id].begin() );
	}
}

// report statistics
//
void SM_Statistics( int frameid, ExtrinsicParameters &pose )
{
	int i;

	// open the file
	FILE *f = fopen( "loca_stats.dat", "a" );

	if ( f == NULL ) {
		LOG(LEVEL_ERROR, "could not open file loca_stats.dat!");
		return;
	}

	// compute the distribution of correspondences
	double accepted=0.0, pending=0.0, unknown=0.0;

	int n = cq.size();

	for (i=0;i<n;i++) {
		if (cq[i].line == NULL)
			continue;
		if (cq[i].line->status == ACCEPTED)
			accepted += 1.0 / n;
		if (cq[i].line->status == PENDING)
			pending += 1.0 / n;
		if (cq[i].line->status == UNKNOWN)
			unknown += 1.0 / n;
	}

	// compute the average and standard deviation of angular error for accepted correspondences
	doubleVector angles;
	Vec3d center = pose.getTranslation();

	for (i=0;i<n;i++) {
		if ( cq[i].line == NULL )
			continue;
		if ( cq[i].line->status != ACCEPTED )
			continue;

		Edge *line = cq[i].line;
		Vec3d a = line->getA()-center;
		Vec3d b = line->getB()-center;
		EdgePlane selected_edgeplane = cq[i].eps[cq[i].eid];
		EdgePlane edgeplane = EdgePlane( a, b, center, selected_edgeplane._cameraId, 0, 0 );
		edgeplane.fromWorldFrameToCameraFrame( pose );
		double angle = toDegrees( edgeplane.angle( selected_edgeplane ) );

		angles.push_back( angle );
	}

	int n_accepted = angles.size();

	double average = 0.0;
	double stdev = 0.0;

	if ( n_accepted > 0 ) {

		for (i=0;i<angles.size();i++)
			average += angles[i] / n_accepted;

		for (i=0;i<angles.size();i++)
			stdev += ( angles[i] - average ) * ( angles[i] - average );

		stdev = sqrt( stdev ) / n_accepted;
	}

	// write the stats into the file
	fprintf(f, "%d %d %.1f %.1f %.1f %.2f %.2f\n", frameid, n, 100.0 * accepted, 100.0 * pending, 100.0 * unknown, average, stdev);

	fclose(f);
}

void SM_UpdateHistory2( ExtrinsicParameters &pose )
{
	Vec3d center = pose.getTranslation();

	// update each correspondence
	for (int i=0;i<cq.size();i++) {

		if ( cq[i].age > 0 )
			cq[i].age--;
		
		if ( !cq[i].valid() ) {
			cq[i].notseen++;
			cq[i].line->status = UNKNOWN;
		}

		if ( cq[i].valid() ) {

			Edge *line = cq[i].line;
			Vec3d a = line->getA()-center;
			Vec3d b = line->getB()-center;
			EdgePlane selected_edgeplane = cq[i].eps[cq[i].eid];
			EdgePlane edgeplane = EdgePlane( a, b, center, selected_edgeplane._cameraId, 0, 0 );
			edgeplane.fromWorldFrameToCameraFrame( pose );
			double angle = edgeplane.angle( selected_edgeplane );
			if ( angle > toRadians( 5.0 ) ) {
				SM_Clear(i);
				cq[i].notseen++;
				LOG(LEVEL_INFO, "[%d][%d] valid but too far (angle = %.3f deg.) [%d,%d]", i, cq[i].line->_id, toDegrees(angle), cq[i].age, cq[i].notseen);
				cq[i].line->status = UNKNOWN;
			} else {
				cq[i].notseen = 0;
				cq[i].age++;
				LOG(LEVEL_INFO, "[%d][%d] valid and continues (angle = %.3f deg.) [%d,%d]", i, cq[i].line->_id, toDegrees(angle), cq[i].age, cq[i].notseen);
				if ( cq[i].age > 4 )
					cq[i].line->status = ACCEPTED;
			}
		}
		if ( cq[i].notseen > 0 && cq[i].notseen < 4 ) {
			int p = history[cq[i]._id].size();
			if ( p-cq[i].notseen-1 >= 0 ) {
				cq[i] = history[cq[i]._id][p-cq[i].notseen-1];
				cq[i].notseen++;
				LOG(LEVEL_INFO, "[%d][%d] not seen but recovered in history [%d,%d]", i, cq[i].line->_id, cq[i].age, cq[i].notseen);
				cq[i].line->status = PENDING;
			} else {
				LOG(LEVEL_INFO, "[%d][%d] not seen and not recovered in history [%d,%d]", i, cq[i].line->_id, cq[i].age, cq[i].notseen);
				cq[i].line->status = UNKNOWN;
			}
		}
		if ( !cq[i].valid() && cq[i].notseen >= 4 ) {
			SM_Clear(i);
			cq[i].notseen++;
			LOG(LEVEL_INFO, "[%d][%d] invalid and not seen for too long [%d,%d]", i, cq[i].line->_id, cq[i].age, cq[i].notseen);
			cq[i].line->status = UNKNOWN;
		}
	}
}

void SM_Clear(int i) 
{
	if ( i < cq.size() ) {

		cq[i].eps.clear();
		cq[i].eid = -1;
		cq[i].age = 0;
	}
}

// return the last element in history for the given line ID
//
bool SM_GetHistory( int line_id, corres &c ) 
{
	for (int i=0;i<history.size();i++) {

		if ( history[i].empty() )
			continue;

		if ( history[i][0].line->_id == line_id ) {
			c = history[i].back();
			return true;
		}
	}

	return false;

}

void SM_ClearHistory()
{
	for (int i=0;i<history.size();i++)
		history[i].clear();

	for (i=0;i<_max_n_correspondences;i++)
		empty_seats.push_back( i );
}

// RANSAC computation of the camera pose from a set of correspondences KS
// each correspondence consists of a line and a set of possible image matches (edgeplanes)
// at each step:
// - select a random set of correspondences S
// - within the subset S, run a RANSAC inner loop to select the top inliers S2
// - compute the camera pose from S2 and score the remaining correspondences (KS - S2)
// - if the score is higher than some threshold, accept; otherwise, continue.
// correspondences which are BLACKLISTED are not taken into account for this calculation
// no change is made on the correspondence state at the end of the computation - this is all up to the state machine
// note that the size of S should be about 15 correspondences
// note that, for performance issues, correspondences are not all updated at the beginning
// instead, they are updated on the fly
bool SM_ComputePose(int pose_max_elements, int pose_min_elements, int pose_max_trials, int pose_threshold_score, corresVector &cs, CorrespondenceVector &inliers)
{
	// save the current camera pose
	ExtrinsicParameters pose = w->_camera->getPose();
	
	PerfTimer timer;

	// initialize a list of light correspondence structures
	std::vector<cr_pose> correspondences;
	int i,k;
	int counter=0;
	for (corresVector::iterator iter = cs.begin(); iter != cs.end(); iter++,counter++) {
		if ( iter->status() != _BLACKLISTED ) {
			cr_pose item = cr_pose( counter, 0, (iter->status() == _CONNECTED) ? 0 : 1, 0, iter->age); 
			correspondences.push_back( item );
		}
	}
	
	int N = correspondences.size();
	int n = MIN( N, POSE_MAX_ELEMENTS);
	
	if ( N < POSE_MIN_ELEMENTS ) // if too few correspondences, quit
		return false;
	
	// iterate until (1) ransac success or (2) max number of trials reached or (3) all possible solutions tried
	int trial = 0;
	double best_score = -1.0;
	ExtrinsicParameters best_pose = pose;
	
	while ( 1 ) {
		
		LOG(LEVEL_INFO,"trial %d", trial);
		
		// exit if max number of trials reached
		if ( trial > POSE_MAX_TRIALS )
			break;

		// reset the camera pose (to be done before any call to PROJ_TEST!)
		w->_camera->setPose( pose );
		
		// shuffle randomly
		std::random_shuffle( correspondences.begin(), correspondences.end() );

		// select the first n valid elements
		corresVector subset; // this is the subset of correspondences that has been selected

		int marker = 0;
		while ( subset.size() < n && marker < correspondences.size() ) {
			Correspondence c;
			if ( correspondences[marker].d == 0 ) { // if the correspondence has not been computed yet, compute it
				corres cc = cs[correspondences[marker].i];
				assert ( cc.line != NULL );
				correspondences[marker].k = w->PROJ_TEST( cc, true, false, CLUSTER_MAX_ANGLE_THRESHOLD ) ? 0 : 1;
				cs[correspondences[marker].i] = cc;
				correspondences[marker].j = cs[correspondences[marker].i].eps.size();
				correspondences[marker].d = 1;
			}
			corres d = cs[correspondences[marker].i];
			
			if ( d.valid() ) { // update the subset with this correspondence 
				subset.push_back( d );
			}
			marker++;
		}
		
		// exit if the table is not complete -- it means that not many correspondences are available so we quit
		//if ( subset.size() < n )  { 
		//	LOG(LEVEL_INFO,"table incomplete ( %d ) . exiting.", subset.size());
		//	break;
		//}
		
		// from the subset S of n correspondences, run a RANSAC inner loop to find the top inliers

		// initialize the counters
		intVector counters, max_counters;
		double max_score = -1.0;
		intVector best_indices;

		for (i=0;i<subset.size();i++) {
			counters.push_back(0);
			max_counters.push_back(subset[i].eps.size());
		}

		inliers.clear();

		for (k=0;k<300;k++) {

			// randomly select the number of correspondences to take
			int p = MIN( subset.size(), POSE_MIN_ELEMENTS - 1 + (double)rand() / (RAND_MAX+1) * (n - POSE_MIN_ELEMENTS + 1));
			//LOG(LEVEL_INFO, "selecting %d elements ( %d elements in the subset)\n", p, subset.size());
			
			// select a random subset of one-to-one correspondences
			CorrespondenceVector v;
			intVector indices;
			selectNRandomInt( p, subset.size(), indices);
			//LOG(LEVEL_INFO, "selected elements out of %d:", subset.size());
			//for (int pp=0;pp<indices.size();pp++)
				//LOG(LEVEL_INFO,"%d",indices[pp]);

			for (i=0;i<indices.size();i++) {
				Correspondence d;
				d.first = subset[indices[i]].line;
				int q = MIN( subset[indices[i]].eps.size(), (double)rand() / (RAND_MAX+1) * subset[indices[i]].eps.size() );
				//LOG(LEVEL_INFO, "element %d: selected %d edgeplane / %d", indices[i], q, subset[indices[i]].eps.size());

				d.second = subset[indices[i]].eps[q];
				v.push_back( d );
			}

			// refine camera pose from selected correspondences
			//LOG(LEVEL_INFO, "refining camera pose...");
			w->_camera->setPose( pose );
			w->refineCameraPoseFromNCorrespondences( v );

			// IDEA: inject ideal position to check!!!!!!!!!!!!!!
			//if ( k == 0 ) {
			//	w->_camera->setPose( w->GetSyntheticPose( w->frameId );
			//	v.clear();
			//	for (i=0;i<subset.size();i++) {
			//LOG(LEVEL_INFO, "done.");

			// compute the residuals
			double score = 0.0;
			for (i=0;i<indices.size();i++) {
				EdgePlane ep = v[i].second;
				score += subset[indices[i]].computeScore( w->_camera->getPose() );
				//ep.fromCameraFrameToWorldFrame( w->_camera->getPose() );
				//double angle = ep.angle( v[i].first );
				//penalty += angle;
			}

			//penalty /= indices.size();
			score /= indices.size();
			//LOG(LEVEL_INFO, "score = %f", score);

			// if better, keep camera pose
			if ( score > max_score ) {
				best_pose = w->_camera->getPose();
				max_score = score;
				inliers = v;
				best_indices = indices;
			}
		}

		// keep the best pose
		if ( max_score > 0.0 ) 
			w->_camera->setPose( best_pose );
		else
			continue;

		// for each inlier, find the best match
		inliers.clear();
		for (i=0;i<best_indices.size();i++) {
			Correspondence d;
			d.first = subset[best_indices[i]].line;
			subset[best_indices[i]].computeScore( w->_camera->getPose() ); // find the best match
			if ( subset[best_indices[i]].best_ep != -1 ) {
				d.second = subset[best_indices[i]].eps[subset[best_indices[i]].best_ep];
				inliers.push_back( d );
			}
		}

		w->_camera->setPose( pose );
		w->refineCameraPoseFromNCorrespondences( inliers );

		// update the trial counter
		trial++;
		//timer.print("pose: refine pose");
		
		// sanity check on speed (rotation and translation)
		if ( !w->_camera->testCameraMotion( pose ) ) {
			//LOG(LEVEL_INFO,"camera motion test failed.");
			continue;
		}
		
		// score the corresponding pose according to the rest of the correspondences
		double score = (marker >= correspondences.size()) ? 1.0 : 0.0;

		for (i=marker;i<correspondences.size();i++) {
			corres d = cs[correspondences[i].i];
			assert( d.line != NULL );

			if ( w->PROJ_TEST( d, true, false, POSE_MAX_ANGLE_THRESHOLD ) ) // project the line and look for a match
				score += 1.0 / (correspondences.size() - marker); 

			if ( ( score > POSE_THRESHOLD_SCORE ) && (score > best_score) ) // if beat the threshold, exit the loop
				break;
		}
		//LOG(LEVEL_INFO, "score: %f", score);
		//timer.print("pose: scoring");

		// keep score if better
		if ( score > best_score ) {
			best_score = score;
			best_pose = w->_camera->getPose();
		}
						  
		// exit if threshold reached
		if ( score > POSE_THRESHOLD_SCORE ) 
			break;
								  
		// cleanup the correspondence table
		correspondences.erase( std::remove_if(correspondences.begin(), correspondences.end(), cr_pose_invalid), correspondences.end() ); 
		
	}
	
	w->_camera->setPose( best_pose );
	//timer.print("finish");

	return best_score;
}

// this method has been developed to allow the state machine to modify its own list of correspondences <cq>
// since <cq> is not accessible from the outside
//
bool SM_ComputePose(int pose_max_elements, int pose_min_elements, int pose_max_trials, int pose_threshold_score, CorrespondenceVector &inliers)
{
	return SM_ComputePose( pose_max_elements, pose_min_elements, pose_max_trials, pose_threshold_score, cq, inliers );
}

// update the model lines according to the current observations
// use only CONNECTED correspondences having a certain age
void SM_UpdateModel()
{
}

// remove deprecated lines and insert new ones
// at this point: if a line is to be removed, its state is REMOVED
// all new lines have state UNKNOWN
// in a first step, the function removes all correspondences for REMOVED lines
// then it swaps through all existing correspondences and set the line flag to ACCEPTED
// then it swaps through the lines of the LUT and creates a new correspondence for each UNKNOWN one
//
void SM_NodeChanged( edgeVector &lines )
{
	int i, j;
	bool exist;

	// 1. search for lines that disappeared and generate new seats

	for (i=0;i<history.size();i++) {

		// if the seat is empty, make sure that it is in the <empty_seat> list
		if ( history[i].empty() ) {

			exist = false;
			for (j=0;j<empty_seats.size();j++) {
				if ( empty_seats[j] == i ) {
					exist = true;
					break;
				}
			}
			if ( !exist ) {
				empty_seats.push_back( i );
			}

			continue;
		}

		// otherwise, if the line is not in the list, empty the seat
		Edge *line = history[i][0].line;

		if ( line == NULL ) {
			LOG(LEVEL_INFO, "inconsistency in the history. exiting...");
			return;
		}

		exist = false;
		for (j=0;j<lines.size();j++) {
			if ( lines[j]->_id == line->_id ) {
				exist =true;
				break;
			}
		}

		if ( exist ) // the line is still being tracked so we don't remove it from the history
			continue;

		// otherwise we empty the seat in history and create a new seat
		LOG(LEVEL_INFO, "removing line %d from history", line->_id);

		line->status = UNKNOWN;

		history[i].clear();
		empty_seats.push_back( i );
	}

	//2. for each line not being tracked, assign a new seat if possible
	for (i=0;i<lines.size();i++) {

		exist = false;

		for (j=0;j<history.size();j++) {
			if ( history[j].empty() )
				continue;
			Edge *line = history[j][0].line;

			if ( line == NULL ) {
				LOG(LEVEL_INFO, "inconsistency in the history. exiting...");
				return;
			}

			if ( line->_id == lines[i]->_id ) {
				exist = true;
				break;
			}
		}

		// if the line is already being tracked, skip it
		if ( exist )
			continue;

		// if there are no more seats available, exit
		if ( empty_seats.empty() ) {
			lines[i]->status = UNKNOWN;
			break;
		}

		// otherwise, assign to a seat
		corres c( lines[i] );

		lines[i]->status = UNKNOWN;

		LOG(LEVEL_INFO, "adding line %d to history", lines[i]->_id);

		SM_InsertItem( c );
	}

	// update the history
	SM_UpdateHistory();

	// regenerate the queue
	cq.clear();	

	for (i=0;i<history.size();i++) {

		if ( !history[i].empty() )
			cq.push_back( history[i].back() );
	}
}

// compute the distribution of correspondence statuses in this order:
// connected, paused, observed, expected, blacklisted, alternate, silent
void SM_ComputeDistribution( intVector &dist )
{
	// reset the distribution
	dist.clear();
	for (int i=0;i<3;i++)
		dist.push_back( 0 );

	// populate the distribution
	for ( corresVector::iterator iter = cq.begin(); iter != cq.end(); iter++ ) {
		switch ( iter->status() ) {
		case _CONNECTED:
			dist[0]++;
			break;
		case _EXPECTED:
			dist[1]++;
			break;
		case _BLACKLISTED:
			dist[2]++;
			break;
		default:
			assert( false );
		}
	}
}
