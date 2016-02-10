#include "main.h"

// this is the main INIT algorithm
// try various algorithms and store the results in the init_poses vector
//
void MyGlWindow::init_pose()
{
	double initial_height = _camera->getTranslation()[2]; // for debugging only

	// compute edges, corners and correspondences from corners
	// detect and chain edges
	//if ( _database == NULL) {
	//	edgePlaneVector edgeplanes;
	//	initIdealCorrespondences( edgeplanes );}
	//else {

	LOG(LEVEL_INFO, "hello");
	std::vector< intVector > edges_buckets;
	
	detect_edges();
	//}
		
	// lookup visibility set
	_LUT.lookup( _camera->came.getTranslation(), 0.0, 0.0, 100, _init_search_depth );
	
	// distribute visible edges into buckets
	LOG(LEVEL_INFO, "distributing edges...");
	distribute_edgeplanes_into_buckets( edges_buckets );
	LOG(LEVEL_INFO, "done.");
	
	// compute corners
	//LOG(LEVEL_INFO, "computing corners");

	// uncomment this line to use corners
	_camera->computeCorners( frameId, FTRACKER_MIN_DIST, FTRACKER_MIN_CORNER_ANGLE  );
	_corner_lines.clear();
	//LOG(LEVEL_INFO, "done.");

	// find the best camera rotation possible
	nodeVector neighbors;
	_LUT.findNodeNeighbors( _LUT._node, _init_search_depth, neighbors, true );

	LOG(LEVEL_INFO, "scoring pose");
	double score = -1.0;
	double best_score = score_camera_pose(_camera->getPose(), edges_buckets);
	ExtrinsicParameters best_pose = _camera->getPose();

	LOG(LEVEL_INFO, "Initial score: %.2f", best_score );
	
	// clear the list of poses
	init_poses.clear();

	// for each init method...
	
	for (int i=0;i<initModes.size();i++) {
		
		InitMode mode = initModes[i];

		// for each node, set the camera pose at different height and run the algorithm
		for (nodeVector::iterator iter = neighbors.begin(); iter != neighbors.end(); iter++ ) {
				
			Node *node = *iter;

			LOG(LEVEL_INFO, "Trying method %d on node %d", mode, node->_id );
			
			_LUT._node = node;

			// lookup visibility set
			_LUT.lookup( node->_pos, 0.0, 0.0, 100, _init_search_depth );

			double height_step = INIT_MAX_CAMERA_HEIGHT / INIT_HEIGHT_STEPS;

			for (double height = 0.0; height <= INIT_MAX_CAMERA_HEIGHT; height += height_step) {
				
				//if ( height > initial_height || height + height_step < initial_height )
				//	continue;

				//height = initial_height - height_step / 2.0;

				// reset the dihedral books
				_LUT.clear_dihedral_books();
	
				// set the camera pose
				_camera->setRotation( Quaternion() );
				_camera->setTranslation( node->_pos );
				_camera->setHeight( height + height_step / 2.0);
				
				// clear the state machine history
				SM_ClearHistory();

				// compute camera pose from rotation voting
				switch ( mode ) {
				case LINE_LINE:
		//			for (k=0;k<100000;k++)
						//compute_pose_from_line_matching( 20.0, 50.0, edges_buckets );
						compute_pose_from_line_matching( height, height + height_step, edges_buckets );
					break;
				case ROTATION_VOTING:
			//		for (k=0;k<100000;k++)
						compute_pose_from_rotation_voting( height, height + height_step, edges_buckets );
					break;
				case CORNERS:
				//	for (k=0;k<100000;k++)
						compute_pose_from_corners( _camera->_frame._corners, _corner_lines );
					break;
				case VANISHING_POINTS:
				//	for (k=0;k<100000;k++)
						compute_pose_from_vps( height, height + height_step, edges_buckets );
					break;
				case EXHAUSTIVE_SEARCH:
				//	for (k=0;k<100000;k++)
						maximize_score( edges_buckets );
					break;
				case INIT_VERTICAL:
					compute_pose_vertical( edges_buckets );
					break;
				default:
					LOG(LEVEL_ERROR, "init mode not supported");
					break;
				}
				
				// score poses by decreasing score
				sort_init_poses();

				// resize if needed
				if ( init_poses.size() > 100 )
					init_poses.resize( 100 );

				if ( init_poses.empty() )
					continue;

				// set the camera pose to the best pose found so far
				_camera->setPose( init_poses[0] );

				// score
				score = score_camera_pose(_camera->getPose(), edges_buckets);
				
				if ( score > best_score ) {
					
					best_score = score;
					best_pose = _camera->getPose();
				}

				if ( mode == INIT_VERTICAL || mode == CORNERS || mode == EXHAUSTIVE_SEARCH) // force loop to finish because this method does not depend on height
					break;
			
				if ( !init->value() ) // stop if user decided so
					break;
			}
			if ( !init->value() ) // stop if user decided so
					break;
		}
		if ( !init->value() ) // stop if user decided so
			break;
	}
	
	// print out best pose
	best_pose.print();

	// set the camera pose to the best pose found so far
	_camera->setPose( best_pose );
	LOG(LEVEL_INFO, "Score before refinement: %f", best_score );

	// refine the camera pose
	refine_camera_pose( edges_buckets );

	// rescore
	best_score = score_camera_pose(_camera->getPose(), edges_buckets);
	LOG(LEVEL_INFO, "Score after refinement: %f", best_score );

	// print out new pose
	_camera->getPose().print();
}

// recover accurate camera pose given a coarse estimate
// use initInitialCorrespondences() to detect N edges and computeInitialCorrespondences() to match them with the set of visible lines
// then use RANSAC to recover the camera pose
// the algorithm keeps the topk edges for each line
//
double MyGlWindow::compute_pose_from_rotation_voting( double h1, double h2, std::vector< intVector > &edges_buckets )
{

	readConfig();

	// lookup the LUT for the current camera pose estimate
	_LUT.lookup( _camera->came.getTranslation(),MIN_SUBTENDED_ANGLE, 0.0, 100, 0);

	edgePlaneVector edgeplanes;
	_ideal_rotation = Vec3d(0,0,0);

	//if ( _database != NULL ) { 	// create real correspondences from observations
	initInitialCorrespondences( edgeplanes );
	//} else {  // for debugging, create a set of ideal correspondences		
	//	_camera->setRotation(Quaternion());
	//	double random_angle = (double)rand()/RAND_MAX * M_PI; // randomly rotate camera
	//	_camera->rotate( Vec3d(0,0,1), random_angle);
	//	LOG(LEVEL_INFO, "rotated camera by %f deg.", toDegrees(random_angle));
	//	Quaternion( Vec3d(0,0,1), random_angle ).toAxisAngle( _ideal_rotation );

	//	initIdealCorrespondences( edgeplanes );
	//	distribute_edgeplanes_into_buckets( edges_buckets );
//
//	}

		// diam is the uncertainty on the camera position ( e.g ~ 50 inches )
	//double diam = _LUT.GRID_STEP; 

	// compute the initial correspondences between edges and lines
	corresVector correspondences;
	std::vector<Vec4d> rotations;
	compute_initial_correspondences_voting( edgeplanes, _LUT._lines, h1, h2, INIT_TOPK, correspondences, rotations );

	if ( rotations.empty() )
		return -1.0;

	// compute the top-k candidate rotations from the set of rotations
	std::vector<Vec3d> best_rotations;
	computeBestRotations( rotations, INIT_TOPK_ROTATIONS, INIT_SEARCH_RADIUS_ROTATION, best_rotations );

	std::vector< std::pair< double, int > > book;
	std::vector<ExtrinsicParameters> temp_poses;

	double best_score = -1.0;

	for (int i=0;i<best_rotations.size();i++) {

		ExtrinsicParameters pose;
		pose.setTranslation( _camera->getTranslation() );
		pose.setRotation( Quaternion( best_rotations[i], len( best_rotations[i] ) ) );
		pose.set_mode( ROTATION_VOTING );

		std::pair< double, int > element;
		element.second = i;
		element.first = refine_camera_translation( pose, h1, h2, edges_buckets );
		book.push_back( element );
	
		pose.set_score( element.first );

		temp_poses.push_back( pose );
		
		best_score = MAX( best_score, element.first );
	}

	//for (i=0;i<book.size();i++)
	//	LOG(LEVEL_INFO, "book %d: score = %f", i, book[i].first);

	std::sort( book.begin(), book.end() );

	for (i=0;i<book.size();i++)
		LOG(LEVEL_INFO, "book %d: score = %f", i, book[i].first);

	std::vector< ExtrinsicParameters> best_poses;

	for (i=book.size()-1; i >=0 ; i-- ) {

		ExtrinsicParameters pose = temp_poses[book[i].second];

		insert_init_pose( pose );

		if ( _LUT.valid_pose( pose ) ) {

			if ( best_poses.empty() )
				best_poses.push_back( pose );
		}
	}

	// set the camera to the best pose found so far
	if ( !best_poses.empty() ) 
		_camera->setPose( best_poses[0] );

	return ( best_score );



	// update the state machine with correspondences
	//SM_Clear();
	//for (int i=0;i<correspondences.size();i++) 
	//	SM_InsertItem( correspondences[i] );

	// run a RANSAC algorithm to recover camera pose
	// cameraPoseFromCorrespondences( correspondences );

	// code for vanishing points approach
	/*
	// compute the LUT vanishing points
	_LUT.computeVanishingPoints();

  // compute the observed vanishing points
	computeVanishingPoints( edgeplanes, _vps );

	// cluster the vanishing points
	clusterVanishingPoints( _vps, _vpc, _LUT._vps.size() );

	LOG(LEVEL_INFO, "%d observed VPs", _vpc.size());
	for (i=0;i<_vpc.size();i++) {
		LOG(LEVEL_INFO,"VP %d: %f %f %f ( length = %f)", _vpc[i].second, _vpc[i].first[0],_vpc[i].first[1], _vpc[i].first[2], len(_vpc[i].first));
	}

	// find the best camera rotation
	alignSetsOfVanishingPoints( _LUT._vps, _vpc, _solutions);
	if ( !_solutions.empty() )
		_camera->setRotation( _solutions[0].getRotation() );

	select_pose->bounds(0,_solutions.size()-1);

	// rotate the vps
	for ( i=0;i<_vps.size();i++) {
		_vps[i].first = _camera->getRotation().rotate( _vps[i].first );
	}

	for ( i=0;i<_vpc.size();i++) {
		_vpc[i].first = _camera->getRotation().rotate( _vpc[i].first );
	}
	return;
	*/

	// RANSAC : compute positions from a set of (presumably) good correspondences
	//ransacCameraPose(300, 0.5);
}

// init a set of ideal correspondences
// this function is used for debugging only
// N is the maximum number of edgeplanes
//
void MyGlWindow::initIdealCorrespondences( edgePlaneVector &edgeplanes )
{
	edgeplanes.clear();

	bool occlusion = false;
	bool clutter = false;
	bool noise = false;

	// simulate a random camera position within the allowed sphere (diameter = INIT_DIAMETER )
	//Vec3d translation = _camera->getTranslation();
	//Vec3d direction = norm(Vec3d((double)rand()/(RAND_MAX+1),(double)rand()/(RAND_MAX+1),(double)rand()/(RAND_MAX+1)));
	//double r = (double)rand()/(RAND_MAX+1) * INIT_DIAMETER / 2.0;
	//_camera->setTranslation( translation + r * direction );

	LOG(LEVEL_INFO, "init: occlusion [%d] clutter [%d] noise [%d]", INIT_OCCLUSION, INIT_CLUTTER, INIT_NOISE );

	LOG(LEVEL_INFO, "[%d] model lines", _LUT._lines.size());

	// for each line, create an ideal edgeplane
	for (int i=0;i<_LUT._lines.size();i++) {
		Edge *line = _LUT._lines[i];

		if ( INIT_OCCLUSION && (double)rand() / RAND_MAX < 0.3 )
			continue;

		EdgePlane edgeplane;
		_camera->makeIdealEdgeplane( line, edgeplane );
		edgeplanes.push_back( edgeplane );
	}

	if (INIT_NOISE) {
		for ( int i=0;i<edgeplanes.size();i++) {
			double angle = toRadians( 2.0 * ( ran1( &_idum ) - 0.5 ) * NOISE_SYNTHETIC_ANGLE_STDEV + NOISE_SYNTHETIC_ANGLE_AVERAGE );
			Vec3d axis = norm( ( edgeplanes[i]._a + edgeplanes[i]._b ) / 2.0 );
			edgeplanes[i].rotate( axis, angle );
			angle = toRadians( 2.0 * ( ran1( &_idum ) - 0.5 ) * NOISE_SYNTHETIC_ANGLE_STDEV + NOISE_SYNTHETIC_ANGLE_AVERAGE );
			axis = norm( edgeplanes[i]._b - edgeplanes[i]._a );
			edgeplanes[i].rotate( axis, angle );
		}
	}

	if ( INIT_CLUTTER ) {
		intVector indices;
		selectNRandomInt(_LUT._lines.size()/2, _LUT._lines.size(), indices);
		for (i=0;i<indices.size();i++) {
			int cameraId = int( ran1( &_idum ) * 6 );
			EdgePlane ep = EdgePlane( Vec3d(ran1( &_idum ),ran1( &_idum ),ran1( &_idum )), Vec3d(ran1( &_idum ),ran1( &_idum ),ran1( &_idum )), \
				_camera->getUnitCameraCenter(cameraId), cameraId, 0, 0 );

			edgeplanes[indices[i]] = ep ;
		}
	}

	// resize the list of edgeplanes if needed
	//if ( edgeplanes.size() > N )
	//	edgeplanes.resize( N );

	// put the edgeplanes in the camera structure
	_camera->clearEdges();
	_camera->_frame.n_edgeplanes_chained = 0;
	for (i=0;i<edgeplanes.size();i++) {
		EdgePlane ep = edgeplanes[i];
		_camera->_frame._edgeplanes_chained[ep._cameraId].push_back(ep);
		_camera->_frame.n_edgeplanes_chained++;
	}

	// restore camera pose
	//_camera->setTranslation( translation );
}

// generate a set of N initial edgeplanes to be used in computeInitialCorrespondences()
// edgeplanes are expressed in the camera coordinate frame
//
void MyGlWindow::initInitialCorrespondences( edgePlaneVector &edgeplanes )
{
	// detect and chain edges
	//_camera->clearEdges();
	//_camera->clearMasks();
	//_camera->initializeEdgeDetectors();
	//_camera->detectLines ( true,false,edgePixels->value(),edgeThresh->value(),edgeSigma->value());
	//_camera->convertLinesToPlanes();
	//_camera->_frame.chainEdges( 0 );  // we keep all chained edges

	// clear the output vector
	edgeplanes.clear();

	// fill in the edgeplane vector
	for (int i=0;i<_camera->_frame._edgeplanes_chained.size();i++) {
		for (int j=0;j<_camera->_frame._edgeplanes_chained[i].size();j++) {
			edgeplanes.push_back( _camera->_frame._edgeplanes_chained[i][j] );
		}
	}

	return;

	// filter edgeplanes (on minimum subtended angle, dihedral angles, and finally topk length)
	/*
	_camera->_frame.filterEdges( edgeplanes, 0.0, toRadians(2.0), N );
	LOG(LEVEL_INFO, "# edges: %d", edgeplanes.size());

	// repopulate the camera frame
	for (i=0;i<_camera->_frame._edgeplanes_chained.size();i++) {
		_camera->_frame._edgeplanes_chained[i].clear();

		for (int j=0;j<edgeplanes.size();j++) {
			if ( edgeplanes[j]._cameraId == i )
				_camera->_frame._edgeplanes_chained[i].push_back( edgeplanes[j] );
		}

		LOG(LEVEL_INFO, "camera %d: %d edges", i, _camera->_frame._edgeplanes_chained[i].size() );
	}
	*/
}

// compute a set of probable correspondences between edges and lines, given:
// a set of N observed edges <edgeplanes> expressed in the camera coordinate frame
// a set of M visible lines <lines> expressed in the world coordinate frame
// an estimate of the camera position encoded in the node, with h1 < height < h2
// output <correspondences>
// this algorithm works by computing the signature of every pair of observed edges and looking for possible matches in pairs of lines
// the algorithm keeps the top-K edges for each model line (topK ~ 5)
//
// note: N edges => N^2 pairs, M lines => M^2 pairs ==> N^2 x M^2 votes.  (N=20, M=20, 160,000 votes)
// optimization: dihedral angles for pairs of edges and pairs of lines are precomputed in a separate table
// this algorithm also computes the list of candidate camera rotations and store is into <rotations>
// each rotation is assigned a weight, hence the Vec4d structure
//
void MyGlWindow::compute_initial_correspondences_voting ( edgePlaneVector &edgeplanes, edgeVector &lines, double h1, double h2, int topk, corresVector &correspondences, 
												std::vector<Vec4d> &rotations )
{
	// first brute-force compute the min and max dihedral angles for each triplet of model lines
	// disregard lines which are closer to the camera position than 3 times the size of <diam>
	int i=0, j=0, k=0, q=0;
	int N = edgeplanes.size();
	int M = lines.size();

	Vec3d position = _LUT._node->_pos;
	position[2] = (h1+h2)/2.0;

	// optimization: precompute the min and max dihedral angle (and max polar angle) in a separate table
	double **book_min;
	double **book_max;
	double **book_max_polar;
	book_min = new double*[M]; //(double**)malloc(M*sizeof(double*));
	book_max = new double*[M]; //(double**)malloc(M*sizeof(double*));
	book_max_polar = new double*[M]; //(double**)malloc(M*sizeof(double*));

	for (j=0;j<M;j++) {
		book_min[j] = new double[M]; //(double*)malloc(M*sizeof(double));
		book_max[j] = new double[M]; //(double*)malloc(M*sizeof(double));
		book_max_polar[j] = new double[M]; //(double*)malloc(M*sizeof(double));
	}

	double min_angle = 0.0, max_angle = 0.0, max_polar_angle = 0.0;

	for (j=0;j<M;j++) {

		for (k=j+1;k<M;k++) {
			_LUT.min_max_dihedral_angle( lines[j], lines[k], h1, h2, min_angle, max_angle, max_polar_angle);

			book_min[j][k] = min_angle;
			book_max[j][k] = max_angle;
			book_max_polar[j][k] = max_polar_angle;

			if ( max_angle - min_angle > 0.5 ) {
				book_min[j][k] = 0.0;
				book_max[j][k] = 0.0;
			}
		}
	}
	
	Fl::check();
	if ( !init->value() )
		return;

	// second, brute-force compute the dihedral angle for each pair of image edges

	// optimization: precompute the dihedral angles in a separate table
	double **book;
	book = new double*[N]; //(double**)malloc(N*sizeof(double*));
	double **book_polar_edge;
	book_polar_edge = new double*[N]; //(double**)malloc(N*sizeof(double*));
	for (i=0;i<N;i++) {
		book[i] = new double[N]; //(double*)malloc(N*sizeof(double));
		book_polar_edge[i] = new double[N];
	}

	for (i=0;i<N;i++) {
		for (k=i+1;k<N;k++) {
			book[i][k] = edgeplanes[i].angle( edgeplanes[k] );
			book_polar_edge[i][k] = maxPolarAngle( edgeplanes[i]._a, edgeplanes[i]._b, edgeplanes[k]._a, edgeplanes[k]._b );
		}
	}

	// compute a NxM compatibility table based on subtended angle: (edge < line)
	int **compatibility = new int*[N];
	for (i=0;i<N;i++)
		compatibility[i] = new int[M];
		
	double *sub_angle_lines = new double[M]; // compute and store the min and max subtended angle for each model line
	for (j=0;j<M;j++) {
		double min_angle = 0.0, max_angle = 0.0;
		_LUT.max_subtended_angle( lines[j], h1, h2, max_angle);
		//minMaxSubtendedAngle( lines[j], position, diam, min_angle, max_angle );
		sub_angle_lines[j] = max_angle;
	}

	for (i=0;i<N;i++) {
		double edge_angle = edgeplanes[i].length();

		for (j=0;j<M;j++) {
			if ( sub_angle_lines[j] * 1.05 < edge_angle ) // allow a 10% error
				compatibility[i][j] = 0;
			else
				compatibility[i][j] = 1;
		}
	}

	// create all edge triplets
	i=0,j=0,k=0;

	// optimization: precompute the match between pairs of lines and pairs of edges
	// for each pair of edges (e1,e2) and pair of lines (l1,l2), a probability function is computed
	// using a gaussian distribution for the edges and a pdf for the lines

	double edge_width = toRadians( 1.0 ); // width of the uniform distribution for a pair of edges

	double **book_pairs = new double*[N*N]; //(double**)malloc(N*N*sizeof(double*));
	for (i=0;i<N*N;i++) 
		book_pairs[i] = new double[M*M]; //(double*)malloc(M*M*sizeof(double));

	double coeff = 1.0; // / (NOISE_SYNTHETIC_ANGLE_STDEV * sqrt(2*M_PI));

	Fl::check();
	if ( !init->value() )
		return;

	// histogram for the minimum rotations


	for (i=0;i<180;i++)
		hist_min_rot[i] = 0;

	FILE *fh = fopen( "rotations.dat", "w" );

	int i1=0,i2=0,i3=0;
	int j1=0,j2=0,j3=0;

	LOG(LEVEL_INFO, "# votes = %d x %d = %d", N*(N-1), M*(M-1), N*(N-1) * M*(M-1));

	for (i1=0;i1<N;i1++) {
		for (i2=i1+1;i2<N;i2++) {
			double angle = book[i1][i2]; // dihedral angle between edge i1 and edge i2
			double min_angle_edge = MAX(0.0, angle - edge_width);
			double max_angle_edge = MIN(M_PI,angle + edge_width);

			if ( fabs(max_angle_edge-min_angle_edge) < 1E-7 )
				continue;

			for (j1=0;j1<M;j1++) {
				for (j2=j1+1;j2<M;j2++) {

					//printf("%d %d / %d\n", i, j, M);

					double min_angle_line = book_min[j1][j2]; // min and max dihedral angle between line j1 and line j2
					double max_angle_line = book_max[j1][j2];

					if ( fabs(min_angle_line-max_angle_line) < 1E-7 )
						continue;

					if ( max_angle_edge < min_angle_line || max_angle_line < min_angle_edge ) { // no overlap, probability = zero
						//book_pairs[i1*N+i2][j1*M+j2] =  0.0;
						continue;
					}
					if ( compatibility[i1][j1] == 0 && compatibility[i1][j2] == 0 ) { // if each edge is incompatible with the two lines, probability = zero
						//book_pairs[i1*N+i2][j1*M+j2] = 0.0;
						continue;
					}
					if ( compatibility[i2][j1] == 0 && compatibility[i2][j2] == 0 ) {
						//book_pairs[i1*N+i2][j1*M+j2] = 0.0;
						continue;
					}

					if ( book_polar_edge[i1][i2] > 1.05 * book_max_polar[j1][j2] ) { // if the max polar angle of the edge pair is larger than the 
						//book_pairs[i1*N+i2][j1*M+j2] = 0.0;					// one of the line pair, probability = zero  (error 5%)
						continue;
					}
					
					double end = MIN(max_angle_edge, max_angle_line);
					double start = MAX(min_angle_edge, min_angle_line);

					double res = 0.0;
					/*
					for (k=0;k<samples;k++) { // compute the product of the gaussian (edge) and pdf (line)
						double x = start + (end-start) * k / samples;
						double p_edge = coeff * exp (-(x-angle)*(x-angle)/(2*NOISE_SYNTHETIC_ANGLE_STDEV*NOISE_SYNTHETIC_ANGLE_STDEV)); // edge gaussian
						double p_line = book_sampling[j1][j2][k];
						res += p_edge * p_line;
					}
					*/

					switch ( INIT_SCORING_MODE ) {
					case 0 : 
						res = 1.0;
						break;
					case 1:
						res = (end-start) * (end - start ) / ( (max_angle_edge - min_angle_edge) * (max_angle_line - min_angle_line) );
						break;
					case 2:
						LOG(LEVEL_INFO, "this mode is not supported anymore!");
						res = res * book_polar_edge[i1][i2] / book_max_polar[j1][j2];
						break;
					}

					//book_pairs[i1*N+i2][j1*M+j2] = res;

					
					Vec3d min_rotation;

					double max_error = M_PI; //MAX( fabs(min_angle_line - angle), fabs(max_angle_line - angle) ) + edge_width;

					Vec3d lj1a = lines[j1]->getA() - position; //_camera->getTranslation();
					Vec3d lj1b = lines[j1]->getB() - position; //_camera->getTranslation();
					Vec3d lj2a = lines[j2]->getA() - position; //_camera->getTranslation();
					Vec3d lj2b = lines[j2]->getB() - position; //_camera->getTranslation();

					Vec3d ei1a = edgeplanes[i1]._a;
					Vec3d ei1b = edgeplanes[i1]._b;
					Vec3d ei2a = edgeplanes[i2]._a;
					Vec3d ei2b = edgeplanes[i2]._b;

					// compute the minimum rotation that brings edges (e1,e2) onto lines (l1,l2)
					double overlap = 0.0;

					//printf("%d %d minRotation2Planes2Lines 1\n", i, j, M);

					if ( minRotation2Planes2Lines ( ei1a, ei1b, ei2a, ei2b, lj1a, lj1b, lj2a, lj2b, max_error, overlap, min_rotation) ) {

							//LOG(LEVEL_INFO, "angle: %f  -- overlap = %f", len(min_rotation), overlap);

							if ( overlap > 1.0 - EPS ) {
								hist_min_rot[ int(len(min_rotation) / M_PI * 179) ]+= MAX(1,int(100 * res)); // store the rotation in a histogram
								fprintf( fh, "%f %f %f\n", min_rotation[0], min_rotation[1], min_rotation[2] );
								rotations.push_back( Vec4d(min_rotation[0], min_rotation[1], min_rotation[2], MAX(1,int(100 * res)))  );
							}

					} else {// compute the minimum rotation that brings edges (e2,e1) onto lines (l1,l2)
						//printf("%d %d minRotation2Planes2Lines 2\n", i, j, M);
						if ( minRotation2Planes2Lines ( ei2a, ei2b, ei1a, ei1b, lj1a, lj1b, lj2a, lj2b, max_error, overlap, min_rotation) ) {

							//LOG(LEVEL_INFO, "angle: %f  -- overlap = %f", len(min_rotation), overlap);

							if ( overlap > 1.0 - EPS ) {
								hist_min_rot[ int(len(min_rotation) / M_PI * 179) ]+= MAX(1,int(100 * res)); // store the rotation in a histogram
								fprintf( fh, "%f %f %f\n", min_rotation[0], min_rotation[1], min_rotation[2] );
								rotations.push_back( Vec4d(min_rotation[0], min_rotation[1], min_rotation[2], MAX(1,int(100 * res)))  );
							}
						}
					}

					//printf("%d %d done\n", i, j, M);

				}
			}
		}
	}

	fclose( fh );

	// print out histogram in a file
	FILE *fg = fopen("hist.dat", "w");
	for (i=0;i<180;i++)
		fprintf(fg, "%d\n", hist_min_rot[i]);
	fclose(fg);

	// free memory (table, duplicate, topk_values)
	for (i=0;i<N;i++) {
		delete[] book[i];
	}
	delete[] book;

	for (i=0;i<N;i++) {
		delete[] book_polar_edge[i];
	}
	delete[] book_polar_edge;

	for (i=0;i<N;i++) {
		delete[] compatibility[i];
	}
	delete[] compatibility;

	for (j=0;j<M;j++) {
		delete[] book_max[j];
	}
	delete[] book_max;

	for (j=0;j<M;j++) {
		delete[] book_min[j];
	}
	delete[] book_min;

	for (j=0;j<M;j++) {
		delete[] book_max_polar[j];
	}
	delete[] book_max_polar;

	for (i=0;i<N*N;i++)
		delete[] book_pairs[i];
	delete[] book_pairs;
}

// compute the best top-k rotations given a set of candidate rotations
// the idea is to look for agglomerations of points and look for the highest density regions over a small sphere of size <search_radius>
// the sphere has a radius of pi
// each rotation is assigned a weight hence the Vec4d structure
//
void MyGlWindow::computeBestRotations( std::vector<Vec4d> rotations, int topk, double search_radius, std::vector<Vec3d> &best_rotations )
{

	// find the topk rotation angles in hist_min_rot[180];
	for (int run = 0; run < topk; run++) {

		int best_i = 0;

		for (int i=0;i<180;i++) {

			if ( hist_min_rot[i] > hist_min_rot[best_i] )
				best_i = i;
		}

		double angle = toRadians((double)best_i);

		// clean up the histogram in the area
		for (i=MAX(0,best_i-5); i<MIN(180,best_i+5); i++)
			hist_min_rot[i] = 0;

		LOG(LEVEL_INFO, "found best angle %f deg.", toDegrees( angle ) );

		// populate the cells for this angle
		std::vector< intVector > cells;

		for (i=0;i<spheretsn->_faces.size();i++) {
			intVector v;
			cells.push_back( v );
		}

		for (i=0;i<rotations.size();i++) {

			Vec3d rotation = Vec3d( rotations[i][0], rotations[i][1], rotations[i][2] );

			if ( fabs( len( rotation ) - angle ) < toRadians( 5.0 ) )
				cells[ spheretsn->closest_cell( rotation ) ].push_back( i );
		}

		// find the top rotation direction
		int best_cell = 0;
		for (i=0;i<cells.size();i++) {
			if ( cells[i].size() > cells[best_cell].size() )
				best_cell = i;
		}

		int n = cells[best_cell].size();

		if ( n == 0 )
			continue;

		// compute the average rotation
		Vec3d average_rotation = Vec3d(0,0,0);

		for (i=0;i<n; i++) {
			Vec3d rotation = Vec3d( rotations[cells[best_cell][i]][0], rotations[cells[best_cell][i]][1], rotations[cells[best_cell][i]][2] );
			average_rotation = average_rotation + norm( rotation );
		}

		average_rotation = angle * norm( average_rotation / n );

		best_rotations.push_back( average_rotation );
	}
}

// compute the best possible camera pose given a set of correspondences
// in each correspondence, a model line is assigned a set of possible edge matches
// the camera translation is assumed to be roughly correct
//
void MyGlWindow::cameraPoseFromCorrespondences( corresVector correspondences )
{
	int i,j,k,p,q;

	int n = correspondences.size();

	// determine the list of visible edge planes (for scoring)
	edgePlaneVector edgeplanes;
	for (i=0;i<_camera->_frame._edgeplanes_chained.size();i++) {
		for (j=0;j<_camera->_frame._edgeplanes_chained[i].size();j++) {
			edgeplanes.push_back( _camera->_frame._edgeplanes_chained[i][j] );
		}
	}

	if ( edgeplanes.empty() )
		return;

	Vec3d translation = _camera->getTranslation();

	double best_score = -1.0;
	ExtrinsicParameters best_pose;

	for (int trial = 0; trial < INIT_RANSAC_TRIALS; trial++ ) {

		// pick three correspondences randomly
		intVector indices;
		selectNRandomInt( 3, n, indices );

		// debug: read the correspondences to use from file
		/*FILE *f = fopen( "user_input.txt", "r" );
		int i1,i2,i3;
		fscanf(f, "%d%d%d", &i1, &i2, &i3);
		indices.push_back( i1 );
		indices.push_back( i2 );
		indices.push_back( i3 );

		fclose(f);
		*/
		corres c1 = correspondences[indices[0]];
		corres c2 = correspondences[indices[1]];
		corres c3 = correspondences[indices[2]];
		
		// first test that the three lines are far from each other
		EdgePlane lc1 = EdgePlane( translation, c1.line->getA() - translation, c1.line->getB(), 0, 0, 0 );
		EdgePlane lc2 = EdgePlane( translation, c2.line->getA() - translation, c2.line->getB(), 0, 0, 0 );
		EdgePlane lc3 = EdgePlane( translation, c3.line->getA() - translation, c3.line->getB(), 0, 0, 0 );

		if ( lc1.angle( lc2 ) < INIT_MIN_DIHEDRAL_ANGLE || lc2.angle( lc3 ) < INIT_MIN_DIHEDRAL_ANGLE  || lc1.angle( lc3 ) < INIT_MIN_DIHEDRAL_ANGLE )
			continue;

		// then find the camera poses that bring the correspondences into alignment
		for (i=0;i<c1.eps.size();i++) {
			// debug: force i
			EdgePlane ec1 = c1.eps[i];

			for (j=0;j<c2.eps.size();j++) {
				EdgePlane ec2 = c2.eps[j];

				for (k=0;k<c3.eps.size();k++) {
					EdgePlane ec3 = c3.eps[k];
					
					ExtrinsicParameters pose;

					if ( !cameraPoseFromThreeCorrespondences( ec1, ec2, ec3, c1.line, c2.line, c3.line, pose ) )
						continue;
			
					if ( len( translation - pose.getTranslation() ) > 2 * INIT_DIAMETER  )  // if the solution is too far from the original camera position,
						continue;															// skip it
				
					// score the pose						
					double score = 0.0;
					
					// compute the transformed edge planes
					//LOG(LEVEL_INFO, "*******  edgeplanes *********");
					edgePlaneVector edgeplanes_world;
					for (p=0;p<edgeplanes.size();p++) {
						EdgePlane ep = edgeplanes[p];
					//	LOG(LEVEL_INFO,"%f %f %f", ep._s[0], ep._s[1],ep._s[2]);
						ep.fromCameraFrameToWorldFrame( pose );
						edgeplanes_world.push_back( ep );
					}
				//	LOG(LEVEL_INFO, "*******  *********** *********");

					// for each remaining correspondence, look for a match and update the score
					for (p=0;p<n;p++) {
						
						if ( p == i || p == j || p == k ) // skip the three correspondences used for alignment
							continue;
						
						corres c = correspondences[p];
						
						c.line->computeWeight( translation );

						for (q=0;q<edgeplanes_world.size();q++) {
							
							EdgePlane ep = edgeplanes_world[q];
							
							if ( ep.angle( c.line ) < INIT_MAX_SCORE_ANGLE  && ep.overlap( c.line ) > 0.5 ) {
								score += ep.overlap( c.line ) / (n-3);
								break;
							}
						}
					}
					
					
					// if beat best score, save it
					if ( score > best_score ) {
					
						best_score = score;
						best_pose  = pose;
						//LOG(LEVEL_INFO, "best pose so far:");
						//best_pose.print();						
					}				
				}
			}
		}
		
		LOG( LEVEL_INFO, "trial %d [%d %d %d]: score = %f", trial, indices[0], indices[1], indices[2], best_score );
	}

	// set the camera pose to the best guess so far
	if ( best_score > -1.0 ) {
		_camera->setPose( best_pose );
	
		LOG(LEVEL_INFO, "Best Camera Pose:");
		_camera->getPose().print();
	} else {
		_camera->setTranslation( translation );
		_camera->setRotation( Quaternion() );
		return;
	}

	// find inliers

	CorrespondenceVector inliers;

	for (p=0;p<n;p++) {
		
		corres c = correspondences[p];
		corres d;
		d.line = c.line;

		for (q=0;q<c.eps.size();q++) {
			
			EdgePlane ep = c.eps[q], tep = ep;
			ep.fromCameraFrameToWorldFrame( best_pose ); // transform the edge plane with the current pose
			
			if ( ep.angle( c.line ) < INIT_REFINEMENT_ANGLE  && ep.overlap( c.line ) > 0.1 ) {
				d.eps.push_back( tep );
			}
		}

		if ( d.valid() )
			SM_InsertItem( d );
	}

	LOG(LEVEL_INFO, "refining from %d correspondences", SM_Size() );

	// refine camera pose from inliers
	SM_ComputePose(POSE_MAX_ELEMENTS, POSE_MIN_ELEMENTS, POSE_MAX_TRIALS, POSE_THRESHOLD_SCORE, inliers);

	LOG(LEVEL_INFO, "%d inliers", inliers.size() );

	// insert a new item for each visible line in the queue

	_LUT.lookup( _camera->came.getTranslation(), 0.0, 0.0, 0, 0);

	SM_Clear();
	for (i=0;i<_LUT._lines.size();i++) {
		SM_InsertItem( _EXPECTED, _LUT._lines[i] );
	}

}

/////////////////////////////////////////////////////////
// lock recovery procedure
//
// recover the accurate camera pose from coarse camera pose estimate
// given a set of model lines and a set of observed features,
// the system determines the set of possible match for each feature
// and runs a RANSAC algorithm to determine the best matches.
//
double MyGlWindow::ransacLockRecovery()
{
	// get the camera pose
	ExtrinsicParameters pose = _camera->getPose();

	// reset line weights
	_LUT.resetLineWeights();

	// init the correspondences
	initCorrespondences( cs );

	// set a large mask for the image
	_camera->_min_mask_width = 50;
	_camera->_max_mask_width = 50;

	// for each model line, find the best matches on the image
	findLineMatches ( cs );

	// compute the camera pose
	CorrespondenceVector inliers;
	double score = SM_ComputePose(3, 3, POSE_MAX_TRIALS, POSE_THRESHOLD_SCORE, cs, inliers);

	// find inliers and refine from them
	refineFromInliers( cs, inliers, RANSAC_MIN_SCORE_INLIER);

	// fill in the state machine
	fillInStateMachine( inliers );

	// return the score
	return score;

}

// cleanup and fill in the state machine with correspondences (assumed _CONNECTED)
void MyGlWindow::fillInStateMachine (  CorrespondenceVector &csv )
{
	// clear the state machine
	SM_Clear();

	// insert CONNECTED items
	for ( int i=0;i<csv.size();i++) {
		SM_InsertItem( _CONNECTED, csv[i].second, csv[i].first, 0 );
		csv[i].first->status = ACCEPTED;
	}

	// insert EXPECTED items
	for ( edgeVector::iterator iter = _LUT._lines.begin(); iter != _LUT._lines.end(); iter++ ) {
		if ( (*iter)->status == UNKNOWN )
			SM_InsertItem( _EXPECTED, *iter );
	}
}

// find the inlier correspondences given a score threshold and refine the camera pose from them
//
void MyGlWindow::refineFromInliers ( corresVector &csv, CorrespondenceVector &inliers, double score_threshold )
{
	// clear inliers
	inliers.clear();

	// cleanup the invalid correspondences
	csv.erase( std::remove_if( csv.begin(), csv.end(), corres_invalid ), csv.end());

	// update each correspondence for the current camera pose
	for (int i=0;i<csv.size(); i++) {
		PROJ_TEST( csv[i], true, false, CLUSTER_MAX_ANGLE_THRESHOLD );

		// find the best edgeplane in the correspondence, if any
		EdgePlane edgeplane;
		double max_score = -1.0;
		for (int j=0;j<csv[i].eps.size();j++) {
			if ( csv[i].scores[j] > max_score ) {
				max_score = csv[i].scores[j];
				if ( max_score > score_threshold ) {
					edgeplane = csv[i].eps[j];
				}
			}
		}

		if ( !csv[i].eps.empty() && max_score > score_threshold ) {
			Correspondence c;
			c.first = csv[i].line;
			c.second = edgeplane;
			inliers.push_back( c );
			csv[i].eps.clear();
			csv[i].eps.push_back( edgeplane );
		}
	}

	refineCameraPoseFromNCorrespondences( inliers );
}

// for each correspondence in the vector <csv>, find the possible matches on the image
// 
void MyGlWindow::findLineMatches ( corresVector &csv )
{
	for (int i=0;i<csv.size();i++) {
		if ( !PROJ_TEST( csv[i], true, false, CLUSTER_MAX_ANGLE_THRESHOLD))
			csv[i].status( _BLACKLISTED );
	}
}

// create a set of correspondences with one (empty) correspondence for each line
// each edgeplane is transformed to the world coord frame in order to find line matches later on
//
void MyGlWindow::initCorrespondences ( corresVector &csv )
{
	csv.clear();

	ExtrinsicParameters pose = _camera->getPose();

	for (edgeVector::iterator iter = _LUT._lines.begin(); iter != _LUT._lines.end(); iter++) {
		csv.push_back( corres( *iter, EdgePlane(), _EXPECTED ) );
	}
}

// given an initial node in the model (where the camera presumably is), determine the most
// probable camera pose assuming that the camera is vertical
// the idea is to perform an exhaustive search on possible camera positions at the given node (for different heights and different orientations)
// and to compute the score at every step. The best position is then retained and a RANSAC lock recovery is run.
//
bool MyGlWindow::ransacInitFromNode( double &best_score )
{
	best_score = -1E10;
	double score = 0.0;

	if ( _LUT._node == NULL )
		return false;
	
	INIT_ANGLE_STEPS = 20;
	INIT_HEIGHT_STEPS = 5;
	
	ExtrinsicParameters pose = _camera->getPose();

	double best_ratio = 0.0;

	for ( int k=0;k<INIT_HEIGHT_STEPS; k++ ) {
		double height = k * 300.0 / INIT_HEIGHT_STEPS;
		
		for ( int i=0;i<INIT_ANGLE_STEPS;i++ ) {
			
			//LOG( LEVEL_INFO, "trial: [%d / %d] [%d / %d]", k, INIT_HEIGHT_STEPS, i, INIT_ANGLE_STEPS );

			// init the camera pose
			Vec3d position = _LUT._node->_pos;
			position[2] = height;
			_camera->setTranslation( position );
			_camera->setRotation( Quaternion() );
			_camera->rotate( Vec3d(0,0,1), 2.0 * M_PI * i / INIT_ANGLE_STEPS );
			
			// run lock recovery
			score = ransacLockRecovery();

			if ( score > best_score ) {
				best_score = score;
				pose = _camera->getPose();
			}
		}
	}
	
	// keep the best camera pose
	_camera->setPose( pose );

	return true;
}

void MyGlWindow::computeRegions( regionVector &regions )
{
	// for each region...
	for (int i=0;i<regions.size();i++) {

		double best_score = 0;
		ExtrinsicParameters best_pose = ExtrinsicParameters();
		bool accepted = false;

		// for each node in the region...
		for (nodeVector::iterator iter = regions[i]._nodes.begin(); iter != regions[i]._nodes.end(); iter++ ) {
			Node *node = *iter;
			assert( node != NULL );

			LOG( LEVEL_INFO, "region [%d] size: %d", i, regions[i]._nodes.size() );

			// lookup the node
			_LUT.lookup( node, 0.0, 0.0, 0, 0 );

			// run a RANSAC search of the best pose
			double score = 0.0;
			ransacInitFromNode( score );

			// keep best score
			if ( score > best_score ) {
				best_score = score;
				best_pose = _camera->getPose();
				accepted = true;
			}
		}

		if ( !accepted )
			continue;

		// refine the camera pose
		_camera->setPose( best_pose );
		best_score = ransacLockRecovery();
		best_pose = _camera->getPose();

		// save the pose and the score for that region
		regions[i]._score = best_score;
		regions[i]._best_pose = best_pose;

		LOG( LEVEL_INFO, "region %d: %f", i, regions[i]._score );
	}

	// sort regions by decreasing score
	std::sort( regions.begin(), regions.end(), greater_region );
}

// RANSAC : compute positions from a set of (presumably) good correspondences stored in the state machine
// pick random triplets of correspondences, compute the camera pose and score
// return success if score > thres
// max_trials: maximum of trials
// 
/*bool MyGlWindow::ransacCameraPose( int max_trials, double thres )
{
	int trial = 0, i=0, j=0;
	double angle_thres = toRadians( 5.0 );

	if ( SM_Size() < 3 ) // must have at least three correspondences to do RANSAC
		return false;

	corresVector correspondences;

	for ( trial = 0; trial < max_trials; trial++) {

		// randomly pick three correspondences
		intVector indexes;
		selectNRandomInt(3,SM_Size(),indexes);

		// for each correspondence, randomly pick one of the possible edgeplane
		CorrespondenceVector v;
		for (i=0;i<3;i++) {
			Correspondence d;
			corres c = SM_Get(indexes[i]);
			d.first = c.line;
			int n = MAX( c.eps.size()-1, (double)rand() / (RAND_MAX+1) * c.eps.size()-1 );
			d.second = c.eps[n];
			v.push_back( d );
		}

		// compute camera pose from triplet
		ExtrinsicParameters pose = _camera->getPose();
		if ( !computeCameraPoseFrom3Correspondences( v[0].first, v[1].first, v[2].first, v[0].second, v[1].second, v[2].second, pose) )
			continue;

		// generate a set of correspondences
		double score = 0.0;
		int counter = 0;
		correspondences.empty();

		// score the pose: for each model line, try to find a match on the image
		for (edgeVector::iterator iter = _LUT._lines.begin(); iter != _LUT._lines.end(); iter++) {
			Edge *line = *iter;
			corres c;
			c.line = line;
			counter++;

			// try to find a match
			for (i=0;i<_camera->_frame._edgeplanes_chained.size();i++) {
				for (j=0;j<_camera->_frame._edgeplanes_chained[i].size();j++) {
					EdgePlane ep = _camera->_frame._edgeplanes_chained[i][j];
					ep.fromCameraFrameToWorldFrame( pose ); // transform the edgeplane to the world coord frame

					// score
					if ( ( ep.overlap( line ) > 0.95 ) && ( ep.angle( line ) < angle_thres ) )
						c.eps.push_back( _camera->_frame._edgeplanes_chained[i][j] );
				}
			}

			if ( c.valid() ) {
				score += 1.0;
				correspondences.push_back( c );
			}
		}

		// test the score
		score /= counter;

		if ( score > thres ) 
			break;
	}

	// if successful, update the state machine and compute the pose
	if ( !correspondences.empty() ) {

		for (i=0;i<correspondences.size();i++)
			SM_InsertItem( correspondences[i] );

		SM_ComputePose(POSE_MAX_ELEMENTS, POSE_MIN_ELEMENTS, POSE_MAX_TRIALS, POSE_THRESHOLD_SCORE, cq, _inliers);

		return true;
	}
	else {
		return false;
	}
}
*/

// compute the camera pose by finding correspondences between the corners and the model lines
// given a camera translation estimate encoded in the node of the LUT
//
double MyGlWindow::compute_pose_from_corners( std::vector< corner > &corners, std::vector< corner_line > &corner_lines )
{
	int i,j,p,q,r,s;

	Vec3d position = _camera->getTranslation();

	// save original camera pose
	ExtrinsicParameters og_pose = _camera->getPose();

	// lookup the visibility field at position t
	_LUT.lookup( _camera->getTranslation(),  MIN_SUBTENDED_ANGLE, 0.0, 100, 0 );

	int M = _LUT._lines.size();
	int N = _LUT._vertices.size();

	LOG(LEVEL_INFO, "model: %d lines and %d corners.", M, N);

	// compute a set of possible correspondences <corner_id, vertex_id, line_corner_1, line_corner_2, line_model_1, line_model_2> 
	
	// clear the dihedral angle book
	_LUT.clear_dihedral_books();

	// for each image corner
	for (i=0;i<corners.size();i++) {

		corner c = corners[i];

		if ( i == 1 )
			printf("hello");

		std::vector< corner_line > cl_vector;

		// for each model corner
		for (j=0;j<N;j++) {

			Vertex *vertex = _LUT._vertices[j];
			Vec3d v = vertex->getPosition();
			Vec3d w = norm(v-position);
				
			// for each pair of edges on this image corner
			for (p=0;p<c.nedges();p++) {
				for (q=p+1;q<c.nedges();q++) {

					double angle = c.dihedral_angle(p,q);

					if ( fabs(angle) < FTRACKER_MIN_CORNER_ANGLE )
						continue;

					// for each pair of lines on this model corner
					for (r=0;r<vertex->nedges();r++) {
						for (s=r+1;s<vertex->nedges();s++) {

							Edge *a = vertex->_edges[r];
							Edge *b = vertex->_edges[s];

							if ( !a->_visible || !b->_visible )
								continue;


							Vec3d v1 = vertex->sibling( a )->getPosition();
							Vec3d v2 = vertex->sibling( b )->getPosition();

							Vec3d n1 = norm(cross(w, norm(v1-v)));
							Vec3d n2 = norm(cross(w, norm(v2-v)));

							double langle = dot(w,cross(n2,n1)) > 0 ? Acos(dot(n1,n2)) : -Acos(dot(n1,n2)); // oriented angle between the two lines

							//double min_dihedral_angle, max_dihedral_angle;

							//_LUT.min_max_dihedral_angle( a, b, INIT_MAX_CAMERA_HEIGHT, min_dihedral_angle, max_dihedral_angle );
							
							//double dihedral_angle = c.dihedral_angle( p , q );

							// test the dihedral angle
							//if ( max_dihedral_angle < dihedral_angle || dihedral_angle < min_dihedral_angle )
							//	continue;

							// test the subtended angle and create a correspondence if match is OK
							double max_subtended_angle_a;
							double max_subtended_angle_b;

							_LUT.max_subtended_angle( a, 0.0, INIT_MAX_CAMERA_HEIGHT, max_subtended_angle_a );
							_LUT.max_subtended_angle( b, 0.0, INIT_MAX_CAMERA_HEIGHT, max_subtended_angle_b );

							double subtended_angle_p = c.subtended_angle( p );
							double subtended_angle_q = c.subtended_angle( q );

							if ( subtended_angle_p < max_subtended_angle_a &&
								subtended_angle_q < max_subtended_angle_b ) {

								corner_line cl;
								cl.camera_id = c.getcameraid();
								cl.corner_id = i; // corners[i]
								cl.vertex_id = j; // _LUT._vertices[j]
								cl.edge_1_id = p;
								cl.edge_2_id = q;
								cl.line_1_id = a->_id;
								cl.line_2_id = b->_id;
								cl.score = (2*M_PI - fabs(langle-angle))/(2*M_PI); //( subtended_angle_p / max_subtended_angle_a ) * ( subtended_angle_q / max_subtended_angle_b );

								//LOG(LEVEL_INFO, "inserting corner_line: camera id %d corner id: %d vertex id: %d edges: %d %d lines: %d %d", cl.camera_id, cl.corner_id, cl.vertex_id,
								//	cl.edge_1_id, cl.edge_2_id, cl.line_1_id, cl.line_2_id);
								cl_vector.push_back( cl );
							}

							if ( subtended_angle_q < max_subtended_angle_a &&
								subtended_angle_p < max_subtended_angle_b ) {

								corner_line cl;
								cl.camera_id = c.getcameraid();
								cl.corner_id = i; // corners[i]
								cl.vertex_id = j; // _LUT._vertices[j]
								cl.edge_1_id = q;
								cl.edge_2_id = p;
								cl.line_1_id = a->_id;
								cl.line_2_id = b->_id;
								cl.score = (2*M_PI - fabs(langle+angle))/(2*M_PI); // ( subtended_angle_q / max_subtended_angle_a ) * ( subtended_angle_p / max_subtended_angle_b );

								//LOG(LEVEL_INFO, "inserting corner_line: camera id %d corner id: %d vertex id: %d edges: %d %d lines: %d %d", cl.camera_id, cl.corner_id, cl.vertex_id,
								//	cl.edge_1_id, cl.edge_2_id, cl.line_1_id, cl.line_2_id);
								cl_vector.push_back( cl );
							}	
						}
					}
				}
			}
		}

		// sort the correspondences and insert them
		std::sort( cl_vector.begin(), cl_vector.end() );

		for (j=0;j<cl_vector.size();j++)
			corner_lines.push_back( cl_vector[j] );
	}

	Fl::check();
	if ( !init->value() )
		return -1.0;
	//return;

	// distribute observed edges into buckets
	// this accelerates the scoring process
	std::vector< intVector > edges_buckets;
	distribute_edgeplanes_into_buckets( edges_buckets );

	// for each two corners, select top-k matches, compute camera pose and score
	//
	double best_score = 0.0;
	ExtrinsicParameters best_pose = og_pose;

	int n = corners.size() * (corners.size()-1);
	int counter = 0;

	progress_bar->value(0);
	progress_bar->maximum(corners.size() * (corners.size()-1) );

	for (i=0;i<corners.size();i++) {
		for (j=i+1;j<corners.size();j++) {

			counter++;
			progress_bar->value( counter );
			Fl::check();

			for (int ii=0;ii<INIT_CORNERS_TOPK;ii++) {

				// get ii-th match for corner i
				int counter = 0;
				corner_line c_i;

				for (int k=0;k<corner_lines.size();k++) {
					
					if ( corner_lines[k].corner_id != i )
						continue;

					c_i = corner_lines[k];

					if ( counter == ii )
						break;

					counter++;
				}

				if ( c_i.corner_id != i || counter != ii )
					continue;

				for (int jj=0;jj<INIT_CORNERS_TOPK;jj++) {

					// get jj-th match for corner j
					counter = 0;
					corner_line c_j;
					
					for ( k=0;k<corner_lines.size();k++) {
						
						if ( corner_lines[k].corner_id != j )
							continue;
						
						c_j = corner_lines[k];
						
						if ( counter == jj )
							break;
						
						counter++;
					}
					
					if ( c_j.corner_id != j || counter != jj )
						continue;

					// at this point, c_i and c_j will be used for pose computation

					Vec3d a,b,s;
					corners[c_i.corner_id].getedge( c_i.edge_1_id, a, b );
					s = _camera->getUnitCameraCenter( c_i.camera_id );
				
				//	if ( i == 6 && j == 9 && ii == 10 && jj == 1 )
				//		LOG(LEVEL_INFO, "warning score:...");

					EdgePlane P = EdgePlane( a, b, s, c_i.camera_id, 0, 0 );

					corners[c_i.corner_id].getedge( c_i.edge_2_id, a, b );
					EdgePlane Q = EdgePlane( a, b, s, c_i.camera_id, 0, 0 );
					
					corners[c_j.corner_id].getedge( c_j.edge_1_id, a, b );
					s = _camera->getUnitCameraCenter( c_j.camera_id );
					EdgePlane R = EdgePlane( a, b, s, c_j.camera_id, 0, 0 );

					Edge *line_p, *line_q, *line_r;
					line_p = _LUT._model->getEdge( c_i.line_1_id );
					line_q = _LUT._model->getEdge( c_i.line_2_id );
					line_r = _LUT._model->getEdge( c_j.line_1_id );


					ExtrinsicParameters pose;
					if ( !cameraPoseFromThreeCorrespondences(P, Q, R, line_p, line_q, line_r, pose ) )
						continue;

					// score the camera pose
					double score = score_camera_pose( pose, edges_buckets );

					//if ( i == 6 && j == 9 && ii == 10 && jj == 1 )
					//	LOG(LEVEL_INFO, "warning score: %f (against %f)", score, best_score);
					
					if ( score > best_score && _LUT.valid_pose( pose )) {
						
						best_score = score;
						best_pose = pose;
					}
					
				}
			}
			
			LOG(LEVEL_INFO, "corners [%d] [%d] best pose:", i, j );
			best_pose.print();
		}
		Fl::check();
		if ( !init->value() ) // quit if the user stopped the init
			break;

		// update the pose vector
		best_pose.set_score( best_score );
		best_pose.set_mode( CORNERS );
		insert_init_pose( best_pose );
		//poses.push_back( best_pose );

	}

	// set the camera pose to the best pose found so far
	_camera->setPose( best_pose );

	return best_score;

}

// brute-force maximize score
double MyGlWindow::maximize_score( std::vector< intVector > &edges_buckets ) 
{

	int step = int( 2 * M_PI / SCORE_MAX_DIHEDRAL_ANGLE );

	double best_score = -1.0;

	ExtrinsicParameters best_pose = _camera->getPose();

	// for a set of possible rotations, compute the score and keep the best one
	for (double x = -M_PI; x <= M_PI; x += 2*M_PI/step) {
		for (double y = -M_PI; y <= M_PI; y += 2*M_PI/step) {
			for (double z = -M_PI; z <= M_PI; z += 2*M_PI/step) {
				
				Vec3d axis = Vec3d(x,y,z);
				
				if ( len(axis) > M_PI )
					continue;
				
				if ( len(axis) < EPS )
					continue;
				
				_camera->setRotation( Quaternion( norm(axis), len(axis) ) );
				
				//PerfTimer timer;
				double score = score_camera_pose( _camera->getPose(), edges_buckets );
				//timer.print( "scoring" );
				
				// insert the score in the score sheet
				std::pair< Vec3d, double > element;
				element.first = axis;
				element.second = score;
				rotation_scoresheet.push_back( element );
				
				if ( score > best_score && _LUT.valid_pose(_camera->getPose()) ) {
					
					best_score = score;
					best_pose = _camera->getPose();
				}
			}
		}

		Fl::check();
		if ( !init->value() )
			break;
	}

	// set the camera pose to the best rotation found so far
	_camera->setPose( best_pose );

	best_pose.set_score( best_score );
	best_pose.set_mode( EXHAUSTIVE_SEARCH );
	insert_init_pose( best_pose );
	//poses.push_back( best_pose );

	return best_score;
}

// compute camera pose from VPs
//
double MyGlWindow::compute_pose_from_vps( double h1, double h2, std::vector< intVector > &edges_buckets )
{
	ExtrinsicParameters pose = _camera->getPose();

	// lookup the LUT for the current camera pose estimate
	_LUT.lookup( _camera->came.getTranslation(),MIN_SUBTENDED_ANGLE, 0.0, 100, 0);

	std::vector< Vec3d > vps_edges;
	std::vector< Vec3d > vps_lines;

	int i;

	// compute the VPs for observed edges
	edgePlaneVector edgeplanes;
	initInitialCorrespondences( edgeplanes );

	compute_vanishing_points( INIT_TOPK_VPS, edgeplanes, vps_edges );

	// store for display
	_vps_edges = vps_edges;

	//for (i=0;i<vps_edges.size();i++)
	//	printf("%f %f %f\n", vps_edges[i][0], vps_edges[i][1], vps_edges[i][2]);

	// compute the VPs for model lines
	edgePlaneVector lines_edgeplanes;
	Vec3d center = pose.getTranslation();

	for (i=0;i<_LUT._lines.size();i++) {
		EdgePlane ep( _LUT._lines[i]->getA() - center, _LUT._lines[i]->getB() - center, center, -1, _LUT._lines[i]->_id, i );
		ep.fromWorldFrameToCameraFrame( pose );
		lines_edgeplanes.push_back( ep );
	}

	compute_vanishing_points( INIT_TOPK_VPS, lines_edgeplanes, vps_lines );

	_vps_lines = vps_lines;

	LOG(LEVEL_INFO, "using %d VPs on image and %d VPs on model", vps_edges.size(), vps_lines.size());

	// for each pair of vanishing points, compute the camera pose, refine translation and score

	double best_score = -1.0;

	progress_bar->value(0);
	progress_bar->minimum(0);
	progress_bar->maximum(vps_edges.size() * (vps_edges.size()-1) * vps_lines.size() * (vps_lines.size()-1) );

	int counter = 0;

	int i1,i2,j1,j2;
	for (i1=0;i1<vps_edges.size();i1++) {

		Vec3d vp_edge1 = vps_edges[i1];

		for (i2=i1+1;i2<vps_edges.size();i2++) {
		
			Vec3d vp_edge2 = vps_edges[i2];

			for (j1=0;j1<vps_lines.size();j1++) {

				Vec3d vp_line1 = vps_lines[j1];

				for (j2=j1+1;j2<vps_lines.size();j2++) {

					Vec3d vp_line2 = vps_lines[j2];

					counter++;
					progress_bar->value( counter );
					
					// check vps angle
					if ( fabs( Acos(dot(vp_edge1, vp_edge2)) - Acos(dot(vp_line1, vp_line2)) ) > INIT_MAX_VPS_ANGLE )
						continue;

					// compute the rotation that brings edges VPs in alignement with lines VPs ( = rotation used in fromCameraFrameToWorldFrame = camera rotation)
					Vec3d rotation;

					if ( !minRotationFourPoints( vp_edge1, vp_edge2, vp_line1, vp_line2, rotation ) )
						continue;

					pose.setRotation( Quaternion( rotation, len(rotation) ) );

					double score = score_camera_pose( pose, edges_buckets); //refine_camera_translation( pose, h1, h2, edges_buckets );

					if ( score > best_score ) {

						best_score = score;

						_camera->setPose( pose );
					}

					if ( !minRotationFourPoints( vp_edge2, vp_edge1, vp_line1, vp_line2, rotation ) )
						continue;

					pose.setRotation( Quaternion( rotation, len(rotation) ) );

					score = score_camera_pose( pose, edges_buckets); //refine_camera_translation( pose, h1, h2, edges_buckets );

					if ( score > best_score && _LUT.valid_pose( pose ) ) {

						best_score = score;
						
						_camera->setPose( pose );
					}

					pose = _camera->getPose();
					pose.set_score( best_score );
					pose.set_mode( VANISHING_POINTS );
					insert_init_pose( pose );
					//poses.push_back( pose );
					
					Fl::check(); // update FLTK

					if ( !init->value() ) // quit function if user stopped init
						break;
				}
				if ( !init->value() )
					break;
			}
			if ( !init->value() )
				break;
		}
		if ( !init->value() )
			break;
	}

	// refine the camera pose
	pose = _camera->getPose();

	best_score = refine_camera_translation( pose, h1, h2, edges_buckets );
	
	_camera->setPose( pose );

	return best_score;
}

// compute the best camera pose assuming vertical pose
// by brute-force search
//
double MyGlWindow::compute_pose_vertical (  std::vector< intVector > &edges_buckets )
{
	double height;
	double alpha,beta,gamma;

	std::vector< ExtrinsicParameters> poses;

	Vec3d center_point = _LUT._node->_pos;

	int total = 10 * 5 * 10 * 10;

	int counter = 0;

	for (height=0.0;height<=INIT_MAX_CAMERA_HEIGHT;height+=INIT_MAX_CAMERA_HEIGHT/10) {

		for (alpha=0.0;alpha<2*M_PI;alpha += M_PI/5.0) {

			for (gamma=0.0;gamma<_LUT.GRID_STEP;gamma+= _LUT.GRID_STEP/10) {

				ExtrinsicParameters pose; 

				// set the camera position
				Vec3d p = center_point + gamma * Vec3d(cos(alpha),sin(alpha),0);
				p[2] = height;

				pose.setTranslation( p );

				for (beta=0.0;beta<2*M_PI;beta+=M_PI/10.0) {

					pose.setRotation( Quaternion(Vec3d(0,0,1), beta) );

					double score = score_camera_pose( pose, edges_buckets );

					pose.set_score( score );
	
					pose.set_mode( INIT_VERTICAL );

					poses.push_back( pose );

					LOG(LEVEL_INFO, "%d / %d", counter, total);

					counter++;

				}
			}
		}
	}
	
	std::sort( poses.begin(), poses.end() );

	if ( poses.size() > 100 )
		poses.resize( 100 );
	
	for (int i=0;i<poses.size();i++)
		init_poses.push_back( poses[i] );

	return poses[0].get_score();
}

// compute camera pose from line matching
//
double MyGlWindow::compute_pose_from_line_matching( double h1, double h2, std::vector< intVector > &edges_buckets )
{

	readConfig();

	// lookup the LUT for the current camera pose estimate
	_LUT.lookup( _camera->came.getTranslation(),MIN_SUBTENDED_ANGLE, 0.0, 100, 0);

	edgePlaneVector edgeplanes;
	_ideal_rotation = Vec3d(0,0,0);
	
	if ( _database != NULL ) { 	// create real correspondences from observations
		initInitialCorrespondences( edgeplanes );
	} else {
		initIdealCorrespondences( edgeplanes ); // useful for debugging only
	}

	//} else {  // for debugging, create a set of ideal correspondences		
		//_camera->setRotation(Quaternion());
		//double random_angle = (double)rand()/RAND_MAX * M_PI; // randomly rotate camera
		//_camera->rotate( Vec3d(0,0,1), random_angle);
		//LOG(LEVEL_INFO, "rotated camera by %f deg.", toDegrees(random_angle));
		//Quaternion( Vec3d(0,0,1), random_angle ).toAxisAngle( _ideal_rotation );

		//initIdealCorrespondences( edgeplanes );
		//distribute_edgeplanes_into_buckets( edges_buckets );

	//}

		// diam is the uncertainty on the camera position ( e.g ~ 50 inches )
	//double diam = _LUT.GRID_STEP; 

	// compute the initial correspondences between edges and lines
	double score = compute_initial_correspondences_matching( edgeplanes, _LUT._lines, h1, h2, INIT_TOPK, edges_buckets );

	if ( score > 0.0 ) {
		ExtrinsicParameters pose = _camera->getPose();
		pose.set_mode( LINE_LINE );
		pose.set_score( score );
		insert_init_pose( pose );
		//if ( _LUT.valid_pose( pose ) )
		//	poses.push_back( pose );
	}

	return score;

}

// compute a set of probable correspondences between edges and lines, given:
// a set of N observed edges <edgeplanes> expressed in the camera coordinate frame
// a set of M visible lines <lines> expressed in the world coordinate frame
// an estimate of the camera position encoded in the node, with h1 < height < h2
// output <correspondences>
// this algorithm works by computing the signature of every pair of observed edges and looking for possible matches in pairs of lines
// the algorithm keeps the top-K edges for each model line (topK ~ 5)
//
// note: N edges => N^2 pairs, M lines => M^2 pairs ==> N^2 x M^2 votes.  (N=20, M=20, 160,000 votes)
// optimization: dihedral angles for pairs of edges and pairs of lines are precomputed in a separate table
// this algorithm also computes the list of candidate camera rotations and store is into <rotations>
// each rotation is assigned a weight, hence the Vec4d structure
//
double MyGlWindow::compute_initial_correspondences_matching ( edgePlaneVector &edgeplanes, edgeVector &lines, double h1, double h2, int topk,
												std::vector< intVector > &edges_buckets )
{
	// first brute-force compute the min and max dihedral angles for each triplet of model lines
	// disregard lines which are closer to the camera position than 3 times the size of <diam>
	int i=0, j=0, k=0, q=0;
	int N = edgeplanes.size();
	int M = lines.size();

	Vec3d position = _LUT._node->_pos;
	position[2] = (h1+h2)/2.0;

	// optimization: precompute the min and max dihedral angle (and max polar angle) in a separate table
	double **book_min;
	double **book_max;
	double **book_max_polar;
	book_min = new double*[M]; //(double**)malloc(M*sizeof(double*));
	book_max = new double*[M]; //(double**)malloc(M*sizeof(double*));
	book_max_polar = new double*[M]; //(double**)malloc(M*sizeof(double*));

	for (j=0;j<M;j++) {
		book_min[j] = new double[M]; //(double*)malloc(M*sizeof(double));
		book_max[j] = new double[M]; //(double*)malloc(M*sizeof(double));
		book_max_polar[j] = new double[M]; //(double*)malloc(M*sizeof(double));
	}

	double min_angle = 0.0, max_angle = 0.0, max_polar_angle = 0.0;

	for (j=0;j<M;j++) {

		for (k=j+1;k<M;k++) {
			_LUT.min_max_dihedral_angle( lines[j], lines[k], h1, h2, min_angle, max_angle, max_polar_angle);

			book_min[j][k] = min_angle;
			book_max[j][k] = max_angle;
			book_max_polar[j][k] = max_polar_angle;

			if ( max_angle - min_angle > 0.5 ) {
				book_min[j][k] = 0.0;
				book_max[j][k] = 0.0;
			}
		}
	}

	//LOG(LEVEL_INFO, "# line triplets: %d", M*(M-1)*(M-2)/6);
	//LOG(LEVEL_INFO, "# edge triplets: %d", N*(N-1)*(N-2)/6);
	
	// second, brute-force compute the dihedral angle for each pair of image edges

	// optimization: precompute the dihedral angles in a separate table
	double **book;
	book = new double*[N]; //(double**)malloc(N*sizeof(double*));
	double **book_polar_edge;
	book_polar_edge = new double*[N]; //(double**)malloc(N*sizeof(double*));
	for (i=0;i<N;i++) {
		book[i] = new double[N]; //(double*)malloc(N*sizeof(double));
		book_polar_edge[i] = new double[N];
	}

	for (i=0;i<N;i++) {
		for (k=i+1;k<N;k++) {
			book[i][k] = edgeplanes[i].angle( edgeplanes[k] );
			book_polar_edge[i][k] = maxPolarAngle( edgeplanes[i]._a, edgeplanes[i]._b, edgeplanes[k]._a, edgeplanes[k]._b );
		}
	}

	// compute a NxM compatibility table based on subtended angle: (edge < line)
	int **compatibility = new int*[N];
	for (i=0;i<N;i++)
		compatibility[i] = new int[M];
		
	double *sub_angle_lines = new double[M]; // compute and store the min and max subtended angle for each model line
	for (j=0;j<M;j++) {
		double min_angle = 0.0, max_angle = 0.0;
		_LUT.max_subtended_angle( lines[j], h1, h2, max_angle);
		//minMaxSubtendedAngle( lines[j], position, diam, min_angle, max_angle );
		sub_angle_lines[j] = max_angle;
	}

	for (i=0;i<N;i++) {
		double edge_angle = edgeplanes[i].length();

		for (j=0;j<M;j++) {
			if ( sub_angle_lines[j] * 1.05 < edge_angle ) // allow a 10% error
				compatibility[i][j] = 0;
			else
				compatibility[i][j] = 1;
		}
	}

	// create all edge triplets
	i=0,j=0,k=0;

	// initialize a large table, with N rows and M columns
	// each cell (i,j) of the table will be incremented if the i-th edge is a valid match for the j-th line

	double **table;
	table = new double*[N]; //(double**)malloc(N*sizeof(double*));
	for (i=0;i<N;i++) {
		table[i] = new double[M]; //(double*)malloc(M*sizeof(double));
	}

	for (i=0;i<N;i++) { // initialize with zeros
		for (j=0;j<M;j++) {
			table[i][j] = 0.0;
		}
	}

	long int nvotes = N*(N-1)*(N-2)*M*(M-1)*(M-2)/36;
	LOG(LEVEL_INFO, "# votes: %d", nvotes);

	// optimization: precompute the match between pairs of lines and pairs of edges
	// for each pair of edges (e1,e2) and pair of lines (l1,l2), a probability function is computed
	// using a gaussian distribution for the edges and a pdf for the lines

	double edge_width = toRadians( 1.0 ); // width of the uniform distribution for a pair of edges

	double **book_pairs = new double*[N*N]; //(double**)malloc(N*N*sizeof(double*));
	for (i=0;i<N*N;i++) 
		book_pairs[i] = new double[M*M]; //(double*)malloc(M*M*sizeof(double));

	double coeff = 1.0; // / (NOISE_SYNTHETIC_ANGLE_STDEV * sqrt(2*M_PI));

	Fl::check();
	if ( !init->value() )
		return -1.0;

	// histogram for the minimum rotations


	for (i=0;i<180;i++)
		hist_min_rot[i] = 0;

	int i1=0,i2=0,i3=0;
	int j1=0,j2=0,j3=0;

	LOG(LEVEL_INFO, "# votes = %d x %d = %d", N*(N-1), M*(M-1), N*(N-1) * M*(M-1));

	for (i1=0;i1<N;i1++) {
		for (i2=i1+1;i2<N;i2++) {
			double angle = book[i1][i2]; // dihedral angle between edge i1 and edge i2
			double min_angle_edge = MAX(0.0, angle - edge_width);
			double max_angle_edge = MIN(M_PI,angle + edge_width);

			for (j1=0;j1<M;j1++) {
				for (j2=j1+1;j2<M;j2++) {
					double min_angle_line = book_min[j1][j2]; // min and max dihedral angle between line j1 and line j2
					double max_angle_line = book_max[j1][j2];

					if ( max_angle_edge < min_angle_line || max_angle_line < min_angle_edge ) { // no overlap, probability = zero
						book_pairs[i1*N+i2][j1*M+j2] =  0.0;
						continue;
					}
					if ( compatibility[i1][j1] == 0 && compatibility[i1][j2] == 0 ) { // if each edge is incompatible with the two lines, probability = zero
						book_pairs[i1*N+i2][j1*M+j2] = 0.0;
						continue;
					}
					if ( compatibility[i2][j1] == 0 && compatibility[i2][j2] == 0 ) {
						book_pairs[i1*N+i2][j1*M+j2] = 0.0;
						continue;
					}

					if ( book_polar_edge[i1][i2] > 1.05 * book_max_polar[j1][j2] ) { // if the max polar angle of the edge pair is larger than the 
						book_pairs[i1*N+i2][j1*M+j2] = 0.0;					// one of the line pair, probability = zero  (error 5%)
						continue;
					}
					
					double end = MIN(max_angle_edge, max_angle_line);
					double start = MAX(min_angle_edge, min_angle_line);

					double res = 0.0;

					switch ( INIT_SCORING_MODE ) {
					case 0 : 
						res = 1.0;
						break;
					case 1:
						res = (end-start) * (end - start ) / ( (max_angle_edge - min_angle_edge) * (max_angle_line - min_angle_line) );
						break;
					case 2:
						LOG(LEVEL_INFO, "this mode is not supported anymore!");
						res = res * book_polar_edge[i1][i2] / book_max_polar[j1][j2];
						break;
					}

					book_pairs[i1*N+i2][j1*M+j2] = res;

				}
			}
		}
	}

	//Fl::check();
	//if ( !init->value() )
	//	return -1.0;

	// fill the table
	// each cell (i,j) of the table will be incremented if the i-th edge is a valid match for the j-th line
	double step = 1E-4;
	int counter_ctable = 0;

	for (int vote = 0; vote < INIT_MAX_VOTES; vote++) {
		
		intVector indices1;
		selectNRandomInt( 3, N, indices1 );
		
		i1 = indices1[0];
		i2 = indices1[1];
		i3 = indices1[2];
		
		
		intVector indices2;
		selectNRandomInt( 3, M, indices2 );
		
		j1 = indices2[0];
		j2 = indices2[1];
		j3 = indices2[2];
		
		if ( book_pairs[i1*N+i2][j1*M+j2] > 0.0 && book_pairs[i2*N+i3][j2*M+j3] > 0.0 && book_pairs[i1*N+i3][j1*M+j3] > 0.0 ) {
			table[i1][j1] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i1][j2] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j1] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j2] += step * book_pairs[i1*N+i2][j1*M+j2];
			
			table[i2][j2] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i2][j3] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j2] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j3] += step * book_pairs[i2*N+i3][j1*M+j2];
			
			table[i1][j1] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i1][j3] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j1] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j3] += step * book_pairs[i1*N+i3][j1*M+j2];

			/*edgePlaneVector edgeplanes_history;
			edgeplanes_history.push_back( edgeplanes[i1] );
			edgeplanes_history.push_back( edgeplanes[i2] );
			edgeplanes_history.push_back( edgeplanes[i3] );

			intVector lines_id_history;
			lines_id_history.push_back( lines[j1]->_id );
			lines_id_history.push_back( lines[j2]->_id );
			lines_id_history.push_back( lines[j3]->_id );

			_line_matching_history.push_back( std::pair< intVector, edgePlaneVector > (lines_id_history, edgeplanes_history) );
			*/
		}
		
		if ( book_pairs[i1*N+i2][j1*M+j2] > 0.0 && book_pairs[i1*N+i3][j2*M+j3] > 0.0 && book_pairs[i2*N+i3][j1*M+j3] > 0.0 ) {
			table[i1][j1] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i1][j2] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j1] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j2] += step * book_pairs[i1*N+i2][j1*M+j2];
			
			table[i1][j2] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i1][j3] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j2] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j3] += step * book_pairs[i1*N+i3][j1*M+j2];
			
			table[i2][j1] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i2][j3] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j1] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j3] += step * book_pairs[i2*N+i3][j1*M+j2];
		}
		if ( book_pairs[i2*N+i3][j1*M+j2] > 0.0 && book_pairs[i1*N+i2][j2*M+j3] > 0.0 && book_pairs[i1*N+i3][j1*M+j3] > 0.0 ) {
			table[i2][j1] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i2][j2] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j1] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j2] += step * book_pairs[i2*N+i3][j1*M+j2];
			
			table[i1][j2] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i1][j3] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j2] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j3] += step * book_pairs[i1*N+i2][j1*M+j2];
			
			table[i1][j1] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i1][j3] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j1] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j3] += step * book_pairs[i1*N+i3][j1*M+j2];
		}
		if ( book_pairs[i2*N+i3][j1*M+j2] > 0.0 && book_pairs[i1*N+i3][j2*M+j3] > 0.0 && book_pairs[i1*N+i2][j1*M+j3] > 0.0 ) {
			table[i2][j1] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i2][j2] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j1] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j2] += step * book_pairs[i2*N+i3][j1*M+j2];
			
			table[i1][j2] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i1][j3] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j2] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j3] += step * book_pairs[i1*N+i3][j1*M+j2];
			
			table[i1][j1] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i1][j3] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j1] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j3] += step * book_pairs[i1*N+i2][j1*M+j2];
		}
		if ( book_pairs[i1*N+i3][j1*M+j2] > 0.0 && book_pairs[i1*N+i2][j2*M+j3] > 0.0 && book_pairs[i2*N+i3][j1*M+j3] > 0.0 ) {
			table[i1][j1] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i1][j2] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j1] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j2] += step * book_pairs[i1*N+i3][j1*M+j2];
			
			table[i1][j2] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i1][j3] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j2] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j3] += step * book_pairs[i1*N+i2][j1*M+j2];
			
			table[i2][j1] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i2][j3] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j1] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j3] += step * book_pairs[i2*N+i3][j1*M+j2];
		}

		if ( book_pairs[i1*N+i3][j1*M+j2] > 0.0 && book_pairs[i2*N+i3][j2*M+j3] > 0.0 && book_pairs[i1*N+i2][j1*M+j3] > 0.0 ) {
			table[i1][j1] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i1][j2] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j1] += step * book_pairs[i1*N+i3][j1*M+j2];
			table[i3][j2] += step * book_pairs[i1*N+i3][j1*M+j2];
			
			table[i2][j2] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i2][j3] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j2] += step * book_pairs[i2*N+i3][j1*M+j2];
			table[i3][j3] += step * book_pairs[i2*N+i3][j1*M+j2];
			
			table[i1][j1] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i1][j3] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j1] += step * book_pairs[i1*N+i2][j1*M+j2];
			table[i2][j3] += step * book_pairs[i1*N+i2][j1*M+j2];
		}

		/*int step_ctable = INIT_MAX_VOTES / 50;

		if ( vote % step_ctable == 0 ) {
			char ctable_filename[256];
			sprintf(ctable_filename, "init_ctable_%d.dat", counter_ctable);
			FILE *fg = fopen(ctable_filename, "w");
			
			for (i=0;i<N;i++) {
				for (j=0;j<M;j++) {
					fprintf(fg, "%f ", table[i][j]);
				}
				fprintf(fg,"\n");
			}
			fclose(fg);
			counter_ctable++;
		}*/
	}

	LOG(LEVEL_INFO, "vote done. (%d in history)", _line_matching_history.size());

	for (i=0;i<N;i++) { // enforce compatibility between edges and lines
		for (j=0;j<M;j++) {
			table[i][j] = table[i][j] * compatibility[i][j];
		}
	}

	// normalize the table
	for (i=0;i<N;i++) {
		double av = 0.0;
		for (j=0;j<M;j++)
			av += table[i][j] / M;

		if ( av > 0.0 ) {
			for (j=0;j<M;j++)
				table[i][j] = table[i][j] / av;
		}
	}

	SM_Clear();

	// print out table in a file
	FILE *fg = fopen("init_ctable.dat", "w");

	for (i=0;i<N;i++) {
		for (j=0;j<M;j++) {
			fprintf(fg, "%f ", table[i][j]);
		}
		fprintf(fg,"\n");
	}
	fclose(fg);

	// keep the top-k matches for each line
	corresVector correspondences;
	int counter = 0;

	for (j=0;j<M;j++) {

		corres c ( lines[j] );
		
		for (k=0;k<topk;k++) {

			int best_i = 0;
			double best_score = table[best_i][j];

			for (i=0;i<N;i++) {
				if ( table[i][j] > table[best_i][j] ) {
					best_i = i;
				}
			}

			table[best_i][j] = 0.0;

			if ( best_i == j )  // for synthetic case only!!!
				counter++;

			//LOG(LEVEL_INFO, "inserting correspondence %d - %d", best_i, j );

			// create a correspondence
			c.eid = 0;
			c.length = edgeplanes[best_i].length();
			c.eps.push_back( edgeplanes[best_i] );
		}

		if ( c.valid() )
			correspondences.push_back( c );

		SM_InsertItem( c ); // insert the correspondence for debugging

	}

	LOG(LEVEL_INFO, "success rate: %f (%d / %d valid correspondences)", (double)counter / M, correspondences.size(), SM_Size());

	//LOG(LEVEL_INFO, "sm size: %d", SM_Size() );

	// refine the camera pose based on these correspondences
	
	double score = refine_camera_pose ( correspondences, edges_buckets );
	
	// free memory (table, duplicate, topk_values)
	for (i=0;i<N;i++) {
		delete[] book[i];
	}
	delete[] book;

	for (i=0;i<N;i++) {
		delete[] book_polar_edge[i];
	}
	delete[] book_polar_edge;

	for (i=0;i<N;i++) {
		delete[] compatibility[i];
	}
	delete[] compatibility;

	for (i=0;i<N;i++) {
		delete[] table[i];
	}
	delete[] table;

	for (j=0;j<M;j++) {
		delete[] book_max[j];
	}
	delete[] book_max;

	for (j=0;j<M;j++) {
		delete[] book_min[j];
	}
	delete[] book_min;

	for (j=0;j<M;j++) {
		delete[] book_max_polar[j];
	}
	delete[] book_max_polar;

	for (i=0;i<N*N;i++)
		delete[] book_pairs[i];
	delete[] book_pairs;

	return score;
}

void MyGlWindow::detect_edges()
{	
	int i,j;

	_camera->clearEdges();
	_camera->clearMasks();
	
	// first try to read the edges from file
	//if ( !_camera->readEdgesFromFile( frameId ) ) {
		
		// otherwise compute edges
		_camera->initializeEdgeDetectors();
		_camera->detectLines (true,false,edgePixels->value(),edgeThresh->value(),edgeSigma->value());
		_camera->convertLinesToPlanes();

		// update color
		int counter = 0;

		for (i=0;i<_camera->_frame._nImages;i++) {
			for ( j=0;j<_camera->_frame._edgeplanes[i].size();j++) {
				EdgePlane edge = _camera->_frame._edgeplanes[i][j];
				edge._uid = counter;
				if ( edge.length() > 1E-7 )
					_camera->updateColor(edge, false);
				_camera->_frame._edgeplanes[i][j] = edge;
				counter++;
			}
		}

		_camera->_frame.chainEdges(N_CHAIN_EDGES);
		
		// update color
		counter = 0;

		for ( i=0;i<_camera->_frame._nImages;i++) {
			for ( j=0;j<_camera->_frame._edgeplanes_chained[i].size();j++) {
				EdgePlane edge = _camera->_frame._edgeplanes_chained[i][j];
				edge._uid = counter;
				_camera->updateColor(edge, false);
				_camera->_frame._edgeplanes_chained[i][j] = edge;
				counter++;
			}
		}


		//LOG(LEVEL_INFO, "n chained edges: %d", _camera->_frame.n_edgeplanes_chained);
		
		//for (int i=0;i<6;i++)
		//	LOG(LEVEL_INFO, "cam %d: %d chained edges.", i, _camera->_frame._edgeplanes_chained[i].size() );
//	}
	
}
