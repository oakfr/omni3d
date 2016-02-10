#include "main.h"

// generate a synthetic sequence from a set of recorded poses
void MyGlWindow::makeSyntheticSequence()
{
	// prompt user for number of frames

	int nposes = 0;

	const char *prompt = fl_input("Specify number of frames","100");
	if (prompt == NULL) {
		return;
	}

	if ( sscanf(prompt, "%d", &nposes ) != 1 ) 
		return;


	// prompt user for directory
	char *f = fl_dir_chooser("Choose target directory", "C:/omni3d/data/", 0); // 0 : full path name
	if ( f == NULL )
		return;

	// test emptiness
	if (!isEmptyDirectory(f)) {
		int ok = fl_choice("This directory is not empty. Do you wish to empty it?","No","Yes",NULL);
		if (ok)
			emptyDirectory(f);
	}

	// create a file for poses
	char pose_filename[256];
	sprintf( pose_filename, "%s\\poses.dat",f );

	// for each pose, make a synthetic image and save it
	

	progress_bar->minimum(0);
	progress_bar->maximum(nposes);
	progress_bar->value(0);

	for (int frameId=0;frameId<nposes;frameId++) {
		_camera->setPose( GetSyntheticPose( frameId ) );

		_LUT.lookup( _camera->getTranslation(), 0.0, 0.0, 0, 0 );

		//Fl::check();
		//redraw();

		LOG( LEVEL_INFO, "generating synthetic image for %d faces", _LUT._faces.size() );

		for (int j=0;j<_camera->_frame._nImages;j++) {

			IplImage *img = cvCreateImage(cvSize(int(_camera->_height), int(_camera->_width)),  IPL_DEPTH_8U, 3);

			_camera->makeSyntheticImage(j,_LUT._faces, _LUT._faces.size(),img);

			char image_filename[MAX_PATH + 1]; 
			sprintf(image_filename,"%s\\%06d-cam%d.bmp",f,frameId,j);

			LOG( LEVEL_INFO, "saving image in %s ( frameId = %d)", image_filename, frameId );

			cvSaveImage(image_filename,img);

			cvReleaseImage( &img );
			//progress_bar->value(6*frameId+j+1);
			//redraw();
		}
		
		// open the pose file
		FILE *pose_file = fopen( pose_filename, "w" );
		if ( pose_file == NULL ) 
			LOG( ERROR, "failed to create pose file %s!", pose_filename );
		
		// save the pose in the file
		_camera->writePose( frameId, pose_file );	
		
		// close the pose file
		fclose( pose_file );
	}
	

	progress_bar->value(0);	
	
	// create data info file
	createDataInfoFile (f,  int(_camera->_height),  int(_camera->_width), nposes, 5, 1);

}

// generate synthetic edge planes from the current camera position
void MyGlWindow::generateSyntheticEdgeplanes ()
{
	// for each visible line, find out which sensors is supposed to see it
	// and generate the corresponding edge plane

	// clear existing edges
	_camera->clearEdges();

	for ( edgeVector::iterator iter = _LUT._lines.begin(); iter != _LUT._lines.end(); iter++ ) {
		Edge *line = *iter;

		intVector cameras;
		_camera->findSensors( line, cameras );

		EdgePlane best_ep = EdgePlane();
		int best_camera = -1;

		if ( cameras.empty() )
			continue;

		for (int i=0;i<cameras.size();i++) {
			int camera = cameras[i];

			Vec3d center = _camera->fromCameraFrameToWorldFrame( _camera->getUnitCameraCenter(camera) );
			Vec3d a = line->getA() - center;
			Vec3d b = line->getB() - center;
			EdgePlane ep = EdgePlane( a, b, center, camera, 0, 0 );
			ep.fromWorldFrameToCameraFrame( _camera->getPose() );
			if ( ep.length() < best_ep.length() )
				continue;
			best_ep = ep;
			best_camera = camera;
		}

		_camera->_frame._edgeplanes_chained[best_camera].push_back( best_ep );

	}
}

// generate synthetic edge planes from the current camera position
void MyGlWindow::generateSyntheticCorrespondences ( corresVector &cs )
{
	// for each visible line, find out which sensors is supposed to see it
	// and generate the corresponding correspondence
	//stRandomGenerators();

	// clear existing correspondences and edges
	_camera->clearEdges();
	cs.clear();
	
	for ( edgeVector::iterator iter = _LUT._lines.begin(); iter != _LUT._lines.end(); iter++ ) {
		
		Edge *line = *iter;
		assert( line != NULL );

		intVector cameras;
		_camera->findSensors( line, cameras );
		
		EdgePlane best_ep = EdgePlane();
		
		if ( cameras.empty() )
			continue;

		for (int i=0;i<cameras.size();i++) {
			int camera = cameras[i];
			
			Vec3d center = _camera->fromCameraFrameToWorldFrame( _camera->getUnitCameraCenter(camera) );
			Vec3d a = line->getA() - center;
			Vec3d b = line->getB() - center;
			EdgePlane ep = EdgePlane( a, b, center, camera, 0, 0 );
			ep.fromWorldFrameToCameraFrame( _camera->getPose() );
			if (  ep.length() < best_ep.length() )
				continue;
			best_ep = ep;
		}
		
		// generate noise if needed
		if ( NOISE_SYNTHETIC ) {
			double angle = toRadians( 2.0 * ( ran1( &_idum ) - 0.5 ) * NOISE_SYNTHETIC_ANGLE_STDEV + NOISE_SYNTHETIC_ANGLE_AVERAGE );
			Vec3d axis = norm( ( best_ep._a + best_ep._b ) / 2.0 );
			best_ep.rotate( axis, angle );
			angle = toRadians( 2.0 * ( ran1( &_idum ) - 0.5 ) * NOISE_SYNTHETIC_ANGLE_STDEV + NOISE_SYNTHETIC_ANGLE_AVERAGE );
			axis = norm( best_ep._b - best_ep._a );
			best_ep.rotate( axis, angle );
		}

		corres c = corres( line, best_ep, _CONNECTED, CONNECTED_MIN_AGE_FOR_POSE, 0.0 );
		c.computeScore( _camera->getPose() );
		//c.print();
		cs.push_back( c );
		
	}
}

// init synthetic mode by setting up synthetic edges or correspondences
//
bool MyGlWindow::initSynthetic ()
{
	if ( _synthetic_mode == 0 )
		return false;

	// print out the synthetic pose
	LOG( LEVEL_INFO, "generating synthetic correspondences for pose:");
	_camera->getPose().print();

	if ( ( _synthetic_mode != 0 ) ) { // synthetic edge planes
		corresVector cs;
		generateSyntheticCorrespondences( cs );
		SM_Clear();
		for (int i=0;i<cs.size();i++) 
			SM_InsertItem( cs[i] );
		return true;
	}

	return true;
}

// synthetic mode 3: generate synthetic correspondences
bool MyGlWindow::syntheticNextFrame () 
{
	if ( _synthetic_mode == 0 )
		return false;

	frameId++;

	if ( _synthetic_mode == 3 ) { 
		_camera->setPose( GetSyntheticPose(frameId) );

		//_camera->getPose().print();
		CorrespondenceVector inliers;
		corresVector cs;
		generateSyntheticCorrespondences( cs );
		SM_Clear();
		for (int i=0;i<cs.size();i++) {
			Correspondence c;
			c.first = cs[i].line;
			c.second = cs[i].head();
			SM_InsertItem( cs[i] ); // insert correspondence in queue for display only
			inliers.push_back( c );
		}
			
		// compute camera pose from perfect correspondences
		refineCameraPoseFromNCorrespondences( inliers );		
		
		// update the pose history
		ExtrinsicParameters pose = _camera->getPose();
		pose.id = frameId;
		pose_history.push_back( pose );

		// write down error
		writeError( "analysis.dat", frameId, _camera->getPose(), GetSyntheticPose( frameId ) );

		return true;
	}

	return false;
}

// return the synthetic pose for a given frame ID
// simple translation
ExtrinsicParameters MyGlWindow::GetSyntheticPose ( int frameId )
{

	Vec3d position = _synthetic_start_pose.getTranslation();

	double d = _LUT.inchToModelUnit( 2.0 );

	position = position + frameId * Vec3d(-d,0,0);

	ExtrinsicParameters pose = _synthetic_start_pose;
	pose.setTranslation( position );

	pose.setRotation( Quaternion ( ) );

	return pose;
}

// generate synthetic frame
void MyGlWindow::generateSyntheticFrame ()
{
	// save camera pose
	ExtrinsicParameters original_pose = _camera->getPose();

	// set camera pose at the next frame
	ExtrinsicParameters pose = GetSyntheticPose( frameId );
	_camera->setPose( pose );

	LOG( LEVEL_INFO, "generating frame %d at pose:", frameId);
	_camera->getPose().print();

	// generate synthetic images
	intVector cameras;
	_camera->makeSyntheticImage( _LUT._faces, _LUT._model->_faces.size(), cameras );
	_camera->_frame.processFrame();
	//frameId++;
	redraw();
	Fl::check();

	// reset camera pose
	_camera->setPose( original_pose );
	_camera->initializeEdgeDetectors();
}

void MyGlWindow::edgeDetectionErrorAnalysis ()
{	
	// for each visible model line, generate a synthetic image, run edge detector and
	// compare measured edge with expected edge
	Vec3d center = _camera->getTranslation();
	int i=0;

	Vertex *v = new Vertex( center[0], center[1], center[2] );
	FILE *f = fopen( "edge_detection_analysis.dat", "w" );
	doubleVector angles1, angles2, subangles;
	double average1=0.0, average2=0.0, std_dev1=0.0, std_dev2=0.0;
	double min1 = M_PI, max1 = 0.0, min2 = M_PI, max2 = 0.0;

	// create a table of lines
	int n = 0;

	edgeVector lines;
	for (i=0;i<_LUT._model->_edges.size();i++) {
		Edge *line = _LUT._model->_edges[i];
		line->computeWeight( center );
		lines.push_back( line );
	}

	std::sort( lines.begin(), lines.end(), std::greater<Edge*>() ); // sort the lines by decreasing subtended angle

	for ( edgeVector::iterator iter = lines.begin(); iter != lines.end(); iter++ ) {
		Edge *line = *iter;
		
		Fl::check();
		if ( !run_analysis->value() )
			break;

		// find out which cameras see this line
		intVector cameras;
		_camera->findSensors( line, cameras );
		if ( cameras.empty() )
			continue;

		for (int k=0;k<cameras.size();k++) {
			int camera = cameras[k]; // select the first sensor to see the line
			
			// generate ideal edge plane on the relevant camera
			Vec3d center = _camera->fromCameraFrameToWorldFrame( _camera->getUnitCameraCenter(camera) );
			Vec3d a = line->getA() - center;
			Vec3d b = line->getB() - center;
			EdgePlane ep = EdgePlane( a, b, center, camera, 0, 0 );
			ep.fromWorldFrameToCameraFrame( _camera->getPose() );
			
			if ( ep.length() < toRadians( 10.0 ) )
				continue;
			
			// make synthetic image with one single face
			Face *face = new Face( 0, v, line->_a, line->_b );
			faceVector faces;
			faces.push_back( face );
			_LUT._selected_lines.clear();
			_LUT._selected_lines.push_back( line );
			_camera->makeSyntheticImage( camera, faces, _LUT._model->_faces.size(), _camera->_frame._grab_img[camera] );
			_camera->_frame.processFrame();
			redraw();
			
			// detect edges
			_camera->clearEdges();
			_camera->clearMasks();
			_camera->initializeEdgeDetectors();
			_camera->detectLines ( true,true,edgePixels->value(),edgeThresh->value(),edgeSigma->value());
			
			// rectify lines and convert to edge planes
			_camera->convertLinesToPlanes();
			
			// select image planes matching the corresponding ideal plane
			// if more than one, select the best one
			double best_angle = M_PI;
			edgePlaneVector edgeplanes;
			EdgePlane rep;
			for (i=0;i<_camera->_frame._edgeplanes[camera].size();i++) {
				if ( ep.angle( _camera->_frame._edgeplanes[camera][i] ) < best_angle ) {
					best_angle = ep.angle( _camera->_frame._edgeplanes[camera][i] );
					rep = _camera->_frame._edgeplanes[camera][i];
				}
			}
			
			// threshold angle error
			if ( rep.angle( ep ) > toRadians( 5.0 ) )
				continue;
			
			if ( rep.length() < toRadians( 10.0 ) )
				continue;
			
			n++;
			
			// compute angular errors with respect to two normal vectors n1 and n2
			Vec3d n1 = norm( ( ep._a + ep._b ) / 2.0 );
			Vec3d n2 = norm( ep._b - ep._a );
			
			double subtended_angle = rep.length();
			double angle1 = toDegrees( fabs( Asin( dot ( cross( ep._normal, rep._normal ), n1 ) ) ) );
			double angle2 = toDegrees( fabs( Asin( dot ( cross( ep._normal, rep._normal ), n2 ) ) ) );
			
			LOG( LEVEL_INFO, "[%d/%d] angles: %.4f %.4f deg.", n, lines.size(), angle1, angle2 );
			
			fprintf( f, "%f %f %f\n", subtended_angle, angle1, angle2 );
			
			// update vectors
			angles1.push_back( angle1 );
			angles2.push_back( angle2 );
			subangles.push_back( subtended_angle );
			
			// update averages
			average1 += angle1;
			average2 += angle2;
			
			// update min and max
			min1 = MIN(angle1, min1);
			max1 = MAX(angle1, max1);
			min2 = MIN(angle2, min2);
			max2 = MAX(angle2, max2);
			
			delete face;
		}
	}
	
	delete v;

	fclose( f );

	// update average
	average1 /= angles1.size();
	average2 /= angles2.size();

	// compute stdev assuming a zero average (the data should be symmetric)
	for (i=0;i<angles1.size();i++) {
		std_dev1 += sqrt( (angles1[i] - 0.0) * (angles1[i] - 0.0)) / angles1.size();
		std_dev2 += sqrt( (angles2[i] - 0.0) * (angles2[i] - 0.0)) / angles2.size();
	}

	// compute histograms
	int hist1[100], hist2[100];

	for (i=0;i<100;i++) {
		hist1[i] = 0;
		hist2[i] = 0;
	}

	for (i=0;i<angles1.size();i++) {
		int index1 = MIN(99,int((angles1[i] - min1) / (max1-min1)));
		int index2 = MIN(99,int((angles2[i] - min2) / (max2-min2)));
		hist1[index1]++;
		hist2[index2]++;
	}

	// print out info
	LOG(LEVEL_INFO, "average min max stdev");
	LOG(LEVEL_INFO, "**********************");
	LOG(LEVEL_INFO, "%.6f %.6f %.6f %.6f", average1, min1, max1, std_dev1);
	LOG(LEVEL_INFO, "%.6f %.6f %.6f %.6f", average2, min2, max2, std_dev2);

	f = fopen( "edge_detection_histograms.dat", "w" );
	for (i=0;i<100;i++) {
		fprintf(f, "%d %d\n", hist1[i], hist2[i]);
	}
	fclose(f);

	// turn off the analysis light
	run_analysis->value(0);


/*
		// debug: print out detected edges
		//LOG(LEVEL_INFO, "observed edge:");
		Edge2D oedge = _camera->_frame._edges[camera][0];
		// merge edges
		//merge2DEdges( _camera->_frame._edges[camera], oedge );
		// pick the longest edge
		for (int k=1;k<_camera->_frame._edges[camera].size();k++) {
			if ( _camera->_frame._edges[camera][k].length() > oedge.length() )
				oedge = _camera->_frame._edges[camera][k];
		}
		Intrinsic intr = _camera->getIntrinsic(camera);
		//oedge.print();
		
		// save edge for display
		_oedge = oedge;
		
		FILE *f = fopen( "output.dat", "a" );
		fprintf(f, "%f %f %f %f\n",oedge._a[0],oedge._a[1],oedge._b[0],oedge._b[1]);
		fclose( f );
		
		// compare with measured edge plane
		Vec3d a3d,b3d;
		_camera->setActiveSensor( camera );
		ExtrinsicParameters pose = _camera->getPose();
		_camera->resetPose();
		_camera->unproject( oedge, a3d, b3d );
		center = _camera->getUnitCameraCenter(camera);
		_camera->setPose( pose );
		
		EdgePlane mep = EdgePlane(a3d - center, b3d - center ,center ,camera,0,0);
		
		// for display, insert items in correspondence table
		SM_Clear();
		corres c = corres( line, mep, _CONNECTED, 0.0 );
		c.computeScore( _camera->getPose() );
		SM_InsertItem( c );
		c = corres( line, ep, _CONNECTED, 0.0 );
		c.computeScore( _camera->getPose() );
		SM_InsertItem( c );
		
		if ( mep.length() <= 10 * M_PI / 180.0 )
			continue;
		
		double angle = ep.angle( mep );
		
		if ( angle > toRadians(5.0) ) {
			ep.print();
			mep.print();
		}
		
		Vec3d n1 = norm( ( ep._a + ep._b ) / 2.0 );
		Vec3d n2 = norm( ep._b - ep._a );
		
		double angle1 = toDegrees( fabs( Asin( dot ( cross( ep._normal, mep._normal ), n1 ) ) ) );
		double angle2 = toDegrees( fabs( Asin( dot ( cross( ep._normal, mep._normal ), n2 ) ) ) );
		
		
		double d = mep.length(); 
		//double d = Acos( fabs( dot( _camera->getZDirection( camera ), ep._normal ) ) );
		
		LOG( LEVEL_INFO, "angle error: %f %f [%d]", angle1, angle2, camera );
		fprintf( fa, "%f %f %f\n", 180.0 * d / M_PI, angle1, angle2 );
		//fprintf( fa, "%f\n", angle * 180.0 / M_PI );
	}

	// run analysis

	FILE *fin = fopen( "input.dat", "r" );
	FILE *fout = fopen( "output.dat", "r" );

	while ( !feof(fin) && !feof(fout) ) {

		double x1,y1,x2,y2;
		double error;
		if (fscanf(fin,"%lf%lf%lf%lf",&x1,&y1,&x2,&y2) != 4 )
			continue;
		Edge2D ein = Edge2D( Vec2d(x1,y1), Vec2d(x2,y2) );

		if (fscanf(fout,"%lf%lf%lf%lf",&x1,&y1,&x2,&y2) != 4 )
			continue;
		Edge2D eout = Edge2D( Vec2d(x1,y1), Vec2d(x2,y2) );

		if ( eout.length() < EPS )
			continue;

		Vec2d a,b;
		error = ein.p2p_distance( eout, a, b );

		//fprintf( fa, "%f %f %f %f %f\n",error,a[0],a[1],b[0],b[1] );
		//fprintf( fa, "%f %f \n %f %f\n",error,a[0],a[1],b[0],b[1] );
	}

	fclose( fa );
	fclose( fin );
	fclose( fout );
	*/
}

// measure the error between the expected pose and the measured pose
// and write down in a file to be displayed with matlab
//
void MyGlWindow::writeError( const char *filename, int frameId, ExtrinsicParameters &pose, ExtrinsicParameters &ref_pose )
{
	double t = len( pose.getTranslation() - ref_pose.getTranslation() ); // translation error in inches

	double angle = CompareTwoRotations( pose.getRotation(), ref_pose.getRotation() );

	FILE *f = fopen( filename, "a" );
	fprintf(f, "%d %f %f\n", frameId, t, angle );
	fclose( f );
}

// this method generates a synthetic image at a random location in the model
// and runs the INIT algorithm on it + returns statistics on the results
//
void MyGlWindow::run_synthetic_init( int nruns )
{	
	progress_bar->value(0);
	progress_bar->minimum(0);
	progress_bar->maximum(nruns);

	// insert INIT methods
	initModes.clear();
	initModes.push_back( VANISHING_POINTS);
	initModes.push_back( CORNERS);
	initModes.push_back( LINE_LINE);
	initModes.push_back( ROTATION_VOTING);
	initModes.push_back( EXHAUSTIVE_SEARCH );

	// init output file
	FILE *f = fopen( "init_runs.txt", "w");

	for (int run = 0; run < nruns; run++) {

		MyTimer timer;
		long int run_id = timer.timestamp();
		
		LOG(LEVEL_INFO, "run %d  run id = %d", run, run_id);

		// check that there exist a current 3D model
		if ( _LUT._model == NULL || _LUT.nodes.empty() )
			return;

		// pick a random node in the model (with bounded height)
		Vec3d position = _LUT._model->random_position( 0.0, INIT_MAX_CAMERA_HEIGHT );

		LOG(LEVEL_INFO, "position: %f %f %f", position[0], position[1], position[2] );

		// lookup the LUT at that location
		_LUT.lookup( position, 0.0, 0.0, 0, 0);

		// if the node has no neighbors, we probably picked a bad position in the structure so skip it
		if ( _LUT._node->_rns.empty() ) {
			run--;
			continue;
		}

		// pick a random rotation
		Quaternion q = random_rotation( _idum );
		
		// set the camera pose
		_camera->setTranslation( position );
		_camera->setRotation( q );
		Vec3d rotation;
		q.toAxisAngle( rotation );
		LOG(LEVEL_INFO, "%f %f %f %f %f %f %d", position[0], position[1], position[2], rotation[0], rotation[1], rotation[2] );

		// generate a synthetic image
		intVector cameras;
		_camera->makeSyntheticImage(_LUT._faces, _LUT._faces.size(),cameras);

		// process frame
		_camera->_frame.processFrame( run_id );

		ExtrinsicParameters initial_pose = _camera->getPose();

		// set the light to on
		init->value(1);

		// run the init algorithm
		init_pose();

		// process the output (essentially, save the best solution found and measure error with initial camera pose

		// print run id, model name, initial camera position and success flag (1 if successful, 0 otherwise)
		bool success = !init_poses.empty();
		fprintf(f, "%d %s %f %f %f %f %f %f %d", run_id, _LUT._model->_dirname.c_str(), position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], success );
		LOG(LEVEL_INFO, "%d %s %f %f %f %f %f %f %d", run_id, _LUT._model->_dirname.c_str(), position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], success );

		if ( !success ) {
			fprintf(f, "\n");
			continue;
		}

		// print translation and rotation error for the best pose, position in the list of solutions, method to find it, score, time to find it
		int best_solution = 0;
		double best_score = 1E10;
		ExtrinsicParameters pose;

		LOG(LEVEL_INFO, "found %d poses for %d modes", init_poses.size(), initModes.size() );
		
		for (int i=0;i<init_poses.size();i++) {

			pose = init_poses[i];

			double translation_error = initial_pose.translation_error( pose );
			double rotation_error = initial_pose.rotation_error( pose );

			double score = sqrt( translation_error * translation_error / ( _LUT.GRID_STEP * _LUT.GRID_STEP ) ) + sqrt( rotation_error * rotation_error / (M_PI * M_PI ) );

			if ( score < best_score ) {

				best_score = score;
				best_solution = i;
			}
		}

		pose = init_poses[best_solution];


		Vec3d ft, fr;
		ft = pose.getTranslation();
		pose.getRotation().toAxisAngle( fr );

		fprintf( f, " %f %f %f %f %f %f %.2f %.2f %d %s %.3f %d\n", ft[0], ft[1], ft[2], fr[0], fr[1], fr[2], initial_pose.translation_error( pose ), initial_pose.rotation_error( pose ), best_solution, pose.get_mode_string().c_str(), 
			pose.get_score(), difftime( initial_pose.get_ctime(), pose.get_ctime() ) );
		LOG( LEVEL_INFO, " %f %f %f %f %f %f %.2f %.2f %d %s %.3f %d\n", ft[0], ft[1], ft[2], fr[0], fr[1], fr[2], initial_pose.translation_error( pose ), initial_pose.rotation_error( pose ), best_solution, pose.get_mode_string().c_str(), 
			pose.get_score(), difftime( initial_pose.get_ctime(), pose.get_ctime() ) );

		// update progress bar
		progress_bar->maximum(nruns);
		progress_bar->value( run );
		Fl::check();
	}

	progress_bar->value(0);

	fclose( f );

}
