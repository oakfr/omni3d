#include "camera/camera.h"
#include "my_colors.h"

/*Vec3d Camera::getCameraTarget( SphericalCoord orientation_eye)
{ 
	Vec3d a;
	a[0] = orientation_eye.rho * sin (orientation_eye.phi) * cos (orientation_eye.theta);
	a[1] = orientation_eye.rho * sin (orientation_eye.phi) * sin (orientation_eye.theta);
	a[2] = orientation_eye.rho * cos (orientation_eye.phi);
	return a;
}

Vec3d Camera::getCameraUp (SphericalCoord orientation_eye)
{
	Vec3d up;
	up[0] = -orientation_eye.rho*cos(orientation_eye.phi)*cos(orientation_eye.theta);
	up[1] = -orientation_eye.rho*cos(orientation_eye.phi)*sin(orientation_eye.theta);
	up[2] = orientation_eye.rho*sin(orientation_eye.phi);
	return up;
}

void Camera::resetEye(int mode, ModelBoundingBox modelbbox) {
	
	if (mode == 0) { // reset viewer orientation
		orientation_eye.theta = 0;
		orientation_eye.phi = 3*M_PI/4;
		orientation_eye.rho = 2 * modelbbox.diam;
		orientation_eye.pitch = 0.0;
		
		eyep.target = modelbbox.box;
		eyep.up = Vec3d(0,0,1);
		
		zoomFactorEye = 1.0;
	}
	
	if (mode == 1) { // reset camera orientation
		orientation_cam.theta = 0.0;
		orientation_cam.phi = M_PI/2;
		orientation_cam.rho = 1.0; // orientation vector is unit
		orientation_cam.pitch = 0.0;

		eyec.eye = modelbbox.box;
		eyec.eye[2] = 68.0; // traditional human height...
		eyep.up = Vec3d(0,0,1);
		eyec.target[0] = sin (orientation_cam.phi) * cos (orientation_cam.theta);
	    eyec.target[1] = sin (orientation_cam.phi) * sin (orientation_cam.theta);
	    eyec.target[2] = cos (orientation_cam.phi);
		
	}
}*/

/* Draw a camera */
void Camera::draw (Viewer &viewer, double radius, const float color[3])
{
	//viewer.drawSphere(came.translation,radius,color,10,10);
	viewer.drawSphere(getTranslation(),radius,YELLOW,10,10);
}

/* Init the camera parameters */
void Camera::init (CameraType type, Ladybug *ladybug, int w, int h, double zoomFactor, Vec3d position, Mat3d rotation, 
				   double SPHERE_TESSELLATION_RESOLUTION) 
{
	
	_cameraType = type;
	_width = w;
	_height = h;
	//_pose.eye = Vec3d(0,0,0);
	_pose.eye = position;
	_pose.up = Vec3d(0,0,1);
	_pose.target = Vec3d(1,0,0);
	_ladybug = ladybug;
	_min_mask_width = 5;
	_max_mask_width = 60; // pixels

	if ((type == CANON_ELURA)||(type == SONY_DFW_V500)) {
		
		readIntrinsic(1); // 1 image
		
		cami.cameraMatrix[0][0] = 952.23952;	// focal length in pixels
		cami.cameraMatrix[1][1] = 869.79232;
		cami.cameraMatrix[0][1] = 0.0;		// skew
		cami.cameraMatrix[0][2] = 361.54911;	// principal point
		cami.cameraMatrix[1][2] = 236.32996;
		cami.cameraMatrix[2][2] = 1.0;
		cami.cameraMatrix[1][0] = 0.0;
		cami.cameraMatrix[2][0] = 0.0;
		cami.cameraMatrix[2][1] = 0.0;
		
		// used for frustrum display in the 3-D model
		kcam.x0 = cami.cameraMatrix[0][2]; 
		kcam.y0 = cami.cameraMatrix[1][2];
		kcam.ax = cami.cameraMatrix[0][0];
		kcam.ay = cami.cameraMatrix[1][1];
		
		_frame.set(type,w,h,1,zoomFactor);

		setActiveSensor(0);

	} else if (type == LADYBUG) {
		
		readIntrinsic(6); // 6 images

		for (int j=0;j<6;j++) {
			Matd M, Minv;
			computeUnitExtrinsic( j, _ladybug_extrinsic[j][0], _ladybug_extrinsic[j][1], _ladybug_extrinsic[j][2], \
				_ladybug_extrinsic[j][3], _ladybug_extrinsic[j][4], _ladybug_extrinsic[j][5], M, Minv );
			_extrinsic.push_back( Minv );
			_extrinsic_inv.push_back( M );
		}

		setActiveSensor(0);

		came.setTranslation(position);

		came.setRotation(Quaternion());

	// check the whole pipeline
		LOG( LEVEL_INFO, "check pipeline" );
		Intrinsic intr = getIntrinsic(0);
		Vec2d t = Vec2d (250.0,70.0);
		Vec2d t_normalized = intr.normalize(t);
		Vec2d t_rectified = intr.rectify(t_normalized);
		Vec3d t_unproject = unproject(t_rectified);
		Vec2d t_rectified_2 = project(t_unproject);
		Vec2d t_normalized_2 = intr.distort(t_rectified_2);
		Vec2d t_2 = intr.denormalize(t_normalized_2);
		LOG(LEVEL_DEBUG,"check:");
		printMatrix(t);
		printMatrix(t_normalized);
		printMatrix(t_rectified);
		printMatrix(t_unproject);
		printMatrix(t_rectified_2);
		printMatrix(t_normalized_2);
		printMatrix(t_2);

		getIntrinsic(0).distorsionChecker(); // for debug only

		Vec3d point = unproject(Vec2d(1,0));
		point = point * getIntrinsic().getFocal() / point[2];
		LOG(LEVEL_DEBUG,"(1,0) => (%f,%f)",point[0],point[1]);
		point = unproject(Vec2d(-1,0));
		point = point * getIntrinsic().getFocal() / point[2];
		LOG(LEVEL_DEBUG,"(-1,0) => (%f,%f)",point[0],point[1]);
		
		_frame.set(LADYBUG,w,h,6,zoomFactor);

		// init edge detector
		_edge_image = new ColorImage( _frame._tmp_img[0]->width,_frame._tmp_img[0]->height);
		_edge_mask_image = new ByteImage( _frame._tmp_img[0]->width,_frame._tmp_img[0]->height);
		memset(_edge_mask_image->getPixels(), 0, _edge_mask_image->getHeight()*_edge_mask_image->getPitch());
		for (int i=0;i<6;i++) {
			_m_EdgeDetector[i] = new CEdgeDetector(_height,_width);
			_m_EdgeDetector[i]->SetMaskImage(*_edge_mask_image);
			_mask_imgs.push_back(cvCreateImage(cvSize(int(_height), int(_width)),  IPL_DEPTH_8U, 1));
		}
	}

	// check geometry
	// pick two points randomly, rotate them and find the rotation that brings them back in position
	/*
	LOG(LEVEL_INFO, "checking minRotationFourPoints...");
	for (int i=0;i<1000;i++) {
		Vec3d a = norm(Vec3d((double)rand()/(RAND_MAX+1), (double)rand()/(RAND_MAX+1), (double)rand()/(RAND_MAX+1) ) );
		Vec3d b = norm(Vec3d((double)rand()/(RAND_MAX+1), (double)rand()/(RAND_MAX+1), (double)rand()/(RAND_MAX+1) ) );
		Vec3d rot = ( M_PI / 4.0 + M_PI / 2.0 * (double)rand() / (RAND_MAX+1) ) * norm(Vec3d((double)rand()/(RAND_MAX+1), (double)rand()/(RAND_MAX+1), (double)rand()/(RAND_MAX+1) ));
		//a = Vec3d(0.091807, 0.851077, 0.516953);
		//b = Vec3d(0.929713, 0.098019, 0.355003);
		//rot = Vec3d(0.000042, 0.000077, 0.000039);
		Quaternion qrot = Quaternion( rot, len(rot) );
		Vec3d ar = qrot.rotate( a );
		Vec3d br = qrot.rotate( b );
		Vec3d rot2;
		minRotationFourPoints( a, b, ar, br, rot2 );
		double check = Acos(dot(norm(rot2),norm(rot)));
		if ( check > 1E-5) {
			LOG(LEVEL_INFO, "error checking minRotationFourPoints function() error = %f", check);
			printf("a = (%f, %f, %f)    b = (%f, %f, %f) rot = (%f, %f, %f)\n", a[0], a[1], a[2], b[0], b[1], b[2], rot[0], rot[1], rot[2]);
			assert(false);
		}
	}
	LOG(LEVEL_INFO, "done.");
	*/

	came.setTranslation(position);
	//came.rotation = rotation;
	came.setRotation(Quaternion());

	_max_rotation_speed = toRadians(18.0); // maximum rotation angle between two frames, in radians.  5 frames = 90 degrees => 18 degrees
	_max_translation_speed = 200.0; // maximum translation between two frames, in mm. 5 frames = 1 meter => 200 mm
	_average_edge_depth = 200.0; // 200 inches

	// initialize rectification display with some typical values
	_rectification_display_scale = 3.4;
	_rectification_display_trans = Vec2d(-600,600);

	int i;
	for (i=0;i<_frame._nImages;i++) {
		CEdgeDetectorEdgeMaskVector vector;
		_edge_masks.push_back(vector);
		CEdgeDetectorEdgeMaskVector vector2;
		_save_edge_masks.push_back(vector);
		
	}

	// init the sphere tessellation
	std::vector<Vec3d> points;
	spherets = new Model();
	spherets->MODEL_RESOLUTION = 0.0;
	spherets->make_icosahedron( points );
	spherets->tessellate( SPHERE_TESSELLATION_RESOLUTION, points );
	
}

/* Clear the list of reprojected edges*/
void Camera::clearReprojectedEdges()
{
	for (int i=0;i<_frame._edges_reprojected.size(); i++)
		_frame._edges_reprojected[i].clear();
}

/* Add a small perturbation in the camera pose
 * This used to debugging/testing only */
void Camera::perturbation ( double angle, double translation ) 
{
	// pick a rotation axis randomly
	Vec3d v = norm( Vec3d( (double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX ) );

	// pick an angle randomly
	double a = (double)rand() / RAND_MAX * angle;
	
	// pick a translation direction randomly
	Vec3d t = norm( Vec3d( (double)rand()/RAND_MAX, (double)rand()/RAND_MAX, (double)rand()/RAND_MAX ) );
	
	// pick a translation value randomly
	double d = (double)rand() / RAND_MAX * translation;

	// rotate, translate
	rotate(v,a);
	translate(t * d);
}

/* Clear all lists of edges */
void Camera::clearEdges()
{
	for (int i=0;i<_frame._nImages;i++) {
		_frame._edges[i].clear();
		_frame._edgeplanes[i].clear();
		_frame._edgeplanes_chained[i].clear();
		_frame._edges_reprojected[i].clear();
	}

	clearSelectedEdges();
}

/* Clear the list of selected edges */
void Camera::clearSelectedEdges()
{
	_frame._selected_edges.clear();
}

/* Read the camera intrinsic parameters from a calibration file */
void Camera::readIntrinsic(int nImages)
{
	FILE *file;
	file = fopen(CALIBRATION_FILE,"r");

	if (file == NULL) {
		printf("Error: camera calibration file not found.\n");
		assert(false);
	}

	_intr.clear();
	_intr_dft.clear();

	while (!feof(file)) {

		int id=0;
		if (fscanf(file,"%d",&id) != 1)
			return;
	
		char *str = (char*)malloc(50*sizeof(char));
		fscanf(file,"%s",str);

		if (strcmp(str,"POLYNOMIAL") == 0) {
			printf("error: the POLYNOMIAL calibration format is not supported.\n");
		} else if (strcmp(str,"SPHERICAL") == 0) {
			int w,h;
			double value,f,cx,cy,pixel_w_mm,pixel_h_mm;
			if ( fscanf(file,"%d%d%lf%lf%lf%lf%lf%lf",&w,&h,&value,&f,&cx,&cy,&pixel_w_mm,&pixel_h_mm) != 8)
				printf("error Camera::readIntrinsic\n");
			_intr.push_back(Intrinsic(id,w,h,f,cx,cy,value,pixel_w_mm,pixel_h_mm));
			_intr_dft.push_back(Intrinsic(id,w,h,f,cx,cy,value,pixel_w_mm,pixel_h_mm));
		} else {
			printf("error: unknown distortion model!\n");
			assert(false);
		}

		delete str;
	}

	fclose(file);
}

/* Make a copy of the intrinsic parameters for manual modification by the user */ 
void Camera::readIntrinsic(int nImages, int sensorId)
{
	_intr[sensorId] = _intr_dft[sensorId];
}

/* Draw the edge masks on the overlay 
 * Masks are deprecated in the latest version of the application 
 */
void Camera::drawMasks (int cameraId, int x, int y, double scale, bool flip)
{
	//double scale = _frame._display_zoom_factor;
	for (int i=0;i<_edge_masks[cameraId].size();i++) {
		_edge_masks[cameraId][i].draw(x,y,_m_EdgeDetector[cameraId]->getWidth(),_m_EdgeDetector[cameraId]->getHeight(),scale,ORANGE,0.25,true,flip,4);
	}
}

/* Set the masks of the edge detector
*/
void Camera::setMasks(int cameraId)
{
	_m_EdgeDetector[cameraId]->clearMasks();

	for (int i=0;i<_edge_masks[cameraId].size();i++) {
		_m_EdgeDetector[cameraId]->AddMask(_edge_masks[cameraId][i]);
	}
}

/* Add a mask for a given edge based on:
 * (1) maximum translational velocity
 * (2) maximum rotational velocity
 * (3) edge depth
 * the mask width given as input acts as a minimum width
 */
void Camera::AddMask(int cameraId, Edge *line, Edge2D edge, bool debug_save)
{
	Intrinsic intr = getIntrinsic(cameraId);
	double depth;
	
	if (line == NULL) {
		depth = _average_edge_depth; // no line info => use average depth
	} else {
		depth = line->shortest_distance( getTranslation() ); // compute edge depth
	}

	Vec2d a,b,c,d;
	intr.computeEdgeBoundingBox(edge,_max_rotation_speed,_max_translation_speed, depth, a, b, c, d, _min_mask_width, _max_mask_width);
	int w = intr.getWidth();
	int h = intr.getHeight();

	_edge_masks[cameraId].push_back(CEdgeDetectorEdgeMask(a,b,c,d,w,h));

	if (debug_save)
		_save_edge_masks[cameraId].push_back(CEdgeDetectorEdgeMask(a,b,c,d,w,h));

}

/* tests whether the camera pose given as input is acceptable for the next (or previous) frame
 * assuming speed limits in term of rotation and translation
 * right now the test is purely on translation
 */
bool Camera::testCameraMotion( ExtrinsicParameters &pose )
{
	if ( inchTomm(len( getTranslation() - pose.getTranslation())) > 2.0 * _max_translation_speed )
		return false;

	return true;
}

/* reproject an edge plane into a set of 2D edges
 * taking distortion into account
 */
int Camera::reprojectEdgePlane (int cameraId, int id, EdgePlane edgeplane, int edge_step, edge2DVector &edges_2d)
{		
	Vec2d a,b,a_rect,b_rect;
	double radius = 50.0;
	edges_2d.clear();
	if (edgeplane._cameraId != cameraId)
		return 0;

	projectAndSubdivide( cameraId, radius*edgeplane._a, radius*edgeplane._b, edge_step, edges_2d);

	for (edge2DVector::iterator iter = edges_2d.begin(); iter != edges_2d.end(); iter++ ) {
		iter->_lineId = id;
		iter->_type = CORRESPONDENCE;
	}

	return edges_2d.size();
}

/* Reproject an set of edgeplanes on the camera plane and draw it
 * the edgeplane is assumed to be expressed in the camera coordinate frame
 */
void Camera::drawEdgeplanes (Viewer &viewer, int cameraId, edgePlaneVector edgeplanes, int edge_step,
							int x, int y, double zoom, bool flipVert, bool flipHoriz,
							int h, int w, int linewidth)
{		
	for (int i=0;i<edgeplanes.size();i++) {
				
		int _cameraId = edgeplanes[i]._cameraId;
		
		if ( cameraId != _cameraId ) {
			continue;
		}
		
		drawEdgeplane( viewer, edgeplanes[i], edge_step, x, y, zoom, _colors[cameraId], flipVert, flipHoriz, h, w, cameraId, linewidth );
	}
}

/* Same as above but in select mode
 */
void Camera::drawEdgeplanesSelectMode (Viewer &viewer, int cameraId, edgePlaneVector edgeplanes, int edge_step,
							int x, int y, double zoom, bool flipVert, bool flipHoriz,
							int h, int w)
{		
	if ( viewer._mode != GL_SELECT )
		return;
	
	glInitNames();

	for (int i=0;i<edgeplanes.size();i++) {
				
		int _cameraId = edgeplanes[i]._cameraId;
		
		if ( cameraId != _cameraId ) {
			continue;
		}
		
		glPushName (cameraId);
		glPushName (i);

		drawEdgeplane( viewer, edgeplanes[i], edge_step, x, y, zoom, _colors[cameraId], flipVert, flipHoriz, h, w, cameraId, 2 );
		
		glPopName();
		glPopName();
	}
}



/* Reproject an edgeplane on the camera plane and draw it
 * the edgeplane is assumed to be expressed in the camera coordinate frame
 */
void Camera::drawEdgeplane (Viewer &viewer, EdgePlane edgeplane, int edge_step,
							int x, int y, double zoom, const float color[3], bool flipVert, bool flipHoriz,
							int h, int w, int cameraId, int linewidth)
{
	// reset the camera pose since the edgeplane is supposed to be expressed in the camera coordinate frame
	resetPose();

	EdgePlane ep = edgeplane;

	if (cameraId != ep._cameraId) {
		restorePose();
		return;
	}

	int id = ep._uid;
	edge2DVector edges;
	
	// project the edgeplane on the camera image
	reprojectEdgePlane( cameraId, id, ep, edge_step, edges);

	// draw the edgeplane
	viewer.drawEdges( edges, x, y, zoom, color, flipVert, flipHoriz, h, w, cameraId , linewidth );

	restorePose();
}

/* Clear all masks in the edge detector
 */
void Camera::clearMasks()
{
	for (int i=0;i<_edge_masks.size();i++) {
		_edge_masks[i].clear();
		_m_EdgeDetector[i]->clearMasks();
	}

}

/* compute the angle histogram of all visible edges
 */
void Camera::computeAngleHistogram ( Histogram &hist )
{
	int n = hist.getSize();

	if ( n == 0 )
		return;

	hist.reset();

	edgePlaneVector planes;
	for ( int i=0; i < _frame._edgeplanes_chained.size(); i++ ) {
		for ( int j=0; j < _frame._edgeplanes_chained[i].size(); j++ ) {
			planes.push_back( _frame._edgeplanes_chained[i][j] );
		}
	}

	for ( i=0;i<planes.size();i++ ) {
		for ( int j=i+1;j<planes.size();j++ ) {

			double val = MAX( 0.0, MIN( 1.0, planes[i].angle( planes[j] ) / M_PI ) );  // 0 < val < 1
			
			int index = int( val * n );

			if ( index == n )
				index--;

			hist.Increment( index );
		}
	}

	hist.normalize();
}

/* Print camera pose to standard output
 */
void Camera::printPose()
{
	LOG(LEVEL_INFO,"%f %f %f",came.getTranslation()[0],came.getTranslation()[1],came.getTranslation()[2]);
	came.getRotation().print();
}

/* Main method for edge detection
 */
void Camera::detectLines(bool first_time, bool save_debug, int m_nMinEdgePixels, double m_dfEdgeThreshold, double m_dfEdgePixelSigma)
{
	for (int img=0;img<_frame._nImages;img++)
		detectLines(first_time,save_debug,img,m_nMinEdgePixels,m_dfEdgeThreshold,m_dfEdgePixelSigma);
}

/* Call edge detector */
void Camera::detectLines(bool first_time, bool save_debug, int cameraId, int m_nMinEdgePixels, double m_dfEdgeThreshold, double m_dfEdgePixelSigma)
{
	_m_EdgeDetector[cameraId]->m_dfEdgePixelSigma = m_dfEdgePixelSigma;
	_m_EdgeDetector[cameraId]->m_dfEdgeThreshold = m_dfEdgeThreshold;
	_m_EdgeDetector[cameraId]->m_nMinEdgePixels = m_nMinEdgePixels;
	
	setMasks(cameraId);

	_m_EdgeDetector[cameraId]->Run(first_time);
	
	// save mask image for display
	//	memcpy(_mask_imgs[cameraId]->imageData,_m_EdgeDetector[cameraId]->m_DisplayMask.getImageData(),_width*_height);

	_frame._edges[cameraId].clear();
	int h = _m_EdgeDetector[cameraId]->getHeight();

	for (int i=0;i<_m_EdgeDetector[cameraId]->m_Edges.size();i++) {
		ColVector2 start,end;
		start = _m_EdgeDetector[cameraId]->m_Edges[i].p_Start;
		end = _m_EdgeDetector[cameraId]->m_Edges[i].p_End;
				
		int id = _frame._edges[cameraId].size();

		_frame._edges[cameraId].push_back(Edge2D(Vec2d(start[0],h-1-start[1]),Vec2d(end[0],h-1-end[1]),id));

		// in debug save mode, write edges on the image
		if (save_debug) {
			//CvPoint cv_p1 = { start[0],start[1] };
			//CvPoint cv_p2 = { end[0],end[1] };
			//cvLine( _frame._copy_img[cameraId], cv_p1, cv_p2, CV_RGB( 255,0,0 ) );
		}
	}

	// in debug save mode, save the image in local directory
	//if ( save_debug ) {
	//	char filename[20];
	//	sprintf(filename,"%d.bmp", cameraId);
	//	cvSaveImage( filename, _frame._copy_img[cameraId] );
	//	LOG(LEVEL_INFO, "debug mode: saved image in %s", filename );
	//}

	//LOG(LEVEL_DEBUG,"[%d]: %d edges",cameraId,_frame._edges[cameraId].size());
	
}

/* Initialize the edge files */
void Camera::initEdgesFiles( int ladybug_id, int w, int h )
{
	FILE *f = fopen( fname_edges.c_str(), "w" );
	if ( f != NULL )
		fclose( f );

	f = fopen( fname_edges_filtered.c_str(), "w" );
	if ( f != NULL )
		fclose( f );

	f = fopen( fname_edges_ref.c_str(), "w" );
	if ( f != NULL ) {
		fprintf( f, "%d %d %d\n", ladybug_id, w, h );
		fclose( f );
	}

}
	
/* Write edges to reference file:
 * <ladybug_id> <width> <height>
 * for each frameid: <frame id> <sensor id> <pointer to edges file> <n edges> <pointer to filtered edges file> <n edges>
 */
bool Camera::writeEdgesToFile( int frameid )
{
	// for each sensor
	for (int sensorid = 0; sensorid < 6; sensorid++ ) {
		
		// open the file in append mode
		FILE *f = fopen( fname_edges.c_str(), "a" );
		
		if ( f == NULL )
			return false;
		
		// get the position of edges file	
		setbuf(f,NULL);
		fprintf(f," ");
		fflush( f );
		long int pos_edges = ftell ( f );
		
		fclose ( f );
		
		f = fopen( fname_edges_filtered.c_str(), "a" );
		
		if ( f == NULL )
			return false;
		
		// get the position of edges filtered file		
		setbuf(f,NULL);
		fprintf(f," ");
		fflush( f );
		long int pos_edges_filtered = ftell ( f );
		
		fclose ( f );
		
		int n1 = _frame._edges[sensorid].size();
		int n2 = _frame._edgeplanes_chained[sensorid].size();
		
		// print pointers into ref file
		FILE *fref = fopen( fname_edges_ref.c_str(), "a" );
		fprintf(fref, "%d %d %d %d %d %d\n", frameid, sensorid, pos_edges, n1, pos_edges_filtered, n2 );
		printf("%d %d %d %d %d %d\n", frameid, sensorid, pos_edges, n1, pos_edges_filtered, n2 );
		fclose( fref );

		// print edges
		f = fopen( fname_edges.c_str(), "a" ); 

		for (int i=0;i<n1;i++) {
			_frame._edges[sensorid][i].print( f );
		}
	
		fprintf( f, "\n" );
		fclose ( f );

		// print filtered edges
		f = fopen( fname_edges_filtered.c_str(), "a" );

		for (i=0;i<n2;i++) {
			_frame._edgeplanes_chained[sensorid][i].print( f );
		}

		fprintf( f, "\n" );
		fclose ( f );
	}

	return true;
}

/* Read edges from file */
bool Camera::readEdgesFromFile( int frameid )
{
	if ( fname_edges_ref.empty() )
		return false;

	FILE *f = fopen( fname_edges_ref.c_str(), "r" );

	if ( f == NULL )
		return false;

	int r_w, r_h, r_ladybug_id;

	if ( fscanf(f, "%d%d%d", &r_ladybug_id, &r_w, &r_h ) != 3 ) {
		fclose( f );
		return false;
	}

	// check consistency on width and height
	if ( r_w != _height || r_h != _width ) {
		LOG(LEVEL_ERROR, "Inconsistency in width and height. Read %d x %d in edge file, while current values are %d x %d!", r_w, r_h, _height, _width );
		fclose ( f );
		return false;
	}

	// read edges
	int r_frameid, r_sensorid, n1, n2;
	long int pos_1, pos_2; // pointers to edges file and edges filtered file

	while ( 1 ) {

		if ( fscanf( f, "%d %d %d %d %d %d", &r_frameid, &r_sensorid, &pos_1, &n1, &pos_2, &n2 ) != 6 ) {
			break;
		}

		if ( r_frameid == frameid ) 
			break;
	}

	// didnt find the frame id in reference file
	if ( r_frameid != frameid ) {
		LOG(LEVEL_INFO, "could not find edges for frame id = %d", frameid );
		fclose( f );
		return false;
	}

	// continue reading...
	_frame.n_edgeplanes_chained = 0;

	for (int sensorid = 0; sensorid < 6; sensorid++ ) {
		
		FILE *f_edges = fopen( fname_edges.c_str(), "r" );
		
		if ( f_edges == NULL )
			return false;
		
		// read edges
		if ( fseek( f_edges, pos_1 - 1, SEEK_SET ) != 0 ) {
			fclose ( f );
			fclose( f_edges );
			return false;
		}
		
		for (int i=0;i<n1;i++) {
			Edge2D edge;
			if ( !edge.read( f_edges ) ) {
				fclose ( f );
				fclose( f_edges );
				return false;
			}
			
			_frame._edges[sensorid].push_back( edge );
		}

		// read edges filtered

		FILE *f_edges_filtered = fopen( fname_edges_filtered.c_str(), "r" );

		if ( f_edges_filtered == NULL ) {
			fclose ( f );
			fclose ( f_edges) ;
			return false;
		}

		if ( fseek( f_edges_filtered, pos_2 - 1, SEEK_SET ) != 0 ) {
			fclose ( f );
			fclose( f_edges );
			fclose( f_edges_filtered );
			return false;
		}

		for (i=0;i<n2;i++) {
			EdgePlane edgeplane;
			if ( !edgeplane.read( f_edges_filtered ) ) {
				fclose ( f );
				fclose( f_edges );
				fclose( f_edges_filtered );
				return false;
			}

			_frame._edgeplanes_chained[sensorid].push_back( edgeplane );

			_frame.n_edgeplanes_chained++;
		}
		
		// read the new file positions for next sensor
		if ( fscanf( f, "%d %d %d %d %d %d", &r_frameid, &r_sensorid, &pos_1, &n1, &pos_2, &n2 ) != 6 ) {
			fclose( f );
			fclose( f_edges );
			return false;
		}

		fclose ( f_edges );
		fclose ( f_edges_filtered );

	}

	fclose ( f );

	return true;
}

/* initialize each edge detector with new image (compute Laplacian of Gaussian)
 * each edge detector will later be queried during the maintenance phase
 */
void Camera::initializeEdgeDetectors()
{
	for ( int cameraId = 0; cameraId <_frame._nImages; cameraId++ ) {
				
		// set color image
		Intrinsic intrinsic = getIntrinsic();
		int w = intrinsic.getWidth();//_frame._tmp_img[cameraId]->width;
		int h = intrinsic.getHeight();//_frame._tmp_img[cameraId]->height;
		memcpy(_edge_image->m_pixels,_frame._tmp_img[cameraId]->imageData,3*w*h);
		_m_EdgeDetector[cameraId]->SetColorImage(*_edge_image);
		
		
		// set projection function
		setActiveSensor(cameraId);
		
		Vec2d center = intrinsic.denormalize(Vec2d(intrinsic.getCenterX(),intrinsic.getCenterY()));
		_m_EdgeDetector[cameraId]->initProjFunction(CProjectionFunc::planar,w,h,intrinsic.getFocalX(),intrinsic.getFocalY(),\
			0.0,center[0],center[1]);
		
		// compute Laplacian of Gaussian
		_m_EdgeDetector[cameraId]->ComputeLaplacianOfGaussian();
		
	}
}

/* Compute the corners
 * min_angle: min angle (in radians) between two edges to validate a corner
 * max_dist: max distance between a point and a line to make a connection (in pixels)
 */
void Camera::computeCorners( int frameid, int max_dist, double min_angle )
{
	resetPose();

	int j;

	PerfTimer timer;

	// clear feature points
	_frame.clearFeaturePoints();

	for (int cameraid = 0; cameraid < 6; cameraid++ ) {

		setActiveSensor( cameraid );

		// compute the feature points
		std::vector<Vec2d> points;

		_frame.computeFeaturePoints( cameraid, frameid, points );


		// unproject the points in the 3D camera coordinate frame
		std::vector< Vec3d > points_3d;

		for (j=0;j<points.size();j++)
			points_3d.push_back( rectifyAndUnproject( points[j] ) );

		// compute the corners
		// pass the sphere tessellation model for sphere subdivision

		_frame.computeCorners( cameraid, points_3d, _frame._edgeplanes_chained[cameraid], getIntrinsic().from_pixels_to_radians( max_dist ), min_angle, spherets );

	}

	restorePose();

	//timer.print( "skeleton computation" );
}

/* Convert a set of 2D image edges (lines) into a list of edge planes */
void Camera::convertLinesToPlanes ()
{
	int i,j;

	ExtrinsicParameters pose = getPose();
	int sensor = _activeSensor;

	resetPose();

	for (i=0;i<_frame._nImages;i++) {

		setActiveSensor(i);

		edgePlaneVector edges;
		unproject( _frame._edges[i], edges, getUnitCameraCenter(i) );

		_frame._edgeplanes[i].clear();

		for (j=0;j<edges.size();j++) {
			_frame._edgeplanes[i].push_back( edges[j] );
		}
	}

	setPose( pose );

	setActiveSensor( sensor );

	return;

}

/* Convert a list of "3D edges" into a list of planes */
void Camera::convert3DEdgesToEdgePlanes (int n, std::vector<edgePlaneVector> edges3D, std::vector<edgePlaneVector> &edgeplanes)
{

	for (int i=0;i<n;i++) {
		edgeplanes[i].clear();
		edgePlaneVector edges;
		for (int k=0;k<edges3D[i].size();k++)
			edges.push_back(edges3D[i][k]); 

		_frame.convert3DEdgesToEdgePlanes(i,edges,edgeplanes[i],getUnitCameraCenter(i));
	}
}

/* Compute the center point of a sensor */
Vec3d Camera::getCenterPoint(int sensorId)
{
	Vecd c = getUnitExtrinsic(sensorId) * Vecd(4,0.0,0.0,0.0,1.0);
	c = c/c[3];
	return Vec3d (c[0],c[1],c[2]);
}

/* Draw a camera frustum */
void Camera::drawFrustum (const float color[3], bool fill)
{
	if (_cameraType == LADYBUG) {

		int currentSensorId = _activeSensor;
		glColor(color);
		glLineWidth(1);
		for (int sensorId = 0; sensorId < 6;sensorId++) {
			setActiveSensor (sensorId);
			Vec3d eye = fromCameraFrameToWorldFrame(Vec3d(0,0,0));
			Vec3d a = fromCameraFrameToWorldFrame(unproject(Vec2d(0,0)));
			Vec3d b = fromCameraFrameToWorldFrame(unproject(Vec2d(_width,0)));
			Vec3d c = fromCameraFrameToWorldFrame(unproject(Vec2d(_width,_height)));
			Vec3d d = fromCameraFrameToWorldFrame(unproject(Vec2d(0,_height)));
			Vec3d e = fromCameraFrameToWorldFrame(unproject(Vec2d(_width/2,_height/2)));

			if (sensorId == currentSensorId)
				glColor(WHITE);
			else
				glColor(color);
			glPyramid(eye,a,b,c,d, fill);
		}
		glLineWidth(1);
		setActiveSensor(currentSensorId);
	}
}

/* Denormalize a 2D edge */
Edge2D Camera::denormalize (int cameraId, Edge2D edge)
{
	return Edge2D (denormalize(cameraId,edge._a),denormalize(cameraId,edge._b));
}

/* Denormalize a 2D point */
Vec2d Camera::denormalize (int cameraId, Vec2d point)
{
	Intrinsic intr = getIntrinsic(cameraId);

	return intr.denormalize(point);
}

/* Distort a 2D point */
Vec2d Camera::distort (int cameraId, Vec2d point)
{
	Intrinsic intr = getIntrinsic(cameraId);

	return intr.denormalize(intr.distort(point));
}

/* Determine whether a 2D point is visible on the image or not */
bool Camera::visible (int cameraId, Vec2d point)
{
	return getIntrinsic(cameraId).visible(point);
}

/* determine whether a 2D edge is visible on the image or not
 * careful: checking the two end points is not sufficient!!!
 */
bool Camera::visible (int cameraId, Edge2D edge)
{
	Edge2D clipped = clip(cameraId,edge);

	return (visible(cameraId,clipped._a) && visible(cameraId,clipped._b));
}

/* Clip a 2D edge */
Edge2D Camera::clip (int cameraId, Edge2D edge)
{
	return getIntrinsic(cameraId).clip(edge);
}


/* compute the projection on the sphere of the image frames
 * this is for display only and allows to see the "overlap" between
 * the six Ladybug sensors
 */
void Camera::computeSphericalFrustum()
{
	int i,j;
	int iCamera=0;
	int nstep = 10;
	Vec2d start,end;
	Vec3d start_sphere,end_sphere;
	_frame._edgeplanes_frustum.clear();
	for (iCamera=0;iCamera<6;iCamera++) {
		setActiveSensor(iCamera);
		Intrinsic intr = getIntrinsic();
		int w = intr.getWidth();
		int h = intr.getHeight();
		//vertical borders
		for ( i=0;i<nstep;i++) {
			start = Vec2d(0,i*h/nstep);
			end = Vec2d(0,(i+1)*h/nstep);
			start_sphere = rectifyAndUnproject(start);
			end_sphere = rectifyAndUnproject(end);
			_frame._edgeplanes_frustum.push_back(EdgePlane(start_sphere,end_sphere,came.getTranslation(),iCamera,0,0));
		}
		for ( i=0;i<nstep;i++) {
			start = Vec2d(w-1,i*h/nstep);
			end = Vec2d(w-1,(i+1)*h/nstep);
			start_sphere = rectifyAndUnproject(start);
			end_sphere = rectifyAndUnproject(end);
			_frame._edgeplanes_frustum.push_back(EdgePlane(start_sphere,end_sphere,came.getTranslation(),iCamera,0,0));
		}
		//horizontal borders
		for ( j=0;j<nstep;j++) {
			start = Vec2d(j*w/nstep,0);
			end = Vec2d((j+1)*w/nstep,0);
			start_sphere = rectifyAndUnproject(start);
			end_sphere = rectifyAndUnproject(end);
			_frame._edgeplanes_frustum.push_back(EdgePlane(start_sphere,end_sphere,came.getTranslation(),iCamera,0,0));
		}
		for ( j=0;j<nstep;j++) {
			start = Vec2d(j*w/nstep,h-1);
			end = Vec2d((j+1)*w/nstep,h-1);
			start_sphere = rectifyAndUnproject(start);
			end_sphere = rectifyAndUnproject(end);
			_frame._edgeplanes_frustum.push_back(EdgePlane(start_sphere,end_sphere,came.getTranslation(),iCamera,0,0));
		}
	}
}

/* Return the camera matrix */
Matd Camera::getCameraMatrix()
{
	return _cameraMatrix;
}

/* Compute the camera matrix */
void Camera::computeCameraMatrix()
{
	Matd P(3,4);
	
	Matd Ep = inv(_extr);

	sub(P,3,3) = sub(Ep,3,3) * came.getRotationMatrix();
	col(P,3) = -sub(Ep,3,3) * came.getRotationMatrix() * came.getTranslation();
	P[0][3] += Ep[0][3];
	P[1][3] += Ep[1][3];
	P[2][3] += Ep[2][3];

	_cameraMatrix = cami.cameraMatrix * P;

	computeInverseCameraMatrix();
}

/* Compute the inverse camera matrix */
void Camera::computeInverseCameraMatrix()
{
	Mat4d P (0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1);
	sub(P,0,0,3,4) = _cameraMatrix;

	Matd Q = inv(P);

	_cameraMatrix_inv = sub(Q,0,0,4,3);

}

/* Unproject a 3D image edge into a 3D line */
Edge Camera::unproject (Edge2D edge)
{
	return Edge (unproject(edge._a),unproject(edge._b));
}

/* Transform a point from 3D world frame to 3D camera frame */
Vec3d Camera::fromWorldFrameToCameraFrame (Vec3d point)
{
	return came.getRotation().inv().rotate (point - came.getTranslation());
}

/* Transform a point from 3D camera frame to 3D world frame */
Vec3d Camera::fromCameraFrameToWorldFrame (Vec3d point)
{
	return came.getRotation().rotate (point) + came.getTranslation();
}

/* Transform an edge from 3D world frame to 3D camera frame */
Edge Camera::fromWorldFrameToCameraFrame (Edge edge)
{
	return Edge (fromWorldFrameToCameraFrame(edge._a->getPosition()),fromWorldFrameToCameraFrame(edge._b->getPosition()));
}

/* Transform a 3D edge into an edge plane */
EdgePlane Camera::fromWorldFrameToCameraFrame( Edge *edge ) 
{
	EdgePlane ep( edge->getA(), edge->getB(), getTranslation(), -1, -1, -1 );
	ep.fromWorldFrameToCameraFrame( getPose() );
	return ep;
}

/* Transform a 3D edge from camera frame to world frame */
Edge Camera::fromCameraFrameToWorldFrame (Edge edge)
{
	return Edge (fromCameraFrameToWorldFrame(edge._a->getPosition()),fromCameraFrameToWorldFrame(edge._b->getPosition()));
}

/* Rectify and unproject a 3D point */
Vec3d Camera::rectifyAndUnproject (Vec2d point)
{
	Intrinsic intr = getIntrinsic();
	Vec2d p1 = intr.rectify(intr.normalize(point));
	return unproject(p1);
}

/* clip the line <a.b> with respect to the camera plane
 * if both points are behind the camera, return false
 * otherwise, return true
 */
bool Camera::clipLine( int cameraId, Vec3d &a, Vec3d &b )
{
	Vec3d z = fromCameraFrameToWorldFrame( getZDirection( cameraId ) ) - getTranslation();
	Vec3d c = getTranslation();
	double f = getIntrinsic( cameraId ).getFocal();
	double lambda;

	bool a_front = ( dot( a - c, z ) > 20 * f );
	bool b_front = ( dot( b - c, z ) > 20 * f );

	// if a is behind...
	if  ( !a_front ) {
		// if b is also behind, reject
		if ( !b_front )
			return false;
		else {
			a = intersectRayPlane( b, a - b, c + 20 * f * z, z, lambda );
			bool res = ( dot( a - c, z ) > f );
			assert( res );
			return true;
		}
	}

	// if b is behind... (a is not behind at this point)
	if ( !b_front ) {
		assert( a_front );
		b = intersectRayPlane( a, b - a, c + 20 * f * z, z, lambda );
		assert( dot( b - c, z ) > f );
		return true;
	}

	// else, a and b are in front, so don't do anything
	return true;
}

/* Create an ideal edgeplane from the current position for the line given as argument
 * the edgeplane is expressed in the camera coordinate frame
 * This method is used for debugging and testing
 */
bool Camera::makeIdealEdgeplane( Edge *line, EdgePlane &edgeplane )
{
	intVector cameras;
	findSensors( line, cameras);
	if ( cameras.empty() ) //if no camera can see this line, pick a camera randomly
		return false;
	
	EdgePlane eps;

	for (int i=0;i<cameras.size();i++) {
		int cameraId = cameras[i];
		Vec3d center = getTranslation();// + getUnitCameraCenter( cameraId );
		Vec3d a = line->getA() - center; // - .2 * line->length() * line->dir - center;
		Vec3d b = line->getB() - center; // + .2 * line->length() * line->dir - center;
		EdgePlane ep = EdgePlane( a, b, center, cameraId, 0, 0 );
		ep.fromWorldFrameToCameraFrame( getPose() );
		if ( i == 0 )
			eps = ep;
		else
			if ( ep.length() > eps.length() )
				eps = ep;
	}

	edgeplane = eps;

	return true;
}

/* Project a 3D point onto the camera
 * and return true if the 3D point is in front of the camera
 */
bool Camera::project (int cameraId, Vec3d point, Vec2d &point_2d)
{
	Intrinsic intr = getIntrinsic(cameraId);
	Vec3d z = getZDirection(cameraId);
	double f = intr.getFocal();

	Vec3d local_point = fromWorldFrameToCameraFrame(point);

	bool front = (dot(local_point,z) > f);

	if ( !front )
		return false;

	Vecd local_point_hmg = toHomg(local_point);

	Vec3d return_point = toNonHomg( getUnitExtrinsicInv(cameraId) * local_point_hmg);

	double z_value = return_point[2];

	if ( z_value < f - 1E-6 ) {

		return false;
	}

	if (fabs(z_value) > f) {
		return_point = return_point * intr.getFocal() / z_value;
		return_point[1] = -return_point[1];
	}

	point_2d = Vec2d( return_point[0], return_point[1] );
	return true;
}

/* Project a 3D point onto the camera (the 3D point being expressed in the camera coordinate frame
 * and return true if the 3D point was in front of the camera
 */
bool Camera::projectLocalPoint (int cameraId, Vec3d point, Vec2d &point_2d)
{
	Intrinsic intr = getIntrinsic(cameraId);
	Vec3d z = getZDirection(cameraId);
	double f = intr.getFocal();

	Vec3d local_point = point;

	bool front = (dot(local_point,z) > f);

	if ( !front )
		return false;

	Vecd local_point_hmg = toHomg(local_point);

	Vec3d return_point = toNonHomg( getUnitExtrinsicInv(cameraId) * local_point_hmg);

	double z_value = return_point[2];

	if ( z_value < f - 1E-6 ) {

		return false;
	}

	if (fabs(z_value) > f) {
		return_point = return_point * intr.getFocal() / z_value;
		return_point[1] = -return_point[1];
	}

	point_2d = Vec2d( return_point[0], return_point[1] );
	return true;
}

/* project a 3D line <a,b> into a 2D edge (including distortion and clipping)
 * and return true if the edge is visible
 */
bool Camera::project( int cameraId, Vec3d a, Vec3d b, Edge2D &edge2D)
{
	// first, clip the 3D line
	if ( !clipLine( cameraId, a, b ) )
		return false;

	// then project each end point
	Vec2d p,q;

	if( !project( cameraId, a, p ) )
		return false;
	if ( !project( cameraId, b, q ) )
		return false;

	Intrinsic intr = getIntrinsic(cameraId);

	// then clip the edge
	edge2D = intr.clip( Edge2D( p, q ) );
	edge2D = Edge2D( p, q );
	
	return true;
}

/* Project a 3D line <a,b> onto the camera and subdivide the image edge
 * into edges of size <edge_step> (in pixels)
 */
bool Camera::projectAndSubdivide( int cameraId, Vec3d a, Vec3d b, int edge_step, edge2DVector &edges)
{
	edges.clear();

	Edge2D edge;
	Intrinsic intr = getIntrinsic(cameraId);

	// project the edge
	if ( !project( cameraId, a, b, edge ) )
		return false;

	// subdivide the edge
	Edge2D temp = Edge2D( distort( cameraId, edge._a ), distort( cameraId, edge._b ) );

	int n = MIN(10,temp.length() / edge_step + 1);

	Vec2d p = edge._a;
	Vec2d q = edge._b;
	Vec2d u = ( q - p ) / n;

	for (int i=0; i < n; i++ ) {
		Vec2d p1 = distort( cameraId, p + i * u );
		Vec2d p2 = distort( cameraId, p + (i + 1) * u );

		if ( intr.visible( p1 ) && intr.visible( p2 ) )
			edges.push_back( Edge2D( p1, p2 ) );
	}

	return true;
}

/* Unproject a 2D point into a 3D ray */
Vec3d Camera::unproject (Vec2d point) 
{
	assert (_cameraType == LADYBUG);
	
	//printf("unproject: %f %f into:\n",point[0],point[1]);
	Vecd X = _extr * Vecd (4,point[0],-point[1],getIntrinsic().getFocal(),1.0);
	Vec3d x = Vec3d(X[0],X[1],X[2]);
	
	//Vec2d out = getIntrinsic().normalize_focal(point);
	x = x;// + getUnitCameraCenter( _activeSensor );

	//Vecd X = Vecd(4,out[0],-out[1],1.0,1.0);  // X, Y, Z, 1.0
	//return came.getRotation().inv().rotate(x);
	return fromCameraFrameToWorldFrame(x);
	//printMatrix(_extr);
	
	//Vecd Xp = _extr*X;
	
	//printMatrix(Xp);
	
	//return Vec3d (Xp[0],Xp[1],Xp[2]);
}

/* Project a 3D point onto the current active sensor */
Vec2d Camera::project (Vec3d point)
{
	Vec2d point_2d;
	project(_activeSensor,point,point_2d);
	return point_2d;	
}

/* Project a 3D edge onto the current active sensor */
Edge2D Camera::project (Edge edge3d)
{
	Vec2d a = project(edge3d._a->getPosition());
	Vec2d b = project(edge3d._b->getPosition());

	Edge2D res =  Edge2D (a,b,0);
	return res;

}

/* Translate the camera */
void Camera::translate (Vec3d v)
{
	came.setTranslation (came.getTranslation() + v);
}

/* Rotate the camera */
void Camera::manual_rotate (int x, int y, int anchorx, int anchory, Vec3d xaxis, Vec3d yaxis)
{
	double theta = (x - anchorx)*1.0/1800;
	rotate(yaxis,-theta);
	theta = (y - anchory)*1.0/1800;
	rotate(xaxis,theta);
}

/* Rotate the camera (axis,angle) definition */
void Camera::rotate (Vec3d w, double theta)
{
	if ( len(w) < EPS )
		return;
	came.setRotation (Quaternion (cos(theta/2),sin(theta/2)*norm(w)) * came.getRotation());
}

/* Update the color information for an edge plane on the image */
bool Camera::updateColor( EdgePlane &ep, bool debug )
{
	Vec2d a,b;
	int i,j;
	int cameraid = ep._cameraId;

	if ( debug ) {
		_color_left.clear();
		_color_right.clear();
	}

	ep._normal = norm(cross(ep._a, ep._b));

	PerfTimer timer;

	// project the end points
	if ( !projectLocalPoint( cameraid, 10.0 * ep._a, a ) )
		return false;
	if ( !projectLocalPoint( cameraid, 10.0 * ep._b, b ) )
		return false;

	a = distort( cameraid, a );
	b = distort( cameraid, b );

	// compute the length in pixels
	double pixel_length = int(len(b-a))+1;

	// if edge too small, quit
	if ( pixel_length < 3 )
		return false;

	// compute the normal vector to the edge plane in the image space
	Vec2d u = Vec2d((a-b)[1], (b-a)[0]);
	u = norm(u);

	// compute the colors along the edge
	std::vector< Vec3d > hsv_left, hsv_right;

	for (i=pixel_length/4;i<3*pixel_length/4;i++) {
		
		double r = (double)i / pixel_length;
		Vec3d p = 10.0 * ( r * ep._a + (1.0-r) * ep._b );
		Vec2d pim;
		
		// project the point on the image
		if ( !projectLocalPoint( cameraid, p, pim ) )
			continue;
		
		pim = distort( cameraid, pim );
		
		// lookup the color information at point pim along the u vector
		// on the left
		for (j=1;j<=5;j++) {
			Vec2d lpim = pim + j*u;
			double r,g,b;
			double h,s,v;
			if ( getColor( cameraid, lpim, &r, &g, &b) ) {
				RGBtoHSV(r, g, b, &h, &s, &v);
				hsv_left.push_back( Vec3d(h,s,v) );
				if ( debug ) 
					_color_left.push_back( Vec3d(r,g,b) );
			}
		}
		// on the right
		for (j=1;j<=5;j++) {
			Vec2d lpim = pim - j*u;
			double r,g,b;
			double h,s,v;
			if ( getColor( cameraid, lpim, &r, &g, &b) ) {
				RGBtoHSV(r, g, b, &h, &s, &v);
				hsv_right.push_back( Vec3d(h,s,v) );
				if ( debug )
					_color_right.push_back( Vec3d(r,g,b) );
			}
		}
	}

	// for debug info, store the pixels in camera memory

	// compute the average and variance in HSV color on each side
	Vec3d l_avg = Vec3d(0,0,0);
	Vec3d r_avg = Vec3d(0,0,0);
	Vec3d l_dev = Vec3d(0,0,0);
	Vec3d r_dev = Vec3d(0,0,0);

	int nl = hsv_left.size();
	int nr = hsv_right.size();

	// compute the averages
	for (i=0;i<nl;i++)
		l_avg = l_avg + hsv_left[i];
	if ( nl > 0 )
		l_avg = l_avg / nl;

	for (i=0;i<nr;i++)
		r_avg = r_avg + hsv_right[i];
	if ( nr > 0 )
		r_avg = r_avg / nr;

	// compute the standard dev
	for (i=0;i<nl;i++) {
		l_dev[0] = l_dev[0] + (hsv_left[i][0]-l_avg[0])*(hsv_left[i][0]-l_avg[0]);
		l_dev[1] = l_dev[1] + (hsv_left[i][1]-l_avg[1])*(hsv_left[i][1]-l_avg[1]);
		l_dev[2] = l_dev[2] + (hsv_left[i][2]-l_avg[2])*(hsv_left[i][2]-l_avg[2]);
	}
	if ( nl > 0 ) {
		l_dev[0] = sqrt(l_dev[0]) / nl;
		l_dev[1] = sqrt(l_dev[1]) / nl;
		l_dev[2] = sqrt(l_dev[2]) / nl;
	}

	for (i=0;i<nr;i++) {
		r_dev[0] = r_dev[0] + (hsv_right[i][0]-r_avg[0])*(hsv_right[i][0]-r_avg[0]);
		r_dev[1] = r_dev[1] + (hsv_right[i][1]-r_avg[1])*(hsv_right[i][1]-r_avg[1]);
		r_dev[2] = r_dev[2] + (hsv_right[i][2]-r_avg[2])*(hsv_right[i][2]-r_avg[2]);
	}
	if ( nr > 0 ) {
		r_dev[0] = sqrt(r_dev[0]) / nr;
		r_dev[1] = sqrt(r_dev[1]) / nr;
		r_dev[2] = sqrt(r_dev[2]) / nr;
	}
		
	// update the edge color info
	ep.l_avg = l_avg;
	ep.r_avg = r_avg;
	ep.l_dev = l_dev;
	ep.r_dev = r_dev;

	// return success
	return true;
}

/* Retrieve the color information at a given image point
*/
bool Camera::getColor( int cameraid, Vec2d point, double *r, double *g, double *b )
{
	int x = roundi( point[0] );
	int y = roundi( point[1] );

	IplImage *img = _frame._tmp_img[cameraid];

	if ( x >= img->width || x < 0 )
		return false;
	if ( y >= img->height || y < 0 )
		return false;

	// flip vertically from Ladybug format to openCV format
	y = img->height-1-y;

	CvPoint pt = {x,y};
	uchar* temp_ptr = &((uchar*)(img->imageData + img->widthStep*pt.y))[x*3];

	*r = temp_ptr[0] / 255.0; //(double)(img->imageData[3*(x+y*img->width)+0]) / 255.0;
	*g = temp_ptr[1] / 255.0; //(double)(img->imageData[3*(x+y*img->width)+1]) / 255.0;
	*b = temp_ptr[2] / 255.0; //(double)(img->imageData[3*(x+y*img->width)+2]) / 255.0;

	return true;
}

/* Unproject a list of 2D image edges into a list of edge planes 
 */
void Camera::unproject (edge2DVector edges2d, edgePlaneVector &edges, Vec3d center)
{
	for (int i=0;i<edges2d.size();i++) {
		Edge2D edge = edges2d[i];
		// flip edges horizontally
		Vec3d a3d = rectifyAndUnproject(edge._a);
		Vec3d b3d = rectifyAndUnproject(edge._b);

		// unproject the edges
		EdgePlane edge3d (a3d-center, b3d-center, center, _activeSensor, i, i);
		
		edges.push_back(edge3d);
	}
}

/* Unproject a 2D image edge into two 3D end points 
 */
void Camera::unproject( Edge2D edge_in, Vec3d &a3d, Vec3d &b3d )
{
	a3d = rectifyAndUnproject(edge_in._a);
	b3d = rectifyAndUnproject(edge_in._b);
}

/* Print the camera pose to a filename 
 */
void Camera::printCameraPose (const char *filename)
{
	FILE *fout;
	fout = fopen(filename,"a");

	if (fout == NULL) {
		fprintf(stderr,"failed to open %s\n",filename);
		return;
	}

	// position (x,y,z)  + 2 unit vectors (target,up)
	fprintf(fout,"%f %f %f %f %f %f %f %f %f %f\n",eyec.eye[0],eyec.eye[1],\
		eyec.eye[2],eyec.target[0],eyec.target[1],eyec.target[2],eyec.up[0],eyec.up[1],eyec.up[2],\
		orientation_cam.pitch);

	fclose(fout);
}

/*void Camera::setPosition (Vec3d position, Vec3d target, Vec3d up, double pitch)
{
	eyec.eye = position;
	eyec.target = target;
	eyec.up = up;

	orientation_cam.pitch = pitch;
}*/

/* Set the current active sensor 
 */
void Camera::setActiveSensor (int id)
{
	assert( (0 <= id) && (id <= 5));

	_saveActiveSensor = _activeSensor;

	_activeSensor = id;

	_extr = getUnitExtrinsic(id);

	cami.cameraMatrix = getIntrinsic().toKMatrix();

}

/* Restore active sensor stored in memory
 */
void Camera::resetActiveSensor ()
{
	setActiveSensor (_saveActiveSensor);
}

//////////////////////////////////////////////////
// PoseSolution
/*
PoseSolution::PoseSolution(edgeVector _lines, edgePlaneVector _planes, ExtrinsicParameters _came) // computes errors and valid
{
	int i;
	assert(_lines.size() == _planes.size());
	n = _lines.size();

	for (i=0;i<n;i++) {
		lines.push_back(_lines[i]);
		planes.push_back(_planes[i]);
		planes[i].fromCameraFrameToWorldFrame(_came.getTranslation(),_came.getRotation());
	}

	came = _came;

	// compute the rotation error
	error_rot = 0.0;
	for (i=0;i<n;i++) {
		error_rot += fabs(dot(planes[i]._normal,lines[i]->dir));
	}

	// compute the translation error
	error_trans = 0.0;
	double lambda = 0.0;
	for (i=0;i<n;i++) {
		intersectRayPlane(lines[i]->_a->getPosition(),planes[i]._normal,planes[i]._s,planes[i]._normal,lambda);
		error_trans += fabs(lambda)/MAX(1.0,len(lines[i]->_a->getPosition()-planes[i]._s));
	}

	// compute the total error
	error = error_rot + error_trans;

	// run the validity test
	valid = testValidity();
}

PoseSolution::PoseSolution(Edge *a, Edge *b, Edge *c , EdgePlane A, EdgePlane B, EdgePlane C, ExtrinsicParameters _came) // computes errors and valid
{
	int i;
	n = 3;

	lines.push_back(a);
	lines.push_back(b);
	lines.push_back(c);
	
	A.fromCameraFrameToWorldFrame(_came.getTranslation(),_came.getRotation());
	B.fromCameraFrameToWorldFrame(_came.getTranslation(),_came.getRotation());
	C.fromCameraFrameToWorldFrame(_came.getTranslation(),_came.getRotation());
	planes.push_back(A);
	planes.push_back(B);
	planes.push_back(C);

	came = _came;

	// compute the rotation error
	error_rot = 0.0;
	for (i=0;i<n;i++) {
		error_rot += fabs(dot(planes[i]._normal,lines[i]->dir));
	}

	// compute the translation error
	error_trans = 0.0;
	double lambda = 0.0;
	for (i=0;i<n;i++) {
		intersectRayPlane(lines[i]->_a->getPosition(),planes[i]._normal,planes[i]._s,planes[i]._normal,lambda);
		error_trans += fabs(lambda)/MAX(1.0,len(lines[i]->_a->getPosition()-planes[i]._s));
	}

	// compute the total error
	error = error_rot + error_trans;

	// run the validity test
	valid = testValidity();
}

// test the validity of a solution based on geometric constraints
// the main constraint is that the lines should project as a plane image
bool PoseSolution::testValidity ()
{
	for (int i=0;i<n;i++) {

		Edge *line = lines[i];
		EdgePlane plane = planes[i];

		// test frontality of line with camera
		test_frontality = plane.testFrontality(line);
		if (!test_frontality)
			return false;

		// test overlap
		test_overlap = plane.testOverlap(line);
		if (!test_overlap)
			return false;
	}

	return true;
}

void PoseSolution::draw(Viewer &viewer)
{
	double radius = 60.0;

	// draw the camera pose
	viewer.drawSphere(came.getTranslation(),5.0,YELLOW,10,10);

	// draw the image planes
	int i;
	for (i=0;i<n;i++)
		viewer.drawEdgePlaneDetails(planes[i],Vec3d(0,0,0),Quaternion(),radius,2,6); // color by camera ID

	// draw the 3D lines
	for (i=0;i<n;i++)
		viewer.drawLine(lines[i]->_a->getPosition(),lines[i]->_b->getPosition(),_colors[planes[i]._cameraId],18); // color by camera ID

	// draw some text info
	//viewer.displayMsg(10,viewer._height-30,RED,0,"Valid: %d",valid);
	//viewer.displayMsg(10,viewer._height-50,RED,0,"Error: %.4f + %.4f = %.4f",error_rot,error_trans,error);
	//viewer.displayMsg(10,viewer._height-70,RED,0,"Position: %.2f %.2f %.2f",came.translation[0],came.translation[1],came.translation[2]);
}

void PoseSolution::print()
{
	printf("Pose Solution (%d correspondences):\n",n);
	printf("valid: %d (%d,%d)\n",valid,test_frontality,test_overlap);
	printf("Error: %f (rotation: %f, translation: %f)\n",error,error_rot,error_trans);
	printf("Camera Position: %f %f %f\n",came.getTranslation()[0],came.getTranslation()[1],came.getTranslation()[2]);
}

bool PoseSolution::equal (PoseSolution pose)
{
	if (len(came.getTranslation() - pose.came.getTranslation()) < 5)
		return true;

	return false;
}
*/

/* Generate a synthetic frame given a camera position and a set of faces
 * each face is generated with a different color in order to test edge detection
 * lighting is off
 * for each pixel on the image, a ray is generated and interesected with the faces
 * n is the total number of faces in the model (needed for unique color generation)
 */
bool Camera::makeSyntheticImage (int sensorId, faceVector &faces, int n, IplImage *image)
{
	LOG( LEVEL_INFO, "generating synthetic image for sensor ID = %d", sensorId );
	
	setActiveSensor( sensorId );
	Intrinsic intr = getIntrinsic( ); // intrinsic parameters
	Vec3d center = getTranslation() + getUnitCameraCenter( sensorId ); // camera center in the world coord frame

	// generate a table of 500 random colors
	float *color;
	color = (float*)malloc(3*sizeof(float));

	// for each pixel...
	for (int i=0;i<intr.getHeight();i++) {
		for (int j=0;j<intr.getWidth();j++) {

			Vec2d pixel = Vec2d( j, i );

			//printf("rectifying...\n");
			Vec3d ray = rectifyAndUnproject( pixel ) - center; // compute the back projected ray

			double lambda = -1.0; // find the intersection with the faces

			Vec3d point = Vec3d(0,0,0);
			
			//printf("intersecting...\n");
			int face_id = intersectRayFaces( center, ray, faces, point );

			if ( face_id == -1 )
				color[0] = color[1] = color[2] = 0.0;
			else {
				color[0] = faces[face_id]->color[0];
				color[1] = faces[face_id]->color[1];
				color[2] = faces[face_id]->color[2];
			}

			// draw the pixel on the image
			CvPoint cv_point = { j, intr.getHeight() - 1 - i };
			cvLine( image, cv_point, cv_point, CV_RGB( color[0] , color[1] , color[2]  ) );

		}
	}

	delete color;
	return true;
}

/* Write the pose into a file
 * assuming that the file is already open!
 */
void Camera::writePose ( int frameId, FILE *pose_file )
{
	if ( pose_file == NULL ) {
		return;
	}

	Vec3d r, t = getTranslation();
	getRotation().toAxisAngle( r );
	fprintf( pose_file, "%d %f %f %f %f %f %f\n",frameId,t[0],t[1],t[2],r[0],r[1],r[2]);
}

/* Generate a synthetic image with lines only
 * this method is somewhat deprecated since a new one incorporates faces
 */
bool Camera::makeSyntheticImage (int sensorId, edgeVector lines, IplImage *image)
{
	// clear image
	int nChannels = image->nChannels;
	bool result = false;

	for (int m=0;m<image->height;m++) {
		for (int n=0;n<image->width;n++) {
			for (int p=0;p<nChannels;p++) {
				image->imageData[nChannels*(m*image->width+n)+p] = 0.0;
			}
		}
	}
	
	// print edges on the image
	for (int i=0; i < lines.size(); i++) {
		Edge *line = lines[i];

		edge2DVector edges;

		projectAndSubdivide( sensorId, line->getA(), line->getB(), DISTORTION_EDGE_STEP, edges);

		// print for debug
		if ( !edges.empty() ) {
			Edge2D start = edges[0];
			Edge2D end = edges[edges.size()-1];
			FILE *f = fopen ( "input.dat", "a");
			fprintf( f, "%f %f %f %f\n",start._a[0],start._a[1],end._b[0],end._b[1]);
			fclose( f );
			result = true;
		}

		for (edge2DVector::iterator iter = edges.begin(); iter != edges.end(); iter++ ) {
			CvPoint point1 = {iter->_a[0],image->height-1-iter->_a[1]};
			CvPoint point2 = {iter->_b[0],image->height-1-iter->_b[1]};
			cvLine(image,point1,point2,CV_RGB(255,255,255));
		}
	}

	return result;
}

/* Generates a synthetic image given a set of visible lines (this method is somewhat deprecated)
 * and store the image in the frame structure
 * calls makeSyntheticImage()
 */
void Camera::makeSyntheticImage (edgeVector lines, intVector &cameras)
{
	for ( int sensorId = 0; sensorId < 6; sensorId++ ) {
		if ( makeSyntheticImage( sensorId, lines, _frame._grab_img[sensorId] ) )
			cameras.push_back( sensorId );
	}
}

/* Generates a synthetic image given a set of visible faces
 * and store the image in the frame structure
 * calls makeSyntheticImage()
 * n is the number of faces in the whole model (needed for color generation)
 */
void Camera::makeSyntheticImage (faceVector &faces, int n, intVector &cameras)
{
	for ( int sensorId = 0; sensorId < 6; sensorId++ ) {
		setActiveSensor( sensorId );
		if ( makeSyntheticImage( sensorId, faces, n, _frame._grab_img[sensorId] ) )
			cameras.push_back( sensorId );
	}
}

////////////////////////////////////////////////////////
// user mask
/*
// clear the user masks
void Camera::clearUserMasks()
{
	user_masks.clear();
}

// add a new user mask (replace if already exists)
void Camera::addUserMask( UserMask mask)
{
	if ( mask.pts.empty() )
		return;

	int n = mask.pts.size() - 1;
	Vec2d start,end;
	Vec2d corner1, corner2;

	switch ( mask.type ) {
	case MASK_DOWN:
		start = Vec2d( mask.pts[0][0], 0 );
		end = Vec2d( mask.pts[n][0], _width );
		corner1 = Vec2d( _height - 1, 0 );
		corner2 = Vec2d( _height - 1, _width );
		break;
	}

	mask.pts[0] = start;
	mask.pts[n] = end;
	mask.pts.push_back( corner2 );
	mask.pts.push_back( corner1 );

	for (std::vector< UserMask >::iterator iter = user_masks.begin(); iter != user_masks.end(); iter++) {
		if ( iter->cameraId == mask.cameraId ) {
			// replace
			*iter = mask;
			return;
		}
	}

	// else add
	user_masks.push_back( mask );
}

// write user masks to file
bool Camera::writeUserMasks( std::string dirname )
{
	std::string filename = dirname + "/" + std::string( USER_MASK_FILE );

	FILE *f = fopen( filename.c_str(), "w" );

	if (f == NULL) 
		return false;

	for (int i=0; i<user_masks.size(); i++)
		user_masks[i].write( f );

	fclose( f );
	return true;
}

// reads the user masks from file
bool Camera::readUserMasks( std::string dirname )
{

	std::string filename = dirname + "/" + std::string( USER_MASK_FILE );

	FILE *f = fopen( filename.c_str(), "r" );

	if ( f == NULL )
		return false;

	while ( !feof( f ) ) {

		UserMask mask;
		if ( mask.read( f ) )
			user_masks.push_back( mask );
	}

	return true;
}

// delete the user mask having the specified camera ID
void Camera::deleteUserMask( int cameraId )
{
	for (std::vector< UserMask >::iterator iter = user_masks.begin(); iter != user_masks.end(); iter++ ) {
		if ( iter->cameraId == cameraId ) {
			user_masks.erase( iter );
			return;
		}
	}
}
*/

/* Draw the masks on top of the image */
void Camera::drawUserMask( Viewer &viewer, int cameraId, int x, int y, double zoom, bool flipVertical )
{
	for (std::vector< UserMask >::iterator iter = user_masks.begin(); iter != user_masks.end(); iter++ ) {
		if ( iter->cameraId == cameraId )
			viewer.drawUserMask( x, y, zoom, iter->pts, flipVertical, _height );
	}
}

/*  Find the sensors that see a given line */
void Camera::findSensors( Edge *line, intVector &ids )
{
	assert( line != NULL );

	Vec3d p = getTranslation();

	for (int i=0;i<6;i++) {

		Vec3d z = getRotation().rotate( getZDirection( i ) );
		Vec3d a = norm( line->getA() - p );
		Vec3d b = norm( line->getB() - p );

		if ( ( dot( z, a ) < 0 ) && ( dot( z, b ) < 0 ) )
			continue;

		if ( fabs( dot( z, norm( cross( line->getA() - p, line->getB() - p ) ) ) ) < COS_30 )
			ids.push_back( i );
	}
}

/* For each pixel on the image, compute a hit on the corresponding face
 */
void Camera::computeColorHits( int cameraid, int level_of_detail, double color_resolution, faceVector &faces, std::vector< color_hit > &hits )
{
	setActiveSensor( cameraid );

	Vec3d center = getTranslation() + getUnitCameraCenter( cameraid ); // camera center in the world coord frame
	
	Intrinsic intr = getIntrinsic( cameraid );
	double pixel_angle = intr.get_pixel_angle();
	
	IplImage *img = _frame._tmp_img[cameraid];

	//uchar* temp_ptr = &((uchar*)(img->imageData + img->widthStep*pt.y))[x*3];
	uchar* temp_ptr = &((uchar*)(img->imageData))[0];

	// for each pixel...
	for (int i=0;i<intr.getHeight();i++) {
		for (int j=0;j<intr.getWidth();j++) {
			
			Vec2d pixel = Vec2d( j, i );
			
			Vec3d ray = rectifyAndUnproject( pixel ) - center; // compute the back projected ray
			
			double lambda = -1.0; // find the intersection with the faces
			
			Vec3d point = Vec3d(0,0,0);
			
			int face_id = intersectRayFaces( center, ray, faces, point );
			
			if ( face_id == -1 )
				continue;
			
			Face *face = faces[face_id];
			
			if ( face->_vertices.size() != 3 && face->_vertices.size() != 4 )
				continue;
			
			if ( !face->inside( point ) )
				continue;
			
			int pi = face->getSubdivisionIndex( point, color_resolution );//,pj;
			if ( -1 == pi)
				continue;

			double r,g,b;
			
			if ( !getColor( cameraid, pixel, &r, &g, &b) ) 
				continue;
			
			
			_color_left.push_back( Vec3d(r,g,b) );

			color_hit hit;
			hit.face_id = face->_id;
			hit.i = pi;
			hit.r = int(r * 255);
			hit.g = int(g * 255);
			hit.b = int(b * 255);
			hit.hits = 1;
			
			hits.push_back( hit );
		}
	}
}
