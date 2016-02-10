#include "main.h"

// this method allows to save images from the Ladybug HD
// in a user-specified location
// had to write this because Ladybug interface is *real crap*
void ladybug_hd_cb(Fl_Widget *w, void *)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	// check if the Ladybug is here
	if (!c->_ladybug->_init)
		return;
	
	// look how many images they are
	int nImages = c->_ladybug->GetRecordedImages();
	printf("%d images on the Ladybug HD.\n",nImages);
	
	if (nImages == 0)
		return;
	
	// ask for start and end counter
	const char *counters = fl_input("Enter start and end images separated by a space. Leave empty for all.","");
	if (counters == NULL) {
		return;
	}
	
	int start=0,end=nImages-1;
	if (sscanf(counters,"%d%d",&start,&end) != 2) {
		start=0;
		end=nImages-1;
	}
	start = MAX(0,MIN(start,nImages-1));
	end = MAX(0,MIN(end,nImages-1));
	
	//int time = (start-end+1)*4/100; // number of minutes to download all images, 100 frames = 4 minutes

	// ask for skipping images
	const char *skip_frames = fl_input("Keep every k frames? Leave 0 for all frames.","0");
	if (skip_frames == NULL )
		return;

	int skip=0;
	if ( sscanf(skip_frames, "%d", &skip) != 1 )
		skip = 0;

	// choose the destination location
	char *f = (char*)malloc(1024*sizeof(char));
	f = fl_dir_chooser("Destination directory", "C:/omni3d/data/", 0); // 0 : full path name
	if (f == NULL) {
		return;
	}
	
	// choose the input size
	int resolution_in = fl_choice("Input resolution:","256x192","512x384","1024x768");
	
	// choose the output size
	int resolution_out = fl_choice("Output resolution:","256x192","512x384","1024x768");
	
	// save images from HD
	c->progress_bar->minimum(start);
	c->progress_bar->maximum(end);
	c->progress_bar->value(start);
	nImages = c->_ladybug->saveImagesFromHD(start,end,skip,f,resolution_in, resolution_out, c->progress_bar);
	c->progress_bar->value(start);
	c->redraw();
	
	// at the end, suggest emptying Ladybug HD, default answer is NO
	int nok = fl_choice("Empty Ladybug HD?","Yes","No",NULL);
	if (!nok) {
		int ok = fl_choice("Are you sure? This will complety erase the Ladybug HD.","No","Yes",NULL);
		if (ok)
			c->_ladybug->emptyHD();
	}
	
	// create the data.ini file
	c->_database->createDataFile(resolution_out,f,nImages, 0);
	
}

// load images from database instead of grabbing from Ladybug
void load_database_hd_cb(Fl_Widget *w, void *)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	if (c->_database != NULL) {
		printf("Error: exit current database first!\n");
		return;
	}
	
	// choose database
	char *f = fl_dir_chooser("Choose database", "C:/omni3d/data/", 0); // 0 : full path name
	if (f == NULL)
		return;
	
	c->_database = new Database();
	
	// init database
	c->_database->init(f);

	// set camera edges file names
	c->_camera->fname_edges = c->_database->_dirname + "/edges.txt";
	c->_camera->fname_edges_filtered = c->_database->_dirname + "/edges_filtered.txt";
	c->_camera->fname_edges_ref = c->_database->_dirname + "/" + CAMERA_EDGES_REF_FILE;
	
	// read the history of poses
	read_history_cb( w, NULL );

	// backup the pose history
	c->pose_history_backup = c->pose_history;

	// setup the frame slider
	c->frame_slider->minimum(0);
	c->frame_slider->maximum(c->_database->_nImages-4);
	c->frame_slider->value(0,3,1,c->_database->_nImages-1);
	c->frame_slider->show();

	// process the first frame
	frame_slider_changed_cb( w, NULL );

	// read the user masks
	//c->_camera->readUserMasks( c->_database->_dirname );
	
	// read the line states history
	c->readLineHistory();

	// set edge parameters
	if ( c->_database != NULL ) {
		c->setEdgeDetectorParameters( false, c->_database->_synthetic != 0 );
	}

	// switch to video mode
	c->_windowMode = MODE_VIDEO;

	// update the frame slider
	frame_slider_changed_cb(w, NULL);

	c->redraw();
}

void frame_slider_changed_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_database->setFrameId(c->frame_slider->value());
	c->frameId = c->frame_slider->value();

	if ( c->_windowMode == MODE_VIDEO ) {
		// update the image
		c->_database->loadFrame(c->_camera->_frame);
		c->_camera->_frame.processFrame();
	}

	// update the camera pose
	for (int i=0;i<c->pose_history.size();i++) {
		if ( c->pose_history[i].id == c->frameId ) {
			c->_camera->setPose( c->pose_history[i] );
			//c->_LUT.lookup( c->_camera->came.getTranslation(), 0.0, 0.0, 0, 0);
			break;
		}
	}

	// get the line history
	if ( !c->getLineHistory() ) {
		c->_LUT._node = NULL;
		c->_LUT.lookup( c->_camera->came.getTranslation(), 0.0, 0.0, 0, 0);
	}


	if ( c->_windowMode == MODE_VIDEO ) {
		// reproject the edges
		reproject_edges_cb(w,NULL);
	}

	//redraw
	c->redraw();
}

void clear_history_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->pose_history.clear();
}

void detect_edges_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->detect_edges();
}

// make a synthetic image for the given camera position
//
void set_camera_pose_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	// generate a string with the current camera pose
	char input[256];
	Vec3d t = c->_camera->getTranslation();
	Vec3d r;
	c->_camera->getRotation().toAxisAngle(r);

	sprintf(input,"%f %f %f %f %f %f", t[0], t[1], t[2], r[0], r[1], r[2] );

	// prompt user for camera pose
	const char *str = fl_input("Enter X Y Z separated by a space.", input);
	if (str == NULL) {
		return;
	}
	
	double x,y,z, rx, ry, rz;
	if (sscanf(str,"%lf%lf%lf",&x,&y,&z) == 3) {
		// set camera translation
		c->_camera->setTranslation(Vec3d(x,y,z));
	}	

	if (sscanf(str,"%lf%lf%lf%lf%lf%lf",&x,&y,&z,&rx,&ry,&rz) == 6) {
		// set camera translation
		c->_camera->setTranslation(Vec3d(x,y,z));
		Vec3d rv = Vec3d(rx,ry,rz);
		c->_camera->setRotation(Quaternion(rv,len(rv)));
	}	


	// lookup LUT
	c->_LUT.lookup( c->_camera->came.getTranslation(), 0.0/*c->MIN_SUBTENDED_ANGLE*/, 0.0, 0, 0);
}

void clear_edges_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	c->_camera->clearEdges();
	//c->_poses.clear();
	c->_locked = false;
	c->select_generic->bounds(0,0);
	c->redraw();
}

// exit the database
void exit_database_hd_cb(Fl_Widget *w, void *)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	if (c->_database == NULL) {
		printf("Error: cannot exit current database!\n");
		return;
	}
	
	delete c->_database;
	c->_database = NULL;

	// set camera edges file names
	c->_camera->fname_edges = "";
	c->_camera->fname_edges_filtered = "";
	c->_camera->fname_edges_ref = "";
	
	c->frame_slider->hide();
	
	c->redraw();
}


void options_cb (Fl_Widget *w, void *)
{
	Fl_Menu_Bar *menubar = (Fl_Menu_Bar*)w;
    const Fl_Menu_Item *item = menubar->mvalue();
	const char *shortname = menubar->text();
	
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	if (strcmp(shortname,"Wireframe") == 0) {
		c->_wireframe = !c->_wireframe;
	}
	
	if (strcmp(shortname,"Depth test") == 0) {
		c->_depth_test = !c->_depth_test;
	}
	
	if (strcmp(shortname,"Synthetic 1") == 0) {
		c->_synthetic_mode = 1;
		c->frameId = 0;
	}

	if (strcmp(shortname,"Synthetic 2") == 0) {
		c->_synthetic_mode = 2;
		c->frameId = 0;
	}

	if (strcmp(shortname,"Synthetic 3") == 0) {
		c->_synthetic_mode = 3;
		c->frameId = 0;
	}

	if (strcmp(shortname,"Error analysis") == 0) {
		c->_synthetic_mode = 4;
		c->frameId = 0;
	}

	if (strcmp(shortname,"Real") == 0) {
		c->_synthetic_mode = 0;
		c->frameId = 0;
	}
	
	w->parent()->show();
}

void set_cb(Fl_Widget *w, void *)
{
	Fl_Menu_Bar *menubar = (Fl_Menu_Bar*)w;
    const Fl_Menu_Item *item = menubar->mvalue();
	const char *shortname = menubar->text();
	
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	c->_historyMode = c->_windowMode;
	
	if (strcmp(shortname,"Video") == 0) {
		c->_windowMode = MODE_VIDEO;

		// update the frame slider
		frame_slider_changed_cb(w, NULL);

	}
	if (strcmp(shortname,"3D Map") == 0) {
		c->_windowMode = MODE_3D_MAP;
	}
	if (strcmp(shortname,"2D Map") == 0) {
		c->_windowMode = MODE_2D_MAP;
	}
	if (strcmp(shortname,"Sphere") == 0) {
		c->_windowMode = MODE_SPHERE;
	}
	if (strcmp(shortname,"Camera calibration") == 0) {
		c->_windowMode = MODE_CALIBRATION;

		// update the frame slider
		frame_slider_changed_cb(w, NULL);
	}
	if (strcmp(shortname,"Rotations") == 0) {
		c->_windowMode = MODE_ROTATIONS;
	}	
	// reset the calibration parameters
	w->parent()->show();
}

void quit_cb(Fl_Widget*, void*)
{
	LOG_DEINIT;
	printf("exiting...\n");
	exit(0);
}

// make a movie sequence with reprojected edges
//
void make_movie_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	c->pose_history.clear();

	c->redraw();

	if ( c->_database == NULL )
		return;

	for (int frameid = c->frameId; frameid < c->_database->_nImages; frameid++) {

		printf("%d\n", frameid);

		// update the frame ID
		c->frame_slider->value(frameid,3,1,c->_database->_nImages-1);
		
		// process the new frame ID
		c->_database->setFrameId(frameid);
		c->frameId = frameid;
		
		if ( c->_windowMode == MODE_VIDEO ) {
			// update the image
			c->_database->loadFrame(c->_camera->_frame);
			c->_camera->_frame.processFrame();
		}
		
		// get the line history
		c->getLineHistory();

		continue;

		// update the camera pose
		for (int i=0;i<c->pose_history_backup.size();i++) {
			if ( c->pose_history_backup[i].id == c->frameId ) {
				c->_camera->setPose( c->pose_history_backup[i] );
				break;
			}
		}
	
		if ( c->_windowMode == MODE_VIDEO ) {
			// reproject the edges
			//c->_LUT.lookup( c->_camera->came.getTranslation(), 0.0, 0.0, 0, 0 );
			c->reprojectEdges( c->_LUT._lines );
		}


		if ( c->_windowMode == MODE_3D_MAP ) {

			c->pose_history.push_back( c->pose_history_backup[frameid] );
			//Sleep(200);
		}

		glClearColor(0,0,0,0);		// clear the window to black
		glClear(GL_COLOR_BUFFER_BIT);	// clear the window

		switch (c->_windowMode) {
		case MODE_VIDEO:
			c->drawVideo(false);
			break;
		case MODE_3D_MAP:
			glEnable(GL_LIGHTING);
			c->draw3DMap();
			break;
		case MODE_2D_MAP:
			c->draw2DMap();
			break;
		case MODE_SPHERE:
			c->drawSphere();
			break;
		case MODE_CALIBRATION:
			c->drawCalibration();
			break;
		case MODE_ROTATIONS:
			c->drawRotations();
			break;
		default:
			exit(0);
		}


		c->redraw();


		// make a snapshot
		int timestamp = frameid;
		screenSnapshot( 0, c->SNAPSHOT_X, c->SNAPSHOT_Y, c->SNAPSHOT_W, c->SNAPSHOT_H, timestamp);

		// redraw
	/*	Fl::check();
		c->activate();
		c->redraw();
		Sleep(100);

		// redraw
		Fl::check();
		c->activate();
		c->redraw();
		Sleep(100);
	
		// redraw
		Fl::check();
		c->activate();
		c->redraw();
		*/

	}

}

void file_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	int timestamp = c->frameId;

	screenSnapshot( 0, c->SNAPSHOT_X, c->SNAPSHOT_Y, c->SNAPSHOT_W, c->SNAPSHOT_H, timestamp);

	//c->_camera->_frame.save("C:/temp/video",c->_ladybug->_frameId);
	
}

void rectify_img_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	c->_camera->setActiveSensor(c->_calibrated_camera);
	
	c->_rectified_done = false;
	if (c->calib_load_img->value())
		c->_camera->rectifyImage(c->_calib_img,c->_calib_rectified_img);
	else
		c->_camera->rectifyImage(c->_camera->_frame._rotated_img[c->_calibrated_camera],\
		c->_camera->_frame._rectified_img[c->_calibrated_camera]);
	
	c->_rectified_done = true;
	c->redraw();
}

void grab_img_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	if (c->_windowMode == MODE_VIDEO)
		c->_camera->_frame.save("C:/temp/video",c->_ladybug->_frameId);
	else if (c->_windowMode == MODE_CALIBRATION)
		c->_camera->_frame.save("C:/temp/video",c->_ladybug->_frameId,c->_calibrated_camera);
}

void calib_load_img_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	if (c->calib_load_img->value() == 0)
		return;
	
	char *f = (char*)malloc(1024*sizeof(char));
	std::string filename;
	
	f = fl_file_chooser("Hello", "*.jpg, *.bmp", "C:/omni3d/camera_calibration/", 0); // 0 : full path name
	if (f == NULL) {
		return;
	}
	
	IplImage *img = cvLoadImage(f);
	
	if (img == NULL)
		return;
	
	if ((img->width == c->_calib_img->width) && (img->height == c->_calib_img->height))
		cvSetImageData(c->_calib_img,img->imageData,3*img->width);
	else if ((img->width == c->_calib_img->height) && (img->height == c->_calib_img->width)) // image needs to be rotated
		rotateCW(img,c->_calib_img);
	
	cvReleaseImage(&img);
}

void open_model_cb (Fl_Widget*w, void*)
{
	char *f = (char*)malloc(1024*sizeof(char));
	std::string filename;
	
	f = fl_file_chooser("Hello", "*.ini", "C:/omni3d/models", 0); // 0 : full path name
	if (f == NULL) {
		return;
	}
	
	filename = std::string(f);
	
	std::string::size_type pos = filename.find (std::string("config.ini"),0);
	assert(pos != std::string::npos);
	
	filename.replace(pos-1,11,std::string(""));
	
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	// init model
	c->_LUT.init (WINDOW_WIDTH-CONTROLBAR_WIDTH,WINDOW_HEIGHT,filename.c_str());
	c->_sphere_viewer._target = c->_LUT._position;
	
	// clear the LUT
	c->_LUT._lines.clear();
	c->_LUT._faces.clear();

	// switch to 3D map view
	c->_windowMode = MODE_3D_MAP;
	
	// read textures
	if ( c->_draw_texture )
		c->_LUT._model->readTextures();

	c->redraw();
}

void edge_clearmasks_cb (Fl_Widget*w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	c->_camera->clearMasks();
}

void select_sensor_cb (Fl_Widget*w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	// determine sensor ID
	const char *sensor = c->calib_camera->value();
	
	if (strcmp(sensor,"Camera 0") == 0)
		c->_calibrated_camera = 0;
	if (strcmp(sensor,"Camera 1") == 0)
		c->_calibrated_camera = 1;
	if (strcmp(sensor,"Camera 2") == 0)
		c->_calibrated_camera = 2;
	if (strcmp(sensor,"Camera 3") == 0)
		c->_calibrated_camera = 3;
	if (strcmp(sensor,"Camera 4") == 0)
		c->_calibrated_camera = 4;
	if (strcmp(sensor,"Camera 5") == 0)
		c->_calibrated_camera = 5;
}

void reproject_edges_cb (Fl_Widget*w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if ( c->view_reprojected_edges->value() ) {
		c->_LUT.lookup( c->_camera->came.getTranslation(), 0.0, 0.0, 0, 0 );
		c->reprojectEdges( c->_LUT._lines );
	} else {
		c->_camera->clearReprojectedEdges();
	}
		
	c->redraw();
}


void redraw_cb (Fl_Widget*w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	c->redraw();

	c->_LUT._draw_grid = c->model_show_grid->value();
	c->_LUT._draw_pixels = c->model_show_pixels->value();

	return;

	if (c->_windowMode == MODE_VIDEO) {
		if ((c->select_generic->changed()) && (c->_camera->_frame._selected_edges.size() == 1)) {
			c->_camera->_frame._selected_edges[0] = c->_camera->_frame.getEdgePlane(c->select_generic->value());
		}
	}

	if (c->_windowMode == MODE_3D_MAP) {
		if ((c->select_generic->changed()) && (c->_LUT._selected_lines.size() == 1)) {
			c->_LUT._selected_lines[0] = c->_LUT._model->getEdge(c->select_generic->value());
		}
	}

	//if (c->_windowMode == MODE_SPHERE) {
	//	c->_corrManager->setSelectedCorrespondence(c->select_generic->value());
	//}

	if (c->_sphere_viewer._type == WALKTHROUGH) {
		c->_sphere_viewer._eye = c->_camera->fromCameraFrameToWorldFrame(c->_camera->getUnitCameraCenter(c->_calibrated_camera));
	}

	// reset fast tracker if flow has been activated
	if ( c->view_flow->changed() && c->view_flow->value() ) {
		for (int i=0;i<6;i++) 
			c->_camera->_frame.ftracker[i]->clear();
	}

	c->redraw();
}

void set_video_mode_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_windowMode = MODE_VIDEO;

	Fl::check();

	redraw_cb(w, NULL);
}

void set_3dmap_mode_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_windowMode = MODE_3D_MAP;

	Fl::check();
	redraw_cb(w, NULL);
}
void set_2dmap_mode_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_windowMode = MODE_2D_MAP;

	Fl::check();
	redraw_cb(w, NULL);
}
void set_sphere_mode_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_windowMode = MODE_SPHERE;

	Fl::check();
	redraw_cb(w, NULL);
}
void set_calibration_mode_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_windowMode = MODE_CALIBRATION;

	Fl::check();
	redraw_cb(w, NULL);
}

void clear_selected_edges_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if (c->_windowMode == MODE_3D_MAP) {
		c->_LUT._selected_lines.clear();
		c->_LUT._selected_lines_sphere.clear();
	}
	else {
		c->_LUT._selected_lines_sphere.clear();
		c->_camera->clearSelectedEdges();
	}

	//c->_poses.clear();
	c->redraw();
}

void sphere_calib_apply_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	// update the camera intrinsic parameters
	/*Mat3d R = fromEulerTo3x3Matrix(c->sphere_calib_yaw->value(),c->sphere_calib_pitch->value(),c->sphere_calib_roll->value());
		
	Matd M = Matd(4,4,R[0][0],R[0][1],R[0][2],0.0,R[1][0],R[1][1],R[1][2],0.0,R[2][0],R[2][1],R[2][2],0.0,0.0,0.0,0.0,1.0);
	
	c->_camera->_extrinsic[c->_calibrated_camera] = M * c->_camera->_extrinsic_dft[c->_calibrated_camera];
	c->_camera->convertLinesToPlanes();
	
	LOG( LEVEL_INFO, "Camera %d", c->_calibrated_camera );
	printMatrix( inv( c->_camera->_extrinsic[c->_calibrated_camera] ) );
*/

	// rotate and translate the camera
	Vec3d position = c->_camera->getTranslation();
	position[2] += c->sphere_calib_z->value();
	c->sphere_calib_z->value(0.0);
	c->_camera->setTranslation( position );

	c->_camera->rotate(Vec3d(0,0,1),c->sphere_calib_yaw->value());
	c->sphere_calib_yaw->value(0.0);
	c->_camera->rotate(Vec3d(1,0,0),c->sphere_calib_pitch->value());
	c->sphere_calib_pitch->value(0.0);
	c->_camera->rotate(Vec3d(0,1,0),c->sphere_calib_roll->value());
	c->sphere_calib_roll->value(0.0);

	// reproject edges if needed
	if ( c->view_reprojected_edges->value() ) {
		c->_LUT.lookup( c->_camera->came.getTranslation(), 0.0, 0.0, 0, 0 );
		c->reprojectEdges( c->_LUT._lines );
	}

	// print camera pose
	c->_camera->printPose();

	//c->redraw();
}

// lock recovery procedure
void lock_recovery_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	// compute camera rotation from 2 correspondences

	if ( c->_camera->_frame._selected_edges.size() == 2  && c->_LUT._selected_lines.size() == 2 ) {

		Vec3d lj1a = c->_LUT._selected_lines[0]->getA() - c->_camera->getTranslation();
		Vec3d lj1b = c->_LUT._selected_lines[0]->getB() - c->_camera->getTranslation();
		Vec3d lj2a = c->_LUT._selected_lines[1]->getA() - c->_camera->getTranslation();
		Vec3d lj2b = c->_LUT._selected_lines[1]->getB() - c->_camera->getTranslation();
		
		Vec3d ei1a = c->_camera->_frame._selected_edges[0]._a ;
		Vec3d ei1b = c->_camera->_frame._selected_edges[0]._b ;
		Vec3d ei2a = c->_camera->_frame._selected_edges[1]._a ;
		Vec3d ei2b = c->_camera->_frame._selected_edges[1]._b ;

		Vec3d rotation = Vec3d(0,0,0);
		double overlap = 0.0;
		minRotation2Planes2Lines( ei1a, ei1b, ei2a, ei2b, lj1a, lj1b, lj2a, lj2b, M_PI, overlap, rotation );
		Quaternion q = Quaternion( rotation, len(rotation) );
		c->_camera->setRotation( q );

		// check
		Vec3d t1 = norm(cross(ei1a,ei1b));
		Vec3d t2 = norm(cross(norm(lj1a),norm(lj1b)));
		LOG(LEVEL_INFO, "check 1 : %f", len(cross(t2,Quaternion(rotation,len(rotation)).rotate(t1))));

		ei1a = q.rotate( ei1a ); // rotate the observed edges
		ei1b = q.rotate( ei1b );
		ei2a = q.rotate( ei2a );
		ei2b = q.rotate( ei2b );
		Vec3d ne1 = norm(cross(norm(ei1a),norm(ei1b)));
		Vec3d ne2 = norm(cross(norm(ei2a),norm(ei2b)));
		Vec3d nl1 = norm(cross(norm(lj1a),norm(lj1b)));
		Vec3d nl2 = norm(cross(norm(lj2a),norm(lj2b)));

		LOG(LEVEL_INFO, "[%d %d] check: %f %f angle = %f -- overlap = %f", c->_LUT._selected_lines[0]->_id, c->_LUT._selected_lines[1]->_id, \
			len(cross(ne1,nl1)), len(cross(ne2,nl2)), len(rotation), overlap );

		return;
	}

	// compute camera pose from 3 correspondences

	if ( c->_camera->_frame._selected_edges.size() == 3  && c->_LUT._selected_lines.size() == 3 ) {
	EdgePlane P = c->_camera->_frame._selected_edges[0];
	EdgePlane Q = c->_camera->_frame._selected_edges[1];
	EdgePlane R = c->_camera->_frame._selected_edges[2];

	Edge *p = c->_LUT._selected_lines[0];
	Edge *q = c->_LUT._selected_lines[1];
	Edge *r = c->_LUT._selected_lines[2];

	ExtrinsicParameters pose;

	c->cameraPoseFromThreeCorrespondences(  P, Q, R, p, q, r, pose );
	return;
	}


	// compute the score
	if ( c->_camera->_frame._selected_edges.size() != 3  || c->_LUT._selected_lines.size() != 3 ) {
		std::vector< intVector > edges_buckets;
		c->distribute_edgeplanes_into_buckets( edges_buckets );
		LOG(LEVEL_INFO, "score current pose: %f", c->score_camera_pose(c->_camera->getPose(), edges_buckets) );
		return;
	}
}

// refine camera pose from a coarse estimate
void refine_camera_pose_cb ( Fl_Widget *w, void* )
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	// compute edges, corners and correspondences from corners
	// detect and chain edges
	if ( c->_database == NULL) {
		edgePlaneVector edgeplanes;
		c->initIdealCorrespondences( edgeplanes );}
	else {
		detect_edges_cb( w, NULL );
	}

	// distribute visible edges into buckets
	std::vector< intVector > edges_buckets;
	c->distribute_edgeplanes_into_buckets( edges_buckets );

	// refine camera pose
	c->refine_camera_pose( edges_buckets );
}

void compute_pose_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if ( c->_camera->_frame._selected_edges.size() == 3 ) {

		EdgePlane P = c->_camera->_frame._selected_edges[0];
		EdgePlane Q = c->_camera->_frame._selected_edges[1];
		EdgePlane R = c->_camera->_frame._selected_edges[2];

		Edge *p, *q, *r;

		if ( c->_LUT._selected_lines.size() == 3 ) {

			p = c->_LUT._selected_lines[0];
			q = c->_LUT._selected_lines[1];
			r = c->_LUT._selected_lines[2];

			ExtrinsicParameters pose;

			if ( c->cameraPoseFromThreeCorrespondences( P, Q, R, p, q, r, pose ) ) {
				LOG(LEVEL_INFO, "manual pose computation SUCCESS.");
				c->_camera->setPose( pose );
			} else {
				LOG(LEVEL_INFO, "manual pose computation FAILED.");
			}
		}
	}
}

void process_init_mode_checks( Fl_Widget *w, void*)
{

	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->initModes.clear();

	for ( int t=0; t < c->_menu_bar->size(); t++ ) {
		Fl_Menu_Item *m = (Fl_Menu_Item*)&(c->_menu_bar->menu()[t]);

		if ( m->label() == NULL ) {
			continue;
		}

		if ( strcmp( m->label(), "Vertical pose" ) == 0 ) {
			if ( m->value() == 4 ) 
				c->initModes.push_back( INIT_VERTICAL );
		}
		if ( strcmp( m->label(), "Line-line matching" ) == 0 ) {
			if ( m->value() == 4 ) {
				c->initModes.push_back( LINE_LINE);
				LOG(LEVEL_INFO, "METHOD: LINE_LINE");
			}
		}
		if ( strcmp( m->label(), "Rotation voting" ) == 0 ) {
			if ( m->value() == 4 ) 
				c->initModes.push_back( ROTATION_VOTING);
		}
		if ( strcmp( m->label(), "Corners matching" ) == 0 ) {
			if ( m->value() == 4 ) 
				c->initModes.push_back( CORNERS);
		}
		if ( strcmp( m->label(), "Vanishing points" ) == 0 ) {
			if ( m->value() == 4 ) 
				c->initModes.push_back( VANISHING_POINTS);
		}
		if ( strcmp( m->label(), "Exhaustive Search" ) == 0 ) {
			if ( m->value() == 4 ) 
				c->initModes.push_back( EXHAUSTIVE_SEARCH );
		}
	}
}

void select_generic_changed_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if ( strcmp( c->sphere_correspondence_mode->value(), "maintenance" ) == 0 ) {
		if ( c->view_correspondences->value() ) {
			if ( c->select_generic->value() < SM_Size() ) {
				corres cc = SM_Get( c->select_generic->value() );
				if ( cc.valid() ) 
					c->_calibrated_camera = cc.eps[cc.eid]._cameraId;
			}
		}
	}

	if ( strcmp( c->sphere_correspondence_mode->value(), "init" ) == 0 ) {

		if ( c->select_generic->value() < c->init_poses.size() ) {

			c->_camera->setPose( c->init_poses[c->select_generic->value()] );

			c->view_reprojected_edges->value(1);

			reproject_edges_cb( w, NULL );
		}
	}

	c->redraw();
}

void select_correspondence_mode_changed_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if ( strcmp( c->sphere_correspondence_mode->value(), "init" ) == 0 ) {

		c->select_generic->value(0);
	
		select_generic_changed_cb( w, NULL );
	}
}

void option_texture_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	c->_draw_texture = !c->_draw_texture;

	c->redraw();
}

void option_vps_display_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_display_vps = ! c->_display_vps;
	
	if ( c->_display_vps ) {
		
		edgePlaneVector edgeplanes;
		// clear the output vector
		edgeplanes.clear();
		
		// fill in the edgeplane vector
		for (int i=0;i<c->_camera->_frame._edgeplanes_chained.size();i++) {
			for (int j=0;j<c->_camera->_frame._edgeplanes_chained[i].size();j++) {
				edgeplanes.push_back( c->_camera->_frame._edgeplanes_chained[i][j] );
			}
		}
		
		c->compute_vanishing_points( c->INIT_TOPK_VPS, edgeplanes, c->_vps_edges );
				
		edgePlaneVector lines_edgeplanes;
		Vec3d center = c->_camera->getTranslation();
		
		for (i=0;i<c->_LUT._lines.size();i++) {
			EdgePlane ep( c->_LUT._lines[i]->getA() - center, c->_LUT._lines[i]->getB() - center, center, -1, c->_LUT._lines[i]->_id, i );
			ep.fromWorldFrameToCameraFrame( c->_camera->getPose() );
			lines_edgeplanes.push_back( ep );
		}
		
		c->compute_vanishing_points( c->INIT_TOPK_VPS, lines_edgeplanes, c->_vps_lines );
	}

	c->redraw();

}

// this method generates a synthetic image at a random location in the model
// and runs the INIT algorithm on it + returns statistics on the results
//
void init_synthetic_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	const char *str = fl_input("Number of runs", "300");
	if (str == NULL) {
		return;
	}
	
	int nruns;
	
	if ( sscanf(str, "%d", &nruns) != 1 )
		return;

	c->run_synthetic_init( nruns );
}

void init_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if ( !c->init->value() )
		return;

	process_init_mode_checks( w, NULL );

	c->init_pose();

	c->init->value(0);
}

void sift_features_cb ( Fl_Widget *w, void*)
{
	return;

	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if ( c->_database == NULL )
		return;

	for ( int t=0; t < c->_menu_bar->size(); t++ ) {
		Fl_Menu_Item *m = (Fl_Menu_Item*)&(c->_menu_bar->menu()[t]);

		if ( m->label() == NULL ) {
			continue;
		}

		if ( strcmp( m->label(), "SIFT" ) == 0 ) {
			if ( m->value() == 4 ) {
				c->_camera->_frame.computeSiftFeatures();
			} else {
				for (int i=0;i<6;i++) 
					c->_camera->_frame._sift[i].clear();
				c->_camera->_frame._sift_computed = false;
			}
		}
	}
}

// set the initialization mode
//
void init_set_mode_cb(Fl_Widget *w, void*)
{
}

// set the search area size (in terms of node neighborhood)
void init_set_search_area_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	const char *input = fl_input("Search depth","0");  // default value: 5fps
	int search_depth = 0;
	if ( sscanf( input, "%d", &search_depth ) != 1 )
		return;

	c->_init_search_depth = search_depth;

	// reset init score
	c->_init_score = -1.0;
}

// compute all edges for the sequence
//
void compute_sequence_edges_cb (Fl_Widget *w, void *)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	// exit if no database
	if ( c->_database == NULL )
		return;

	// prompt for user confirmation
	int ok = fl_choice("Are you sure?","No","Yes",NULL);

	if ( !ok )
		return;

	// compute all edges for this sequence

	// open edge files
	c->_camera->fname_edges = c->_database->_dirname + "/edges.txt";
	c->_camera->fname_edges_filtered = c->_database->_dirname + "/edges_filtered.txt";
	c->_camera->fname_edges_ref = c->_database->_dirname + "/" + CAMERA_EDGES_REF_FILE;

	// init edges files
	c->_camera->initEdgesFiles( c->_database->_ladybug_id, c->_database->_width, c->_database->_height );

	// reset frame id
	c->frameId = 0;
	c->_database->_frameId = 0;

	// for each frame, compute edges
	for (int frameid = 0; frameid < c->_database->_nImages; frameid++) {

		// read that frame
		c->_database->loadFrame(c->_camera->_frame);
		c->_camera->_frame.processFrame();
		c->frameId = c->_database->_frameId;
		c->frame_slider->value(c->frameId,3,1,c->_database->_nImages-1);
		c->redraw();
		Fl::check();

		// compute edges
		detect_edges_cb( w, NULL );
		c->redraw();
		Fl::check();

		// store edges
		c->_camera->writeEdgesToFile( frameid );

		// move to next frame
		c->_database->nextFrame();

	}
}

// find pose given 3 correspondences
void run_analysis_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->edgeDetectionErrorAnalysis ();

	c->redraw();
}

void run_maintenance_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	c->_last_recovery = 0;
	

	if ( c->_synthetic_mode != 0 ) {
		while ( c->run_maintenance->value() ) {
			next_frame_cb(w,NULL);
			Fl::check();
			c->redraw();
		}
	} else {
		while ( c->run_maintenance->value() && (c->frameId < c->_database->_nImages-1) ) {
			
			next_frame_cb(w,NULL);
			//c->drawVideo(true);
			Fl::check();
			c->redraw();
			//c->snapshotVideo();
			//c->snapshotVideo();
			//Fl::check();
			//c->redraw();
		}
	}

}

void next_frame_cb (Fl_Widget *w, void*)
{
		// display info about the GL library
	//LOG( LEVEL_INFO, "%s", glGetString( GL_RENDERER ) ); 
	//LOG( LEVEL_INFO, "%s", glGetString( GL_VENDOR ) ); 
	//LOG( LEVEL_INFO, "%s", glGetString( GL_VERSION ) ); 

	PerfTimer timer;

	int i;

	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	// run synthetic mode if needed
	if ( c->syntheticNextFrame() ) {// for synthetic mode 3, generate synthetic correspondences and return true
		c->redraw();
		return;
	}

	timer.print("new frame");

	if ( c->_synthetic_mode == 0 ) {
		// clear the reprojected edges
		//c->_camera->clearReprojectedEdges();
		
		// read the next frame image
		if ( c->_database == NULL ) {
			LOG(LEVEL_ERROR, "open a database!");
			return;
		}
		c->_database->nextFrame();
		c->_database->loadFrame(c->_camera->_frame);
		c->_camera->_frame.processFrame();
		c->frameId = c->_database->_frameId;
		c->frame_slider->value(c->frameId,3,1,c->_database->_nImages-1);
		c->redraw();
		Fl::check();
		
		LOG(LEVEL_DEBUG,"load: %f (frame ID = %d)",timer.elapsed(),c->_database->_frameId);
		
		// first initialize the edge detector (compute Laplacian of Gaussian)
		c->_camera->initializeEdgeDetectors();
	}

	timer.print("read frame");

	// set camera min and max mask width
	//c->_camera->_min_mask_width = 5;
	//c->_camera->_max_mask_width = 20;

	// synthetic mode 1: generate synthetic images
	if ( c->_synthetic_mode == 1 ) {
		c->generateSyntheticFrame();
	}

	// detect edges
	c->detect_edges();

	// sort edges
	c->distribute_edgeplanes_into_buckets( c->_edges_buckets );

	/*
	// if tracker mode and pose exists, get the pose and update the tracker only
	if ( strcmp( c->sphere_correspondence_mode->value(), "tracker" ) == 0 ) {
			
		// get the pose
		bool pose_found = false;

		for (i=0;i<c->pose_history.size();i++) {
			if ( c->pose_history[i].id == c->frameId ) {
				c->_camera->setPose( c->pose_history[i] );
				pose_found = true;
				break;
			}
		}
		
		if ( pose_found )
			c->updateEdgeTracker();

		return;
	}*/

	// second query the visibility LUT
	// and fire NODE_CHANGED if the node has changed
	if ( c->_LUT.lookup( c->_camera->getTranslation(), c->MIN_SUBTENDED_ANGLE, 0.0, 100, 0) ) {

		SM_NodeChanged( c->_LUT._lines );

		//relock_cb( w, NULL );

		//init_correspondences_cb( w, NULL );

	}

	timer.print("node change");

	// update the state machine history
	SM_UpdateHistory();

	timer.print( "history updated" );

	// update correspondences and compute camera pose
	bool ok = c->update_correspondences();

	if ( !ok ) {

		corresVector correspondences;
		c->init_correspondences( correspondences, c->_edges_buckets, c->MAINTENANCE_DIHEDRAL_ANGLE, c->MAINTENANCE_MIN_OVERLAP, true );
		
		ok = c->update_correspondences();
	}

	if ( !ok ) {

		c->relock( toRadians(20.0), 5000 );

		ok = c->update_correspondences();
	}

	// if tracker mode and pose exists, get the pose and update the tracker only
	if ( strcmp( c->sphere_correspondence_mode->value(), "tracker" ) == 0 ) {
			
		// get the pose
		bool pose_found = false;

		for (i=0;i<c->pose_history.size();i++) {
			if ( c->pose_history[i].id == c->frameId ) {
				//c->_camera->setPose( c->pose_history[i] );
				pose_found = true;
				break;
			}
		}
	}

	SM_UpdateHistory2( c->_camera->getPose() );

	// refine the camera pose from accepted correspondences
	// c->refine_from_accepted_correspondences();

	SM_Statistics( c->frameId, c->_camera->getPose() );

	// update the line states history
	c->updateLineHistory();

	// write out the line states history
	c->writeLineHistory();
	
	//	SM_InsertEvent( NODE_CHANGED );

	// process the queue
	//SM_ProcessItems();

	// print the queue
	//SM_Print();
	timer.print("process items");

	// compute the camera pose
	//SM_ComputePose(POSE_MAX_ELEMENTS, POSE_MIN_ELEMENTS, POSE_MAX_TRIALS, POSE_THRESHOLD_SCORE, c->_inliers);

	timer.print("compute pose");

	// recompute the distribution (for display only)
	SM_ComputeDistribution( c->correspondence_distribution );

	// reproject edges if needed
	reproject_edges_cb(w,NULL);
	
	// update the pose history
	ExtrinsicParameters pose = c->_camera->getPose();
	pose.id = c->frameId;

	c->pose_history.push_back( pose );

	// save the pose in a file
	FILE *pose_file = fopen("poses.dat","a");
	c->_camera->writePose(c->frameId, pose_file);
	fclose(pose_file);

	// write down error in synthetic mode
	if ( c->_synthetic_mode != 0 ) {
		c->writeError( "analysis.dat", c->frameId, c->_camera->getPose(), c->GetSyntheticPose( c->frameId ) );
	} else {
		if ( c->_database->_synthetic ) {
			ExtrinsicParameters ref_pose;
			if ( c->_database->readPose( c->frameId, ref_pose ) ) {
				double error_trans = len(ref_pose.getTranslation() - c->_camera->getTranslation());
				c->writeError( "synth_loc_error.dat", c->frameId, c->_camera->getPose(), ref_pose );
				if ( error_trans > c->_LUT.inchToModelUnit( 5.0 ) )	// stop the calculation if the error is too large
					c->run_maintenance->value(0);
			}
		}
	}
	
	timer.print("debug printing");
	
	// re-lock if things went wrong
	if ( SM_Size() <= 10 ) {	
		
		//relock_cb(w,NULL);

	}	
		// detect edges
	//	c->detect_edges();
		
		// distribute visible edges into buckets
	//	std::vector< intVector > edges_buckets;
	//	c->distribute_edgeplanes_into_buckets( edges_buckets );
		
	//	c->refine_camera_pose( edges_buckets );
	//	timer.print("relock");
	//}

	Fl::check();

	c->redraw();
	timer.print("redraw");
	return;
}

void average_pose_history_cb ( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	const char *input = fl_input("Frames","0");  

	int frames = 0;

	if ( c->_database == NULL )
		return;

	if (sscanf(input, "%d", &frames) != 1 )
		return;

	if (frames == 0)
		return;

	int frame_start = 0;

	if ( c->frameId > frames ) 
		frame_start = c->frameId - frames;
	else
		frames = c->frameId;

	for (int frameid = frame_start; frameid < c->frameId; frameid++) {

		double alpha = 1.0 - (double)(frameid-frame_start) / frames;

		c->pose_history[frameid].setTranslation( 4.0 * Vec3d(ran1(&c->_idum),ran1(&c->_idum),ran1(&c->_idum)) + alpha * c->pose_history[frame_start].getTranslation() + (1.0-alpha) * c->pose_history[c->frameId].getTranslation() );
	}

}

void read_config_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->readConfig();

	c->redraw();
}

void select_pose_cb (Fl_Widget *w, void*)
{

	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	int n = c->select_pose->value();

	if (n> c->_solutions.size())
		return;

	// select new camera pose
	c->_camera->setRotation( c->_solutions[n].getRotation() );

	c->redraw();
}

// clear the state machine
//
void clear_sm_cb ( Fl_Widget *w, void*)
{
	SM_Clear();
}

// add a correspondence in the state machine
void add_sm_cb  ( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if ( c->_camera->_frame._selected_edges.empty() )
		return;

	if ( c->_LUT._selected_lines.empty() )
		return;

	Edge *line = c->_LUT._selected_lines[0];
	EdgePlane edgeplane = c->_camera->_frame._selected_edges[0];

	SM_InsertItem( _CONNECTED, edgeplane, line, CONNECTED_MIN_AGE_FOR_POSE );
	
	c->_LUT._selected_lines.clear();
	c->_camera->_frame._selected_edges.clear();
}

void relock_cb ( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	
	c->relock( toRadians(20.0), 5000 );
}

void init_correspondences_cb ( Fl_Widget *w, void*)
{

	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	// detect edges
	c->detect_edges();

	// distribute visible edges into buckets
	std::vector< intVector > edges_buckets;
	c->distribute_edgeplanes_into_buckets( edges_buckets );

	corresVector correspondences;
	c->init_correspondences( correspondences, edges_buckets, c->MAINTENANCE_DIHEDRAL_ANGLE, c->MAINTENANCE_MIN_OVERLAP, true );

	//c->update_init_pose( edges_buckets );

	LOG(LEVEL_INFO, "%d correspondences.", correspondences.size() );

}


void screenshot_cb ( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);
	unsigned char *image;
	image = (unsigned char*)malloc(WINDOW_WIDTH*WINDOW_HEIGHT*3*sizeof(unsigned char));

	c->activate();
	fl_read_image( image, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT );
	cvSetImageData(c->_screenshot_image,image,WINDOW_WIDTH*3);

	char filename[256];
	sprintf(filename, "snaps/%d.bmp", c->_timestamp);
	cvSaveImage(filename,c->_screenshot_image);
	c->_timestamp++;
	delete image;
}

void switch_view_cb (Fl_Widget *w, void*)
{

	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if (c->_sphere_viewer._type == OUTSIDE) {
		c->_sphere_viewer._type = WALKTHROUGH;
		c->_sphere_viewer.restoreView();
		c->_sphere_viewer._eye = c->_camera->fromCameraFrameToWorldFrame(c->_camera->getUnitCameraCenter(c->_calibrated_camera));
	} else {
		c->_sphere_viewer._type = OUTSIDE;
		c->_sphere_viewer.restoreView();
	}
}

// start/stop recording of camera poses
void make_synthetic_cb (Fl_Widget *w, void *)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_synthetic_start_pose = c->_camera->getPose();

	c->makeSyntheticSequence();
}

// write the history of camera poses in a file
void write_history_cb (Fl_Widget *w, void*)
{

	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if ( c->_database == NULL )
		return;

	std::string filename = /*c->_database->_dirname + "/" + */ POSE_HISTORY_FILE;

	FILE *f = fopen( filename.c_str(), "w" );

	if ( f == NULL )
		return;

	for ( int i=0; i<c->pose_history.size(); i++) {
		fprintf(f, "%d ", i);
		c->pose_history[i].write( f );
	}

	fclose( f );
}

// read the history of camera poses in a file
void read_history_cb (Fl_Widget *w, void*)
{

	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	if ( c->_database == NULL )
		return;

	c->pose_history.clear();

	std::string filename = c->_database->_dirname + "/" + POSE_HISTORY_FILE;

	FILE *f = fopen( filename.c_str(), "r" );

	if ( f == NULL )
		return;

	int frameid;
	int counter = 0;
	ExtrinsicParameters pose;

	double distance = 0.0;

	while ( !feof( f ) ) {
		if ( fscanf(f, "%d", &frameid ) != 1 )
			break;

		if ( frameid < counter ) {
			pose.read( f );
			continue;
		}

		if ( frameid > counter ) {
			while ( frameid > counter ) {
				c->pose_history.push_back( pose );
				counter++;
			}
		}

		if ( pose.read( f ) ) {
			pose.id = frameid;
			Vec3d p1;
			if ( c->pose_history.size() > 1 ) 
				p1 = c->pose_history.back().getTranslation();

			c->pose_history.push_back( pose );

			if ( c->pose_history.size() > 2 )
				distance += len(p1 - pose.getTranslation());

			//LOG(LEVEL_INFO, "distance parcourue: %f", distance);

			counter++;
		}
	}

	LOG(LEVEL_INFO, "distance parcourue: %f", distance);

	fclose( f );

	LOG(LEVEL_INFO, "read %d poses in history", c->pose_history.size());
}

void ladybug_record_cb ( Fl_Widget *w, void*)
{

	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	// check that the Ladybug is present
	if (!c->_ladybug->_init)
		return;

	// stop record
	if ( c->_ladybug_recording ) {
		
		c->_ladybug_recording = false;

		if ( c->_database == NULL ) 
			return;

		c->_database->_nImages = c->_ladybug_record_nimages;
	
		// create a database info file
		c->_database->createDataFile();

		// exit the current database
		exit_database_hd_cb( w, NULL );

		//if ( c->xsens_log_file ) {
		//	fclose(c->xsens_log_file);
		//	c->xsens_log_file = NULL;
		//}

		return;
	}

	// start record

	// exit the current database
	exit_database_hd_cb( w, NULL );

	// create a new database and prompt the user for a new location
	char *f = fl_dir_chooser("Choose database", "C:/omni3d/data/", 0); // 0 : full path name
	if (f == NULL)
		return;
	
	c->_database = new Database();
	
	// prompt user for frame rate
	const char *input = fl_input("Frame rate","5.0");  // default value: 5fps
	double fps = 0.0;
	if ( sscanf( input, "%lf", &fps ) != 1 )
		return;

	c->_ladybug_record_framerate = fps;

	// init database
	c->_database->initNewDatabase ( f, c->_ladybug_record_framerate );

	// set the record flag
	c->_ladybug_recording = true;

	c->_ladybug_record_nimages = 0;

	c->frameId = 0;

	c->_database->_frameId = 0;

	// reset xsens log file
	FILE *fd = fopen( XSENS_LOG_FILE, "w" );
	fclose(fd);

	while ( c->_ladybug_recording ) {
		
		fd = fopen( XSENS_LOG_FILE, "a" );
		fprintf(fd, "%d\n", c->frameId);
		fclose(fd);	
		fflush(fd);

		c->_ladybug->grab(c->_camera->_frame._grab_img);
		
		c->_camera->_frame.processFrame();

		for (int sensorId = 0; sensorId < 6; sensorId++ ) {
			std::string filename = c->_database->_dirname + "//" + c->_database->getLadybugImageFilename( sensorId, c->frameId );

			cvSaveImage( filename.c_str(), c->_camera->_frame._tmp_img[sensorId] );
		}

		c->_ladybug_record_nimages++;
		c->frameId++;
		c->_database->_frameId = c->frameId;

		//c->_ladybug->grab(); // grab an image on the ladybug
		//Sleep(1000/fps);
		printf("FRAME ID = %d\n", c->frameId );
		//c->frameId++;
	

		Fl::check();
	}

	c->redraw();
}

void switch_bgd_color_cb( Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->BACKGROUND_WHITE = 1 - c->BACKGROUND_WHITE;
}



void color_read_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	// prompt user for level of detail
	const char *input = fl_input("Level of detail","0");

	if ( input == NULL )
		return;

	int lod;
	if (sscanf(input,"%d", &lod) != 1 )
		return;

	c->_LUT._model->_lod = lod;

	// read colors in model
	c->_LUT._model->readColorHits();
}

void color_process_frame_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->computeColorHits( c->_LUT._model->_lod );

	c->redraw();
}

void color_process_frame_series_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_processing_frame_colors = !c->_processing_frame_colors;

	if ( !c->_processing_frame_colors )
		return;

	while ( c->_processing_frame_colors ) {

		// process the frame color
		c->computeColorHits( c->_LUT._model->_lod );

		// go to next frame
		int frameid = c->frame_slider->value();

		if ( c->_database == NULL || frameid+4 >= c->_database->_nImages ) {
			c->_processing_frame_colors = false;
			return;
		}

		c->frame_slider->value(frameid+4,3,1,c->_database->_nImages-1);

		frame_slider_changed_cb(w, NULL);

		// write colors for backup
		color_write_cb(w, NULL);

		Fl::check();

		c->redraw();
	}
}

void color_write_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_LUT._model->writeColorHits();

	c->redraw();

}

void color_clear_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_LUT._model->clearColorHits();

}

void option_line_color_coding_cb (Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_line_color_coding = !c->_line_color_coding;

	c->redraw();
}

void anchor_add_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_adding_anchor = true;
	c->_updating_anchor = false;
	c->_removing_anchor = false;

	c->redraw();
}

void anchor_remove_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_adding_anchor = false;
	c->_updating_anchor = false;
	c->_removing_anchor = true;

}

void anchor_update_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_adding_anchor = false;
	c->_updating_anchor = true;
	c->_removing_anchor = false;
}

void anchor_clear_all_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_anchors.clear();
}

void anchor_hide_show_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_show_anchor = ! c->_show_anchor;

	c->redraw();
}

void tracker_write_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->WriteEdgeTracker();
}

void tracker_read_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->ReadEdgeTracker();

	frame_slider_changed_cb( w, NULL );
}

void tracker_clear_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->_tracked_edges.clear();
}

void tracker_compute_cb(Fl_Widget *w, void*)
{
	MyGlWindow* c = (MyGlWindow*)w->parent()->child(0);

	c->processTrackedEdges();

	c->redraw();
}
