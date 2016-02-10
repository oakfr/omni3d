#include "main.h"

void init(MyGlWindow *gl_main)
{
	int i;

	// read the config file
	gl_main->readConfig();

	gl_main->_initGraphics = false;
	
	gl_main->_ladybug = new Ladybug();
	Ladybug *ladybug = gl_main->_ladybug;
	if (!ladybug->init())
		printf("warning: failed to initialize ladybug!\n");
	gl_main->_camera = new Camera();
	gl_main->_locked = false;
	
	// load the extrinsic calibration file
	char fullname[256];
	sprintf(fullname, "%s/config/%s", PROJECT_HOME, gl_main->LADYBUG_CALIBRATION_FILE);
	gl_main->_ladybug->loadCalibrationFile( fullname	);
	for (i=0;i<6;i++) {
		double params[6];
		gl_main->_ladybug->getUnitExtrinsic( i, params );
		for (int j=0;j<6;j++)
			gl_main->_camera->_ladybug_extrinsic[i][j] = params[j];
	}

	// by default, init model on stata 32-33x
	gl_main->_LUT.init (WINDOW_WIDTH-CONTROLBAR_WIDTH,WINDOW_HEIGHT-MENUBAR_HEIGHT, "");
	//gl_main->_sphere_viewer._target = gl_main->_LUT._position;
	//gl_main->_camera->came.getTranslation() = gl_main->_LUT._position;
	//gl_main->_LUT.lookup(true, gl_main->_camera->came.getTranslation());


	//gl_main->_zoomFactor = 2;
	gl_main->_camera->init(LADYBUG,gl_main->_ladybug,ladybug->_uiCols,ladybug->_uiRows,gl_main->_zoomFactor,Vec3d(0,0,0),
		fromEulerTo3x3Matrix(0,0,0), gl_main->SPHERE_TESSELLATION_RESOLUTION);

	gl_main->_camera->computeSphericalFrustum();
	gl_main->_display_3d_edges = true;
	
	// init the SFM viewer
	gl_main->_sphere_viewer.set(WINDOW_WIDTH-CONTROLBAR_WIDTH,WINDOW_HEIGHT-MENUBAR_HEIGHT,0);
	gl_main->_sphere_viewer.init(gl_main->_LUT._position,100.0,3*M_PI/4,M_PI/2,45,1.0,10000.0,OUTSIDE);
	gl_main->_pauseVideo = false;
	
	// init the rotations viewer
	gl_main->_rotations_viewer.set(WINDOW_WIDTH-CONTROLBAR_WIDTH,WINDOW_HEIGHT-MENUBAR_HEIGHT,0);
	gl_main->_rotations_viewer.init(Vec3d(0,0,0),3*M_PI,3*M_PI/4,M_PI/2,45,1.0,10000.0,OUTSIDE);

	// init the video viewer
	gl_main->_video_viewer.set(WINDOW_WIDTH-CONTROLBAR_WIDTH,WINDOW_HEIGHT,0);
	
	gl_main->_calib_img = cvCreateImage(cvSize(int(gl_main->_camera->_width), int(gl_main->_camera->_height)),  IPL_DEPTH_8U, 3);
	gl_main->_calib_rectified_img = cvCreateImage(cvSize(int(gl_main->_camera->_width), int(gl_main->_camera->_height)),  IPL_DEPTH_8U, 3);

		// by default load database on stata 33x
	gl_main->_database = NULL;
	
	// init the (empty) correspondence table
	//gl_main->_corrManager = new CorrespondenceManager(/*gl_main->_LUT._model*/);
	
	gl_main->_timestamp = 0; // for snapshot generation
	gl_main->_screenshot_image = cvCreateImage(cvSize(WINDOW_WIDTH,WINDOW_HEIGHT), IPL_DEPTH_8U, 3);
	//gl_main->SNAPSHOT_START_X = 300;
	//gl_main->SNAPSHOT_START_Y = 250;
	
	gl_main->_synthetic_mode = 0; // no synthetic mode a priori
	gl_main->frameId = 0;
	gl_main->_idum = -1; // initialize the random seed  -  see numerical.cpp

	gl_main->_ladybug_recording = false;

	if ( !gl_main->_config.load( "config.ini" )) {
		LOG(LEVEL_ERROR, "missing config file config.ini!");
	}

	// init the fast tracker
	for (i=0;i<6;i++) 
		gl_main->_camera->_frame.ftracker[i] = new fasttracker( gl_main->_camera->_height, gl_main->_camera->_width, 
			gl_main->FTRACKER_WINDOW_SIZE, gl_main->FTRACKER_MAX_FEATURES, gl_main->FTRACKER_MIN_DIST, gl_main->FTRACKER_TIME_TO_LIVE );
	
	// init the sphere tessellation for edgeplanes normals
	std::vector<Vec3d> points;
	gl_main->spheretsn = new Model();
	gl_main->spheretsn->MODEL_RESOLUTION = 0.0;
	gl_main->spheretsn->make_icosahedron( points );
	gl_main->spheretsn->tessellate( gl_main->SCORE_MAX_DIHEDRAL_ANGLE, points );

	gl_main->_init_search_depth = 0;
	gl_main->_init_score = -1.0;

	gl_main->_draw_texture = false;
	gl_main->_wireframe = false;

	gl_main->_processing_frame_colors = false;
	gl_main->_line_color_coding = true;

	gl_main->_adding_anchor = false;
	gl_main->_updating_anchor = false;
	gl_main->_removing_anchor = false;
	gl_main->_show_anchor = true;
	gl_main->_display_vps = false;

	gl_main->_n_correspondences = 0;


}

bool MyGlWindow::readConfig()
{
	if ( !_config.load( "config.ini" ) ) {
		LOG(LEVEL_ERROR, "missing config file config.ini!");
		return false;
	}

	INIT_DIAMETER = _config.getDoubleValue ( "INIT_DIAMETER" );
	MIN_SUBTENDED_ANGLE = toRadians( _config.getDoubleValue("MIN_SUBTENDED_ANGLE") );
	MIN_DIHEDRAL_ANGLE = toRadians( _config.getDoubleValue("MIN_DIHEDRAL_ANGLE") );

	INIT_OCCLUSION = _config.getBoolValue("INIT_OCCLUSION");
	INIT_CLUTTER = _config.getBoolValue("INIT_CLUTTER");
	INIT_NOISE = _config.getBoolValue("INIT_NOISE");
	INIT_SCORING_MODE = _config.getIntValue( "INIT_SCORING_MODE" );
	INIT_TOPK = _config.getIntValue( "INIT_TOPK" );
	INIT_MIN_DIHEDRAL_ANGLE  = toRadians( _config.getDoubleValue ( "INIT_MIN_DIHEDRAL_ANGLE" ) );

	MAINTENANCE_MIN_OVERLAP = _config.getDoubleValue( "MAINTENANCE_MIN_OVERLAP" );
	MAINTENANCE_DIHEDRAL_ANGLE = toRadians(_config.getDoubleValue( "MAINTENANCE_DIHEDRAL_ANGLE" ));
	MAINTENANCE_MAX_CORRESPONDENCES  = _config.getIntValue("MAINTENANCE_MAX_CORRESPONDENCES");

	INIT_RANSAC_TRIALS = _config.getIntValue( "INIT_RANSAC_TRIALS" );
	INIT_MAX_SCORE_ANGLE  = toRadians( _config.getDoubleValue( "INIT_MAX_SCORE_ANGLE" ) );
	INIT_REFINEMENT_ANGLE  = toRadians( _config.getDoubleValue( "INIT_REFINEMENT_ANGLE" ) );
	INIT_REFINEMENT_OVERLAP = _config.getDoubleValue( "INIT_REFINEMENT_OVERLAP" );
	INIT_TOPK_VPS  = _config.getIntValue( "INIT_TOPK_VPS" );
	BACKGROUND_WHITE = 0;

	INIT_MAX_VOTES = _config.getIntValue( "INIT_MAX_VOTES" );
	INIT_MAX_VPS_ANGLE = toRadians( _config.getDoubleValue( "INIT_MAX_VPS_ANGLE" ) );

	MAINTENANCE_HISTORY_SIZE = _config.getIntValue("MAINTENANCE_HISTORY_SIZE");

	INIT_TOPK_ROTATIONS = _config.getIntValue( "INIT_TOPK_ROTATIONS" );
	INIT_SEARCH_RADIUS_ROTATION = toRadians( _config.getDoubleValue( "INIT_SEARCH_RADIUS_ROTATION" ));

	FTRACKER_WINDOW_SIZE = _config.getIntValue( "FTRACKER_WINDOW_SIZE" );
	FTRACKER_MAX_FEATURES = _config.getIntValue( "FTRACKER_MAX_FEATURES" );
	FTRACKER_MIN_DIST = _config.getIntValue( "FTRACKER_MIN_DIST" );
	FTRACKER_TIME_TO_LIVE = _config.getIntValue( "FTRACKER_TIME_TO_LIVE" );
	FTRACKER_MIN_CORNER_ANGLE = toRadians( _config.getDoubleValue( "FTRACKER_MIN_CORNER_ANGLE" ) );

	SPHERE_TESSELLATION_RESOLUTION  = toRadians( _config.getDoubleValue( "SPHERE_TESSELLATION_RESOLUTION" ) );

	LADYBUG_CALIBRATION_FILE = (char*)_config.getStringValue( "LADYBUG_CALIBRATION_FILE" );

	N_CHAIN_EDGES  = _config.getIntValue ("N_CHAIN_EDGES" );

	INIT_MAX_CAMERA_HEIGHT = _config.getDoubleValue( "INIT_MAX_CAMERA_HEIGHT" );
	INIT_HEIGHT_STEPS = _config.getIntValue ( "INIT_HEIGHT_STEPS" );

	SCORE_MIN_OVERLAP = _config.getDoubleValue( "SCORE_MIN_OVERLAP" );
	SCORE_MAX_DIHEDRAL_ANGLE = toRadians( _config.getDoubleValue( "SCORE_MAX_DIHEDRAL_ANGLE" ) );
	INIT_CORNERS_TOPK = _config.getIntValue( "INIT_CORNERS_TOPK" );
	INIT_MIN_CORRESPONDENCES = _config.getIntValue( "INIT_MIN_CORRESPONDENCES" );

	LADYBUG_ID = _config.getIntValue("LADYBUG_ID");

	SNAPSHOT_X = _config.getIntValue("SNAPSHOT_X");
	SNAPSHOT_Y = _config.getIntValue("SNAPSHOT_Y");
	SNAPSHOT_W = _config.getIntValue("SNAPSHOT_W");
	SNAPSHOT_H = _config.getIntValue("SNAPSHOT_H");

	COLOR_THRESHOLD = _config.getDoubleValue( "COLOR_THRESHOLD" );

	STATE_COLOR_CODED = _config.getIntValue("STATE_COLOR_CODED");

	ANCHOR_SIZE  = _config.getDoubleValue("ANCHOR_SIZE");

	MIN_KEYFRAMES_PER_TRACK = _config.getIntValue ( "MIN_KEYFRAMES_PER_TRACK" );
	MAX_TRACK_AGE = _config.getIntValue ( "MAX_TRACK_AGE" );
	KEYFRAME_DISTANCE = _config.getDoubleValue ( "KEYFRAME_DISTANCE" );
	MIN_EDGE_SIZE_TRACKING = toRadians( _config.getDoubleValue( "MIN_EDGE_SIZE_TRACKING" ) );

	_zoomFactor = _config.getIntValue("VIDEO_DISPLAY_ZOOM");

	LOG(LEVEL_INFO, "INIT_DIAMETER: %f", INIT_DIAMETER);
	LOG(LEVEL_INFO, "MIN_SUBTENDED_ANGLE: %f", MIN_SUBTENDED_ANGLE);
	LOG(LEVEL_INFO, "MIN_DIHEDRAL_ANGLE: %f", MIN_DIHEDRAL_ANGLE);
	LOG(LEVEL_INFO, "INIT_OCCLUSION: %d", INIT_OCCLUSION);
	LOG(LEVEL_INFO, "INIT_CLUTTER: %d", INIT_CLUTTER);
	LOG(LEVEL_INFO, "INIT_NOISE: %d", INIT_NOISE);
	LOG(LEVEL_INFO, "INIT_SCORING_MODE = %d", INIT_SCORING_MODE );
	LOG(LEVEL_INFO, "INIT_TOPK = %d", INIT_TOPK);
	LOG(LEVEL_INFO, "INIT_MIN_DIHEDRAL_ANGLE = %.2f deg.", toDegrees( INIT_MIN_DIHEDRAL_ANGLE ) );
	LOG(LEVEL_INFO, "INIT_RANSAC_TRIALS = %d", INIT_RANSAC_TRIALS);
	LOG(LEVEL_INFO, "INIT_MAX_SCORE_ANGLE = %.2f deg.", toDegrees( INIT_MAX_SCORE_ANGLE ) );
	LOG(LEVEL_INFO, "INIT_REFINEMENT_ANGLE = %.2f deg.", toDegrees( INIT_REFINEMENT_ANGLE ) );
	LOG(LEVEL_INFO, "FTRACKER_MIN_CORNER_ANGLE = %f", toDegrees( FTRACKER_MIN_CORNER_ANGLE ) );

	LOG(LEVEL_INFO, "SPHERE_TESSELLATION_RESOLUTION = %f", toDegrees( SPHERE_TESSELLATION_RESOLUTION ) );
	LOG(LEVEL_INFO, "SCORE_MIN_OVERLAP = %f", SCORE_MIN_OVERLAP );
	LOG(LEVEL_INFO, "SCORE_MAX_DIHEDRAL_ANGLE = %f deg.", toDegrees(SCORE_MAX_DIHEDRAL_ANGLE));
	LOG(LEVEL_INFO, "INIT_CORNERS_TOPK = %d", INIT_CORNERS_TOPK );
	LOG(LEVEL_INFO, "LADYBUG_ID = %d", LADYBUG_ID);

	return true;
}

void initMainWindow(MyGlWindow *gl_main, MyControlBarWindow *gl_control/*, Fl_Menu_Bar *menu_bar*/) 
{
	int x = WINDOW_WIDTH-CONTROLBAR_WIDTH;
	int y = MENUBAR_HEIGHT+CONTROLBAR_HEIGHT;
	int w = CONTROLBAR_WIDTH;
	int h = 25;
	int step = 45;
	int yref;

	//************************************************************************
	//** general widgets
	// a button to toggle video ON/OFF
	gl_main->pause_video = new  Fl_Light_Button(x,y,w,h,"Video ON/OFF");
	gl_main->pause_video ->value(1);
	gl_main->pause_video->when(FL_WHEN_RELEASE);
	y += h;
	
	/* 6 */ gl_main->calib_camera = new Fl_Input_Choice(x,y,w,h,"");
	gl_main->calib_camera->add("Camera 0");
	gl_main->calib_camera->add("Camera 1");
	gl_main->calib_camera->add("Camera 2");
	gl_main->calib_camera->add("Camera 3");
	gl_main->calib_camera->add("Camera 4");
	gl_main->calib_camera->add("Camera 5");
	gl_main->calib_camera->value("Camera 0");
	gl_main->calib_camera->callback(select_sensor_cb);
	y += h;
	
	// a button to toggle edge display
	gl_main->view_flow = new  Fl_Light_Button(x,y,w,h,"FAST");
	gl_main->view_flow ->value(0);
	gl_main->view_flow->callback(redraw_cb);
	gl_main->view_flow->when(FL_WHEN_RELEASE);
	y += h;
	
	// a button to run edge display once
	gl_main->detect_edges_button = new  Fl_Button(x,y,w,h,"Detect Edges");
	gl_main->detect_edges_button->callback(detect_edges_cb);
	gl_main->detect_edges_button->when(FL_WHEN_RELEASE);
	y += h;
	
	// a button to clear edges
	gl_main->clear_edges = new  Fl_Button(x,y,w,h,"Clear Edges");
	gl_main->clear_edges->callback(clear_edges_cb);
	gl_main->clear_edges->when(FL_WHEN_RELEASE);
	y += h;
			
	// a button to clear selected edges
	gl_main->clear_selected_edges = new  Fl_Button(x,y,w,h,"Clear selection");
	gl_main->clear_selected_edges->value(0);
	gl_main->clear_selected_edges->callback(clear_selected_edges_cb);
	gl_main->clear_selected_edges->when(FL_WHEN_RELEASE);
	y += h;
	
	// a button to compute the pose
	gl_main->run_analysis = new  Fl_Light_Button(x,y,w,h,"Run analysis");
	gl_main->run_analysis->value(0);
	gl_main->run_analysis->callback(run_analysis_cb);
	gl_main->run_analysis->when(FL_WHEN_RELEASE);
	y += h;
	
	// a counter to select the pose solution
	gl_main->select_pose = new  Fl_Counter(x,y,w,h,"");
	gl_main->select_pose->value(0);
	gl_main->select_pose->bounds(0,1000);
	gl_main->select_pose->step(1.0);
	gl_main->select_pose->type(FL_SIMPLE_COUNTER);
	gl_main->select_pose->callback(redraw_cb);
	y += h;
	
	// a check box to filter solutions
	gl_main->compute_pose = new Fl_Button(x,y,w,h,"Compute pose");
	gl_main->compute_pose->callback(compute_pose_cb);
	y += h;
		
	// a generic counter to select edges, lines etc.
	gl_main->select_generic = new  Fl_Counter(x,y,w,h,"");
	gl_main->select_generic->value(0);
	gl_main->select_generic->bounds(0,0);
	gl_main->select_generic->step(1.0);
	gl_main->select_generic->callback(select_generic_changed_cb);
	y += h;
	
	// a drop-down menu to choose which pool to draw (PRIMARY by default)
	/*gl_main->select_pool = new Fl_Input_Choice(x,y,w,h,"");
	gl_main->select_pool->add( "PRIMARY" );
	gl_main->select_pool->add( "VALIDATION" );
	gl_main->select_pool->add( "BACKUP" );
	gl_main->select_pool->value( "PRIMARY" );
	gl_main->select_pool->callback( redraw_cb );
	y += h;*/
	
	// a button to run RANSAC
	gl_main->lock_recovery = new  Fl_Button(x,y,w,h,"lock_recovery");
	gl_main->lock_recovery->callback(lock_recovery_cb);
	gl_main->lock_recovery->when(FL_WHEN_RELEASE);
	y += h;
	
	// a button to run RANSAC init
	gl_main->init = new  Fl_Light_Button(x,y,w,h,"Init");
	gl_main->init->callback(init_cb);
	gl_main->init->when(FL_WHEN_RELEASE);
	y += h;
		
	// a button for next frame
	gl_main->next_frame = new Fl_Button (x,y,w,h,"Next frame");
	gl_main->next_frame->callback (next_frame_cb);
	y += h;
	
	// a button for next frame
	gl_main->run_maintenance = new Fl_Light_Button (x,y,w,h,"Next frame++");
	gl_main->run_maintenance->callback (run_maintenance_cb);
	y += h;
	
	/* 6 */ gl_main->view_correspondences = new Fl_Light_Button(x,y,w,h,"View Correspondences");
	gl_main->view_correspondences->callback(redraw_cb);
	gl_main->view_correspondences->value(0);
	y += h;

	/* 6 */ gl_main->set_camera_pose = new Fl_Button(x,y,w,h,"Set camera pose");
	gl_main->set_camera_pose->callback(set_camera_pose_cb);
	gl_main->set_camera_pose->value(0);
	y += h;
	
	gl_main->snapshot = new Fl_Light_Button( x, y, w, h, "Screenshot" );
	gl_main->snapshot->callback( redraw_cb );
	y += h;
	
	gl_main->clear_sm = new Fl_Button( x, y, w, h, "Clear SM" );
	gl_main->clear_sm->callback( clear_sm_cb );
	y += h;
	
	gl_main->add_sm = new Fl_Button( x, y, w, h, "Add to SM" );
	gl_main->add_sm->callback( add_sm_cb );
	y += h;
	
	// a multi-purpose progress bar
	gl_main->progress_bar = new Fl_Progress(x,WINDOW_HEIGHT-20,w,20,"");
	gl_main->progress_bar->minimum(0);
	gl_main->progress_bar->maximum(1000);
	gl_main->progress_bar->value(0);
	gl_main->progress_bar->selection_color(FL_RED);
	
	// a tickle for frame ID in database mode
	gl_main->frame_slider = new Fl_Scrollbar(x,WINDOW_HEIGHT,w,20,"");
	gl_main->frame_slider->type(FL_HORIZONTAL);
	gl_main->frame_slider->minimum(0);
	gl_main->frame_slider->maximum(10);
	gl_main->frame_slider->linesize(1);
	gl_main->frame_slider->value(0,3,1,10);
	gl_main->frame_slider->selection_color(FL_YELLOW);
	gl_main->frame_slider->hide();
	gl_main->frame_slider->callback(frame_slider_changed_cb);
	
	yref = y;
	
	//************************************************************************
	// ** widgets for the "3D Map" window
	
	y = yref;
	
	/* 6 */ gl_main->model_show_grid = new Fl_Light_Button(x,y,w,h,"Show Grid");
	gl_main->model_show_grid->callback(redraw_cb);
	gl_main->model_show_grid->value(0);
	y += h;
	
	/* 6 */ gl_main->model_show_pixels = new Fl_Light_Button(x,y,w,h,"Show Pixels");
	gl_main->model_show_pixels->callback(redraw_cb);
	gl_main->model_show_pixels->value(0);
	y += h;
	
	//************************************************************************
	// ** widgets for the "Video" window
	// three sliders to play with edge detection parameters
	y = yref;
	/* 3 */ gl_main->edgePixels = new Fl_Value_Slider(x,y,w,h, "Min Edge size (pixels)");
	gl_main->edgePixels->type(FL_HORIZONTAL);
	gl_main->edgePixels->minimum(1);
	gl_main->edgePixels->maximum(100);
	gl_main->edgePixels->value(EDGE_SIZE_INIT_REAL); 
	y += step;
	
	/* 4 */ gl_main->edgeSigma = new Fl_Value_Slider(x,y,w,h, "Edge Sigma");
	gl_main->edgeSigma->type(FL_HORIZONTAL);
	gl_main->edgeSigma->minimum(0.0);
	gl_main->edgeSigma->maximum(4.0);
	gl_main->edgeSigma->value(EDGE_SIGMA_INIT_REAL); 
	y += step;
	
	/* 5 */ gl_main->edgeThresh = new Fl_Value_Slider(x,y,w,h,"Edge Threshold");
	gl_main->edgeThresh->type(FL_HORIZONTAL);
	gl_main->edgeThresh->minimum(0.0);
	gl_main->edgeThresh->maximum(100.0);
	gl_main->edgeThresh->value(EDGE_THRESHOLD_INIT_REAL); 
	y += step;
	
	/* 6 */ gl_main->view_mask = new Fl_Light_Button(x,y,w,h,"View Mask");
	gl_main->view_mask->callback(redraw_cb);
	gl_main->view_mask->value(0);
	y += h;
	
	/* 6 */ gl_main->view_reprojected_edges = new Fl_Light_Button(x,y,w,h,"View Reprojected Edges");
	gl_main->view_reprojected_edges->callback(reproject_edges_cb);
	gl_main->view_reprojected_edges->value(0);
	y += h;
	
	/* 7 */ gl_main->edge_clearmask = new Fl_Button(x,y,w,h,"Clear Masks");
	gl_main->edge_clearmask->callback(edge_clearmasks_cb);
	y += step;
	
	gl_control->_main_w_ptr = gl_main;
	gl_control->_perf_time = 0.0;
	
	gl_main->_windowMode = MODE_VIDEO;
	gl_main->_calibrated_camera = 0;
	gl_main->_rectified_done = false;
	
	gl_main->_menu_bar->menu(mainMenu);
    
	init(gl_main);
	
	//************************************************************************
	// ** widgets for the "Sphere" window
	y = yref;

	// a toggle for vanishing points
	gl_main->vanishing_points = new Fl_Check_Button(x,y,w,h,"Vanishing points");
	gl_main->vanishing_points->value(0);
	y += h;

	// a toggle for vanishing points
	gl_main->rotation_scores = new Fl_Check_Button(x,y,w,h,"Rotation scores");
	gl_main->rotation_scores->value(0);
	y += h;

	// a button to toggle axis drawing
	gl_main->draw_axis = new Fl_Light_Button(x,y,w,h,"Draw axis");
	gl_main->draw_axis->value(0);
	gl_main->draw_axis->callback(redraw_cb);
	y += h;
	
	// a button to toggle axis drawing
	gl_main->sphere_draw_frustum = new Fl_Light_Button(x,y,w,h,"Draw Frustums");
	gl_main->sphere_draw_frustum->value(0);
	gl_main->sphere_draw_frustum->callback(redraw_cb);
	y += h;
	
	// a button to switch view
	gl_main->switch_view = new  Fl_Button(x,y,w,h,"Switch View");
	gl_main->switch_view->value(0);
	gl_main->switch_view->callback(switch_view_cb);
	gl_main->switch_view->when(FL_WHEN_RELEASE);
	y += h;
	
	// a drop-down menu to choose which correspondences are shown (maintenance or recovery)
	gl_main->sphere_correspondence_mode = new Fl_Input_Choice(x,y,w,h,"");
	gl_main->sphere_correspondence_mode->add( "inliers" );
	gl_main->sphere_correspondence_mode->add( "maintenance" );
	gl_main->sphere_correspondence_mode->add( "init" );
	gl_main->sphere_correspondence_mode->add( "tracker" );
	gl_main->sphere_correspondence_mode->value( "maintenance" );
	gl_main->sphere_correspondence_mode->callback(select_correspondence_mode_changed_cb);
	y += h;
	
	// three sliders to adjust the head unit calibration
	gl_main->sphere_calib_z = new Fl_Roller(x+step,y+step,w*.6,h*.6,"");
	gl_main->sphere_calib_z->type(FL_HORIZONTAL);
	gl_main->sphere_calib_z->minimum(-10.0);
	gl_main->sphere_calib_z->maximum(10.0);
	gl_main->sphere_calib_z->precision(1);
	gl_main->sphere_calib_z->value(0);
	gl_main->sphere_calib_z->callback(sphere_calib_apply_cb);
	gl_main->sphere_calib_z->when( FL_WHEN_CHANGED );
	y += h;
	gl_main->sphere_calib_yaw = new Fl_Roller(x+step,y+step,w*.6,h*.6,"");
	gl_main->sphere_calib_yaw->type(FL_HORIZONTAL);
	gl_main->sphere_calib_yaw->minimum(-M_PI);
	gl_main->sphere_calib_yaw->maximum(M_PI);
	gl_main->sphere_calib_yaw->precision(3);
	gl_main->sphere_calib_yaw->value(0);
	gl_main->sphere_calib_yaw->callback(sphere_calib_apply_cb);
	gl_main->sphere_calib_yaw->when( FL_WHEN_CHANGED );
	y += h;
	gl_main->sphere_calib_pitch = new Fl_Roller(x+step,y+step,w*.6,h*.6,"");
	gl_main->sphere_calib_pitch->type(FL_HORIZONTAL);
	gl_main->sphere_calib_pitch->minimum(-M_PI/2);
	gl_main->sphere_calib_pitch->maximum(M_PI/2);
	gl_main->sphere_calib_pitch->precision(3);
	gl_main->sphere_calib_pitch->value(0);
	gl_main->sphere_calib_pitch->callback(sphere_calib_apply_cb);
	gl_main->sphere_calib_pitch->when( FL_WHEN_CHANGED );
	y += h;
	gl_main->sphere_calib_roll = new Fl_Roller(x+step,y+step,w*.6,h*.6,"");
	gl_main->sphere_calib_roll->type(FL_HORIZONTAL);
	gl_main->sphere_calib_roll->minimum(-M_PI/2);
	gl_main->sphere_calib_roll->maximum(M_PI/2);
	gl_main->sphere_calib_roll->precision(3);
	gl_main->sphere_calib_roll->value(0);
	gl_main->sphere_calib_roll->callback(sphere_calib_apply_cb);
	gl_main->sphere_calib_roll->when( FL_WHEN_CHANGED );
	y += h;
	
	//************************************************************************
	// ** widgets for the "Camera Calibration" window
	// a drop-down menu to choose the camera to calibrate
	y = yref;
	
	// a button to toggle image rectification
	gl_main->calib_img_rectification = new  Fl_Button(x,y,w,h,"Rectify Image");
	gl_main->calib_img_rectification ->callback(rectify_img_cb);
	y += h;
	
	// a button to toggle image rectification
	gl_main->calib_img_mask = new  Fl_Light_Button(x,y,w,h,"Mask Image");
	gl_main->calib_img_mask->callback(redraw_cb);
	y += h;
	
	// a button to save an image
	gl_main->grab_img = new  Fl_Button(x,y,w,h,"Save image");
	gl_main->grab_img->callback(grab_img_cb);
	y += h;
	
	// a button to load an image
	gl_main->calib_load_img = new  Fl_Light_Button(x,y,w,h,"Load image");
	gl_main->calib_load_img->value(0);
	gl_main->calib_load_img->callback(calib_load_img_cb);
	y += h;
	
	/* 7 */ 
	// two sliders to play with the focal length
	gl_main->calib_focal_x = new Fl_Value_Slider(x,y,w,h,"Focal X (pixels)");
	gl_main->calib_focal_x->type(FL_HORIZONTAL);
	gl_main->calib_focal_x->minimum(gl_main->_camera->getFocalX()/2);
	gl_main->calib_focal_x->maximum(gl_main->_camera->getFocalX()*2);
	gl_main->calib_focal_x->value(gl_main->_camera->getFocalX());
	y += step;
	
	/* 8 */ gl_main->calib_focal_y = new Fl_Value_Slider(x,y,w,h,"Focal Y (pixels)");
	gl_main->calib_focal_y->type(FL_HORIZONTAL);
	gl_main->calib_focal_y->minimum(gl_main->_camera->getFocalY()/2);
	gl_main->calib_focal_y->maximum(gl_main->_camera->getFocalY()*2);
	gl_main->calib_focal_y->value(gl_main->_camera->getFocalY());
	y += step;
	
	// two rollers to play with the center point
	gl_main->calib_center_x = new Fl_Roller(x,y,h*.6,w*.6,"Center X");
	gl_main->calib_center_x->type(FL_VERTICAL);
	gl_main->calib_center_x->minimum(0);
	gl_main->calib_center_x->minimum(gl_main->_camera->_frame._rotated_img[0]->width);
	gl_main->calib_center_x->precision(1); //Sets the step value to 1/10^digits.
	gl_main->calib_center_x->value(gl_main->_camera->getCenterX());
	
	gl_main->calib_center_y = new Fl_Roller(x+step,y+step,w*.6,h*.6,"Center Y");
	gl_main->calib_center_y->type(FL_HORIZONTAL);
	gl_main->calib_center_y->minimum(0);
	gl_main->calib_center_y->minimum(gl_main->_camera->_frame._rotated_img[0]->height);
	gl_main->calib_center_y->precision(1); //Sets the step value to 1/10^digits.
	gl_main->calib_center_y->value(gl_main->_camera->getCenterY());
	y += w*.7;
	
	// five sliders for the five radial distorsion parameters
	for (int i=0;i<5;i++) {
		gl_main->calib_distortion.push_back(new Fl_Value_Slider(x,y,w,h,""));
		gl_main->calib_distortion[i]->type(FL_HORIZONTAL);
		gl_main->calib_distortion[i]->minimum(gl_main->_camera->getKC(i)/10);
		gl_main->calib_distortion[i]->maximum(gl_main->_camera->getKC(i)*10);
		gl_main->calib_distortion[i]->value(gl_main->_camera->getKC(i));
		y += 20;
	}

}

// set the edge detector parameters depending on synthetic/real dataset and init/maintenance phase
//
void MyGlWindow::setEdgeDetectorParameters( bool init, bool synthetic ) 
{
	if ( init ) {
		if ( synthetic ) {
			edgePixels->value(EDGE_SIZE_INIT_SYNT);
			edgeSigma->value( EDGE_SIGMA_INIT_SYNT);
			edgeThresh->value(EDGE_THRESHOLD_INIT_SYNT); 
		} else {
			edgePixels->value(EDGE_SIZE_INIT_REAL);
			edgeSigma->value( EDGE_SIGMA_INIT_REAL);
			edgeThresh->value(EDGE_THRESHOLD_INIT_REAL); 
		}
	} else {
		if ( synthetic ) {
			edgePixels->value(EDGE_SIZE_MAIN_SYNT);
			edgeSigma->value( EDGE_SIGMA_MAIN_SYNT);
			edgeThresh->value(EDGE_THRESHOLD_MAIN_SYNT); 
		} else {
			edgePixels->value(EDGE_SIZE_MAIN_REAL);
			edgeSigma->value( EDGE_SIGMA_MAIN_REAL);
			edgeThresh->value(EDGE_THRESHOLD_MAIN_REAL); 
		}
	}
}
