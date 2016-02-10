#include "main.h"
#include "ImageEdge.h"
#include "recovery.h"

void MyGlWindow::draw3DMap()
{
	int i,j,k;

	if ( BACKGROUND_WHITE )
		_LUT._viewer_3d.clearWindow(false);
	else
		_LUT._viewer_3d.clearWindow(true);

	_LUT._viewer_3d.setup3D();

	if (_LUT._draw_pixels) {
		_LUT.drawScene_3d(_wireframe,_depth_test,!view_correspondences->value(),_draw_texture, _line_color_coding, _calibrated_camera);
		return;
	}
	
	PerfTimer timer;
	_LUT._viewer_3d.setMode(GL_RENDER);
	_LUT._viewer_3d.setup3D();
	_LUT._viewer_3d.glLighting( _LUT._model->_centroid + Vec3d(0,0,_LUT._model->_diameter));
	//timer.print("setup");

	_LUT.drawScene_3d(_wireframe,_depth_test,!view_correspondences->value(),_draw_texture, _line_color_coding, _calibrated_camera);
	//timer.print("draw scene");

	Correspondence c;



	// draw correspondence corner in the 3d map
	/*if (view_correspondences->value()) {

		// draw corners
		int counter = 0;
		for (int i=0;i<_corner_lines.size();i++) {
			corner_line cl = _corner_lines[i];

			if ( cl.corner_id != select_generic->value() )
				continue;

			if (counter == select_pose->value()) {
				_LUT._viewer_3d.drawEdge(  _LUT._model->getEdge( cl.line_1_id ), RED, 10 );
				_LUT._viewer_3d.drawEdge(  _LUT._model->getEdge( cl.line_2_id ), RED, 10 );
			}

			counter++;
		}
		//timer.print("draw corners");

		// draw orange line for correspondence
		if ( _LUT._selected_lines.size() == 1 ) {
			Edge *line = _LUT._selected_lines[0];
			_LUT._viewer_3d.drawEdge(  line, ORANGE, 15 );
		}
	}*/
	
	// draw the camera
	_LUT._viewer_3d.drawFrustum( _camera->getPose(), 4.0, YELLOW );
	//Vec3d position = _camera->came.getTranslation();
	//_LUT._viewer_3d.drawSphere(position,3.0,YELLOW,10,10);
	//timer.print("draw camera");

	// draw the camera history
	drawPoseHistory( _LUT._viewer_3d, 4.0, BLUE );
	//timer.print("draw pose history");

	// draw the LUT history
	_LUT.drawHistory( _LUT._viewer_3d, RED, 4.0 );
	//timer.print("draw LUT history");
	
	// draw the computed poses
	//int i;
	//for (int i=0;i<_save_good_poses.size();i++)
	//	_LUT._viewer_3d.drawSphere(_save_good_poses[i].getPose().getTranslation(),3.0,GREEN,10,10);
	//for ( i=0;i<_poses.size();i++)
	//	_LUT._viewer_3d.drawSphere(_poses[i].getPose().getTranslation(),3.0,RED,10,10);
	
	// display some text info
	/*if (!_LUT._selected_lines.empty()) {
		_LUT._viewer_3d.setup2D();
		_LUT._viewer_3d.displayMsg(10,WINDOW_HEIGHT-100,RED,0,"# selected: %d",_LUT._selected_lines.size());
		for (i=0;i<_LUT._selected_lines.size();i++)
			_LUT._viewer_3d.displayMsg(200+30*i,WINDOW_HEIGHT-100,RED,0,"%d (%f)",_LUT._selected_lines[i]->_id, _LUT._selected_lines[i]->_weight);
		_LUT._viewer_3d.displayMsg(10,WINDOW_HEIGHT-120,GREEN,0,"Position: %.2f %.2f %.2f",\
			_camera->came.getTranslation()[0],_camera->came.getTranslation()[1],_camera->came.getTranslation()[2]);
	}*/

	/*if ( snapshot->value()) {
		SNAPSHOT_START_X = 250;
		SNAPSHOT_START_Y = 300;
		_LUT._viewer_3d.screenshot( _timestamp, SNAPSHOT_START_X, SNAPSHOT_START_Y, SNAPSHOT_WIDTH, SNAPSHOT_HEIGHT );
		_timestamp++;
	}*/
	
	// draw tracked edges
	if ( strcmp( sphere_correspondence_mode->value(), "tracker" ) == 0 ) {
		
		// draw each edgeplane 
		for (i=0;i<_tracked_edges.size();i++) {
			
			if ( i != select_generic->value() )
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
						_LUT._viewer_3d.drawEdgePlaneDetails( ep, Vec3d(0,0,0), Quaternion(), 50.0, 0, 3 ); // draw the edgeplane in bold
						edgeplanes.push_back( ep );
					}
				}
			}
			
			// if less than three hits, skip it
			if ( _tracked_edges[i].first < 3 )
				continue;

			Edge *line = new Edge();
			if ( !intersectEdgeplanesToLine( edgeplanes, line ) ) {
				delete line;
				_LUT._viewer_3d.setup2D();
				_LUT._viewer_3d.displayMsg( 100, 80, RED, 0, "Line (failed): %d hits",  _tracked_edges[i].first );
				_LUT._viewer_3d.setup3D();
			} else {
				_LUT._viewer_3d.setup2D();
				_LUT._viewer_3d.displayMsg( 100, 80, RED, 0, "Line (success): %d hits",  _tracked_edges[i].first );
				_LUT._viewer_3d.setup3D();
				line->draw( RED, 3 );
			}
		}
	}

	// draw the selected region
	if ( _LUT._draw_grid ) {
		if ( select_pose->value() < _LUT.regions.size() ) {
			_LUT.regions[select_pose->value()].draw( _LUT._viewer_3d, YELLOW, .15 );
		}
	}

	// draw X,Y,Z axis and compass

	if ( draw_axis->value() ) {
		
		_LUT._viewer_3d.drawLine( _LUT._model->_centroid, _LUT._model->_centroid + Vec3d(100,0,0), GREEN, 5 ); // x axis
		_LUT._viewer_3d.drawCone( _LUT._model->_centroid + Vec3d(100,0,0), Vec3d(15,0,0), 10, GREEN, 10, 10 );
		_LUT._viewer_3d.drawLine( _LUT._model->_centroid, _LUT._model->_centroid + Vec3d(0,100,0), BLUE, 5 ); // y axis
		_LUT._viewer_3d.drawCone( _LUT._model->_centroid + Vec3d(0,100,0), Vec3d(0,15,0), 10, BLUE, 10, 10 );
		_LUT._viewer_3d.drawLine( _LUT._model->_centroid, _LUT._model->_centroid + Vec3d(0,0,100), RED, 5 ); // z axis
		_LUT._viewer_3d.drawCone( _LUT._model->_centroid + Vec3d(0,0,100), Vec3d(0,0,15), 10, RED, 10, 10 );

		// draw the compass
		Vec3d compass_center = _LUT._model->_centroid + Vec3d(_LUT._model->_diameter/2, _LUT._model->_diameter/2, 0 );
		_LUT._viewer_3d.drawCircle( compass_center, Vec3d(0,0,1), 100, WHITE, 5, 100 ); 
		Vec3d north_dir = Quaternion(Vec3d(0,0,1), _LUT._model->NORTH_ANGLE).rotate(Vec3d(0,1,0));

		_LUT._viewer_3d.drawLine( compass_center, compass_center + 100*north_dir, RED, 5 ); // z axis
		_LUT._viewer_3d.drawCone( compass_center + 50 * north_dir, 50 * north_dir, 10, RED, 10, 10 );
		

	}

	// draw the anchors
	if ( _show_anchor )
		drawAnchors(0,0,1.0,true,true,0,0,0);

	// display histograms
	//Histogram cameraHistogram = Histogram( 35 );
//
//	_camera->computeAngleHistogram( cameraHistogram );

	_LUT._viewer_3d.setup2D();

	// display the model name and other info
	if ( _LUT._model != NULL ) {
		_LUT._viewer_3d.displayMsg( 10, 10, RED, 0, _LUT._model->_dirname.c_str() );

		int x = 10;
		for (int i=0;i<_LUT._selected_lines.size();i++) {
			_LUT._viewer_3d.displayMsg( x, 30, RED, 0, "%d", _LUT._selected_lines[i]->_id );
			x += 50;
			for (int j=0;j<_LUT._selected_lines[i]->_faces.size();j++) {
				_LUT._viewer_3d.displayMsg( x, 30, YELLOW, 0, "%d", _LUT._selected_lines[i]->_faces[j]->_id );
				x += 50;
			}
			_LUT._viewer_3d.displayMsg( x, 30, GREEN, 0, "%d", _LUT._selected_lines[i]->_a->_id );
			x += 50;
			_LUT._viewer_3d.displayMsg( x, 30, GREEN, 0, "%d", _LUT._selected_lines[i]->_b->_id );
			x += 50;
		}

		x = 10;
		if ( _LUT._node != NULL )
			_LUT._viewer_3d.displayMsg( x, 50, RED, 0, "Node: %d", _LUT._node->_id );

		if (view_correspondences->value()) {
			int counter = 0;
			for (int i=0;i<_corner_lines.size();i++) {
				corner_line cl = _corner_lines[i];
				
			if ( cl.corner_id != select_generic->value() )
				continue;

				if (counter == select_pose->value()) {
					_LUT._viewer_3d.displayMsg( x, 30, RED, 0, "Corner %d - %d: [lines %d %d] [edges %d %d] score: %f", cl.corner_id, cl.vertex_id, cl.line_1_id, cl.line_2_id, cl.edge_1_id, cl.edge_2_id, cl.score );
				}

				counter++;
			}

			// draw orange line ID for correspondence
		if ( SM_get_correspondence( select_generic->value(), c ) )
			_LUT._viewer_3d.displayMsg( x, 80, ORANGE, 0, "Line: %d",  c.first->_id  );

		}
	}
}

void MyGlWindow::hideAll()
{
	view_mask->hide();
	edge_clearmask->hide();
	edgePixels->hide();
	edgeSigma->hide();
	edgeThresh->hide();
	calib_focal_x->hide();
	calib_focal_y->hide();
	calib_img_rectification->hide();
	calib_center_x->hide();
	calib_center_y->hide();
	for (int i=0;i<5;i++)
		calib_distortion[i]->hide();
	calib_load_img->hide();
	draw_axis->hide();
	grab_img->hide();
	sphere_draw_frustum->hide();
	sphere_calib_yaw->hide();
	sphere_calib_z->hide();
	sphere_calib_pitch->hide();
	sphere_calib_roll->hide();
	switch_view->hide();
	model_show_grid->hide();
	select_pose->hide();
	select_generic->hide();
	vanishing_points->hide();
	rotation_scores->hide();
	calib_img_mask->hide();
	next_frame->hide();
	run_maintenance->hide();
	view_reprojected_edges->hide();
	model_show_pixels->hide();
	view_correspondences->hide();
	sphere_correspondence_mode->hide();
	snapshot->hide();
	clear_sm->hide();
	add_sm->hide();
	//select_pool->hide();
}

void MyGlWindow::showAndHide()
{
	// show and hide items depending on window mode
	hideAll();
	
	switch (_windowMode) {
	case MODE_VIDEO:
		edgePixels->show();
		edgeSigma->show();
		edgeThresh->show();
		select_generic->maximum( FTRACKER_MAX_FEATURES );
		select_generic->bounds(0,100000);
		if (_camera->_frame._selected_edges.size() == 1) {
			select_generic->bounds(0,MAX(0,_camera->_frame.n_edgeplanes_chained-1));
			select_generic->value(_camera->_frame._selected_edges[0]._uid);
			select_generic->lstep(10.0);
			select_generic->show();
		}
		next_frame->show();
		run_maintenance->show();
		view_mask->show();
		view_reprojected_edges->show();
		view_correspondences->show();
		lock_recovery->show();
		init->show();
		clear_sm->show();
		add_sm->show();
		sphere_calib_yaw->show();
		sphere_calib_pitch->show();
		sphere_calib_roll->show();
		sphere_calib_z->show();
		select_pose->show();
		select_generic->lstep(10.0);
		select_generic->show();
		sphere_correspondence_mode->show();
		break;
	case MODE_CALIBRATION:
		//calib_focal_x->show();
		//calib_focal_y->show();
		//calib_center_x->show();
		//calib_center_y->show();
		calib_img_rectification->show();
		next_frame->show();
		run_maintenance->show();
		//for (i=0;i<5;i++)
		//	calib_distortion[i]->show();
		calib_img_mask->show();
		clear_sm->show();
		add_sm->show();
		view_correspondences->show();
	//	if ( strcmp( sphere_correspondence_mode->value(), "inliers" ) == 0 ) {
	//		if ( !_inliers.empty() )
	//			select_generic->bounds(0, MAX(0,_inliers.size()-1));
	//	} else 
	//		select_generic->bounds(0,MAX(0,SM_Size()-1));

		select_generic->lstep(10.0);
		select_generic->show();
		sphere_correspondence_mode->show();
		break;
	case MODE_SPHERE:
	case MODE_ROTATIONS:
		draw_axis->show();
		sphere_draw_frustum->show();
		sphere_calib_yaw->show();
		sphere_calib_pitch->show();
		sphere_calib_roll->show();
		switch_view->show();
		select_pose->show();
		view_correspondences->show();
		sphere_correspondence_mode->show();
//		if ( strcmp( sphere_correspondence_mode->value(), "inliers" ) == 0 ) {
//			if ( !_inliers.empty() )
//				select_generic->bounds(0, MAX(0,_inliers.size()-1));
//		} else 
//			select_generic->bounds(0,MAX(0,SM_Size()-1));

		select_generic->lstep(10.0);
		select_generic->show();
		vanishing_points->show();
		rotation_scores->show();
		next_frame->show();
		run_maintenance->show();
		lock_recovery->show();
		init->show();
		clear_sm->show();
		add_sm->show();
		sphere_correspondence_mode->show();
		break;
	case MODE_3D_MAP:
		draw_axis->show();
		model_show_pixels->show();
		model_show_grid->show();
		select_pose->show();
//		if (_LUT._selected_lines.size() == 1) {
//			select_generic->bounds(0,_LUT._model->_maxEdgeId);
//			select_generic->value(_LUT._selected_lines[0]->_id);
//			select_generic->lstep(100.0);
//			select_generic->show();
//		}
		next_frame->show();
		run_maintenance->show();
		view_correspondences->show();
		lock_recovery->show();
		init->show();
		clear_sm->show();
		add_sm->show();
		select_generic->show();
		sphere_correspondence_mode->show();
		break;
	default:
		return;
	}

	if ( select_generic->value() > select_generic->maximum() )
		select_generic->value(0);

	snapshot->show();
}

void MyGlWindow::drawSnapshotScreen()
{
	// setup 2D
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0,WINDOW_WIDTH,0,WINDOW_HEIGHT+MENUBAR_HEIGHT);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);

	// draw RED box
	glColor( RED );
	glLineWidth( 3 );

	/*glBegin( GL_LINES );
	glVertex2f( SNAPSHOT_START_X - 3, SNAPSHOT_START_Y - 3 );
	glVertex2f( SNAPSHOT_START_X + 3 + SNAPSHOT_WIDTH, SNAPSHOT_START_Y  - 3 );

	glVertex2f( SNAPSHOT_START_X + 3 + SNAPSHOT_WIDTH, SNAPSHOT_START_Y - 3 );
	glVertex2f( SNAPSHOT_START_X + 3 + SNAPSHOT_WIDTH, SNAPSHOT_START_Y + 3 + SNAPSHOT_HEIGHT);
	
	glVertex2f( SNAPSHOT_START_X + 3 + SNAPSHOT_WIDTH, SNAPSHOT_START_Y + 3 + SNAPSHOT_HEIGHT);
	glVertex2f( SNAPSHOT_START_X - 3, SNAPSHOT_START_Y + 3 + SNAPSHOT_HEIGHT);
	
	glVertex2f( SNAPSHOT_START_X - 3, SNAPSHOT_START_Y + 3 + SNAPSHOT_HEIGHT);
	glVertex2f( SNAPSHOT_START_X - 3, SNAPSHOT_START_Y - 3 );
	glEnd();
	*/
}

// drawing function for the rotations mode
// draw the rotations in the unit sphere
//
void MyGlWindow::drawRotations()
{
	if ( BACKGROUND_WHITE )
		_rotations_viewer.clearWindow(false);
	else
		_rotations_viewer.clearWindow(true);

	// lighting 
	_rotations_viewer.setup3D();
	_rotations_viewer.glLighting( Vec3d(0,0,100) );

	glEnable(GL_DEPTH_TEST);

	// draw the icosahedron
	spheretsn->drawFaces( false );

	return;

	// draw the rotations
	glColor(BLUE);
	glPointSize(1);
	glBegin(GL_POINTS);
	
	for (int i=0;i<_rotations.size();i++) {
		glVertex3f(_rotations[i][0], _rotations[i][1], _rotations[i][2] );
	}
	
	glEnd();


	// draw the selected rotations
	glColor(RED);
	glPointSize(3);
	glBegin(GL_POINTS);
	
	for (i=0;i<_best_rotations.size();i++) {
		
		//_rotations_viewer.drawSphere( _best_rotations[i], INIT_SEARCH_RADIUS_ROTATION, RED, 10, 10 );
		glVertex3f(_best_rotations[i][0], _best_rotations[i][1], _best_rotations[i][2] );
	}
	glEnd();
	
	// draw the ideal rotation
	glColor(GREEN);
	glPointSize(3);
	glBegin(GL_POINTS);
	glVertex(_ideal_rotation);
	glEnd();

	// draw the axis
	_rotations_viewer.drawAxis(Vec3d(-2*M_PI,-2*M_PI,0),M_PI);


}

void MyGlWindow::drawCalibration()
{
	int i;

	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	
	_calibration_viewer.set(WINDOW_WIDTH-CONTROLBAR_WIDTH,WINDOW_HEIGHT,0);
	_calibration_viewer.setup2D();
	if ( BACKGROUND_WHITE )
		_calibration_viewer.clearWindow(false);
	else
		_calibration_viewer.clearWindow(true);

	int x = CALIBRATION_DRAW_X;
	int y = CALIBRATION_DRAW_Y;
	int width = _camera->_frame._rotated_img[_calibrated_camera]->width;
	int height = _camera->_frame._rotated_img[_calibrated_camera]->height;
	int x2 = x + 1.2 * width; // do not change these values otherwise you will break the anchor code
	int y2 = y;

	double zoom = 1.0;

	if (!_pauseVideo)
		_calibration_viewer.displayMsg(x,WINDOW_HEIGHT-100,"Press button to see rectified image",RED,0);
	
	// draw the user mask
	_camera->drawUserMask( _calibration_viewer, _calibrated_camera, CALIBRATION_DRAW_X, CALIBRATION_DRAW_Y, 1.0, false );

	//_calibration_viewer.displayMsg(x,y-20,"Raw image",WHITE,0);
	if (calib_load_img->value())
		_calibration_viewer.drawImage(_calib_img,x,y,true,true);
	else
		_calibration_viewer.drawImage(_camera->_frame._rotated_img[_calibrated_camera],x2,y2,true,true);
	
	_calibration_viewer.drawImage(_camera->_frame._copy_img[_calibrated_camera],x,y,true,true);

	_calibration_viewer.drawBox( x, y, x + width, y + height, 2, GREY );
	_calibration_viewer.drawBox( x2, y2, x2 + width, y2 + height, 2, GREY );

	if ( BACKGROUND_WHITE )
		_calibration_viewer.displayMsg( x, y - 20, BLACK, 0, "Camera %d", _calibrated_camera );
	else
		_calibration_viewer.displayMsg( x, y - 20, _colors[_calibrated_camera], 0, "Camera %d", _calibrated_camera );

	// draw the edges
	//_calibration_viewer.drawEdges(_camera->_frame._edges[_calibrated_camera],x,y,1.0,RED,true,false,\
	//	_camera->_frame._rotated_img[_calibrated_camera]->height,0.0,_calibrated_camera);
	
	// draw the anchors on calibration window
	if ( _show_anchor )
		drawAnchors (x2, y2, zoom, true, false, _calibrated_camera, height, width);
	
	// draw the correspondence
	if ( view_correspondences->value() ) {
		if ( strcmp( sphere_correspondence_mode->value(), "maintenance" ) == 0 ) { // in maintenance display mode, just display the correspondence
			if ( select_generic->value() < SM_Size() ) {
				
				corres c = SM_Get( select_generic->value() );

				_calibration_viewer.drawCorrespondenceInfo( c , _camera->getPose(), x2 +width + 50, _calibration_viewer._height - 100, RED );

				//retrieve history info and draw it on the left image
				corres c_history;
				if ( SM_GetHistory(c.line->_id, c_history) ) {
					if ( c_history.valid() && c_history.eid != -1) {
						_camera->drawEdgeplane( _calibration_viewer, c_history.eps[c_history.eid], DISTORTION_EDGE_STEP, 
							x,y,zoom,RED,true,false,height,width,_calibrated_camera, 2);
					}
				}

				// draw the expected line on the right image
				EdgePlane edgeline = EdgePlane( c.line->getA() - _camera->getTranslation(), c.line->getB() - _camera->getTranslation(), _camera->getTranslation(), _calibrated_camera, 0, 0 );
				edgeline.fromWorldFrameToCameraFrame( _camera->getPose() );
				//_camera->drawEdgeplane( _calibration_viewer, edgeline, DISTORTION_EDGE_STEP, 
				//			x2,y2,zoom,  GREEN,  true,false,height,width,_calibrated_camera, 2);
				//_camera->getPose().print();

				// perform a search on new image and display edges on the right image
				_camera->initializeEdgeDetectors();
				
				// draw all possible matches
				edgePlaneVector edges_display;
				bool test = PROJ_TEST_DISPLAY( c, true, true, CLUSTER_MAX_ANGLE_THRESHOLD, edges_display );

				for (i=0;i<edges_display.size();i++) {
					EdgePlane ep = edges_display[i];
					_camera->drawEdgeplane( _calibration_viewer, ep, DISTORTION_EDGE_STEP, 
						x2,y2,zoom,  YELLOW,  true,false,height,width,_calibrated_camera, 2);
				}

				// draw the match on next image
				double hl,sl,vl,hr,sr,vr;
				double rl,gl,bl,rr,gr,br;
				for (i=0;i<c.eps.size();i++) {
					EdgePlane ep = c.eps[i];
					_camera->drawEdgeplane( _calibration_viewer, ep, DISTORTION_EDGE_STEP, 
						x2,y2,zoom,  RED,  true,false,height,width,_calibrated_camera, 2);
					hl = ep.l_avg[0];
					sl = ep.l_avg[1];
					vl = ep.l_avg[2];
					hr = ep.r_avg[0];
					sr = ep.r_avg[1];
					vr = ep.r_avg[2];

				}

				HSVtoRGB( hl, sl, vl, &rl, &gl, &bl);
				HSVtoRGB( hr, sr, vr, &rr, &gr, &br);

				_camera->drawMasks( _calibrated_camera, x2, y2, zoom, false );
				_calibration_viewer.displayMsg( 100,_calibration_viewer._height - 130,RED, 1, "Projection test: %d (%d candidates)", test, edges_display.size() ); 

				_calibration_viewer.displayMsg( 100,_calibration_viewer._height - 150,RED, 1, "RGB left [%d %d %d]  right [%d %d %d]", \
				int(255.0*rl), int(255.0*gl), int(255.0*bl), int(255.0*rr), int(255.0*gr), int(255.0*br)); 
			}
			
		} else { //  in inlier display mode, display the inlier
			if (  !_inliers.empty() ) {
				if (select_generic->value() < _inliers.size()) {
					Correspondence d = _inliers[select_generic->value()];
					_calibrated_camera = d.second._cameraId;
					_LUT._selected_lines.clear(); // select the line for debugging
					_LUT._selected_lines.push_back( d.first );
					_camera->drawEdgeplane( _calibration_viewer, d.second, DISTORTION_EDGE_STEP, x, y, zoom, _colors[_calibrated_camera], true, false, height, width, _calibrated_camera, 2);
					_calibration_viewer.displayMsg( 100, 100, RED, 1, "Line %d", d.first->_id, 2 );
					EdgePlane ep = d.second;
					ep.fromCameraFrameToWorldFrame( _camera->getPose() );
					_calibration_viewer.displayMsg( 100, 80, RED, 1, "Angle : %f deg.", toDegrees(ep.angle( d.first)) );
					EdgePlane lep = EdgePlane( d.first->getA() - ep._s, d.first->getB() - ep._s, ep._s, ep._cameraId, 0, 0); // line edgeplane
					lep.fromWorldFrameToCameraFrame( _camera->getPose() );
					_camera->drawEdgeplane( _calibration_viewer, lep, DISTORTION_EDGE_STEP, x, y, zoom, GREEN, true, false, height, width, _calibrated_camera, 2 );
					
				}
			}
		}
	} else if ( !_LUT._selected_lines.empty() ) { // for debug, create a dummy correspondence for the selected line and run PROJ_TEST on it
		_camera->initializeEdgeDetectors();
		corres c = corres( _LUT._selected_lines[0], EdgePlane(), _EXPECTED );
		bool test = PROJ_TEST( c, true, true, CLUSTER_MAX_ANGLE_THRESHOLD );
		for (i=0;i<c.eps.size();i++) {
			EdgePlane ep = c.eps[i];
			
			_camera->drawEdgeplane( _calibration_viewer, ep, DISTORTION_EDGE_STEP, 
				x,y,zoom,YELLOW,true,false,height,width,_calibrated_camera, 2);
		}
		_camera->drawMasks( _calibrated_camera, x, y, zoom, false );
		LOG( LEVEL_INFO, "proj test: %d", test );
		
	}
	
	x += _camera->_frame._rotated_img[_calibrated_camera]->width+30;
	
	// display rectified image + rectified edges if needed
	if (_rectified_done) {
		_calibration_viewer.displayMsg(x,y-20,"Rectified image",WHITE,0);
		if (calib_load_img->value())
			_calibration_viewer.drawImage(_calib_rectified_img,x,y,true,true);
		else
			_calibration_viewer.drawImage(_camera->_frame._rectified_img[_calibrated_camera],x,y,true,true);
	}
	
	// draw the mask image if needed
	if (calib_img_mask->value()) {
		_calibration_viewer.drawImage(_camera->_mask_imgs[_calibrated_camera],x,y,false,true);
	}
	
	// draw the user mask
	_camera->drawUserMask( _calibration_viewer, _calibrated_camera, CALIBRATION_DRAW_X, CALIBRATION_DRAW_Y, 1.0, false );

	redraw();
}

void MyGlWindow::drawVideo( bool snap)
{
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	if ( BACKGROUND_WHITE )
		_video_viewer.clearWindow(false);
	_video_viewer.setup2D();
	if ( BACKGROUND_WHITE )
		_video_viewer.clearWindow(false);
	else
		_video_viewer.clearWindow(true);
	
	// display error message if no camera
	if (( !_ladybug->_init ) && ( _database == NULL ) && ( _synthetic_mode == 0 ) ) {
		_video_viewer.displayMsg(100,WINDOW_HEIGHT-100,"Camera not connected",RED,0);
		// try to init the camera again
		_ladybug->init();
		return;
	}
	
	drawVideoImages();
	
	
	//viewer.drawImage(_camera->_frame._rotated_img[_calibrated_camera],10,100,true,true);
	
	// code to make a snapshot video
	
	/*printf("frame ID = %d\n",_ladybug->_frameId);
	char *name;
	name = (char*)malloc(15*sizeof(char));
	if (_ladybug->_frameId < 10)
	sprintf(name,"snap0%d.jpg",_ladybug->_frameId);
	else
	sprintf(name,"snap%d.jpg",_ladybug->_frameId);
	
	  viewer.screenshot(name,0,WINDOW_HEIGHT-MENUBAR_HEIGHT-3*int(_camera->_height/_zoomFactor),2*int(_camera->_width/_zoomFactor),3*int(_camera->_height/_zoomFactor));
	delete name;*/	
	drawVideoEdges();
	
	drawVideoPoints();
	
	drawVideoInfo();
	
	if ( snapshot->value() && !snap) {
		int y = WINDOW_HEIGHT-MENUBAR_HEIGHT-2*int(_camera->_height/_zoomFactor);
		//SNAPSHOT_START_X = 5;
		//SNAPSHOT_START_Y = y;
		_video_viewer.screenshot( frameId, SNAPSHOT_X, SNAPSHOT_Y, SNAPSHOT_W, SNAPSHOT_H );
	}

	// display info message in recording mode
	if ( _ladybug_recording ) {
		_video_viewer.displayMsg(100,300,RED, 0, "Camera recording @ %d fps : %f frames", _ladybug_record_framerate, _ladybug_record_nimages);
	}
}

void MyGlWindow::snapshotVideo()
{
//	if ( snapshot->value() ) {
//		drawVideo();
//		_video_viewer.screenshot( _timestamp, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT );
//		_timestamp++;
//	}	
}

void MyGlWindow::drawSphere()
{
	glDisable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	
	Vec3d position = _camera->came.getTranslation();
	_sphere_viewer._target = position;
	
	double radius = 50.0;
	int i;
	
	if ( BACKGROUND_WHITE )
		_sphere_viewer.clearWindow(false);
	_sphere_viewer.setup3D();
	if ( BACKGROUND_WHITE )
		_sphere_viewer.clearWindow(false);
	else
		_sphere_viewer.clearWindow(true);

	int w = _camera->_frame._rotated_img[0]->width;
	int h = _camera->_frame._rotated_img[0]->height;

	// vanishing points?
	if ( vanishing_points->value() ) {

		// draw the axis
		_sphere_viewer.drawAxis(position + Vec3d(0,1.5*radius,0),radius/3);
		_sphere_viewer.drawAxis(position + Vec3d(0,-1.5*radius,0),radius/3);

		// draw camera center
		_camera->draw( _sphere_viewer, 2.0, YELLOW );

		// draw observed vanishing points (color coded by cluster)
		Vec3d c = _sphere_viewer._target;

		_sphere_viewer.drawSphere(c,radius*.96,BLACK,10,10);
		glPointSize(2);
		glColor(RED);
		for (i=0;i<_vps.size();i++) {
			glColor(_colors[(_vps[i].second+2)%NCOLORS]);
			glBegin(GL_POINTS);
			glVertex(c + radius * _vps[i].first);
			glEnd();
	}

		// draw the clustered vanishing points 
		_sphere_viewer.drawSphere(c,radius*.96,BLACK,10,10);
		glPointSize(8);
		for (i=0;i<_vpc.size();i++) {
			glColor(_colors[(_vpc[i].second+2)%NCOLORS]);
			glBegin(GL_POINTS);
			glVertex(c + radius * _vpc[i].first);
			glEnd();
		}

		// draw the LUT vanishing points
		//c += Vec3d(0,3*radius, 0);
		
		_sphere_viewer.drawSphere(c,radius*.96,BLACK,10,10);
		glColor(GREEN);
		glPointSize(4);
		glBegin(GL_POINTS);
		for(i=0;i<_LUT._vps.size();i++)
			glVertex(c + radius * _LUT._vps[i].first);
		glEnd();
		glPointSize(1);
		return;
	}

	if ( rotation_scores->value() ) {

		for (int i=0;i<rotation_scoresheet.size();i++) {

			float color[3];
			score_to_color( rotation_scoresheet[i].second, color );
			//LOG(LEVEL_INFO, "%f %f %f", color[0], color[1], color[2]);

			_sphere_viewer.drawPoint( _sphere_viewer._target + radius * rotation_scoresheet[i].first, 6, color, .25 );
		}

		return;
	}

	// draw the selected pool correspondence
	if ( view_correspondences->value() ) {
	
		if ( strcmp( sphere_correspondence_mode->value(), "inliers" ) == 0 ) {
			if (  !_inliers.empty() ) {
				if (select_generic->value() < _inliers.size()) {
					Correspondence d = _inliers[select_generic->value()];
					_sphere_viewer.drawEdge( d.first, RED, 4 );
					_sphere_viewer.drawEdgePlane( d.second, _camera->getTranslation(), _camera->getRotation(), radius, 2, 2);
					_sphere_viewer.displayMsg( 100, 100, RED, 1, "Line %d", d.first->_id );
					EdgePlane ep = d.second;
					ep.fromCameraFrameToWorldFrame( _camera->getPose() );
					_calibration_viewer.displayMsg( 100, 80, RED, 1, "Angle : %f deg.", toDegrees(ep.angle( d.first)) );

					_sphere_viewer.setup3D();
				}
			}
		} else {
			if (select_generic->value() < SM_Size() && select_generic->value() > -1) {
				//printf("1\n");
				corres c = SM_Get( select_generic->value() );
				Edge *line = c.line;

				// set this line to be displayed in the 3D model
				_LUT._selected_lines.clear();
				_LUT._selected_lines.push_back( line );

				int cameraId = _calibrated_camera;
				_sphere_viewer.drawCorrespondence( c , _camera->getPose(), RED );
				//printf("2\n");
				EdgePlane epc = EdgePlane( norm( line->getA() - _camera->getTranslation() ), norm( line->getB() - _camera->getTranslation() ), _camera->getTranslation(),\
					cameraId, 0, 0);
				epc.fromWorldFrameToCameraFrame( _camera->getPose() );
				_sphere_viewer.drawCorrespondenceInfo( c , _camera->getPose(), 100, _sphere_viewer._height - 100, RED );
				//printf("3\n");
				//_camera->drawEdgeplanes( _sphere_viewer, cameraId, c.eps, DISTORTION_EDGE_STEP, 
				//	150,198,3,true,false,h,w,2);
				//_sphere_viewer.drawImage(_camera->_frame._display_img[cameraId],150,200,true,true);

				_sphere_viewer.setup3D();

				// debug display
				/*if ( _LUT._selected_lines.size() > 2 ) {
					double lambda;
					Vec3d p1 = intersectRayPlane( _LUT._selected_lines[0]->getA(), _LUT._selected_lines[0]->dir, _camera->getTranslation(), _LUT._selected_lines[0]->dir, lambda);
					Vec3d p2 = intersectRayPlane( _LUT._selected_lines[1]->getA(), _LUT._selected_lines[1]->dir, _camera->getTranslation(), _LUT._selected_lines[1]->dir, lambda);

					Vec3d u = norm(p2-p1);
					Vec3d z = norm(cross(p2-_camera->getTranslation(), p1-_camera->getTranslation()));
					Vec3d v = norm(cross(u,z));
					double c = len(p2-p1)/(2.0*tan(mline1_angle));
					Vec3d center = (p1+p2)/2.0 + c * v;
					LOG(LEVEL_INFO, "check: %f %f %f", len(p1-center),len(p2-center),len(_camera->getTranslation() - center));
					LOG(LEVEL_INFO, "angle check: %f %f", mline1_angle, Acos(dot(norm(p1-_camera->getTranslation()), norm(p2-_camera->getTranslation()))));
					double rad = c / cos(mline1_angle);
					_sphere_viewer.drawLine( p1, p2, YELLOW, 2);
					glBegin(GL_POINTS);
					for (int q=0;q<300;q++) {
						glVertex(center + rad * v );
						v = Quaternion(z,2*M_PI/300).rotate(v);
					}
					glEnd();
				}*/
			}
		}
	} else {
		
		// draw the spherical edges
		for ( i=0;i<_camera->_frame._nImages;i++) {
			if (!_camera->_frame._edgeplanes_chained[i].empty()) {
				_sphere_viewer.drawEdgePlanes(_camera->_frame._edgeplanes_chained[i],\
					_camera->getTranslation(),_camera->getRotation(),radius,_calibrated_camera,2,4);
				_sphere_viewer.drawEdgePlanesDetails(_camera->_frame._selected_edges,\
					_camera->getTranslation(),_camera->getRotation(),radius,_calibrated_camera,2,6);
				if ( _LUT._selected_lines.size() > 2 && _camera->_frame._selected_edges.size() > 2 ) {
					_sphere_viewer.drawEdge( *_LUT._selected_lines[2], RED, 4 );
					
					glEnable(GL_BLEND);
					glBlendFunc( GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
					glColor4f( _colors[_camera->_frame._selected_edges[2]._cameraId][0], _colors[_camera->_frame._selected_edges[2]._cameraId][1],
						_colors[_camera->_frame._selected_edges[2]._cameraId][2], .25 );
					glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
					glBegin( GL_POLYGON);
					glVertex( _camera->getTranslation() );
					glVertex( _LUT._selected_lines[2]->getA() );
					glVertex( _LUT._selected_lines[2]->getB() );
					glEnd();
					glDisable( GL_BLEND );
				}

			}
			else {
				_sphere_viewer.drawEdgePlanes(_camera->_frame._edgeplanes[i],\
					_camera->getTranslation(),_camera->getRotation(),radius,_calibrated_camera,2,2);
				_sphere_viewer.drawEdgePlanesDetails(_camera->_frame._selected_edges,\
					_camera->getTranslation(),_camera->getRotation(),radius,_calibrated_camera,2,6);
			}
		}
	}
	
	if ( _LUT._selected_lines.size() > 0 ) {
		_sphere_viewer.drawEdge( *_LUT._selected_lines[0], RED, 4 );
	}
	// draw the camera as a 2-inch sphere (and a vertical line for user information)
	_camera->draw( _sphere_viewer, 2.0, YELLOW );
	
	// draw the camera history
	drawPoseHistory( _sphere_viewer, 4.0, BLUE );
	
	// draw the LUT history
	_LUT.drawHistory( _sphere_viewer, RED, 4.0 );

	// draw the 3D features
	_sphere_viewer.drawEdges(_LUT._lines,GREEN,4,true);
	
	// draw axis
	if (draw_axis->value())
		_sphere_viewer.drawAxis(position,radius/2);
	
	// draw camera frustums
	if (sphere_draw_frustum->value()) {
		_sphere_viewer.drawEdgePlanes(_camera->_frame._edgeplanes_frustum,_camera->getTranslation(),_camera->getRotation(),radius,_calibrated_camera,2,4);
		for (i=0;i<_camera->_frame._nImages;i++) {
			_sphere_viewer.drawSphere(_camera->fromCameraFrameToWorldFrame(radius*_camera->getUnitCameraCenter(i)),1.0,_colors[i],10,10);
			Vec3d z = _camera->getRotation().rotate( _camera->getZDirection( i ) );
			_sphere_viewer.drawLine( _camera->getTranslation(), _camera->getTranslation() + radius * z, _colors[i], 5 );
		}
	}
	
	// draw 2D text info
	
	if ( view_correspondences->value() ) {
		
		if ( strcmp( sphere_correspondence_mode->value(), "maintenance" ) == 0 ) {
			if (select_generic->value() < SM_Size()) {
				_sphere_viewer.drawCorrespondenceInfo( SM_Get( select_generic->value() ) , _camera->getPose(), 100, _sphere_viewer._height - 100, RED );
			}
		}
	}


	/*if ( strcmp( sphere_correspondence_mode->value(), "recovery" ) == 0 ) {
		if (  !cs.empty() ) {
			if (select_generic->value() < cs[_calibrated_camera].size()) {				
				_camera->drawEdgeplanes( _sphere_viewer,cs[_calibrated_camera][select_generic->value()].eps, DISTORTION_EDGE_STEP, 
					150,198,3,true,false,h,w);
			}
		}
	}

	if ( snapshot->value()) {
		SNAPSHOT_START_X = 150;
		SNAPSHOT_START_Y = 200;
		_sphere_viewer.screenshot( _timestamp, SNAPSHOT_START_X, SNAPSHOT_START_Y, SNAPSHOT_WIDTH, SNAPSHOT_HEIGHT );
		_timestamp++;
	}*/
	
}

void MyGlWindow::drawVideoInfo()
{
	_video_viewer.setup2D();
	
	if (_database != NULL) {
		_video_viewer.displayMsg(10,10,RED,0,"Frame ID: %d",_database->_frameId);
	}
	
	Correspondence c;

	// draw orange line ID for correspondence
	if ( _LUT._selected_lines.size() == 1 ) {
		Edge *line = _LUT._selected_lines[0];
		_video_viewer.displayMsg(10,10,RED,0,"Line: %d",line->_id);
	}

	// display info in init mode
	if ( strcmp(sphere_correspondence_mode->value(), "init") == 0 && select_generic->value() < init_poses.size() ) {
		ExtrinsicParameters pose = init_poses[select_generic->value()];

		std::string mode = pose.get_mode_string();

		_video_viewer.displayMsg(10,40,RED, 0, "Pose %d / %d  Score %.2f  Mode: %s", int(select_generic->value()), init_poses.size()-1, pose.get_score(), (char*)mode.c_str());
	}

}

void MyGlWindow::drawVideoImages()
{
	int i,counter=0;
	int _x=VIDEO_DRAW_X,_y=WINDOW_HEIGHT-MENUBAR_HEIGHT-2*int(_camera->_height/_zoomFactor);
	int width = int(_camera->_width/_zoomFactor);
	int height = int(_camera->_height/_zoomFactor);

	if ( _database != NULL ) {
		// display frame ID and frame rate
		_video_viewer.displayMsg( _x, _y +height + 20, RED, 0, "Frame : %d / %d", frameId, _database->_nImages );
		_video_viewer.displayMsg( _x, _y +height + 40, RED, 0, "Frame rate : %d Hz", (int)_database->_framerate );
	}

	for (i=0;i<7;i++) {
		
		if (i == 4)
			continue;
		
		counter = (i+1)%6;
		
		_video_viewer.drawImage(_camera->_frame._display_img[counter],_x,_y,true,true);
		_camera->drawUserMask( _video_viewer, counter, _x, _y, _zoomFactor, false);
		_video_viewer.drawBox( _x, _y, _x + width, _y + height, 2, GREY );
		if ( BACKGROUND_WHITE )
			_video_viewer.displayMsg( _x, _y - 20, BLACK, 0, "Camera %d", counter );
		else
			_video_viewer.displayMsg( _x, _y - 20, _colors[counter], 0, "Camera %d", counter );
		_x += width;
	}
	
	// draw camera 5 image
	_x = 10+4*int(_camera->_width/_zoomFactor);
	_y = WINDOW_HEIGHT-MENUBAR_HEIGHT-int(_camera->_height/_zoomFactor);
	_video_viewer.drawImage(_camera->_frame._display_img[5],_x,_y,true,true);
	_video_viewer.drawBox( _x, _y, _x + width, _y + height, 2, GREY );
	_camera->drawUserMask( _video_viewer, 5, _x, _y, _zoomFactor, false );
	if ( BACKGROUND_WHITE )
		_video_viewer.displayMsg( _x - 1.2*width/2, _y + height - 20, BLACK, 0, "Camera %d", 5);
	else
		_video_viewer.displayMsg( _x - 1.2*width/2, _y + height - 20, _colors[5], 0, "Camera %d", 5);
	
	// draw flipped camera 5 image
	_x = 10+int(1.5*int(_camera->_width/_zoomFactor));
	_video_viewer.drawImage(_camera->_frame._flipped_img,_x,_y,true,true);
	_video_viewer.drawBox( _x, _y, _x + width, _y + height, 2, GREY );
	_camera->drawUserMask( _video_viewer, 5, _x, _y, _zoomFactor, true );
}

void MyGlWindow::drawVideoPoints ()
{
	int i,counter=0;

	_camera->resetPose();

	int _x=10,_y=WINDOW_HEIGHT-MENUBAR_HEIGHT-2*int(_camera->_height/_zoomFactor);
	int k,h = _camera->_frame._display_img[0]->height;
	int w = _camera->_frame._display_img[0]->width;
	int size = 4;
	Vec2d point;
	
	// draw the horizontal images on one row
	for (i=0;i<7;i++) {
		
		if (i == 4)
			continue;
		
		counter = (i+1)%6;
				
		// draw the corners in purple
		/*for (k=0;k<_camera->_frame._corners.size();k++) {
			corner c = _camera->_frame._corners[k];
			if ( c.getcameraid() != counter )
				continue;
			Vec2d point;
			
			if ( _camera->project(counter, 10.0 * Vec3d(c.getx(),c.gety(),c.getz()), point) ) {
				point = _camera->distort( counter,point );
				point = ladybugflip( point, h, _camera->_frame._display_zoom_factor );
				if ( k == select_generic->value() ) {
					_video_viewer.drawBox(_x+point[0]-size,_y+point[1]-size,_x+point[0]+size,_y+point[1]+size,2,YELLOW);
					//c.print();
					// draw the connected edges
					for (int p=0;p<c.nedges();p++) {
						Vec3d pa, pb;
						c.getedge( p, pa, pb );

						EdgePlane ep = EdgePlane( pa, pb, Vec3d(0,0,0), c.getcameraid(), 0, 0 );

						_camera->drawEdgeplane( _video_viewer, ep, DISTORTION_EDGE_STEP, _x, _y, _zoomFactor, PURPLE, true, false, h, w, counter, 2);
					}
				}
				else
					_video_viewer.drawBox(_x+point[0]-size,_y+point[1]-size,_x+point[0]+size,_y+point[1]+size,2,PURPLE);
			}
			
		}*/

		// draw the point features in red
		if ( view_mask->value() ) {
			for (k=0;k<_camera->_frame._points[counter].size();k++) {
				Vec2d point = _camera->_frame._points[counter][k];
				point = ladybugflip( point, h, _camera->_frame._display_zoom_factor );
				if ( k == select_generic->value() )
					_video_viewer.drawBox(_x+point[0]-size,_y+point[1]-size,_x+point[0]+size,_y+point[1]+size,2,YELLOW);
				else
					_video_viewer.drawBox(_x+point[0]-size,_y+point[1]-size,_x+point[0]+size,_y+point[1]+size,1,RED);
				
			}
		}

		// draw the tracked feature and bounding box in white
		/*if ( view_mask->value() ) {
			std::vector< Vec2d > boxes;
			if ( _camera->_frame.getFeaturePointBox( _calibrated_camera, frameId, select_generic->value(), boxes ) ) {
				
				for (int k=0;k<boxes.size() / 2; k++) {
					Vec2d a = ladybugflip( boxes[2*k], h, _camera->_frame._display_zoom_factor );
					Vec2d b = ladybugflip( boxes[2*k+1], h, _camera->_frame._display_zoom_factor );
					_video_viewer.drawBox( _x+a[0], _y+a[1], _x+b[0], _y+b[1], 1, WHITE );
				}
			}
		}*/

		_x += int(_camera->_width/_zoomFactor);

	}
	
	_x = 10+4*int(_camera->_width/_zoomFactor);
	_y = WINDOW_HEIGHT-MENUBAR_HEIGHT-int(_camera->_height/_zoomFactor);

	// draw the corners in purple
	for (k=0;k<_camera->_frame._corners.size();k++) {
		corner c = _camera->_frame._corners[k];
		if ( c.getcameraid() != 5 )
			continue;
		Vec2d point;
		
		if ( _camera->project(5, 10.0 * Vec3d(c.getx(),c.gety(),c.getz()), point) ) {
			point = _camera->distort( 5,point );
			point = ladybugflip( point, h, _camera->_frame._display_zoom_factor );
			_video_viewer.drawBox(_x+point[0]-size,_y+point[1]-size,_x+point[0]+size,_y+point[1]+size,1,PURPLE);
		}
	}

	// draw the point features in red
	if ( view_mask->value() ) {
		for (k=0;k<_camera->_frame._points[5].size();k++) {
			Vec2d point = _camera->_frame._points[5][k];
			point = ladybugflip( point, h, _camera->_frame._display_zoom_factor );
			if ( k == select_generic->value() )
				_video_viewer.drawBox(_x+point[0]-size,_y+point[1]-size,_x+point[0]+size,_y+point[1]+size,2,YELLOW);
			else
				_video_viewer.drawBox(_x+point[0]-size,_y+point[1]-size,_x+point[0]+size,_y+point[1]+size,1,RED);
		}
	}

	_camera->restorePose();

	// draw the tracked feature
	/*if ( 5 == _calibrated_camera && _camera->_frame.getFeaturePoint( _calibrated_camera, frameId, select_generic->value(), point, age ) ) {
		point = ladybugflip( point, h, _camera->_frame._display_zoom_factor );
		_video_viewer.drawBox(_x+point[0]-size,_y+point[1]-size,_x+point[0]+size,_y+point[1]+size,2,YELLOW);

		// draw the bounding box in white
		if ( view_mask->value() ) {
			std::vector< Vec2d > boxes;
			if ( _camera->_frame.getFeaturePointBox( _calibrated_camera, frameId, select_generic->value(), boxes ) ) {
				for (int k=0;k<boxes.size() / 2; k++) {
					Vec2d a = ladybugflip( boxes[2*k], h, _camera->_frame._display_zoom_factor );
					Vec2d b = ladybugflip( boxes[2*k+1], h, _camera->_frame._display_zoom_factor );
					_video_viewer.drawBox( _x+a[0], _y+a[1], _x+b[0], _y+b[1], 1, WHITE );
				}
			}		
		}
	}*/

	// update the tracker debug ID
	_camera->_frame.ftracker[_calibrated_camera]->dbg_id = select_generic->value();
}

void MyGlWindow::drawVideoEdges (int cameraId, int _x, int _y, bool flip)
{
	int w = _camera->_frame._rotated_img[cameraId]->width;
	int h = _camera->_frame._rotated_img[cameraId]->height;
	double zoom = _camera->_frame._display_zoom_factor;
	
	// draw the reprojected edges
	//if (view_reprojected_edges->value()) {
	if ( STATE_COLOR_CODED ) {
		_video_viewer.drawEdgesColorCoded(_camera->_frame._edges_reprojected[cameraId],_x,_y,zoom,true,flip,\
			h,w,cameraId,2);
	} else {
		_video_viewer.drawEdges(_camera->_frame._edges_reprojected[cameraId],_x,_y,zoom,GREEN,true,flip,\
			h,w,cameraId);
	}
	//}

	// draw the tracked edges
	if ( strcmp( sphere_correspondence_mode->value(), "tracker" ) == 0 ) {

		if ( _LUT._selected_lines.size() == 1 ) {

			Edge *line = _LUT._selected_lines[0];

			EdgePlane ep;

			//write_correspondences(1);
			if ( read_correspondence( frameId, line->_id, &ep ) ) {

				//printf("read successful!\n");
				_camera->drawEdgeplane(_video_viewer, ep, DISTORTION_EDGE_STEP, 
					_x, _y, zoom, RED/*_colors[cameraId]*/, true, flip, h, w, cameraId, 4);
			}
		}

		/*
		if ( select_generic->value() < _tracked_edges.size() ) {

			int hits = _tracked_edges[select_generic->value()].first;

			_video_viewer.displayMsg( 100, 100, _colors[cameraId], 0, "hits = %d  age = %d", hits, _tracked_edges[select_generic->value()].second.size() );

			for (int p=0;p<_tracked_edges[select_generic->value()].second.size();p++) {

				EdgePlane ep = _tracked_edges[select_generic->value()].second[p].second;

				if ( frameId == _tracked_edges[select_generic->value()].second[p].first ) {

					_camera->drawEdgeplane(_video_viewer, ep, DISTORTION_EDGE_STEP, 
						_x, _y, zoom, _colors[cameraId], true, flip, h, w, cameraId, 2);
				}
			}
		}
		*/
		return;
	}

	// draw the vanishing points in red
	if ( _display_vps ) {
		
		for (int p = 0; p < _vps_edges.size();p++) {
			Vec2d vp;
			int size = 14;
			if ( _camera->project( cameraId, _camera->fromCameraFrameToWorldFrame(10.0 * _vps_edges[p]), vp )) {
				vp = _camera->distort( cameraId, vp );
				_video_viewer.drawFace(vp+Vec2d(size,-size), vp+Vec2d(size,size), vp+Vec2d(-size,size), vp+Vec2d(-size,-size), _x, _y, zoom, RED,
					true, flip, h, w);
			}
			if ( _camera->project( cameraId, _camera->fromCameraFrameToWorldFrame(-10.0 * _vps_edges[p]), vp )) {
				vp = _camera->distort( cameraId, vp );
				_video_viewer.drawFace(vp+Vec2d(size,-size), vp+Vec2d(size,size), vp+Vec2d(-size,size), vp+Vec2d(-size,-size), _x, _y, zoom, RED,
					true, flip, h, w);
			}
		}
		
		for (p = 0; p < _vps_lines.size();p++) {
			Vec2d vp;
			int size = 14;
			if ( _camera->project( cameraId, _camera->fromCameraFrameToWorldFrame(10.0 * _vps_lines[p]), vp )) {
				vp = _camera->distort( cameraId, vp );
				_video_viewer.drawFace(vp+Vec2d(size,-size), vp+Vec2d(size,size), vp+Vec2d(-size,size), vp+Vec2d(-size,-size), _x, _y, zoom, BLUE,
					true, flip, h, w);
			}
			if ( _camera->project( cameraId, _camera->fromCameraFrameToWorldFrame(-10.0 * _vps_lines[p]), vp )) {
				vp = _camera->distort( cameraId, vp );
				_video_viewer.drawFace(vp+Vec2d(size,-size), vp+Vec2d(size,size), vp+Vec2d(-size,size), vp+Vec2d(-size,-size), _x, _y, zoom, BLUE,
					true, flip, h, w);
			}
		}
	}

	// draw anchors on video window
	if ( _show_anchor )
		drawAnchors (_x, _y, zoom, true, flip, cameraId, h, w);
	
		if ( view_correspondences->value() ) {
			
			// draw the corner correspondence
			/*int counter = 0;
			for (int i=0;i<_corner_lines.size();i++) {
				corner_line cl = _corner_lines[i];
				
				if ( cl.corner_id != select_generic->value() )
					continue;
							
				if (counter == select_pose->value()) {
					Vec3d a,b;
					Vec3d center = _camera->getUnitCameraCenter( cameraId );
					_camera->_frame._corners[cl.corner_id].getedge(cl.edge_1_id, a , b);
					EdgePlane ep1 = EdgePlane( a, b, center, cameraId, 0, 0 );
					_camera->_frame._corners[cl.corner_id].getedge(cl.edge_2_id, a , b);
					EdgePlane ep2 = EdgePlane( a, b, center, cameraId, 0, 0 );
					
					if ( cl.camera_id == cameraId ) {
						_camera->drawEdgeplane( _video_viewer, ep1, DISTORTION_EDGE_STEP, _x, _y, zoom, PURPLE, true, flip, h, w, cameraId, 2);
						_camera->drawEdgeplane( _video_viewer, ep2, DISTORTION_EDGE_STEP, _x, _y, zoom, PURPLE, true, flip, h, w, cameraId, 2);
					}
				}
				
				counter++;
			}*/
			
			if ( strcmp( sphere_correspondence_mode->value(), "inliers" ) == 0 ) { // draw recovery correspondences
				//if (  !_inliers.empty() ) {
				//	if (select_generic->value() < _inliers.size()) {
				//		Correspondence d = _inliers[select_generic->value()];
				//		_camera->drawEdgeplane( _video_viewer, d.second, DISTORTION_EDGE_STEP, _x, _y, zoom, _colors[cameraId], true, flip, h, w, cameraId, 2);
				//	}
				//}
	
				// shows the triplets matched during line-edge matching at initialization
				if ( select_generic->value() < _line_matching_history.size() ) {

				_LUT._selected_lines.clear();

				for (int k=0;k<_line_matching_history[select_generic->value()].first.size();k++)
					_LUT._selected_lines.push_back( _LUT._model->getEdge( _line_matching_history[select_generic->value()].first[k] ) );

				for (k=0;k<_line_matching_history[select_generic->value()].second.size();k++)
					_camera->drawEdgeplane( _video_viewer, _line_matching_history[select_generic->value()].second[k], DISTORTION_EDGE_STEP, _x, _y, zoom, _colors[cameraId], true, flip, h, w, cameraId, 2);

				_camera->setPose( _line_matching_pose_history[select_generic->value()] );
				
				if ( view_reprojected_edges->value() ) {
					reprojectEdges( _LUT._lines );
					if ( STATE_COLOR_CODED ) {
						_video_viewer.drawEdgesColorCoded(_camera->_frame._edges_reprojected[cameraId],_x,_y,zoom,true,flip,\
							h,w,cameraId,2);
					} else {
						_video_viewer.drawEdges(_camera->_frame._edges_reprojected[cameraId],_x,_y,zoom,GREEN,true,flip,\
							h,w,cameraId);
					}
				} else {
					_camera->clearReprojectedEdges();
				}
				}


			} else { // draw the maintenance correspondence
				if ( select_generic->value() < SM_Size() ) {
					_camera->drawEdgeplanes( _video_viewer, cameraId, SM_Get( select_generic->value() ).eps, DISTORTION_EDGE_STEP, 
						_x,_y,zoom,true,flip,h,w, 2);
				}
			}
		} else {
			// draw the chained edges (select mode and render mode)
			//_video_viewer.drawEdgesSelectId(_camera->_frame._edges[cameraId],_x,_y,zoom,_colors[cameraId],true,flip,\
			//	h,w,_camera->_frame._edgeplanes_chained[cameraId],cameraId,2);
			_camera->drawEdgeplanes( _video_viewer, cameraId, _camera->_frame._edgeplanes_chained[cameraId], DISTORTION_EDGE_STEP, 
				_x,_y,zoom,true,flip,h,w, 2);
			_camera->drawEdgeplanesSelectMode( _video_viewer, cameraId, _camera->_frame._edgeplanes_chained[cameraId], DISTORTION_EDGE_STEP, 
				_x,_y,zoom,true,flip,h,w);
			
			_camera->drawEdgeplanes( _video_viewer, cameraId, _camera->_frame._selected_edges, DISTORTION_EDGE_STEP, 
				_x,_y,zoom,true,flip,h,w, 10);


			// draw the selected edges (render mode only)
			//if (_video_viewer._mode == GL_RENDER)
			//	_video_viewer.drawEdgesSelectId(_camera->_frame._edges[cameraId],_x,_y,zoom,_colors[cameraId],true,flip,\
			//	h,w,_camera->_frame._selected_edges,cameraId,10);
			
			// draw the selected lines (based on _lineId)
			//_video_viewer.drawEdgesSelectId (_camera->_frame._edges_reprojected[cameraId],_x,_y,zoom,GREEN,true,flip,\
			//	h,w,_LUT._selected_lines,cameraId,8);
		}
		
		

	_video_viewer.drawSiftFeatures( _camera->_frame._sift[cameraId], _x, _y, zoom, RED, false, flip, h, w);

	// display masks
	if (view_mask->value())
		_camera->drawMasks(cameraId,_x,_y,zoom,flip);
}

void MyGlWindow::drawVideoEdges ()
{
	// otherwise, display images
	int i,counter=0;
	int _x=10,_y=WINDOW_HEIGHT-MENUBAR_HEIGHT-2*int(_camera->_height/_zoomFactor);
	
	// for debugging, distribute edges in buckets
	EdgePlane ref_edgeplane;
	std::vector< intVector > edges_buckets;
	distribute_edgeplanes_into_buckets( edges_buckets );
	edgePlaneVector lines_edgeplanes;
	int ref_bucket_id = -1;

	// draw the horizontal images on one row
	int width = _camera->_frame._rotated_img[_calibrated_camera]->width;
	int height = _camera->_frame._rotated_img[_calibrated_camera]->height;

	for (i=0;i<7;i++) {
		
		if (i == 4)
			continue;
		
		counter = (i+1)%6;
		
		drawVideoEdges(counter,_x,_y,false);
		
		// draw orange line for correspondence
	
		if ( _LUT._selected_lines.size() == 1 ) {
			Edge *line = _LUT._selected_lines[0];
			
			Correspondence c;
			//for (int j=0;j<SM_Size();j++) {
			//	SM_get_correspondence(j, c );
			//	if ( c.second._cameraId == counter && c.first->_id == line->_id )
			//		_camera->drawEdgeplane( _video_viewer, c.second, DISTORTION_EDGE_STEP, _x, _y, _camera->_frame._display_zoom_factor, ORANGE, true, false, height, width, counter, 2);
			//}
		}

		if ( ref_bucket_id != -1 && ref_bucket_id < edges_buckets.size() ) {
			for (int j=0;j<edges_buckets[ref_bucket_id].size();j++) {
				EdgePlane epp;
				if ( _camera->_frame.get_edgeplane_chained(edges_buckets[ref_bucket_id][j], epp) && select_generic->value() != edges_buckets[ref_bucket_id][j] &&
					epp.overlap( ref_edgeplane ) > SCORE_MIN_OVERLAP)
					_camera->drawEdgeplane( _video_viewer, epp , DISTORTION_EDGE_STEP, _x, _y, _camera->_frame._display_zoom_factor, YELLOW, true, false, height, width, counter, 2);
			}
		}

		_x += int(_camera->_width/_zoomFactor);
		
	}

	counter = 5;
	// draw the top image (camera 5) on the right
	_x = 10+4*int(_camera->_width/_zoomFactor);
	_y = WINDOW_HEIGHT-MENUBAR_HEIGHT-int(_camera->_height/_zoomFactor);
	drawVideoEdges(5,_x,_y,false);
	if ( _LUT._selected_lines.size() == 1 ) {
		Edge *line = _LUT._selected_lines[0];
		
		//Correspondence c;
		//for (int j=0;j<SM_Size();j++) {
		//	SM_get_correspondence(j, c );
		//	if ( c.second._cameraId == counter && c.first->_id == line->_id )
		//		_camera->drawEdgeplane( _video_viewer, c.second, DISTORTION_EDGE_STEP, _x, _y, _camera->_frame._display_zoom_factor, ORANGE, true, false, height, width, counter, 2);
		//}
	}

	// draw the flipped top image (on the left) along with edges
	_x = 10+int(1.5*int(_camera->_width/_zoomFactor));
	drawVideoEdges(5,_x,_y,true);
	
}

int MyGlWindow::keyboard(int event, int key)
{
	if (event != FL_KEYDOWN)
		return 1;
	
	//printf("key pressed: %d\n",Fl::event_key());
	
	if (Fl::get_key(FL_Tab))
		restoreMode();
	
	if (Fl::event_key() == 32) // space key 
	{
		if (_windowMode == MODE_VIDEO)
			_camera->_frame.save("C:/temp/video",_ladybug->_frameId);
		else if (_windowMode == MODE_CALIBRATION)
			_camera->_frame.save("C:/temp/video",_ladybug->_frameId,_calibrated_camera);
	}
	
	double step = 5.0;
	
	//printf("key: %d\n", key);
	//int key = Fl::event_key();

	//if ( Fl::get_key()
	/*if (Fl::get_key(FL_Up))
	_camera->_diff_intr.CC[0] -= step;
	if (Fl::get_key(FL_Down))
	_camera->_diff_intr.CC[0] += step;
	if (Fl::get_key(FL_Left))
	_camera->_diff_intr.CC[1] -= step;
	if (Fl::get_key(FL_Right)) {
	//printf("adjusting center point.\n");
	_camera->_diff_intr.CC[1] += step;
}*/
	
	return 1;
}

void MyGlWindow::video_picking_2d (int x, int y)
{
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	
	_video_viewer.setMode(GL_SELECT);
	if ( BACKGROUND_WHITE )
		_video_viewer.clearWindow(false);
	else
		_video_viewer.clearWindow(true);
	_video_viewer.setup2D(x,y);
	
	bool chained = false;
	
	drawVideoEdges();
	
	intVector hits;
	_video_viewer.stopVideoPicking(hits);
	EdgePlane edgeplane;
	
	if (hits.size() > 1) {
		int cameraId = hits[0];
		int id = hits[1];
		if ( cameraId < 6 && id < _camera->_frame._edgeplanes_chained[cameraId].size() )
			_camera->_frame._selected_edges.push_back( _camera->_frame._edgeplanes_chained[cameraId][id] );

		/*if (type == DETECTED) {
			id = hits[2];
			assert ((id>=0) && (id < _camera->_frame._edges[cameraId].size())); 
			//_camera->_frame._selected_edges.push_back(_camera->_frame._edges[cameraId][id]);
		} else if (type == CHAINED) {
			id = hits[3];
			assert ((id >= 0) && (id < _camera->_frame._edgeplanes_chained[cameraId].size()));
			edgeplane = _camera->_frame._edgeplanes_chained[cameraId][id];
			_camera->_frame._selected_edges.push_back(edgeplane);
		} else if (type == REPROJECTED) {
			id = hits[4];
			assert ((id >= 0) && (id < _LUT._model->_maxEdgeId));
			_LUT._selected_lines.push_back(_LUT._model->getEdge(id));
		} else if (type == CORRESPONDENCE) {
			id = hits[4];
			//assert ((id >= 0) && (id < _corrManager->size()));
			//_corrManager->setSelectedCorrespondence(id);
			//Correspondence temp = _corrManager->getCorrespondence(frameId,id);
			//edge2DVector edges_tmp;
			//_camera->reprojectEdgePlane(cameraId,id,temp.getPlane(),10,edges_tmp);
		}
		*/
	}
}

void MyGlWindow::sphere_picking_3d (int x, int y)
{
	glDisable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	_sphere_viewer.setMode(GL_SELECT);
	if ( BACKGROUND_WHITE )
		_sphere_viewer.clearWindow(false);
	else
		_sphere_viewer.clearWindow(true);
	_sphere_viewer.setup3D(x,y);
	
	// draw the spherical edges
	bool chained = false;
	int i;
	Vec3d position = _camera->came.getTranslation();
	double radius = 50.0;
	
	glPushName(0);
	for ( i=0;i<_camera->_frame._nImages;i++) {
		if (!_camera->_frame._edgeplanes_chained[i].empty()) {
			_sphere_viewer.drawEdgePlanes(_camera->_frame._edgeplanes_chained[i],\
				_camera->getTranslation(),_camera->getRotation(),radius,i,2,30);
			chained = true;
		}
		else
			_sphere_viewer.drawEdgePlanes(_camera->_frame._edgeplanes[i],_camera->getTranslation(),_camera->getRotation(),radius,i,2,30);
	}
	glPopName();
	
	// draw the 3D lines
	glPushName(1);
	_sphere_viewer.drawEdgeBoxes(_LUT._model->_edges,0.005);
	glPopName();
	
	intVector hits;
	_sphere_viewer.stopPicking(hits);
	
	if (hits.size() > 1) {
		int code = hits[0];
		
		if (code == 0) {
			int cameraId = hits[1];
			
			if (!chained)
				_camera->_frame._selected_edges.push_back(_camera->_frame._edgeplanes[cameraId][hits[2]]);
			else {
				_camera->_frame._selected_edges.push_back(_camera->_frame._edgeplanes_chained[cameraId][hits[2]]);
			}
		} else {
			_LUT._selected_lines_sphere.push_back(_LUT._model->_edges[hits[1]]);
		}
	}
}

void MyGlWindow::model_picking_3d (int x, int y)
{
	_LUT.model_picking_3d(x,y,_wireframe,_depth_test);
}

// draw a sphere at every position in the history
void MyGlWindow::drawPoseHistory( Viewer &viewer, int radius, const float color[3] )
{
	if ( pose_history.empty() )
		return;
	
	int i = pose_history.size() - 1;
	
	while (i >= 0) {

		viewer.drawFrustum( pose_history[i], radius, color );
		//viewer.drawSphere( pose_history[i].getTranslation(), radius, color, 10, 10 );
		i--;
	}

	//viewer.drawSphere( get_expected_position(8), radius, ORANGE, 10, 10 );

	for (i=0;i<_point_cloud.size();i++)
		viewer.drawPoint( _point_cloud[i], 5, color, .25 );

	for (i=0;i<_point_cloud_centroids.size();i++) {
		viewer.drawPoint( _point_cloud_centroids[i], 5, RED, 1.0 );
		viewer.drawLine( _point_cloud_centroids[i], _point_cloud_centroids[i] + Vec3d(0,0,5), RED, 1);
	}
}

// draw the anchors
void MyGlWindow::drawAnchors (int x, int y, double zoom, bool flip1, bool flip2, int cameraid, int h, int w) 
{
	int i;
	
	if ( _windowMode == MODE_3D_MAP ) {
		
		for (i=0;i<_anchors.size();i++) {
			
			// draw the anchor
			_LUT._viewer_3d.drawFace( _anchors[i].a, _anchors[i].b, _anchors[i].c, _anchors[i].d, _anchors[i].n, .5, PINK );
			
			Vec3d target = (_anchors[i].a + _anchors[i].b + _anchors[i].c + _anchors[i].d) / 4.0;
			
			// draw a line between the center and the target
			_LUT._viewer_3d.drawLine( _anchors[i].cc, target, PINK, 1 );

			// draw a green debug line for shortest distance
			_LUT._viewer_3d.drawLine( _anchors[i].c1, _anchors[i].c2, GREEN, 1 );
				
		}
	}
	
	if ( _windowMode == MODE_CALIBRATION || _windowMode == MODE_VIDEO ) {
	
		for (i=0;i<_anchors.size();i++) {
	
			Vec2d a,b,c,d;

			// project the anchor on the image
			if ( !_camera->project( cameraid, _anchors[i].a, a ) )
				continue;
			
			a = _camera->distort( cameraid, a );

			if ( !_camera->getIntrinsic(cameraid).visible(a) ) 
				continue;

			if ( !_camera->project( cameraid, _anchors[i].b, b ) )
				continue;
			
			b = _camera->distort( cameraid, b );
			
			if ( !_camera->getIntrinsic(cameraid).visible(b) ) 
				continue;

			if ( !_camera->project( cameraid, _anchors[i].c, c ) )
				continue;
			
			c = _camera->distort( cameraid, c );

			if ( !_camera->getIntrinsic(cameraid).visible(c) ) 
				continue;
			
			if ( !_camera->project( cameraid, _anchors[i].d, d ) )
				continue;
			
			d = _camera->distort( cameraid, d );

			if ( !_camera->getIntrinsic(cameraid).visible(d) ) 
				continue;

			// draw the anchor
			_calibration_viewer.drawFace(a, b, c, d, x, y, zoom, PINK, flip1, flip2, h, w);

			// draw a link between the anchor center and the anchor target

			Vec3d target = (_anchors[i].a + _anchors[i].b + _anchors[i].c + _anchors[i].d) / 4.0;

			edge2DVector edges;
			_camera->projectAndSubdivide( cameraid, _anchors[i].cc + 1.0 * norm(target - _anchors[i].cc), target, 10, edges);

			_video_viewer.drawEdges( edges,x,y,zoom,PINK,flip1,flip2,h,w,cameraid);
		}
	}
}


