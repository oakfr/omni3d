// simple OpenGL / FlTk program
// written 10/16/99, Michael L Gleicher


#include "main.h"

void MyGlWindow::draw2DMap()
{
	_LUT.drawScene_3d_topview();

		// draw the camera as a 2-inch sphere (and a vertical line for user information)
	_camera->draw( _sphere_viewer, 2.0, YELLOW );
	
	// draw the camera history
	drawPoseHistory( _LUT._viewer_3d_topview, 2.0, BLUE );

	if ( snapshot->value()) {
		//SNAPSHOT_START_X = 150;
		//SNAPSHOT_START_Y = 200;
		 _LUT._viewer_3d_topview.screenshot( _timestamp, SNAPSHOT_X, SNAPSHOT_Y, SNAPSHOT_W, SNAPSHOT_H);
		_timestamp++;
	}
	
}


void MyGlWindow::draw()
{	// the draw method must be private

	glClearColor(0,0,0,0);		// clear the window to black
	glClear(GL_COLOR_BUFFER_BIT);	// clear the window

	showAndHide();
	glDisable(GL_LIGHTING);

	switch (_windowMode) {
	case MODE_VIDEO:
		drawVideo(false);
		break;
	case MODE_3D_MAP:
		glEnable(GL_LIGHTING);
		draw3DMap();
		break;
	case MODE_2D_MAP:
		draw2DMap();
		break;
	case MODE_SPHERE:
		drawSphere();
		break;
	case MODE_CALIBRATION:
		drawCalibration();
		break;
	case MODE_ROTATIONS:
		drawRotations();
		break;
	default:
		exit(0);
	}
	
	//if ( snapshot->value() )
	//	drawSnapshotScreen();
	//redraw();
};

void MyGlWindow::restoreMode ()
{
	WindowMode temp = _windowMode;
	_windowMode = _historyMode;
	_historyMode = temp;
	redraw();
}

int MyGlWindow::handle(int e)
{

	//int ret = Fl_Button::handle(e);
	//cout<<endl<<count++<<" ******** button "<<label()<<" receives ";
	switch(e)
	{
	case FL_FOCUS:
	case FL_UNFOCUS:
	case FL_KEYBOARD:
		return keyboard(e,Fl::event_key());
	case FL_WHEN_RELEASE:
		_pauseVideo = 1-pause_video->value();
		redraw();
		break;
	case FL_PUSH:
		
		_mousex = Fl::event_x();
		_mousey = Fl::event_y();
		
		if (_windowMode == MODE_3D_MAP) {
			_LUT.mouse_3d(Fl::event_button(),GLUT_DOWN,Fl::event_x(),Fl::event_y(),\
				fltk2gl_mousemodifier());
			if (fltk2gl_mousemodifier() == GLUT_ACTIVE_CTRL) {
				model_picking_3d(Fl::event_x(),Fl::event_y());
			} else if (fltk2gl_mousemodifier() == GLUT_ACTIVE_SHIFT) {
				_camera->came.getTranslation() = _LUT._viewer_3d.raycast(Fl::event_x(),WINDOW_HEIGHT-Fl::event_y());
				//_LUT._position = _camera->came.getTranslation();
				_sphere_viewer._target = _camera->came.getTranslation();
			}
		}
		else if (_windowMode == MODE_2D_MAP) {
			_LUT.mouse_3d_topview( Fl::event_button(),GLUT_DOWN,Fl::event_x(),Fl::event_y());
		} 
		else if (_windowMode == MODE_SPHERE) {
			_sphere_viewer.mouse(Fl::event_button(),GLUT_DOWN,Fl::event_x(),Fl::event_y(),fltk2gl_mousemodifier());
			if (fltk2gl_mousemodifier() == GLUT_ACTIVE_CTRL) {
				sphere_picking_3d(Fl::event_x(),Fl::event_y());
			}
		} 
		else if (_windowMode == MODE_VIDEO) {
			if (fltk2gl_mousemodifier() == GLUT_ACTIVE_CTRL) {
				video_picking_2d(Fl::event_x(),Fl::event_y());
			}
			if ( Fl::event_button() == FL_LEFT_MOUSE ) {
				if ( _adding_anchor ) {
					addAnchor( _mousex, _mousey, _calibrated_camera );
					_adding_anchor = false;
				}
				if ( _updating_anchor ) {
					updateAnchor(  _mousex, _mousey, _calibrated_camera );
					_updating_anchor = false;
				}
				
				if ( _removing_anchor ) {
					removeAnchor(  _mousex, _mousey, _calibrated_camera );
					_removing_anchor = false;
				}
			} else if (  Fl::event_button() == FL_RIGHT_MOUSE ) {
				LOG(LEVEL_INFO, "cancelled selection");
				_adding_anchor = false;
				_updating_anchor = false;
				_removing_anchor = false;
			}			
		}
		else if ( _windowMode == MODE_CALIBRATION ) { // let the user select points to create an anchor
			if ( Fl::event_button() == FL_LEFT_MOUSE ) {
				LOG(LEVEL_INFO, "selected (%d,%d)", _mousex, _mousey);
				if ( _adding_anchor ) {
					addAnchor( _mousex, _mousey, _calibrated_camera );
					_adding_anchor = false;
				}
				if ( _updating_anchor ) {
					updateAnchor(  _mousex, _mousey, _calibrated_camera );
					_updating_anchor = false;
				}

				if ( _removing_anchor ) {
					removeAnchor(  _mousex, _mousey, _calibrated_camera );
					_removing_anchor = false;
				}

			} else if (  Fl::event_button() == FL_RIGHT_MOUSE ) {
				LOG(LEVEL_INFO, "cancelled selection");
				_adding_anchor = false;
				_updating_anchor = false;
				_removing_anchor = false;
			}
		}
		else if ( _windowMode == MODE_ROTATIONS ) {
			_rotations_viewer.mouse(Fl::event_button(),GLUT_DOWN,Fl::event_x(),Fl::event_y(),fltk2gl_mousemodifier());
		}
		return 1;
	case FL_RELEASE:
	case FL_DRAG:
		if (_windowMode == MODE_3D_MAP) {
			_LUT.motion_3d(Fl::event_x(),Fl::event_y());
			if (fltk2gl_mousemodifier() == GLUT_ACTIVE_SHIFT) {
				
				if (e == FL_RELEASE) {
					// when released, compute the visible 3D features
					//_LUT.nframes = FRAMES_JUMP; // force the node jump
					_LUT.lookup( _camera->came.getTranslation(), MIN_SUBTENDED_ANGLE, 0.0, 0, 0);
				} else {
					// drag the camera position
					//double z = _camera->getTranslation()[2];
					_camera->came.setTranslation(_LUT._viewer_3d.raycast(Fl::event_x(),WINDOW_HEIGHT-Fl::event_y()));
					//_camera->setTranslation( Vec3d( _camera->getTranslation()[0], _camera->getTranslation()[1], z ) );
					//_LUT._position = _camera->came.getTranslation();
					_sphere_viewer._target = _camera->came.getTranslation();
				}
			} 
			redraw();
		} 
		else if (_windowMode == MODE_SPHERE) {
			_sphere_viewer.motion(Fl::event_x(),Fl::event_y(),4.0,400.0,4.0);
		}
		else if (_windowMode == MODE_2D_MAP) {
			_LUT.motion_3d_topview(Fl::event_x(),Fl::event_y());
		}
		else if ( _windowMode == MODE_ROTATIONS ) {
			_rotations_viewer.motion( Fl::event_x(),Fl::event_y(),4.0,400.0,4.0);
		}
		//redraw();
		return 1;
		//case FL_HIDE:
		//	 return 1;
	case FL_SHOW:
		show();
		return 1;
	default:
		return Fl_Widget::handle(e);
	}	
	return 1;
}

void initGraphics()
{
	if (_gl_main->_initGraphics)
		return;

	_gl_main->_initGraphics = true;

}

// this callback is called every second
void callback(void*) 
{	

	PerfTimer timer;
	
	initGraphics();

	_gl_main->redraw();

	//Fl::repeat_timeout(.5, callback);

	//return;

	if (!_gl_main->pause_video->value()) {
		Fl::repeat_timeout(_tempo, callback);
		return;
	}
	
	//refresh the control bar
	_wind->child(2)->redraw();
	
	// for performance reasons...	
	_gl_main->_camera->setActiveSensor(_gl_main->_calibrated_camera);
	
	// check database first
	if (_gl_main->_database == NULL) {
		// grab ladybug images
		if (_gl_main->_ladybug->_init) {
			_gl_main->_ladybug->grab(_gl_main->_camera->_frame._grab_img);
			_gl_main->_camera->_frame.processFrame();
			_tempo = 0.05;
		} else {
			_tempo = 1.0;
		}
	}

	// save frame if recording
	if ( _gl_main->_ladybug_recording ) {

		_tempo = .5;

		_gl_main->_ladybug->grab(_gl_main->_camera->_frame._grab_img); // grab an image on the ladybug
		_gl_main->_camera->_frame.processFrame();

		for (int sensorId = 0; sensorId < 6; sensorId++ ) {
			std::string filename = _gl_main->_database->_dirname + "//" + _gl_main->_database->getLadybugImageFilename( sensorId, _gl_main->frameId );

			cvSaveImage( filename.c_str(), _gl_main->_camera->_frame._tmp_img[sensorId] );
		}

		_gl_main->_ladybug_record_nimages++;
		_gl_main->frameId++;
		_gl_main->_database->_frameId = _gl_main->frameId;
		_gl_main->redraw();
	}

	// compute FAST features and update the tracker

	if ( _gl_main->view_flow->value() ) {
	}
		
	// refresh the main window
	if ((_gl_main->_windowMode == MODE_VIDEO) || (_gl_main->_windowMode == MODE_CALIBRATION))
		_gl_main->redraw();
	
	// refresh the main window
	if (_gl_main->_windowMode == MODE_SPHERE)
		_gl_main->redraw();
	
	_gl_control->_perf_time = timer.elapsed();
	//Fl::repeat_timeout(MAX(0.05,1.5*_gl_control->_perf_time), callback);
	Fl::repeat_timeout(_tempo, callback);
}

// the main routine makes the window, and then runs an even loop
// until the window is closed
int main(int argc, char *argv[])
{
	LOG_INIT( "log.txt" );

	srand(time(NULL));

	_wind = new Fl_Double_Window(6,MENUBAR_HEIGHT,WINDOW_WIDTH,WINDOW_HEIGHT+MENUBAR_HEIGHT,"Omnivision");
	
	/* 0 */ _gl_main = new MyGlWindow(0,MENUBAR_HEIGHT,WINDOW_WIDTH-CONTROLBAR_WIDTH,WINDOW_HEIGHT); // main window
	/* 1 */ _gl_main->_menu_bar = new Fl_Menu_Bar(0, 0, WINDOW_WIDTH-CONTROLBAR_WIDTH, MENUBAR_HEIGHT); // menu bar
	/* 2 */ _gl_control = new MyControlBarWindow(WINDOW_WIDTH-CONTROLBAR_WIDTH,MENUBAR_HEIGHT,CONTROLBAR_WIDTH,\
			CONTROLBAR_HEIGHT); // control bar

	_gl_main->_control_bar = _gl_control;

	initMainWindow(_gl_main,_gl_control);

		// init the state machine
	SM_Init( _gl_main, _gl_main->MAINTENANCE_MAX_CORRESPONDENCES, _gl_main->MAINTENANCE_HISTORY_SIZE );

	_wind->begin();	

	_tempo = 0.0;
	Fl::add_timeout(_tempo, callback);

	_wind->end();
	_wind->show();

	/*_wind->begin();
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(WINDOW_WIDTH,WINDOW_HEIGHT);
	int ww = glutCreateWindow(argv[0]);
	LOG(LEVEL_INFO, "created window %d\n", ww);
	GLuint textures[10];
	glGenTextures( 10, textures );
	_wind->end();*/


	return Fl::run();
	//delete _gl_main;
	//delete _gl_control;

	//return 1;
}