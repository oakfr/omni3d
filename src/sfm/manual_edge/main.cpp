// this program allows a user to manually generate
// an edge file in the sfm format for 3D reconstruction
//
// input: a database of images, a camera (for rectification and intrinsic parameters)
// output: an edge file

#include "main.hpp"

void display()
{
	// clear the window
	//_viewer.clearWindow(true);

	// if temporary black screen, display nothing
	if (_blackScreen) {
		glutSwapBuffers();
		return;
	}

	// setup the 2D view
	_viewer.setup2D();

	// draw the image
	_viewer.drawImage (_camera._frame._display_img[_sensorId],0,0,1.0,true,false);
	
	// draw the edges
	if (_drawEdges)
		_viewer.drawEdges(_camera._frame._edges[0],_database._height,0,0,1.0,RED);

	// draw the selected edge
	if (_edgeIdValid) {
		if (_all_edges.size()%2 == 0)
			_viewer.drawEdge(_camera._frame._edges[0][_edgeId].flipOpenGL(_database._height),BLUE,3);
		else
			_viewer.drawEdge(_camera._frame._edges[0][_edgeId].flipOpenGL(_database._height),GREEN,3);
	}

	if (_manualEdgeValid) 
		_viewer.drawEdge(_manualEdge.flipOpenGL(_database._height),ORANGE,3);
			

	// draw the edges read in the log file
	if (_all_read_edges.size() > _database._frameId) {
		printf("drawing %d edges (read).\n",_all_read_edges[_database._frameId].size());
	_all_read_edges[_database._frameId][0].print();
		_viewer.drawEdges(_all_read_edges[_database._frameId],_database._height,0,0,1.0,RED);
	}

	// swap buffers
	glutSwapBuffers();
}

void processFrame ()
{
	// store the selected edge (actually, the rectified version!)
	if (_edgeIdValid||_manualEdgeValid) {

		Edge2D edge;

		if (_edgeIdValid)
			edge = _camera._frame._rectified_edges[0][_edgeId];
		else
			edge = _camera.rectifyEdge(_manualEdge);
		
		// flip edges for display
		edge._b[1] = _database._height-edge._b[1];
		edge._a[1] = _database._height-edge._a[1];

		int frameId;
		if (_database._frameId == 0)
			frameId = _database._nImages-1;
		else 
			frameId = _database._frameId - 1;

		Edge2D norm_edge = _camera.normalize(edge);
		norm_edge._id = frameId;
		_edges.push_back(norm_edge);
	}

	// if we have gone through all the frames, store the edge series
	if (!_edges.empty() && (_database._frameId == 0)) {

		_all_edges.push_back(_edges);
		_edges.clear();
	}

	_edgeIdValid = false;
	_manualEdgeValid = false;

	// clear the frame
	_camera._frame.clear();

	// load the frame
	_database.loadFrame(_camera._frame);

	// convert the image to grayscale
	//_camera._frame.convertToGrayscale(); 

	// detect the edges (or read them from memory)
	edge2DVector edges,rectified_edges;

	if (_save_edges.size() > _database._frameId) {
		edges = _save_edges[_database._frameId];
		rectified_edges = _save_rectified_edges[_database._frameId];
	} else {
		detectEdges(_camera._frame._img[_sensorId],edges);
		// rectify the edges
		_camera.rectifyEdges(edges,rectified_edges);
	}
	
	_camera._frame._edges.push_back(edges);
	_camera._frame._rectified_edges.push_back(rectified_edges);

		// save edges for optimization
	if (_save_edges.size() < _database._nImages) {
		_save_edges.push_back(edges);
		_save_rectified_edges.push_back(rectified_edges);
	}

	printf("frame ID = %d\n", _database._frameId);

}

void keyboard ( unsigned char key, int x, int y )
{	
	int i,j;

	switch ( key ) {
	case 'q':
	case 'Q':
	case 27: // exit
		exit(0);
		break;
	case ' ':
		if (_database._frameId == _database._nImages-1) {
			if (!_blackScreen) {
				_blackScreen = true;
				break;
			} else 
				_blackScreen = false;
		}

		_database.nextFrame(); // next frame
		processFrame();
		break;
	case '-':
		_database.prevFrame();  // prev frame
		processFrame();
		break;
	case 'd': // done: write down the edges
		writeEdgesVector (_all_edges, "33x.log", _database._nImages);
		break;
	case 'r':
		readEdgesVector (_all_read_edges, "33x.log");
		_camera._intr.print();
		for (i=0;i<_all_read_edges.size();i++) {
			for (j=0;j<_all_read_edges[i].size();j++) {
				Edge2D edge = _camera.denormalize(_all_read_edges[i][j]);
				// flip edges for display
				edge._a[1] = _database._height-edge._a[1];
				edge._b[1] = _database._height-edge._b[1];
				_all_read_edges[i][j] = edge;
			}
		}

		break;
	case 'e':
		_drawEdges = !_drawEdges;
		break;
	default:
		break;
	}
	_viewer.refresh();
}

void mousePicking (int button, int state, int x, int y)
{	
	_viewer.setMode(GL_SELECT);

	_viewer.setup2D(x,y);

	// draw the edges (in select mode, so edges are not actually displayed)
	_viewer.drawEdges(_camera._frame._edges[0],_database._height,0,0,1.0,RED);

	intVector hits;
	_viewer.stopPicking(hits);

	if (!hits.empty()) {
		if (_edgeIdValid && (hits[0] == _edgeId)) {
			_edgeIdValid = false;
		} else {
			_edgeId = hits[0];
			_edgeIdValid = true;
		}
	}
}

void mouse (int button, int state, int x, int y)
{
	if (state != GLUT_DOWN) return;

	if (button == GLUT_LEFT_BUTTON) {
		_manualEdgeValid = false;
		mousePicking(button, state, x, y);
	} else if (button == GLUT_RIGHT_BUTTON) {
		_edgeIdValid = false;
		
		if (_manualEdgeBegin) {
			_manualEdgeBegin = false;
			_manualEdge._a = Vec2d(x,y);
		} else {
			_manualEdgeBegin = true;
			_manualEdge._b = Vec2d(x,y);
			_manualEdgeValid = true;
		}
	}

	_viewer.refresh();
}

int main(int argc, char **argv)
{
	// init the database
	//_database.init("20050720_002_33x"); //20050906_001_33x
	//_database.init("20050907_smf_sample_blocks");
	_database.init("20050927_001_33x_testing");

	// init the sensor ID
	_sensorId = 0;
	_camera.setActiveSensor(_sensorId);

	// init the camera
	//_camera.init(LADYBUG,_database._width,_database._height,1.0);
	//_camera.init(CANON_ELURA,_database._width,_database._height,1.0);
	_camera.init(LADYBUG,_database._width,_database._height,1.0);

	// setup the frame
	//_camera._frame.set (LADYBUG, _database._width, _database._height, 6, 1.0);
	_camera._frame.set (LADYBUG, _database._width, _database._height, 1, 1.0);

	// process the first frame
	processFrame();

	// init parameters
	_edgeIdValid = false;
	_manualEdgeValid = false;
	_manualEdgeBegin = true;
	_blackScreen = false;
	_drawEdges = true;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	
	// init 3D viewer window
	glutInitWindowPosition(300,100);
	glutInitWindowSize(_database._width, _database._height);
	_viewer.set(_database._width, _database._height, glutCreateWindow("Manual Edge Tracker"));
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);

	// main loop
	glutMainLoop();

	return 0;
}
