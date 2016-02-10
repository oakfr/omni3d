#include "main.h"

// an empty drawing function
//

const char *DEFAULT_DIRECTORY = "models.txt";


// the main computation function
//
void compute()
{
	progress_bar->minimum( 0 );
	progress_bar->maximum( _lut.nodes.size() );
	progress_bar->show();
	for (int i=0;i<dirnames.size();i++) {

		LOG(LEVEL_INFO, "processing %s...", dirnames[i].c_str() );

		// remove the visibility files
		_lut._model->_dirname = dirnames[i];
		fileRemove( _lut._model->toFilename(MODEL_VISI_FILE).c_str() );
		fileRemove( _lut._model->toFilename(MODEL_NODE_FILE).c_str() );

		//LOG(LEVEL_INFO, "removing file %s", _lut._model->toFilename(MODEL_VISI_FILE).c_str() );

		// init the LUT
		_lut.init(800,800,dirnames[i]);

		// init the viewer
		viewer.init(_lut._model->_centroid,_lut._model->_diameter,3*M_PI/4,M_PI/2,45,1.0,10000.0,OUTSIDE);

		// compute visibility
		_lut.automate( progress_bar );

		_lut.clear();

	}
	progress_bar->hide();
	exit(0);
}


void draw() {

	if ( _lut._node == NULL ) {
		if ( _lut._model == NULL )
			return;
		viewer._target = _lut._model->_centroid;
	} else {
		viewer._target = _lut._node->_pos;
	}

	viewer._spheric.phi = M_PI/2;
	viewer._spheric.theta = M_PI/2;
	
	viewer.clearWindow(true);
	viewer.setup3D();

	// draw the faces
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	viewer.clearWindow(true);

	// draw each face with a different color
	// and reset the visible flag to false
	int n = _lut._model->_faces.size();

	float *color;
	color = (float*)malloc(3*sizeof(float));
	for (int i=0;i<n; i++) {
		indexToColor( n, i, color );
		_lut._model->getFace( i )->draw( true, color );
		if ( !_lut._model->getFace( i )->_matched)
			_lut._model->getFace( i )->_visible = false;
		//int index;
		//colorToIndex( n, index, color );
		//printf( "%d -> (%f,%f,%f) -> %d\n", i, color[0],color[1],color[2], index );
		//assert( i == index );
	}

	delete color;

	glutSwapBuffers();

	compute();
}

// any key press starts the computation
// (except exit keys)
//
void keyboard ( unsigned char key, int x, int y )
{
	switch ( key ) {
	case 'q':
	case 'Q':
	case 27: // exit
		exit(0);
		break;
	default:
		compute();
		break;
	}
}

void init( char *dirname )
{
	char init_name[256];
	sprintf(init_name, "%s/config.ini", dirname);

	if ( fileExist( init_name ) ) {  // in this mode, the user inputs a single directory to process
		dirnames.push_back( dirname );
	} else { // in this mode, the user inputs a list of directories to process

		if ( !fileExist( dirname ) ) {
			LOG(LEVEL_ERROR, "%s does not exist!", dirname);
			exit(0);
		}

		std::ifstream inFile (dirname);
		std::string line;

		while (std::getline(inFile,line,'\n')) {
			//line.erase( line.size()-1, 1 );
			dirnames.push_back( line );
		}
	}

	// print out
	LOG(LEVEL_INFO, "list of directories to process");
	for (int i=0;i<dirnames.size();i++)
		LOG(LEVEL_INFO, "%s.", dirnames[i].c_str() );
}


// the main function
//
int main(int argc, char **argv) {
	
	// init glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glEnable(GL_DEPTH_TEST);

	// clear the visibility files
	_lut._model->cleanup( false, true );

	// read the input parameters
	if ( argc < 2 ) {
		printf("usage: viscomp <directory_name> (set parameters in config.ini file in model directory)\n");
		init ((char*)DEFAULT_DIRECTORY);
	} else {
		printf("directory: %s\n", argv[1]);
		init(argv[1]);
	}

	// init visibility window
	glutInitWindowPosition(150,200);
	glutInitWindowSize(300,300);
	glutCreateWindow("Visibility window");
	glutDisplayFunc(draw);
	glutKeyboardFunc(keyboard);

	// init the progress bar
	progress_bar = new Fl_Progress(150,200,150,20,"Progress");
	progress_bar->hide();
	
	glutMainLoop();

	return 0;
}

