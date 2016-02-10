#include "database.h"


/* Init a new database using default values
 */
void Database::initNewDatabase ( std::string dirname, double framerate )
{
	// date
	MyTimer timer;
	char date[256];
	sprintf( date, "%d%d%d", timer.tyear(), timer.tmonth(), timer.tday() );
	sscanf( date, "%d", &_date );

	// dirname
	_dirname = dirname;

	// width and height
	_width = 512;
	_height = 384;

	// frame rate
	_framerate = framerate;

	// camera type
	_cameraType = LADYBUG;

	// synthetic mode
	_synthetic = 0;

	// number of images
	_nImages = 0;

	// frame ID
	_frameId = 0;

	// bmp by default
	_filetype = std::string("bmp");

	temp = NULL;

	_ladybug_id = DEFAULT_LADYBUG_ID;

	// print for debug
	print();
}


/* Init an existing database by reading the config file
 */
void Database::init (std::string dirname)
{
	// init config file
	rude::Config config;
	std::string fullname = dirname + "/data.ini";
	printf("config file name: %s\n",fullname.c_str());
	bool res = config.load(fullname.c_str());
	assert( res );

	// read config info
	//_dirname = dirname;
	_date = config.getIntValue("DATE");
	_dirname = dirname;
	_width = config.getIntValue("WIDTH");
	_height = config.getIntValue("HEIGHT");
	_framerate = config.getDoubleValue("FRAME RATE");
	std::string cameraType = std::string(config.getStringValue("CAMERA"));
	_synthetic = config.getIntValue("SYNTHETIC");
	_filetype = config.getStringValue("FILETYPE");
	_ladybug_id = config.getIntValue( "LADYBUG_ID" );

	if (cameraType == "LADYBUG")
		_cameraType = LADYBUG;
	else if (cameraType == "CANON_ELURA")
		_cameraType = CANON_ELURA;
	else if (cameraType == "SONY_DFW_V500")
		_cameraType = SONY_DFW_V500;
	_nImages = config.getIntValue("N_IMAGES");
	_frameId=0;

	temp = NULL;
	print();
}

/* For synthetic datasets, read the pose in pose file for a given frame ID
 * return true for successful read, false otherwise
 */
bool Database::readPose( int frameId, ExtrinsicParameters &pose )
{
	char filename[256];
	sprintf(filename,"%s//%s", _dirname.c_str(),POSE_HISTORY_FILE);
	FILE *f = fopen( filename, "r" );
	if ( f == NULL )
		return false;

	while (!feof(f) ) {
		int id;
		double x,y,z,rx,ry,rz;
		if (fscanf(f,"%d%lf%lf%lf%lf%lf%lf", &id, &x, &y, &z, &rx, &ry, &rz ) == 7 ) {
			if ( id == frameId ) {
				pose.setTranslation( Vec3d(x,y,z) );
				Quaternion q = Quaternion( Vec3d(rz,ry,rz), len(Vec3d(rx,ry,rz)) );
				pose.setRotation( q );
				fclose( f );
				return true;
			}
		}
	}

	fclose( f );
	return false;
}

/* Print out a database to std output
 */
void Database::print()
{
	// print info
	printf("DATABASE:\n");
	printf("date: %d\n",_date);
	printf("location: %s\n",_dirname.c_str());
	printf("Image size: %d x %d\n",_width,_height);
	printf("Frame rate: %f\n",_framerate);
	if (_cameraType == LADYBUG)
		printf("Camera: LADYBUG\n");
	else if (_cameraType == CANON_ELURA)
		printf("Camera: CANON_ELURA\n");
	else if (_cameraType == SONY_DFW_V500)
		printf("Camera: SONY_DFW_V500\n");
	printf("# images: %d\n",_nImages);
	printf("Synthetic: %d\n",_synthetic);
}

/* Load a frame from the database
 */
void Database::loadFrame(Frame &frame)
{
	int sensorId;

	

	if (frame._cameraType == LADYBUG) {
		PerfTimer timer;

		for (sensorId=0;sensorId<6;sensorId++) {

			std::string fname = _dirname + "/" + getLadybugImageFilename(sensorId,_frameId);
			
			if (temp != NULL)
				cvReleaseImage(&temp); // important to avoid memory leaks
			temp = NULL;
			
			temp = cvLoadImage (fname.c_str(), 1); // 0: grayscale, 1: color
			cvCvtColor(temp,frame._grab_img[sensorId],CV_BGR2RGBA /*CV_BGR2BGRA*/);  // switch to CV_BGR2BGRA if you want a nice blue effect
				
			assert(temp != NULL);
		}
	}

	if ((frame._cameraType == CANON_ELURA)||(_cameraType == SONY_DFW_V500)) {
		
		sensorId = 0;
		std::string fname = util_toFilename(DATABASE_HOME,_dirname,getLadybugImageFilename(sensorId,_frameId));
		cvReleaseImage(&frame._img[sensorId]); // important to avoid memory leaks
		frame._img[sensorId] =  cvLoadImage (fname.c_str(), 1); // 0: grayscale, 1: color
		
		IplImage *temp = cvCreateImage(cvSize(frame._display_img[sensorId]->width, frame._display_img[sensorId]->height),IPL_DEPTH_8U, 3);
		IplImage *temp2 = cvCreateImage(cvSize(frame._display_img[sensorId]->width, frame._display_img[sensorId]->height),IPL_DEPTH_8U, 3);
		cvResize (frame._img[sensorId],temp); // resize the image
		cvConvertImage (temp, frame._display_img[sensorId], CV_CVTIMG_SWAP_RB | CV_CVTIMG_FLIP);
		
		cvReleaseImage(&temp);
		cvReleaseImage(&temp2);
		
	}

}

/* Next frame
 */
void Database::nextFrame()
{
	_frameId = (_frameId+1)%_nImages;
}

/* Prev frame
 */
void Database::prevFrame()
{
	_frameId--;

	if (_frameId<0)
		_frameId = _nImages-1;

	assert(_frameId >= 0);
}

/* Build Ladybug filename from sensor ID and frame ID
 */
std::string Database::getLadybugImageFilename (int sensorId, int frameId)
{	
	int offset = int (log(frameId)/log(10.0));
	char t1[20],t2[20];
	sprintf(t1,"%d",frameId);
	sprintf(t2,"%d",sensorId);

	std::string str = "";
	for (int i=0;i<5-offset;i++)
		str = str + "0";

	str = str + std::string(t1);

	str = str + "-cam" + std::string(t2) + "." + _filetype;//".bmp";

	return str;
}

/* Create database info file
 */
void Database::createDataFile(int resolution_code, char *dirname, int nimages, int synthetic)
{
	
	int w,h;
	switch (resolution_code) {
	case 2:
		w = 1024;
		h = 768;
		break;
	case 1:
		w = 512;
		h = 384;
		break;
	case 0:
		w = 256;
		h = 192;
		break;
	default:
		assert(false);
	}
	
	createDataInfoFile (dirname,  w,  h,  nimages, _framerate, synthetic);
	
}

/* Create database info file
 */
void Database::createDataFile ()
{
	createDataFile( 1, (char*)_dirname.c_str(), _nImages, _synthetic );
}
