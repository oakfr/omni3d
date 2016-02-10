//=============================================================================
// 
// Author: Olivier Koch, koch@csail.mit.edu
//
// Date: 05-June-2007
//
// Reference: O. Koch and S. Teller, Wide-Area Localization from Known 3D Structure, CVPR '07
//
// Documentation for this code is available in my MSc Thesis:
// 
// http://rvsn.csail.mit.edu/omni3d
//
//=============================================================================

#include "my_ladybug.h"

/* constructor
 */
Ladybug::Ladybug ()
{
	_init = false;
	_resolution = LADYBUG_RESOLUTION_512x384;
	_dataformat = LADYBUG_DATAFORMAT_INTERLEAVED;
	_framerate = LADYBUG_FRAMERATE_7_5;
	_uiCols = 384;
	_uiRows = 512;
	_context = NULL;
	_config_loaded = false;
}

/* get resolution of the Ladybug
 */
void Ladybug::getResolution(int &w, int &h)
{
	switch (_resolution)
	{

	case LADYBUG_RESOLUTION_128x96:
		w = 96;
		h = 128;
		break;
	case LADYBUG_RESOLUTION_256x192:
		w = 192;
		h = 256;
		break;
	case LADYBUG_RESOLUTION_512x384:
		w = 384;
		h = 512;
		break;
	case LADYBUG_RESOLUTION_640x480:
		w = 480;
		h = 640;
		break;
	case LADYBUG_RESOLUTION_1024x768:
		w = 768;
		h = 1024;
		break;
	default:
		w = 384;
		h = 512;
		break;
	}
}

/* check whether a sensor is active
 */
bool Ladybug::isOneOfProcessingCameras(unsigned int uiProcessingCameras, unsigned int uiCamera)
{
	unsigned int thisProcessingCameraBit;
	switch( uiCamera )
	{
	case 0 : 
		thisProcessingCameraBit = LADYBUG_UNIT_0;
		break;
	case 1 : 
		thisProcessingCameraBit = LADYBUG_UNIT_1;
		break;
	case 2 : 
		thisProcessingCameraBit = LADYBUG_UNIT_2;
		break;
	case 3 : 
		thisProcessingCameraBit = LADYBUG_UNIT_3;
		break;
	case 4 : 
		thisProcessingCameraBit = LADYBUG_UNIT_4;
		break;
	default : // case 5 : 
		thisProcessingCameraBit = LADYBUG_UNIT_5;
		break;
	}
	
	return ( ( uiProcessingCameras & thisProcessingCameraBit ) > 0 );
}

/* rectify edges using PointGrey Research rectification table
 * turns out to be lower quality than our own rectification
 * see distortion.cpp
 */
void Ladybug::rectifyEdges (LadybugContext context, int cameraId, char *filename)
{
	LadybugError	  error;
	
	char *filenew = (char*)malloc(256*sizeof(char));
	sprintf(filenew,"%s_rect",filename);
	
	FILE *f = fopen(filename,"r");
	if (f == NULL) {
		delete filenew;
		return;
	}
	
	FILE *fo = fopen(filenew,"w");
	int id;
	double x1,y1,z1,x2,y2,z2;
	double x1r,y1r,x2r,y2r;
	while (fscanf(f,"%d%lf%lf%lf%lf%lf%lf",&id,&x1,&y1,&z1,&x2,&y2,&z2) == 7) {
		error = ::ladybugRectifyPixel(context,cameraId,y1,x1,&x1r,&y1r);
		_HANDLE_ERROR;
		error = ::ladybugRectifyPixel(context,cameraId,y2,x2,&x2r,&y2r);
		_HANDLE_ERROR;
		printf("%f %f  =>  %f %f\n",x1,y1,x1r,y1r);
		fprintf(fo,"%d %f %f %f %f %f %f\n",id,y1r,x1r,z1,y2r,x2r,z2);
	}
	
	fclose(f);
	fclose(fo);
	delete filenew;
}

/* load Ladybug calibration file
 */
void Ladybug::loadCalibrationFile()
{
	char	 pszConfigFileName[256];
	LadybugError	  error;
	LadybugCameraInfo	  ladybugCamInfo;
	error = ladybugGetCameraInfo( _context, &ladybugCamInfo );
	_HANDLE_ERROR;
	sprintf( pszConfigFileName, "../../config/ladybug%7d.cal", ladybugCamInfo.serialHead );

	loadCalibrationFile( pszConfigFileName );
}

/* load Ladybug calibration file
 */
void Ladybug::loadCalibrationFile( char *pszConfigFileName )
{
	if (_config_loaded)
		return;

	LadybugError	  error;
	printf( "Loading config %s.\n", pszConfigFileName );
	error = ladybugLoadConfig( _context, pszConfigFileName );
	if( error == LADYBUG_COULD_NOT_OPEN_FILE )
	{
		error = ladybugLoadConfig( _context, "config.ladybug" );
	}
	_HANDLE_ERROR;

	_config_loaded = true;

}

/* Get Ladybug extrinsic parameters
 */
void Ladybug::getUnitExtrinsic( int cameraId, double *parameters )
{
	LadybugError	  error;

	double *params = (double*)malloc(6*sizeof(double));
	error = ::ladybugGetCameraUnitExtrinsics (_context, cameraId, params);
	parameters[0] = params[0];
	parameters[1] = params[1];
	parameters[2] = params[2];
	parameters[3] = params[3];
	parameters[4] = params[4];
	parameters[5] = params[5];

	delete params;
}

/* Get Ladybug calibration info
 */
void Ladybug::getCalibrationInfo(int cameraId)
{
	LadybugError	  error;
	// load the appropriate ladybug configuration from file
	loadCalibrationFile();
		
	double *params = (double*)malloc(6*sizeof(double));
	error = ::ladybugGetCameraUnitExtrinsics (_context, cameraId, params);
	printf("Extrinsic camera %d: %f %f %f %f %f %f\n",cameraId,params[0],params[1],params[2],params[3],params[4],params[5]);
	delete params;
}

/* Init a Ladybug camera connected to the firewire port
 */
bool Ladybug::init ()
{
	_init = false;
	_frameId = 0;

	LadybugError	  error;
	int iCamera;
	//
	// Initialize context.
	//

	if (_context != NULL)
		ladybugDestroyContext( &_context);

	error = ::ladybugCreateContext( &_context );
	if (error != LADYBUG_OK)
		return false;
	_HANDLE_ERROR;
	
	//
	// Initialize the first ladybug on the bus.
	//
	error = ::ladybugInitializeFromIndex( _context, 0 );
	if (error != LADYBUG_OK)
		return false;
	_HANDLE_ERROR;
	
   	// Optional : comment out any units that are not needed, those images will not
	//	 be processed or saved
	unsigned int uiProcessingCameras = LADYBUG_UNIT_0 | LADYBUG_UNIT_1 | LADYBUG_UNIT_2 | LADYBUG_UNIT_3 | LADYBUG_UNIT_4 | LADYBUG_UNIT_5;
	
	//
	// Start up the camera
	//
	error = ::ladybugStart(
		_context,
		_resolution, 
		//LADYBUG_DATAFORMAT_PLANAR_JPEG, 
		_dataformat,
		//LADYBUG_DATAFORMAT_SEQUENTIAL,
		_framerate);
	//LADYBUG_FRAMERATE_ANY );
	if (error != LADYBUG_OK)
		return false;
	_HANDLE_ERROR;
	
	//
	// Set up the set of proceesing cameras
	// Note : by default without this call, the full set will be processed
	//
	error = ::ladybugSetProcessingCameras( _context, uiProcessingCameras );
	if (error != LADYBUG_OK)
		return false;
	_HANDLE_ERROR;
	
	::ladybugSetColorProcessingMethod( 
         _context, LADYBUG_RIGOROUS/*LADYBUG_NEAREST_NEIGHBOR_FAST*/ );

	_init = true;
	
	// allocate memory for the six images
	for (iCamera=0;iCamera<6;iCamera++)
		_arpBGRABuffers[ iCamera ] = new unsigned char[ _uiCols * _uiRows * 4 ];

	printf("done.\n");

	// uncomment this to get some useful info
	for (iCamera=0;iCamera<6;iCamera++)
		getCalibrationInfo(iCamera);

	return true;
	
}

/* Specify color processing mode
 */
bool Ladybug::setColorProcessingMethod (LadybugColorProcessingMethod method)
{

		LadybugError error = ::ladybugSetColorProcessingMethod( 
         _context, method);

		if (error != LADYBUG_OK) {
			printf("warning: setColorProcessingMethod failed.\n");
			return false;
		}

		return true;
}

/* Grab an image
 */
int Ladybug::grab()
{
	assert(_init);
	
	LadybugError	  error;
	LadybugImage	  image;
	
	unsigned int   uiCols = 0;
	unsigned int   uiRows = 0;
	unsigned int uiCamera = 0;
				
	ladybugUnlockAll( _context );
	error = ladybugLockNext( _context, &image );

	return 0;
}

/* Grab an image
 * warning: the ladybug only grabs images with 4 channels
 * use cvConvertImage to convert to 3 channels
 */
int Ladybug::grab(std::vector<IplImage*> images)
{
	assert(_init);
	
	LadybugError	  error;
	LadybugImage	  image;
	
	unsigned int   uiCols = 0;
	unsigned int   uiRows = 0;
	unsigned int uiCamera = 0;
				
	error = ::ladybugGrabImage( _context, &image );

	if (error != LADYBUG_OK) {
		_init = false;
		return 1;
	}
	_HANDLE_ERROR;
	
	// get the falloff correction parameters
	bool falloffCorrectionFlag = false;
	float falloffCorrectionValue = 0.0;

	error = ::ladybugSetFalloffCorrectionFlag( _context, true);

	error = ::ladybugGetFalloffCorrectionFlag( _context, &falloffCorrectionFlag);

	_HANDLE_ERROR;

	error = ::ladybugGetCorrectionAttenuation( _context, &falloffCorrectionValue );

	_HANDLE_ERROR;

	// read images from ladybug
	// (this is the most time-consuming part)

	error = ladybugConvertToMultipleBGRU32( 
      _context, &image, _arpBGRABuffers, NULL );
	_HANDLE_ERROR;

	uiCols = image.uiCols;
	uiRows = image.uiRows;
	
	assert(images.size() == 6);
	int img = 0;
	
	for (img=0;img<6;img++) {
		
		IplImage *im = images[img];
				
		// check image size
		assert(im->width == _uiRows);
		assert(im->height == _uiCols);
		
		cvSetImageData(im,(char*)(_arpBGRABuffers[img]),4*_uiRows);

	}

	_frameId++;

	return 0;
}

/* Terminate context
 */
void Ladybug::terminate()
{
	LadybugError	  error;
	
	   printf( "Destroying context.\n" );
	   error = ::ladybugDestroyContext( &_context );
	   _HANDLE_ERROR;
	   printf( "Done.\n" );
}

/* Get Ladybug filename
 */
std::string Ladybug::getLadybugImageFilename (int sensorId, int frameId)
{	
	int offset = int (log(frameId)/log(10.0));
	char t1[20],t2[20];
	sprintf(t1,"%d",frameId);
	sprintf(t2,"%d",sensorId);

	std::string str = ""; //"cam" + std::string(t2) + "/";
	for (int i=0;i<5-offset;i++)
		str = str + "0";

	str = str + std::string(t1);

	str = str + "-cam" + std::string(t2) + ".bmp";

	return str;
}

/* unproject point <x,y> on image (image width=w and height=h)
 * points are expressed in the ladybug coordinates frame
 * the size of the map is 1024x768
 */
bool Ladybug::unproject( int cameraId, int w, int h, double x, double y, 
						double &X, double &Y, double &Z )
{
	int _x = Round ( 1024.0 * x / w );
	int _y = Round ( 768.0 * y / h );

	if ( ( _x < 0 ) || ( _x >= 1024 ) )
		return false;
	if ( ( _y < 0 ) || ( _y >= 768 ) )
		return false;
	
	if ( ( cameraId < 0 ) || ( cameraId > 6 ) )
		return false;

	if ( _config_loaded == NULL )
		return false;

	LadybugPoint3d point = _map[cameraId]->ppoints[_x * 768 + _y];

	X = point.fX;
	Y = point.fY;
	Z = point.fZ;

	return true;
}

/* Project a 3D point onto a Ladybug camera
 */
bool Ladybug::project( int cameraId, int w, int h, double X, double Y, double Z,
					  double &x, double &y )
{
	if ( !_config_loaded )
		return false;

	LadybugError	  error;

	double row = 0.0, col=0.0;
	error = ::ladybugXYZtoRC( _context, X, Y, Z, cameraId, &row, &col, NULL );

	if (error != LADYBUG_OK) {
		return false;
	}

	if ( ( row < 0 ) || ( col < 0 ) )
		return false;

	row = row * h / 1024;
	col = col * w / 768;

	x = row;
	y = col;

	return true;
}

/* Integer round
 */
int Ladybug::Round( double a )
{
	int _a = int(a);

	if (a-_a > 0.5)
		return _a+1;
	else
		return a;
}