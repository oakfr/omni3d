#ifndef _LADYBUG_H__
#define _LADYBUG_H__

//=============================================================================
// System Includes
//=============================================================================
//#include <windows.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "basic.h"
#include "image.h"
#include <FL/Fl_Progress.H> // for the silly progress bar in saveImagesFromHD()
//=============================================================================
// PGR Includes
//=============================================================================

//=============================================================================
// Project Includes
//=============================================================================
#include "ladybug.h"
#include "ladybuggeom.h"

#define _HANDLE_ERROR \
   if( error != LADYBUG_OK ) \
   { \
      printf( \
	 "Error! Ladybug library reported %s\n", \
	  ::ladybugErrorToString( error ) ); \
      assert( false ); \
      exit( 1 ); \
   } \
\

class Ladybug {
public:

	// ** methods
	Ladybug();
	~Ladybug() {};

	bool Ladybug::isOneOfProcessingCameras(unsigned int uiProcessingCameras, unsigned int uiCamera);
	void Ladybug::rectifyEdges (LadybugContext context, int cameraId, char *filename);
	bool Ladybug::init ();
	bool Ladybug::setColorProcessingMethod (LadybugColorProcessingMethod method);
	void Ladybug::getCalibrationInfo(int cameraId);
	int Ladybug::GetRecordedImages();
	void Ladybug::getResolution(int &w, int &h);
	int Ladybug::saveImagesFromHD (int start, int end, int skip, char *dest, int resolution_in, int resolution_out, Fl_Progress *progress);
	void Ladybug::emptyHD();

	int Ladybug::grab(std::vector<IplImage*> images);
	int Ladybug::grab();
	void Ladybug::terminate();
	std::string Ladybug::getLadybugImageFilename (int sensorId, int frameId);
	bool unproject( int cameraId, int w, int h, double x, double y, double &X, double &Y, double &Z );
	bool project( int cameraId, int w, int h, double X, double Y, double Z, 
					  double &x, double &y );

	int Round( double a ); // used to round pixels
	void Ladybug::getUnitExtrinsic( int cameraId, double *parameters );
	void Ladybug::loadCalibrationFile( char *pszConfigFileName );

	// ** attributes
	bool _init;
	LadybugContext _context;
	LadybugResolution _resolution;
	LadybugDataFormat _dataformat;
	LadybugFramerate _framerate;
	int _uiCols, _uiRows;
	long int _frameId;
	unsigned char* _arpBGRABuffers[ LADYBUG_NUM_CAMERAS ];
	bool _config_loaded;
	void Ladybug::loadCalibrationFile();
	LadybugImage3d* _map[6]; // map from image to sphere

};

#endif
