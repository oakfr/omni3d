#ifndef _VIDEO_H__
#define _VIDEO_H__

#include "aviReader/visAviReader.h"
#include "basic.h"

#define MAX_COUNT 200
#define CORNER_QUALITY 0.001
#define CORNER_MIN_DISTANCE 5.0

class Video {
public:
	
	// ----------- AVI file components -----------------
	AviReader aviReader;
	HRESULT hr;
	WAVEFORMATEX wfx; 
	int nrows, ncols, nfrms;
	int pitch;
	CvPoint2D32f* corners;
	int corner_count;

	int width_video, height_video;
	double video_scale;

	unsigned int frame; // current frame displayed in the openGL window
	unsigned char* pixels;
	unsigned char* pixels_new;
	unsigned char* pixels_resized;

	IplImage *image, *gray, *eig_image, *temp_image;

	void Video::getImageData(unsigned char *data);
	void Video::initVideo(const char * fname, int, int, double);
	void Video::flipPixels();
	void Video::imageLoad();
	void Video::imageResize();
	void Video::dumpImage(const char *filename, unsigned char *buffer, int w, int h);
	void Video::detectCorners();
	void Video::moveToNextFrame();
	void Video::stop();
};


#endif
