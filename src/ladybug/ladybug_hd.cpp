#include "my_ladybug.h"
#include "PGRBitmap.h"
#include <FL/Fl_Progress.H>

/* Get the number of images recorded on the HD
 */
int Ladybug::GetRecordedImages()
{
	if (!_init)
		return 0;
	
	LadybugError	  error;
	unsigned int n = 0;
	
	error = ::ladybugGetRecordedImages(_context, &n);
	
	assert(error == LADYBUG_OK);
	
	return n;
}

/* Empty the HD -- warning, this is a one-way trip...
 */
void Ladybug::emptyHD()
{
	if (!_init)
		return;
	
	LadybugError	  error;
	error = ::ladybugResetWritePointer(_context);
	
	if (error != LADYBUG_OK)
		printf("ERROR: failed to reset write pointer on Ladybug.\n");
}

/* Save images from HD to specific location
 */
int Ladybug::saveImagesFromHD (int start, int end, int skip, char *dest, int resolution_in, int resolution_out, Fl_Progress *progress)
{
	if (end < start)
		return 0;
	
	if (!_init)
		return 0;
	
	printf("saving %d images from HD: %d --> %d skipping every %d frames\n",end-start+1,start,end, skip);

	LadybugError	  error;
	int wo,ho;
	int w,h;
	switch (resolution_in) {
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
	switch (resolution_out) {
	case 2:
		wo = 1024;
		ho = 768;
		break;
	case 1:
		wo = 512;
		ho = 384;
		break;
	case 0:
		wo = 256;
		ho = 192;
		break;
	default:
		assert(false);
	}
	
	int k;	
	unsigned char *buffer;
	buffer = (unsigned char*)malloc(6*w*h*4*sizeof(unsigned char)); // six 4-channel images
	unsigned char *destBuffer[6];

	IplImage *image_size = cvCreateImage(cvSize(int(wo), int(ho)),  IPL_DEPTH_8U, 3);
	for ( k=0;k<6;k++)
		destBuffer[k] = (unsigned char*)malloc(w*h*4*sizeof(unsigned char));
				
	LadybugImageInfo imgInfo;
			
	LadybugImage ladybug_image;
	ladybug_image.uiCols = w;
	ladybug_image.uiRows = h;
	ladybug_image.dataFormat = _dataformat;
	ladybug_image.resolution = LADYBUG_RESOLUTION_1024x768;
	ladybug_image.pData = (unsigned char*)malloc(6*w*h*4*sizeof(unsigned char));


	int counter = 0;

	for (int img=start;img<=end;img++) {
		
		if ( skip > 0 && img % skip != 1 )
			continue;
		
		// extract image
		error = ladybugExtractImageFromHD(_context,img,buffer,&imgInfo);
		_HANDLE_ERROR;
		
		memcpy(ladybug_image.pData,buffer,6*w*h*4*sizeof(unsigned char));
		
		
		error = ladybugConvertToMultipleBGRU32( 
			_context, &ladybug_image, destBuffer, NULL );
		_HANDLE_ERROR;
		
		for (k=0;k<6;k++) {
			PGRBitmap bitmap( 
				w , h, 32, destBuffer[ k ] );
			
			char pszFilename[ _MAX_PATH ];
			sprintf( pszFilename, "%s/%06d-cam%d.bmp", dest, counter, k );
			
			if( !bitmap.saveImageToBMP( pszFilename ) )
				return counter;

			IplImage *image = cvLoadImage(pszFilename,1);
			cvResize(image,image_size);
			cvSaveImage(pszFilename,image_size);
			cvReleaseImage(&image);
		}

		progress->value(img);
		Fl::check();
		counter++;

		printf("--> %04d\n", img);
	}

	for (k=0;k<6;k++) {
		delete [] destBuffer[k];
		destBuffer[k] = NULL;
	}
	delete buffer;
	delete ladybug_image.pData;
	cvReleaseImage(&image_size);

	return counter;
	
}
