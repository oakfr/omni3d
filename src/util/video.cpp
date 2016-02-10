#include "video.hpp"

void Video::getImageData(unsigned char *data)
{
    unsigned char *pix;
    aviReader.LockSurface(ncols, nrows, pix, pitch);
	
    memcpy(data, pix, nrows * pitch);
	
    aviReader.UnlockSurface();
}

void Video::initVideo(const char * fname, int width, int height, double scale) {
	nrows = 0;
	ncols = 0;
	nfrms = 0;
	pitch = 0;
	frame = 0;
	
	width_video = width;
	height_video = height;
	video_scale = scale;

	if (FAILED(hr = aviReader.Initialize(fname, FALSE)))
	{
		fprintf(stderr, "failed to open video file %s\n", fname);
		return;
	}
	aviReader.Start();
	ncols = aviReader.getWidth();
	nrows = aviReader.getHeight();
	pitch = aviReader.getPitch();
	nfrms = aviReader.getNumFrames();
	
	wfx = aviReader.getAudioFormat();
	
	// to make sure that image sizes are accurate!
	unsigned char *pix;
	aviReader.LockSurface(ncols, nrows, pix, pitch);
	//memcpy(data, pix, nrows * pitch);
	aviReader.UnlockSurface();
	
	pixels = new unsigned char[3*width_video*height_video];
	pixels_new = new unsigned char[3*width_video*height_video];
	pixels_resized = new unsigned char [3*width_video/video_scale*height_video/video_scale];
	corners = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(corners[0]));

	// create temporary images for corner detection
	CvSize imageSize;
	imageSize.width = width_video;
	imageSize.height = height_video;
	
	image = cvCreateImage(imageSize, IPL_DEPTH_8U, 3); // depth = 8, channels = 3
	gray = cvCreateImage (imageSize, IPL_DEPTH_8U, 1);
	eig_image = cvCreateImage (imageSize, IPL_DEPTH_32F, 1);
	temp_image = cvCreateImage (imageSize, IPL_DEPTH_32F, 1);

	printf("There are %d frames in the AVI\n", (int) aviReader.getNumFrames());
	printf("frameTime/duration %d/%d\n", (int) aviReader.getFrameTime(), (int) aviReader.getDuration());
	imageLoad();

	//image_orig = cvCreateImage(cvSize(width_image, height_image), IPL_DEPTH_8U, 3); // depth = 8, channels = 3
	//image_resized = cvCreateImage(cvSize(width_image/video_scale, height_image/video_scale), IPL_DEPTH_8U, 3); // depth = 8, channels = 3

}

void Video::flipPixels() {
	int i;
	
	for (i=height_image-1;i>=0;i--) {
		for (int j=0;j<width_image;j++) {
			pixels_new[3*(height_image-1-i)*width_image+3*j] = pixels[3*width_image*i+3*j+2];
			pixels_new[3*(height_image-1-i)*width_image+3*j+1] = pixels[3*width_image*i+3*j+1];
			pixels_new[3*(height_image-1-i)*width_image+3*j+2] = pixels[3*width_image*i+3*j];
		}
	}
}

void Video::imageLoad() {
	
	aviReader.Seek(frame);
	aviReader.Update();
	getImageData (pixels);
	flipPixels();
	imageResize();
}

void Video::stop() 
{
	aviReader.Stop();
}

void Video::moveToNextFrame() {
	frame = (frame + 1)%nfrms;
}

void Video::detectCorners() {
	
	//if (corners != NULL) cvFree((void**)&corners);
	
	cvSetImageData(image,pixels_new,width_image*3);
		
	cvCvtColor(image, gray, CV_BGR2GRAY);
	
	corner_count = MAX_COUNT;
	cvGoodFeaturesToTrack (gray, eig_image, temp_image, corners, &corner_count, CORNER_QUALITY, \
		CORNER_MIN_DISTANCE);
}

void Video::imageResize() {
	static IplImage *image_orig, *image_resized; 
	static int firstTime = 1;
	
	if (firstTime) {
		image_orig = cvCreateImage(cvSize(width_image, height_image), IPL_DEPTH_8U, 3); // depth = 8, channels = 3
		image_resized = cvCreateImage(cvSize(width_image/video_scale, height_image/video_scale), IPL_DEPTH_8U, 3); // depth = 8, channels = 3
		firstTime = 0;
	}
	cvSetImageData(image_orig,pixels_new,width_image*3);
	cvResize(image_orig,image_resized);
	//cvFlip(image_resized,NULL,1);
	pixels_resized = (unsigned char*)image_resized->imageData;
}

void Video::dumpImage(const char *filename, unsigned char *buffer, int w, int h) {
	IplImage* image = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3); // depth = 8, channels = 3
	cvSetImageData(image,pixels,w*3);
	cvSaveImage("toto.jpg",image);
}
