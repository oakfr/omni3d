#include "basic.h"
#include "distorsion.h"
#include "math/numerical.h"
#include <math.h>

/*void Camera::resetPixels(unsigned char *pixels)
{
	for (int i=0;i<3*_width*_height;i++)
		pixels[i] = 0;
}


bool Camera::checkPixel (int x, int y)
{
	int x_ = MAX(0,MIN(x,_width-1));
	int y_ = MAX(0,MIN(y,_height-1));
	return ((x == x_) && (y == y_));
}

int Camera::getPixelIndex (int x, int y)
{
	assert(checkPixel(x,y));
	return 3*(_width*y+x);
}

void Camera::applyPixel (unsigned char *pixels, unsigned char *pixels_new, Vec2d x, Vec2d xd)
{
	int x_d = xd[0];
	int y_d = xd[1];
	
	int x_o = x[0];
	int y_o = x[1];
	
	if (!checkPixel(x_d,y_d))
		return;
	
	if (!checkPixel(x_o,y_o))
		return;
	
	pixels_new[getPixelIndex(x_d,y_d)] = pixels[getPixelIndex(x_o,y_o)];
	pixels_new[getPixelIndex(x_d,y_d)+1] = pixels[getPixelIndex(x_o,y_o)+1];
	pixels_new[getPixelIndex(x_d,y_d)+2] = pixels[getPixelIndex(x_o,y_o)+2];
}
void Camera::applyPixelColor (unsigned char *pixels, unsigned char *pixels_new, double error, Vec2d xd)
{
	int x_d = xd[0];
	int y_d = xd[1];
	
	if (!checkPixel(x_d,y_d)) {
		//printf("%d %d *",x_d,y_d);
		return;
	}
	
	pixels_new[getPixelIndex(x_d,y_d)] = MIN(255,int(error/25.0*255.0));
	pixels_new[getPixelIndex(x_d,y_d)+1] = 0;
	pixels_new[getPixelIndex(x_d,y_d)+2] = 0;
}
*/

/* Rectify a 2D image point */
Vec2d Camera::rectifyPixel (Vec2d x)
{
	Intrinsic intr = getIntrinsic();
	//intr.print();
	Vec2d xn = intr.normalize(x);
	Vec2d xd = intr.rectify(xn);
	Vec2d xd_im = intr.denormalize(xd);
	
	return xd_im;
}

/* Rectify a 2D image edge */
Edge2D Camera::rectifyEdge (Edge2D edge)
{
	Vec2d ar,br;
	
	ar = rectifyPixel(edge._a);
	br = rectifyPixel(edge._b);
	
	return Edge2D (ar,br,edge._id);
}

/*
void Camera::rectifyImages(std::vector<IplImage*> images, std::vector<IplImage*> output_images)
{
	PerfTimer timer;
	int nImages = images.size();
	if (nImages == 0)
		return;
	
	int w,h;
	
	assert (images.size() == output_images.size());
	
	for (int img=0;img<1;img++) {
		w = images[img]->width;
		h = images[img]->height;
		
		assert((w==_width) && (h=_height));
		setActiveSensor(img);
		rectifyImage(images[img], output_images[img]);
	}
	
	//printf("rectified 6 %dx%d images in %f seconds.\n",w,h,timer.elapsed());
	
}
*/

/* Rectify an image */
void Camera::rectifyImage (IplImage *image, IplImage *output_image)
{
	int w = image->width;
	int h = image->height;
	
	memset(output_image->imageData, 0, output_image->height*output_image->width*output_image->nChannels);
	
	assert((w==output_image->width) && (h==output_image->height));
	
	assert ((w == _width) && (h == _height)); // rotated image
	
	Vec2d X0 = rectifyPixel(Vec2d(h/2,0));
	Vec2d X1 = rectifyPixel(Vec2d(h-1,w/2));
	Vec2d X2 = rectifyPixel(Vec2d(h/2,w-1));
	Vec2d X3 = rectifyPixel(Vec2d(0,w/2));
	
	double w_x = X1[0]-X3[0];
	double w_y = X2[1]-X0[1];
	_rectification_display_scale = 1.3*MAX(w_x/h,w_y/w);
	_rectification_display_trans = rectifyPixel(Vec2d(h/20,0));

	for (int i=0;i<h;i++) {
		for (int j=0;j<w;j++) {

			Vec2d rp = rectifyPixel(Vec2d(i,j))-_rectification_display_trans;
			rp = rp/_rectification_display_scale;
			rp[0] = round(rp[0]);
			rp[1] = round(rp[1]);
			if (rp[0]<0) continue;
			if (rp[1]<0) continue;
			if (rp[0]>=output_image->height) continue;
			if (rp[1]>=output_image->width) continue;
			int offset_in = (i*image->width+j)*image->nChannels;
			int offset_out = (int(rp[0])*output_image->width+int(rp[1]))*output_image->nChannels;
			if (offset_out >= output_image->width*output_image->height*output_image->nChannels-4)
				continue;
			if (offset_out < 0)
				continue;
			memcpy(output_image->imageData+offset_out,image->imageData+offset_in,image->nChannels);
		}
	}
}
