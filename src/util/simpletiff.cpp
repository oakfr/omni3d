// Based on libtiff documentation and libtiff tutorial at
// http://www.cs.wisc.edu/graphics/Courses/cs-638-1999/libtiff_tutorial.htm

//#include <afxwin.h>
#include <string.h>
#include <tiffio.h>
#include "simpletiff.hpp"

//using namespace std;

simpleTiff::simpleTiff(int w, int h, unsigned char *b) {
  width = w;
  height = h;
  buffer = b;
  ownsBuffer = false;
}

simpleTiff::simpleTiff(int w, int h) {
  width = w;
  height = h;
  buffer = new unsigned char [w * h * 3];
  ownsBuffer = true;
}

simpleTiff::simpleTiff() {
  buffer = NULL;
  ownsBuffer = true;
}

simpleTiff::~simpleTiff() {
  if (ownsBuffer) delete [] buffer;
}

bool simpleTiff::write(std::string filename) {
  TIFF *tif;
  tif = TIFFOpen(filename.c_str(), "w");
  if (tif == NULL) return false;

  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, width);
  TIFFSetField(tif, TIFFTAG_IMAGELENGTH, height);
  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tif, width * 3));
  
  TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
  TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);

  tsize_t linebytes = width * 3;
  unsigned char *buf = NULL;
  if (TIFFScanlineSize(tif) < linebytes)
    buf = (unsigned char *)_TIFFmalloc(linebytes);
  else
    buf = (unsigned char *)_TIFFmalloc(TIFFScanlineSize(tif));
  
  for (int row = 0; row < height; row++) {
    memcpy(buf, &buffer[row * linebytes], linebytes);
    if (TIFFWriteScanline(tif, buf, row, 0) < 0)
      break;
  }

  // Close the file
  TIFFClose(tif);

  if (buf)
    _TIFFfree(buf);
  
  return true;
}

bool simpleTiff::read(std::string filename) {
  TIFF *tif;
  tif = TIFFOpen(filename.c_str(), "r");
  if (tif == NULL) return false;
  uint32 *ob;
  
  TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
  TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
  int npixels = width * height;
  ob = (uint32 *)_TIFFmalloc(npixels * sizeof(uint32));
  if (ob == NULL) return false;

  if (buffer != NULL) delete [] buffer;
  buffer = new unsigned char[width * height * 3];

  if (!TIFFReadRGBAImage(tif, width, height, ob, 0)) return false;

  for (int y = 0; y < height; y++)
    for (int x = 0; x < width; x++)
      memcpy(buffer + (width * y + x) * 3, ob + (width * (height - 1 - y) + x), 3);

  _TIFFfree(ob);
  TIFFClose(tif);
  return true;
}

/*
int simpleTiff::getint( ifstream& infile )
 {
 // get the 4 bytes:
 char a, b, c, d;
 infile.get(a);
 infile.get(b);
 infile.get(c);
 infile.get(d);
 // form the value using bigendian (little endian?) form:
 return ((d*256+c)*256+b)*256 + a;
}

int simpleTiff::getshort( ifstream& infile )
 {
 // get the 2 bytes:
 char a, b;
 infile.get(a);
 infile.get(b);
 // form the value using bigendian (little endian?) form:
 return b*256 + a;
 }

int simpleTiff::getIndex( int row, int col )
 {
 return row*ncols*4 + col*4;
 }

bool simpleTiff::readBMP (const char * filename_in, const char * filename_out) {
		
		GLubyte* pixels;

		ifstream infile;

		infile.open( filename_in, ios::binary );
		
		unsigned char code1, code2;
		infile >> code1;
		infile >> code2;
		
		GLuint fileSize = getint( infile );
		GLuint reserved = getint( infile );
		GLuint offset = getint( infile );
		GLuint headerSize = getint( infile );
		
		ncols = getint( infile );
		nrows = getint( infile );
		
		GLushort planes = getshort( infile );
		GLushort bitsPerPixel = getshort( infile );
		GLuint compression = getint( infile );
		GLuint dataSize = getint( infile );
		GLuint hRes = getint( infile );
		GLuint vRes = getint( infile );
		GLuint colors = getint( infile );
		GLuint important = getint( infile );
		
		cout << "found complete file size = " << fileSize << endl <<
			" reserved = " << reserved << endl <<
			" offset to bitmap data = " <<
			offset << endl;
		cout << "header size = " << headerSize << endl <<
			" width in pixels = " <<
			ncols << " height in pixels = " << nrows << endl;
		cout << " planes = " << planes << endl <<
			" bits per pixel = " << bitsPerPixel << endl;
		cout << " compression = " << compression << endl <<
			" data size = " << dataSize << endl <<
			" horiz resolution = " << hRes <<
			" vert res. = " << vRes << endl <<
			" colors = " << colors <<
			" number important colors = " << important << endl;
		
		// if format is not correct, stop, otherwise
		// grab all the values and store them in a suitable grid

		if( ! (
			nrows>0 && bitsPerPixel==24 && compression==0 &&
			important==0 && headerSize==40
			)
			)
		{// problem?
			cout << "Hmmm, this program can't process this bmp format" << endl;
			cout << nrows << "," << bitsPerPixel << "," << compression << "," << \
				important << "," << headerSize << endl;
			return false;
		}
			// grab the bitmap itself and store in array for convenience
			// (note: BMP format seems to use b g r order for a pixel)
			// provide space for A, but BMP doesn't include it
			pixels = new GLubyte[nrows*ncols*4];
		//GLubyte temp;
			char temp;
		
		int k, j;
		
		for( k=0; k<nrows; k++ )
			for( j=0; j<ncols; j++ )
			{
				// read 3 unsigned bytes for pixel at row k, col j
				// (GL_RGBA format uses order RGBA)
				infile.get( temp );
				pixels[ getIndex(k,j) + 2 ] = temp; // Blue
				infile.get( temp );
				pixels[ getIndex(k,j) + 1 ] = temp; // Green
				infile.get( temp );
				pixels[ getIndex(k,j) + 0 ] = temp; // Red
				
				// store 0 for A
				pixels[ getIndex(k,j) + 3 ] = 0; // A
			}
			
			infile.close();
			
			cout << "The BMP file " << filename_in << " has been read" << endl;
	
		write(filename_out);

  return true;
}*/

/*
void simpleTiff::readBmp (string filename_in, string filename_out, int width, int height) {
	//load Bitmap

HBITMAP hImage = (HBITMAP)LoadImage(NULL, filename_in.c_str(), IMAGE_BITMAP,
0, 0, LR_LOADFROMFILE|LR_CREATEDIBSECTION|LR_DEFAULTSIZE);

CBitmap* m_Bitmap = CBitmap::FromHandle(hImage);

//Memory allocation is still 600x600 in your code..
BYTE* bmpBuffer=(BYTE*)GlobalAlloc(GPTR, width*height);//allocate memory


// Size of bitmap as I draw by using x,y points...
m_Bitmap->GetBitmapBits(width*height ,bmpBuffer);


TIFF *image;

  // Open the TIFF file
  if((image = TIFFOpen(filename_out.c_str(), "w")) == NULL)
  {
    printf("Could not open output.tif for writing\n");
  }

  TIFFSetField(image, TIFFTAG_IMAGEWIDTH,width);
  TIFFSetField(image, TIFFTAG_IMAGELENGTH,height);
  TIFFSetField(image, TIFFTAG_BITSPERSAMPLE,8);
  TIFFSetField(image, TIFFTAG_SAMPLESPERPIXEL,1);

  uint32 rowsperstrip = TIFFDefaultStripSize(image, -1); 
  //<REC> gives better compression

  TIFFSetField(image, TIFFTAG_ROWSPERSTRIP, rowsperstrip);
  TIFFSetField(image, TIFFTAG_COMPRESSION, COMPRESSION_PACKBITS); 

// Start CCITTFAX3 setting
  
  uint32 group3options = GROUP3OPT_FILLBITS+GROUP3OPT_2DENCODING;
  TIFFSetField(image, TIFFTAG_GROUP3OPTIONS, group3options);
  TIFFSetField(image, TIFFTAG_FAXMODE, FAXMODE_CLASSF);
  TIFFSetField(image, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
  TIFFSetField(image, TIFFTAG_ROWSPERSTRIP, -1L);


// End CCITTFAX3 setting

  TIFFSetField(image, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);

  TIFFSetField(image, TIFFTAG_FILLORDER, FILLORDER_MSB2LSB);
  TIFFSetField(image, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
 
  TIFFSetField(image, TIFFTAG_RESOLUTIONUNIT, RESUNIT_INCH);
  TIFFSetField(image, TIFFTAG_XRESOLUTION, 100.0);
  TIFFSetField(image, TIFFTAG_YRESOLUTION, 100.0);

 
   char page_number[20];
    sprintf(page_number, "Page %d", 1);

    TIFFSetField(image, TIFFTAG_SUBFILETYPE, FILETYPE_PAGE);
    TIFFSetField(image, TIFFTAG_PAGENUMBER, 1, 1);
    TIFFSetField(image, TIFFTAG_PAGENAME, page_number);



  // Write the information to the file
BYTE *bits;
for (int y = 0; y < height; y++)
  {
    bits= bmpBuffer + y*width;
    //if (TIFFWriteScanline(image,bits, y, 0)==-1) cout << "Complete or error";
	TIFFWriteScanline(image,bits, y, 0);
  }

  // Close the file
  TIFFClose(image);
}*/
