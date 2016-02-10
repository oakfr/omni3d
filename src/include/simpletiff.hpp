// Based on libtiff documentation and libtiff tutorial at
// http://www.cs.wisc.edu/graphics/Courses/cs-638-1999/libtiff_tutorial.htm

#ifndef SIMPLETIFF_HPP
#define SIMPLETIFF_HPP

#include <string>
//#include "autocadreader.h"

class simpleTiff {
private:
  bool ownsBuffer;
//    int getshort( ifstream& infile );
 // int getIndex( int row, int col );
 // int getint( ifstream& infile );

public:
  int width;
  int height;
  unsigned char *buffer;

  simpleTiff();
  simpleTiff(int w, int h);
  simpleTiff(int w, int h, unsigned char *b);
  ~simpleTiff();
  bool write(std::string filename);
  bool read(std::string filename);
  //bool readBMP (const char * filename_in, const char * filename_out);

  int ncols, nrows;

};

#endif
