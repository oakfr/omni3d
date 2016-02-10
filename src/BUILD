System requirements
- Windows XP
- Microsoft Visual C++ 6.0
- Firewire port (to connect the Ladybug)


Download
Install the code in your C:/ directory :
% svn co svn+ssh://svn.csail.mit.edu/afs/csail.mit.edu/group/rvsn/repositories/omni3d

Installation
Install the following third-party software:
- ImageMagick 6.2.7 (from source)www.imagemagick.org
- OpenCV sourceforge.net/projects/opencvlibrary/
- VL Library 1.3.2 vl-1.3.2.zip
- PointGrey Ladybug API and calibration file.

For each of them, compile the source in Debug Multi-Threaded DLL mode.

The following two libraries are already compiled in the project. However, the GNU Scientific Library must be completely installed on the system.
- RudeConfig : rudeserver.com
- GNU Scientific Library for Windows: gnuwin32.sourceforge.net
For GSL, make sure that libgsl.a is in the C:/omni3d/lib/ directory (as opposed to libgsl.lib).

Build
Build the FLTK library in C:/omni3d/fltk-1.1.7/visualc/fltk.dsw.

Setup
Open the workspace framework.dsw in MSVC++ 6.0. Check that the project is linking to the following libraries:


Release mode:

kernel32.lib user32.lib gdi32.lib winspool.lib wsock32.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib opengl32.lib comctl32.lib geomR.lib cxcore.lib cv.lib highgui.lib vl.lib fltk.lib fltkgl.lib fltkimages.lib glu32.lib glaux.lib databaseR.lib toolboxR.lib mathR.lib linearR.lib modelR.lib signatureR.lib rudeconfig.lib timerR.lib my_ladybugR.lib ladybug.lib cameraR.lib vision_atlasR.lib libgsl.a libgslcblas.a siftR.lib 

Debug mode:

kernel32.lib user32.lib gdi32.lib winspool.lib wsock32.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib fltkd.lib fltkgld.lib fltkimagesd.lib opengl32.lib comctl32.lib geom.lib cxcore.lib cv.lib highgui.lib vl.lib glu32.lib glaux.lib toolbox.lib sift.lib math.lib linear.lib model.lib signature.lib rudeconfig.lib timer.lib my_ladybug.lib ladybug.lib fast.lib camera.lib vision_atlas.lib libgsl.a libgslcblas.a 

Check that the code is generated in Debug Multi-Threaded DLL mode.

Add the following paths to the include directories:

C:\Program Files\Microsoft Visual Studio\VC98\INCLUDE
C:\Program Files\Microsoft Visual Studio\VC98\MFC\INCLUDE
C:\Program Files\Microsoft Visual Studio\VC98\ATL\INCLUDE
C:\IMAGEMAGICK-6.3.0
C:\IMAGEMAGICK-6.3.0\VISUALMAGICK\BIN
C:\IMAGEMAGICK-6.3.0\MAGICK++
C:\IMAGEMAGICK-6.3.0\VISUALMAGICK\LIB
C:\IMAGEMAGICK-6.3.0\VISUALMAGICK\CONFIGURE
C:\IMAGEMAGICK-6.3.0\MAGICK++\LIB
C:\PROGRAM FILES\OPENCV\OTHERLIBS\_GRAPHICS\INCLUDE
C:\PROGRAM FILES\OPENCV\CXCORE\INCLUDE
C:\PROGRAM FILES\OPENCV\OTHERLIBS\HIGHGUI
C:/omni3d\SRC\INCLUDE
C:/omni3d\SRC
C:\VL-1.3.2\INCLUDE
C:\VL-1.3.2\INCLUDE\VL
C:/omni3d\FLTK-1.1.7
C:\PROGRAM FILES\POINT GREY RESEARCH\PGR LADYBUG\INCLUDE
C:/omni3d\VISION_ATLAS
C:/omni3d\VISION_ATLAS\MOOS\MOOSLIB
C:\PROGRAM FILES\GNUWIN32\INCLUDE
C:\PROGRAM FILES\OPENCV\CXCORE
C:\PROGRAM FILES\OPENCV\CV\INCLUDE

Add the following paths to the lib directories:

C:\Program Files\Microsoft Visual Studio\VC98\LIB
C:\Program Files\Microsoft Visual Studio\VC98\MFC\LIB
C:\IMAGEMAGICK-6.3.0\VISUALMAGICK\LIB
C:\IMAGEMAGICK-6.3.0
C:\IMAGEMAGICK-6.3.0\VISUALMAGICK\BIN
C:\IMAGEMAGICK-6.3.0\VISUALMAGICK\LIB
C:\PROGRAM FILES\OPENCV\OTHERLIBS\HIGHGUI
C:\PROGRAM FILES\OPENCV\LIB
C:/omni3d\SRC\LIB
C:\VL-1.3.2\RELEASE
C:\VL-1.3.2\DEBUG
C:\PROGRAM FILES\GNUWIN32\LIB
C:/omni3d\FLTK-1.1.7\LIB


Compilation
Build > Rebuild All...
