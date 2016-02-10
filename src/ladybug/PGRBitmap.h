//=============================================================================
// Copyright © 2000 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: PGRBitmap.h,v 1.1 2002/10/15 19:13:13 mwhite Exp $
//=============================================================================
#ifndef __PGRBITMAP_H__
#define __PGRBITMAP_H__

//=============================================================================
// System Includes
//=============================================================================
#define WIN32_LEAN_AND_MEAN
//#include <windows.h>
#include <cstdio>
#include <cassert>

//=============================================================================
// PGR Includes
//=============================================================================


/**
 * This class encapsulates the necessary functions to draw a bitmap to a win32
 * window, plus some extra goodies.
 */
class PGRBitmap
{
public:

   //==========================================================================
   // Construction/Destruction
   //==========================================================================

   /**
    * Construct a bitmap of specified dimensions using the passed external
    * pointer.
    */
   PGRBitmap( 
      int	     iWidth, 
      int	     iHeight, 
      int	     iBitsPerPixel, 
      unsigned char* pImageData );

   /**
    * Default destructor
    */
   virtual ~PGRBitmap();

   
   //==========================================================================
   // Public Methods
   //==========================================================================
   
   /**
    * Returns a pointer to the data, for setting the image contents directly.
    * If the image size changes, a call to setImageDimensions() must also be 
    * made.
    *
    * @see setImageDimensions()
    */
   unsigned char* getDataPointer();

   /**
    * Allows the user to directly set the data pointer.  This should only be
    * if the bitmap does not own its own data.  This has similar functionality
    * to setBitmap().
    */
   BOOL setDataPointer( unsigned char* pBuffer );

   /**
    * Set the image dimensions.  Note that this does not change the buffer
    * size, it only changes the output size.
    */
   int setImageDimensions( int iWidth, int iHeight );

   /**
    * Retrieve the image dimensions.
    */
   int getImageDimensions( int&	iWidth, int& iHeight );

   /**
    * Set the bitmap from the specified parameters.  If the current memory
    * is owned by the bitmap, it will be freed.
    */
   void	 setBitmap( int iWidth, int iHeight, int iBitsPerPixel, unsigned char* pData );

   /**
    * Paint the current bitmap to a device.
    *
    * @param hDC  The handle to the device context.  Use GetSaveHDC() (or
    *             something) under win32.
    *
    * @param iDestXOrigin  Destination X origin in the window.  Use 0.
    * @param iDestYOrigin  Destination Y origin in the window.  Use 0.
    * @param iDestWidth	   Width to use.  This determintes stretching if 
    *                      necessary.  Don't supply this param.
    * @param iDestHeight   Height to use.  THis determines stretching if 
    *                      necessary.  Don't supply this param.
    */
   int paintToDevice( 
      HDC hDC, 
      int iDestXOrigin, 
      int iDestYOrigin,
      int iDestWidth = -1, 
      int iDestHeight = -1 );

   /**
    * Save image in bitmap format to the specified path and filename.
    * 
    * @note Must be in BGR (24 bit) or BGRU (32 bit) format.
    */
   BOOL saveImageToBMP( const char* pszFilename );

   /**
    * Save image in .ppm format to the specified path and filename.
    *
    * @note Assumes RGB format. (does not munge the bytes before writing to
    *       disk.)
    * @note Assumes 24 bit image.
    */
   BOOL saveImageToPPM( const char* pszFilename );

   /**
    * Save image in .ppm format to the specified path and filename.
    *
    * @note Must be in BGR (24 bit) or BGRU (32 bit) format.
    */
   BOOL saveImageBGRToPPM( const char* pszFilename );

   /**
    * Save an 8-bit image to PGM format.
    *
    * @note Assumes 8 bit image.
    */
   BOOL saveImageToPGM( const char* pszFilename );

   /**
    * Fills the current pointed-to memory with an "attractive" colour ramp.
    * @note Assumes 24 or 32 bits!
    */
   void	 fillWithColourRamp();

   /**
    * Fills the current pointed-to memory with an "attractive" b/w ramp.
    * @note Assumes 8 bits!
    *
    */
   void	 fillWithBWRamp();


protected:
   //=============================================================================
   // Private Methods
   //=============================================================================

   /**
    * Internal helper function.
    */
   void initBitmapInfo();

   /**
    * Internal helper function.
    */
   void initBitmapInfoHeader();


protected:
   //=============================================================================
   // Private Member Variables
   //=============================================================================
   
   /**
    * Whether this object owns the data or not (ie, whether it has created it and
    * will free it upon deletion.
    */
   BOOL	 m_bOwnsData;

   /**
    * Image bpp.
    */
   int	 m_iBitsPerPixel;

   /**
    * Image cols.
    */
   int	 m_iWidth;

   /**
    * Image rows.
    */
   int	 m_iHeight;

   /**
    * The image data, in any format.  For higher colour depths than 8, this
    * is always assumed to be BGR.  But really, it doesn't matter unless you're
    * displaying or saving images.
    */
   unsigned char*  m_pData;
   
   /**
    * Bitmap info structure for painting to a windows device handle.
    */
   BITMAPINFO* m_pBitmapInfo;
};


#endif // !__PGRBITMAP_H__
