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
// $Id: PGRBitmap.cpp,v 1.1 2002/10/15 19:13:13 mwhite Exp $
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include "stdafx.h"
#include <memory.h>

//=============================================================================
// PGR Includes
//=============================================================================

//=============================================================================
// Project Includes
//=============================================================================
#include "PGRBitmap.h"




PGRBitmap::PGRBitmap( 
		     int	    iWidth, 
		     int	    iHeight, 
		     int	    iBitsPerPixel, 
		     unsigned char* pImageData )
{
   assert( iBitsPerPixel % 8 == 0 );

   m_pData = (unsigned char*)pImageData;

   m_bOwnsData = FALSE;
   m_iBitsPerPixel = iBitsPerPixel;
   m_iWidth = iWidth;
   m_iHeight = iHeight;

   initBitmapInfo();
}


void
PGRBitmap::initBitmapInfo()
{
   //
   // If the colourdepth is 8 bits or lower, we need a colour palette 
   // embedded in the bitmap info structure.
   //

   if ( m_iBitsPerPixel == 8 )
   {
      // we'll need room for the colour palette.

      m_pBitmapInfo = 
	 (BITMAPINFO*)malloc( sizeof( BITMAPINFO ) + sizeof( RGBQUAD ) * 256 );
      assert( m_pBitmapInfo != NULL );

      //
      // assume that it's greyscale for now.
      //
      for( int i = 0; i < 256; i++ )
      {
	 m_pBitmapInfo->bmiColors[i].rgbBlue     = (unsigned char)i;
	 m_pBitmapInfo->bmiColors[i].rgbGreen    = (unsigned char)i;
	 m_pBitmapInfo->bmiColors[i].rgbRed      = (unsigned char)i;
	 m_pBitmapInfo->bmiColors[i].rgbReserved = (unsigned char)0;
      }
   }
   else
   {
      m_pBitmapInfo = (BITMAPINFO*)malloc( sizeof( BITMAPINFO )  );
      assert( m_pBitmapInfo != NULL );
   }

   m_pBitmapInfo->bmiHeader.biSize = sizeof( BITMAPINFOHEADER );
   m_pBitmapInfo->bmiHeader.biPlanes = 1;
   m_pBitmapInfo->bmiHeader.biCompression = BI_RGB;
   m_pBitmapInfo->bmiHeader.biXPelsPerMeter = 100;
   m_pBitmapInfo->bmiHeader.biYPelsPerMeter = 100;
   m_pBitmapInfo->bmiHeader.biClrUsed = 0;
   m_pBitmapInfo->bmiHeader.biClrImportant = 0;

   initBitmapInfoHeader();
}


void
PGRBitmap::initBitmapInfoHeader()
{
   m_pBitmapInfo->bmiHeader.biWidth	  = m_iWidth;
   m_pBitmapInfo->bmiHeader.biHeight	  = -m_iHeight; // top-down bitmap, negative height
   m_pBitmapInfo->bmiHeader.biBitCount	  = (unsigned short)m_iBitsPerPixel;
   m_pBitmapInfo->bmiHeader.biSizeImage	  = ::abs( m_iWidth * m_iHeight );
}


PGRBitmap::~PGRBitmap()
{
   if( m_pData != NULL && m_bOwnsData == TRUE )
   {
      delete[] m_pData;
      m_pData = NULL;
   }

   if( m_pBitmapInfo != NULL )
   {
      free( m_pBitmapInfo );
      m_pBitmapInfo = NULL;
   }
}


void	 
PGRBitmap::setBitmap( 
		     int	    iWidth, 
		     int	    iHeight, 
		     int	    iBitsPerPixel, 
		     unsigned char* pData )
{
   assert( pData != NULL );

   m_iBitsPerPixel   = iBitsPerPixel;   
   m_iWidth	     = iWidth;
   m_iHeight	     = iHeight;

   initBitmapInfoHeader();

   if( m_bOwnsData )
   {
      delete[] m_pData;
   }

   m_pData = (unsigned char*)pData;

   m_bOwnsData = FALSE;
}


int
PGRBitmap::paintToDevice(
			 HDC  hDC,
			 int  iDestXOrigin, 
			 int  iDestYOrigin, 
			 int  iDestWidth, 
			 int  iDestHeight )
{
   if ( hDC == (HDC)NULL )
   {
      return 0;
   }

   if ( iDestWidth == -1 )
   {
      iDestWidth = m_pBitmapInfo->bmiHeader.biWidth;
   }
   if ( iDestHeight == -1 )
   {
      iDestHeight = ::abs( m_pBitmapInfo->bmiHeader.biHeight );
   }
   
   int iAbsHeight = ::abs( m_pBitmapInfo->bmiHeader.biHeight );
   
   if ( iDestWidth == m_pBitmapInfo->bmiHeader.biWidth && iDestHeight == iAbsHeight )
   {
      
      return ::SetDIBitsToDevice(
         hDC,
         iDestXOrigin, 
	 iDestYOrigin,
         m_pBitmapInfo->bmiHeader.biWidth, 
	 iAbsHeight,
         0, 
	 0,
         0, 
	 iAbsHeight,
	 m_pData,
	 m_pBitmapInfo, 
	 DIB_RGB_COLORS );
   }
   else
   {
      //
      // Set the stretching mode - the default mode screws up the colour
      // palette.
      //
      ::SetStretchBltMode( hDC, COLORONCOLOR );

      return ::StretchDIBits(
         hDC,
         iDestXOrigin,
         iDestYOrigin,
         iDestWidth,
         iDestHeight,
         0,
         0,
         m_pBitmapInfo->bmiHeader.biWidth, 
	 iAbsHeight,
	 m_pData, 
         m_pBitmapInfo, 
         DIB_RGB_COLORS,
         SRCCOPY );
   }
}


BOOL
PGRBitmap::saveImageToPPM( const char* pszPpmFileName )
{
   //
   //  myk revisit - this method assumes 24 bpp RGB.
   //
   assert( m_iBitsPerPixel == 24 );

   if( pszPpmFileName == NULL )
   {
      return FALSE;
   }

   int iWidth = m_pBitmapInfo->bmiHeader.biWidth;
   int iHeight = ::abs( m_pBitmapInfo->bmiHeader.biHeight );

   FILE* pfile = fopen( pszPpmFileName, "wb" );
   if( pfile )
   {
      ::fprintf( pfile, "P6\n%d %d\n255\n", iWidth, iHeight );
      ::fwrite( ((RGBTRIPLE*)m_pData), iWidth * iHeight * sizeof( RGBTRIPLE ), 1, pfile );
      ::fclose( pfile );

      return TRUE;
   }

   return FALSE;
}


BOOL 
PGRBitmap::saveImageBGRToPPM( const char* pszFilename )
{
   if( pszFilename == NULL )
   {
      return FALSE;
   }

   if( m_iBitsPerPixel != 24 && m_iBitsPerPixel != 32 )
   {
      assert( false );
      return FALSE;
   }

   const int iWidth = m_pBitmapInfo->bmiHeader.biWidth;
   const int iHeight = ::abs( m_pBitmapInfo->bmiHeader.biHeight );
   const int iPixels = iWidth * iHeight;
   assert( iPixels > 0 );

   FILE* pfile;

   if( ( pfile = ::fopen( pszFilename, "wb" ) ) == NULL )
   {
      assert( false );
      return FALSE;
   }
   
   ::fprintf( 
      pfile, 
      "P6\n%d %d\n255\n",
      iWidth,
      iHeight  );
   
   if( m_iBitsPerPixel == 24 )
   {
      //
      // Write out bytes in RGB order for .ppm.
      //
      for( int iPixel = 0; iPixel < iPixels; iPixel++ )
      {
	 const unsigned char* pPixel = &m_pData[ iPixel * 3 ];
	 
	 // 
	 // Write BGR pixel in the correct order (RGB) to .PPM.
	 //
	 ::fwrite( pPixel + 2, 1, 1, pfile );	
	 ::fwrite( pPixel + 1, 1, 1, pfile );	
	 ::fwrite( pPixel + 0, 1, 1, pfile );
      }
   }
   else if( m_iBitsPerPixel == 32 )
   {
      //
      // Write out bytes in RGB order for .ppm.
      //
      for( int iPixel = 0; iPixel < iPixels; iPixel++ )
      {
	 const unsigned char* pPixel = &m_pData[ iPixel * 4 ];
	 
	 // 
	 // Write BGR pixel in the correct order (RGB) to .PPM.
	 //
	 ::fwrite( pPixel + 2, 1, 1, pfile );	
	 ::fwrite( pPixel + 1, 1, 1, pfile );	
	 ::fwrite( pPixel + 0, 1, 1, pfile );
      }
   }
   else
   {
      assert( false );
   }  
   
   ::fclose( pfile );

   return TRUE;
}


BOOL 
PGRBitmap::saveImageToPGM( const char* pszFilename )
{
   assert( m_iBitsPerPixel == 8 );

   if( pszFilename == NULL )
   {
      return FALSE;
   }

   const int iWidth = m_pBitmapInfo->bmiHeader.biWidth;
   const int iHeight = ::abs( m_pBitmapInfo->bmiHeader.biHeight );

   FILE* pfile;

   if( ( pfile = ::fopen( pszFilename, "wb" ) ) == NULL )
   {
      assert( false );
      return FALSE;
   }

   ::fprintf( 
      pfile, 
      "P5\n%d %d\n255\n",
      iWidth,
      iHeight  );

   ::fwrite( m_pData, iWidth * iHeight, 1, pfile );
   ::fclose( pfile );

   return TRUE;
}


BOOL 
PGRBitmap::saveImageToBMP( const char* pszFilename )
{
   if( pszFilename == NULL )
   {
      return FALSE;
   }
   
   if( m_iBitsPerPixel != 24 && m_iBitsPerPixel != 32 )
   {
      assert( false );
      return FALSE;
   }
   
   BITMAPFILEHEADER bfh;
   BITMAPINFOHEADER bih;
   
   bfh.bfType = (WORD)(('M'<<8) + 'B');
   bfh.bfReserved1 = 0;
   bfh.bfReserved2 = 0;

   //
   // The wierd %4 stuff here is to pad things out for windows BMP
   // NOTE: THIS MAKES NO SENSE BUT WORKS!!!
   //
   bfh.bfSize =
      sizeof( BITMAPFILEHEADER )
      + sizeof( BITMAPINFOHEADER ) 
      + ::abs((m_pBitmapInfo->bmiHeader.biHeight+(m_pBitmapInfo->bmiHeader.biWidth%4))*m_pBitmapInfo->bmiHeader.biWidth) 
      * sizeof( RGBTRIPLE );
   
   bfh.bfOffBits = sizeof( BITMAPFILEHEADER ) + sizeof( BITMAPINFOHEADER );
   
   bih = m_pBitmapInfo->bmiHeader;
   bih.biHeight = ::abs( bih.biHeight ); // Positive height
   
   FILE *outfile = fopen( pszFilename, "wb" );
   if( outfile == NULL )
   {
      return FALSE;
   }
   
   ::fwrite( &bfh, sizeof( BITMAPFILEHEADER ), 1, outfile );
   ::fwrite( &bih, sizeof( BITMAPINFOHEADER ), 1, outfile );
   

   if( bih.biBitCount == 24 )
   {
      for( int i = bih.biHeight - 1; i >= 0; i-- )
      {
	 ::fwrite(
	    &( (RGBTRIPLE*)m_pData )[ i * bih.biWidth ],
	    bih.biWidth * sizeof( RGBTRIPLE ),
	    1,
	    outfile );
	 //
	 // The wierd %4 stuff here is to pad things out for windows BMP
	 // NOTE: THIS MAKES NO SENSE BUT WORKS!!!
	 //
	 ::fwrite( m_pData, ( bih.biWidth % 4 ), 1, outfile );
      }      
   }      
   else if( bih.biBitCount == 32 )
   {
      for( int i = bih.biHeight - 1; i >= 0; i-- )
      {
	 ::fwrite(
	    &( (RGBQUAD*)m_pData )[ i * bih.biWidth ],
	    bih.biWidth * sizeof( RGBQUAD ),
	    1,
	    outfile );
	 //
	 // The wierd %4 stuff here is to pad things out for windows BMP
	 // NOTE: THIS MAKES NO SENSE BUT WORKS!!!
	 //
	 ::fwrite( m_pData, ( bih.biWidth % 4 ), 1, outfile );
      }      
   }
   else
   {
      assert( false );
   }  
   
   ::fclose( outfile );
   
   return TRUE;
}


unsigned char*
PGRBitmap::getDataPointer()
{
   return m_pData;
}


BOOL 
PGRBitmap::setDataPointer( unsigned char* pBuffer )
{
   if( pBuffer == NULL )
   {
      assert( false );
      return FALSE;
   }

   if( m_bOwnsData )
   {
      assert( false );
      return FALSE;
   }

   m_pData = pBuffer;

   return TRUE;
}


BOOL
PGRBitmap::setImageDimensions( int iWidth, int iHeight )
{
   if( iWidth < 0 || iHeight < 0 )
   {
      assert( FALSE );
      return FALSE;
   }
   
   m_iWidth = iWidth;
   m_iHeight = iHeight;

   // New dimentions; set infoheader information
   
   m_pBitmapInfo->bmiHeader.biWidth	  = iWidth;
   m_pBitmapInfo->bmiHeader.biHeight	  = -iHeight; // top-down bitmap, negative height
   m_pBitmapInfo->bmiHeader.biSizeImage	  = ::abs( iWidth * iHeight );

   return TRUE;
}


BOOL
PGRBitmap::getImageDimensions( int& nWidth, int& nHeight )
{
   nWidth = m_pBitmapInfo->bmiHeader.biWidth;
   nHeight = ::abs( m_pBitmapInfo->bmiHeader.biHeight );
   return TRUE;
}


void	 
PGRBitmap::fillWithColourRamp()
{
   if( m_iBitsPerPixel != 24 && m_iBitsPerPixel != 32 )
   {
      assert( false );
      return;
   }

   if ( m_pData == NULL )
   {
      assert( FALSE );
      return;
   }

   int	 iWidth;
   int	 iHeight;
   getImageDimensions( iWidth, iHeight );

   if( m_iBitsPerPixel == 24 )
   {
      for( int iRow = 0; iRow < iHeight; iRow++ )
      {
	 unsigned char ucPixel = (unsigned char)( iRow % 256 );
	 
	 for( int iCol = 0; iCol < iWidth; iCol++ )
	 {
	    m_pData[ 3 * (iRow * iWidth + iCol) ] = (unsigned char)iCol;
	    m_pData[ 3 * (iRow * iWidth + iCol) + 1 ] = 0x0;
	    m_pData[ 3 * (iRow * iWidth + iCol) + 2 ] = ucPixel;
	 }
      }
   }
   else if ( m_iBitsPerPixel == 32 )
   {
      for( int iRow = 0; iRow < iHeight; iRow++ )
      {
	 unsigned char ucPixel = (unsigned char)( iRow % 256 );
	 
	 for( int iCol = 0; iCol < iWidth; iCol++ )
	 {
	    m_pData[ 4 * (iRow * iWidth + iCol) ] = (unsigned char)iCol;
	    m_pData[ 4 * (iRow * iWidth + iCol) + 1 ] = 0x0;
	    m_pData[ 4 * (iRow * iWidth + iCol) + 2 ] = ucPixel;
	 }
      }
   }
   else
   {
      assert( false );
   }
}


void	 
PGRBitmap::fillWithBWRamp()
{
   if( m_iBitsPerPixel != 8  )
   {
      assert( false );
      return;
   }

   if ( m_pData == NULL )
   {
      assert( FALSE );
      return;
   }

   int	 iWidth;
   int	 iHeight;
   getImageDimensions( iWidth, iHeight );
   
   int iMaxValue = 256;
   
   for( int iRow = 0; iRow < iHeight; iRow++ )
   {
      unsigned char ucPixel = (unsigned char)(iRow % iMaxValue );
      
      for( int iCol = 0; iCol < iWidth; iCol++ )
      {
	 m_pData[ iRow*iWidth + iCol ] = ucPixel;
      }
   }
}


