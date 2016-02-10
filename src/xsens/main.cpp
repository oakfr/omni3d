/*+==========================================================================
  File:      Xbus Example.cpp

  Summary:   Example code for reading MTi/MTx Data

             This code uses the Xsens Xbus Class v1.1
			 
  Functions: main
			   Gets user inputs, initializes xbus class, reads MTi/MTx data from
			   COM port and writes to screen.

----------------------------------------------------------------------------
  This file is part of the Xsens SDK Code Samples.

  Copyright (C) Xsens Technologies B.V., 2005.  All rights reserved.

  This source code is intended only as a supplement to Xsens
  Development Tools and/or documentation.  See these other
  materials for detailed information regarding Xsens code samples.

  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
  KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  PARTICULAR PURPOSE.
==========================================================================+*/


#ifndef WIN32
#include <sys/ioctl.h>
#endif

#include "Xbus.h"
#include <time.h>
#include <math.h>

#define XSENS_LOG_FILE "C:/omni3d/src/framework/xsens.dat"
#define RADTODEG 57.295779513082

CXbus xbus;
int portNumber;
char deviceName[15];
int outputMode;
int outputSettings;

double _fx, _fy, _fz;
double _calib_yaw;
double _dt;

LARGE_INTEGER _ticksPerSecond;
LARGE_INTEGER _start;

//////////////////////////////////////////////////////////////////////////
// gotoxy
//
// Sets the cursor position at the specified console position
//
// Input
//	 x	: New horizontal cursor position
//   y	: New vertical cursor position
void gotoxy(int x, int y)
{
#ifdef WIN32
	COORD coord;
	coord.X = x;
	coord.Y = y;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
#else
	char essq[100];		// String variable to hold the escape sequence
    char xstr[100];		// Strings to hold the x and y coordinates
    char ystr[100];		// Escape sequences must be built with characters

    /*
    ** Convert the screen coordinates to strings
    */
    sprintf(xstr, "%d", x);
    sprintf(ystr, "%d", y);

    /*
    ** Build the escape sequence (vertical move)
    */
    essq[0] = '\0';
    strcat(essq, "\033[");
    strcat(essq, ystr);

    /*
    ** Described in man terminfo as vpa=\E[%p1%dd
    ** Vertical position absolute
    */
    strcat(essq, "d");

    /*
    ** Horizontal move
    ** Horizontal position absolute
    */
    strcat(essq, "\033[");
    strcat(essq, xstr);
    // Described in man terminfo as hpa=\E[%p1%dG
    strcat(essq, "G");

    /*
    ** Execute the escape sequence
    ** This will move the cursor to x, y
    */
    printf("%s", essq);
#endif
}

//////////////////////////////////////////////////////////////////////////
// clrscr
//
// Clear console screen
void clrscr() 
{
#ifdef WIN32
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD coord = {0, 0};
	DWORD count;
	
	GetConsoleScreenBufferInfo(hStdOut, &csbi);
	FillConsoleOutputCharacter(hStdOut, ' ', csbi.dwSize.X * csbi.dwSize.Y, coord, &count);
	SetConsoleCursorPosition(hStdOut, coord);
#else
	int i;

    for (i = 0; i < 100; i++)
    // A bunch of new lines for now. It's blank, hey!
		putchar('\n');
	gotoxy(0,0);
#endif
}

#ifndef WIN32
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif

void
xsens_setParameters ( int port )
{
	portNumber = port;
	outputMode = 3;
	outputMode <<= 1;
	outputSettings = /*OUTPUTSETTINGS_QUATERNION*/ OUTPUTSETTINGS_EULER;
	outputSettings |= OUTPUTSETTINGS_SAMPLECNT;
}

//////////////////////////////////////////////////////////////////////////
// getUserInputs
//
// Request user for output data
void getUserInputs()
{
	clrscr();

#ifdef WIN32
	printf("Enter COM port: ");
	scanf("%d", &portNumber);
#else
	printf("Enter device name (eg /dev/ttyS0 or /dev/ttyUSB0): ");
	scanf("%s", deviceName);
#endif
	clrscr();

	do{
		printf("Select desired output:\n");
		printf("1 - Calibrated data\n");
		printf("2 - Orientation data\n");
		printf("3 - Both Calibrated and Orientation data\n");
		printf("Enter your choice: ");
		scanf("%d", &outputMode);
		// flush stdin
		while (getchar() != '\n') continue;

		if (outputMode < 1 || outputMode > 3) {
			printf("\n\nPlease enter a valid output mode\n");
		}
	}while(outputMode < 1 || outputMode > 3);
	clrscr();

	// Update outputMode to match data specs of SetOutputMode
	outputMode <<= 1;

	if ((outputMode & OUTPUTMODE_ORIENT) != 0) {
		do{
			printf("Select desired output format\n");
			printf("1 - Quaternions\n");
			printf("2 - Euler angles\n");
			printf("3 - Matrix\n");
			printf("Enter your choice: ");
			scanf("%d", &outputSettings);
			// flush stdin
			while (getchar() != '\n') continue;

			if (outputSettings < 1  || outputSettings > 3) {
				printf("\n\nPlease enter a valid choice\n");
			}
		}while(outputSettings < 1 || outputSettings > 3);

		// Update outputSettings to match data specs of SetOutputSettings
		switch(outputSettings) {
		case 1:
			outputSettings = OUTPUTSETTINGS_QUATERNION;
			break;
		case 2:
			outputSettings = OUTPUTSETTINGS_EULER;
			break;
		case 3:
			outputSettings = OUTPUTSETTINGS_MATRIX;
			break;
		}
	}
	else{
		outputSettings = 0;
	}
	outputSettings |= OUTPUTSETTINGS_SAMPLECNT;
	clrscr();
}

//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global xbus class
bool doMtSettings(void) 
{
	// Put MTi/MTx in Config State
	if(xbus.writeMessage(MID_GOTOCONFIG) != XBRV_OK){
		printf("No device connected\n");
		return false;
	}

	// Request DID
	unsigned long value = 0xFFFFFFFF;
	xbus.reqSetting(MID_REQDID, value);
	if ((value & DID_TYPEH_MASK) != DID_TYPEH_MTI_MTX) {
		printf("MTi / MTx has not been detected\n");
		return false;
	}

	if (xbus.setDeviceMode(outputMode, outputSettings) != XBRV_OK) {
		printf("Could not set device mode\n");
		return false;
	}

	// Put MTi/MTx in Measurement State
	xbus.writeMessage(MID_GOTOMEASUREMENT);

	return true;
}

//////////////////////////////////////////////////////////////////////////
// writeHeaders
//
// Write appropriate headers to screen
void xsens_writeHeaders( )
{
	//gotoxy(0,2);
	unsigned char data[MAXMSGLEN];
	float fdata[18];
	short datalen;
	unsigned short samplecounter;
	

	if ( xbus.readDataMessage(data, datalen) != XBRV_OK ) {
		printf("error reading Xsens data...\n");
		return;
	}

	xbus.getValue(VALUE_SAMPLECNT, samplecounter, data, BID_MASTER);
			
	/*xbus.getValue(VALUE_ORIENT_QUAT, fdata, data);

	double s = fdata[0];
	
	double a = acos(s);

	if ( fdata[3] < 0 ) 
		a = -a;

	//printf("[%d] %6.3f %6.3f %6.3f %6.3f\n", samplecounter, fdata[0], fdata[1], fdata[2], fdata[3]);
	printf( "orient: %6.3f\n", a * 180.0 / 3.14159265359);
	
	return;
	*/	
	xbus.getValue(VALUE_CALIB_GYR, fdata, data);

	float fx = _fx + fdata[0] * _dt;
	float fy = _fy + fdata[1] * _dt;
	double fz = _fz + ( fdata[2] - _calib_yaw ) * _dt;

	// compute elapsed time
	//LARGE_INTEGER end;
	//QueryPerformanceCounter(&end);

	//LONGLONG diff = end.QuadPart - _start.QuadPart;
	//double dt = (double)(diff)/_ticksPerSecond.QuadPart;

	//fz += 0.64814 * dt;

	//fz -= _calib_yaw;

	int n = 1500;

	if ( samplecounter == n ) {
		
		LARGE_INTEGER end;
		QueryPerformanceCounter(&end);
		
		LONGLONG diff = end.QuadPart - _start.QuadPart;
		_dt = (double)(diff)/_ticksPerSecond.QuadPart / n;
		
		_calib_yaw = fz / n;
		fz = 0.0;
	}

	printf("xsens gyro [%d]: %6.4f\n", samplecounter, fz * RADTODEG); //fdata[0], fdata[1], fdata[2] );

	time_t rawtime;
	struct tm * timeinfo;
	
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	
	FILE *f = fopen( XSENS_LOG_FILE, "a" );
	fprintf(f, "%d %d %d %d %6.6f %6.6f %6.2f\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, samplecounter, \
			fdata[2], _calib_yaw, fz );
	fclose(f);	
	fflush(f);

	_fx = fx;
	_fy = fy;
	_fz = fz;

	return;

	if ((outputMode & OUTPUTMODE_CALIB) != 0) {
		printf("Calibrated sensor data");
		//gotoxy(0,3);
		printf(" Acc X\t Acc Y\t Acc Z");
		//gotoxy(23, 4);
		printf("(m/s^2)");
		//gotoxy(0,5);
		printf(" Gyr X\t Gyr Y\t Gyr Z");
		//gotoxy(23, 6);
		printf("(rad/s)");
		//gotoxy(0,7);
		printf(" Mag X\t Mag Y\t Mag Z");
		//gotoxy(23, 8);
		printf("(a.u.)");
		//gotoxy(0,10);
	}

	if ((outputMode & OUTPUTMODE_ORIENT) != 0) {
		printf("Orientation data\n");
		switch(outputSettings & OUTPUTSETTINGS_ORIENTMODE_MASK) {
		case OUTPUTSETTINGS_QUATERNION:
			printf("    q0\t    q1\t    q2\t    q3\n");
			break;
		case OUTPUTSETTINGS_EULER:
			printf("  Roll\t Pitch\t   Yaw\n");
			printf("                       degrees\n");
			break;
		case OUTPUTSETTINGS_MATRIX:
			printf(" Matrix\n");
			break;
		default:
			;
		}			
	}
}

void
xsens_init ( int port )
{
	if (xbus.openPort(port) != XBRV_OK) {
		printf("error init port xsens %d\n", port);
	}

	if(doMtSettings() == false) {
		printf("error setting up xsens...\n");
	}

	QueryPerformanceFrequency(&_ticksPerSecond); 
	QueryPerformanceCounter(&_start);

	_fx = _fy = _fz = 0.0;
	_calib_yaw = 0.0;
	_dt = 1.0;
}
//////////////////////////////////////////////////////////////////////////
// main
// 
// Example program for setting MTi/MTx settings and reading MTi/MTx data
//

int main(int argc, char *argv[]) 
{
	unsigned char data[MAXMSGLEN];
	short datalen;
	float fdata[18] = {0};
	unsigned short samplecounter;

	// Skip factor for writing data to screen, make screen seem a bit smoother
	short screenSkipFactor = 10;
	short screenSkipFactorCnt = screenSkipFactor;

	//getUserInputs();	
	xsens_setParameters( 9 );
	xsens_init( 9 );

	while  (1) {
		xsens_writeHeaders();
	}

	return 0;

	// Open and initialize serial port
#ifdef WIN32
	if (xbus.openPort(portNumber) != XBRV_OK) {
		for (int k=0;k<10000000;k++)
			printf("Cannot open COM port %d\n", portNumber);
#else
	if (xbus.openPort(deviceName) != XBRV_OK) {
		printf("Cannot open COM port %s\n", deviceName);
#endif
		return XBRV_INPUTCANNOTBEOPENED;
	}	

	if(doMtSettings() == false)
		return XBRV_UNEXPECTEDMSG;

	clrscr();

	//writeHeaders();
	
	while(xbus.readDataMessage(data, datalen) == XBRV_OK && !_kbhit()) {
		xbus.getValue(VALUE_SAMPLECNT, samplecounter, data, BID_MASTER);
		
		gotoxy(0,0);
		printf("Sample Counter %05d\n", samplecounter);
		
		if (screenSkipFactorCnt++ == screenSkipFactor) {
			screenSkipFactorCnt = 0;
			gotoxy(0,4);			
			if ((outputMode & OUTPUTMODE_CALIB) != 0) {
				// Output Calibrated data
				xbus.getValue(VALUE_CALIB_ACC, fdata, data);
				printf("%6.2f\t%6.2f\t%6.2f", fdata[0], 
											  fdata[1], 
											  fdata[2]);
				gotoxy(0,6);
				xbus.getValue(VALUE_CALIB_GYR, fdata, data);
				printf("%6.2f\t%6.2f\t%6.2f", fdata[0], 
											  fdata[1], 
											  fdata[2]);
				gotoxy(0,8);
				xbus.getValue(VALUE_CALIB_MAG, fdata, data);
				printf("%6.2f\t%6.2f\t%6.2f", fdata[0], 
											  fdata[1], 
											  fdata[2]);
				gotoxy(0,12);
			}

			if ((outputMode & OUTPUTMODE_ORIENT) != 0) {
				switch(outputSettings & OUTPUTSETTINGS_ORIENTMODE_MASK) {
				case OUTPUTSETTINGS_QUATERNION:
					// Output: quaternion
					xbus.getValue(VALUE_ORIENT_QUAT, fdata, data);
					printf("%6.3f\t%6.3f\t%6.3f\t%6.3f\n",
							fdata[0],
							fdata[1], 
							fdata[2], 
							fdata[3]);
					break;
				case OUTPUTSETTINGS_EULER:
					// Output: Euler
					xbus.getValue(VALUE_ORIENT_EULER, fdata, data);
					printf("%6.1f\t%6.1f\t%6.1f\n",
							fdata[0],
							fdata[1], 
							fdata[2]);
					break;
				case OUTPUTSETTINGS_MATRIX:
					// Output: Cosine Matrix
					xbus.getValue(VALUE_ORIENT_MATRIX, fdata, data);
					printf("%6.3f\t%6.3f\t%6.3f\n",fdata[0], 
												   fdata[1], 
												   fdata[2]);
					printf("%6.3f\t%6.3f\t%6.3f\n",fdata[3],
												   fdata[4], 
												   fdata[5]);
					printf("%6.3f\t%6.3f\t%6.3f\n",fdata[6], 
												   fdata[7], 
												   fdata[8]);
					break;
				default:
					;
				}
			}
		}
	}

	// When done, close the serial port
	xbus.close();

	return XBRV_OK;
}
