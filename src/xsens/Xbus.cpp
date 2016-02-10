// Xbus.cpp: implementation of the CXbus class.
//
// Version 1.1.1
//	Public release
//
// v1.1.1
// 01-09-2005 - Added defines for Extended output mode
//			  - Added reqSetting (byte array in + out & no param variant)
//
// v1.1
// 08-08-2005 - Added file read and write support
//			  - Added functions for data retrieval (getValue etc)
//				  for easy data retrieval of acc, gyr, mag etc
//			  - ReadMessageRaw:
//				- added a temporary buffer for unprocessed bytes
//				- check for invalid length messages
//			  - Changed BID_MT into 1 and added BID_MASTER (=0xFF)
//			  - Changed various ....SerialPort functions to .....Port
//			  - Changed mtSendMessage functions to mtWriteMessage
//			  - Added numerous defines
//			  - Deleted obsolete functions
//			  - Changed function getLastErrorCode into getLastDeviceError
//			  - Changed OpenPort function for compatiblity with Bluetooth ports
//			  - Added workaround for lockup of USB driver (close function) 
//			  - Added workaround for clock() problem with read function of USB driver
//
// v1.0.2
// 29-06-2005 - Inserted initSerialPort with devicename input
//			  - Changed return value defines names from X_ to XBRV_
//			  - Removed unneeded includes for linux
//
// v1.0.1
// 22-06-2005 - Fixed ReqSetting functions (byte array & param variant)
//				mtSendRawString had wrong length input
//
// v1.0.0
// 20-06-2005 - Initial release
//
// ----------------------------------------------------------------------------
//  This file is an Xsens Code Examples.
//
//  Copyright (C) Xsens Technologies B.V., 2005.  All rights reserved.
//
//  This source code is intended only as a example of the implementation
//	of the Xsens Xbus protocol.
//	It was written for cross platform capabilities.
//
//  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
//  KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//  PARTICULAR PURPOSE.
//////////////////////////////////////////////////////////////////////

#include "Xbus.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
//
CXbus::CXbus()
{
	m_portOpen = false;
	m_fileOpen = false;
	m_deviceError = 0;		// No error
	m_retVal = XBRV_OK;
	m_timeOut = TO_DEFAULT;
	m_nTempBufferLen = 0;
	for(int i=0;i<MAXDEVICES+1;i++){
		m_storedOutputMode[i] = INVALIDSETTINGVALUE;
		m_storedOutputSettings[i] = INVALIDSETTINGVALUE;
		m_storedDataLength[i] = 0;
	}
}

CXbus::~CXbus()
{
	close();
}

////////////////////////////////////////////////////////////////////
// clockms
//
// Calculates the processor time used by the calling process.
// For linux use gettimeofday instead of clock() function
// When using read from FTDI serial port in a loop the 
// clock() function very often keeps returning 0.
//
// Output
//   returns processor time in milliseconds
//
clock_t CXbus::clockms()
{
	clock_t clk;
#ifdef WIN32
	clk = clock();		// Get current processor time

	clk /= (CLOCKS_PER_SEC / 1000);

#else
	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);
	clk = tv.tv_sec * 1000 + (tv.tv_usec / 1000);
#endif
	return clk;
}

////////////////////////////////////////////////////////////////////
// openPort (serial port number as input parameter)
//
// Opens a 'live' connection to a MT or XM
//
// Input
//   portNumber	 : number of serial port to open (1-99)
//   baudrate	 : baudrate value (One of the PBR_* defines), default = PBR_115K2
//   inqueueSize : size of input queue, default = 4096
//   outqueueSize: size of output queue, default = 1024
//
// Output
//   returns XBRV_OK if serial port is successfully opened, else XBRV_INPUTCANNOTBEOPENED
//
short CXbus::openPort(const int portNumber, const unsigned long baudrate, const unsigned long inqueueSize, const unsigned long outqueueSize)
{
	if (m_fileOpen || m_portOpen) {
		return (m_retVal = XBRV_ANINPUTALREADYOPEN);
	}
#ifdef WIN32	
	char pchFileName[10];
	
	sprintf(pchFileName,"\\\\.\\COM%d",portNumber);		// Create file name

	m_handle = CreateFile(pchFileName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (m_handle == INVALID_HANDLE_VALUE) {
		return (m_retVal = XBRV_INPUTCANNOTBEOPENED);
	}

	// Once here, port is open
	m_portOpen = true;

	//Get the current state & then change it
	DCB dcb;

	GetCommState(m_handle, &dcb);	// Get current state

	dcb.BaudRate = baudrate;			// Setup the baud rate
	dcb.Parity = NOPARITY;				// Setup the Parity
	dcb.ByteSize = 8;					// Setup the data bits
	dcb.StopBits = TWOSTOPBITS;			// Setup the stop bits
	dcb.fDsrSensitivity = FALSE;		// Setup the flow control 
	dcb.fOutxCtsFlow = FALSE;			// NoFlowControl:
	dcb.fOutxDsrFlow = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	if (!SetCommState(m_handle, (LPDCB)&dcb)) {// Set new state
		// Bluetooth ports cannot always be opened with 2 stopbits
		// Now try to open port with 1 stopbit. 
		dcb.StopBits = ONESTOPBIT;
		if (!SetCommState(m_handle, (LPDCB)&dcb)) {
			CloseHandle(m_handle);
			m_handle = INVALID_HANDLE_VALUE;
			m_portOpen = false;
			return (m_retVal = XBRV_INPUTCANNOTBEOPENED);
		}
	}

	// Set COM timeouts
	COMMTIMEOUTS CommTimeouts;

	GetCommTimeouts(m_handle,&CommTimeouts);	// Fill CommTimeouts structure

	CommTimeouts.ReadTotalTimeoutConstant = 0;	
	CommTimeouts.ReadIntervalTimeout = MAXDWORD;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0; 
	SetCommTimeouts(m_handle, &CommTimeouts);	// Set CommTimeouts structure

	// Other initialization functions
	EscapeCommFunction(m_handle, SETRTS);		// Enable RTS (for Xbus Master use)
	SetupComm(m_handle,inqueueSize,outqueueSize);	// Set queue size

	// Remove any 'old' data in buffer
	PurgeComm(m_handle, PURGE_TXCLEAR | PURGE_RXCLEAR);

	return (m_retVal = XBRV_OK);
#else	
	char chPort[15];
	struct termios options;

	/* Open port */
	sprintf(chPort,"/dev/ttyS%d",(portNumber - 1));
	
	m_handle = open(chPort, O_RDWR | O_NOCTTY);

	// O_RDWR: Read+Write
	// O_NOCTTY: Raw input, no "controlling terminal"
	// O_NDELAY: Don't care about DCD signal

	if (m_handle < 0) {
		// Port not open
		return (m_retVal = XBRV_INPUTCANNOTBEOPENED);
	}

	// Once here, port is open
	m_portOpen = true;

	/* Start configuring of port for non-canonical transfer mode */
	// Get current options for the port
	tcgetattr(m_handle, &options);
	
	// Set baudrate. 
	cfsetispeed(&options, baudrate);
	cfsetospeed(&options, baudrate);
	
	// Enable the receiver and set local mode
	options.c_cflag |= (CLOCAL | CREAD);
	// Set character size to data bits and set no parity Mask the characte size bits
	options.c_cflag &= ~(CSIZE|PARENB);
	options.c_cflag |= CS8;		// Select 8 data bits
	options.c_cflag |= CSTOPB;	// send 2 stop bits
	// Disable hardware flow control
	options.c_cflag &= ~CRTSCTS;
	options.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	// Disable software flow control
	options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	// Set Raw output
	options.c_oflag &= ~OPOST;
	// Timeout 0.005 sec for first byte, read minimum of 0 bytes
	options.c_cc[VMIN]     = 0;
	options.c_cc[VTIME]    = 5;

	// Set the new options for the port
	tcsetattr(m_handle,TCSANOW, &options);
	
	tcflush(m_handle, TCIOFLUSH);

	return (m_retVal = XBRV_OK);
#endif	
}

////////////////////////////////////////////////////////////////////
// openPort (string as input parameter)
//
// Opens a 'live' connection to a MT or XM
//
// Input
//   portName	 : device name of serial port ("/dev/ttyUSB0" or "\\\\.\\COM1")
//   baudrate	 : baudrate value (One of the PBR_* defines), default = PBR_115K2
//   inqueueSize : size of input queue, default = 4096
//   outqueueSize: size of output queue, default = 1024
//
// Output
//   returns XBRV_OK if serial port is successfully opened, else XBRV_INPUTCANNOTBEOPENED
//
short CXbus::openPort(const char *portName, const unsigned long baudrate, const unsigned long inqueueSize, const unsigned long outqueueSize)
{
	if (m_fileOpen || m_portOpen) {
		return (m_retVal = XBRV_ANINPUTALREADYOPEN);
	}
#ifdef WIN32	
	m_handle = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (m_handle == INVALID_HANDLE_VALUE) {
		return (m_retVal = XBRV_INPUTCANNOTBEOPENED);
	}

	// Once here, port is open
	m_portOpen = true;

	//Get the current state & then change it
	DCB dcb;

	GetCommState(m_handle, &dcb);	// Get current state

	dcb.BaudRate = baudrate;			// Setup the baud rate
	dcb.Parity = NOPARITY;				// Setup the Parity
	dcb.ByteSize = 8;					// Setup the data bits
	dcb.StopBits = TWOSTOPBITS;			// Setup the stop bits
	dcb.fDsrSensitivity = FALSE;		// Setup the flow control 
	dcb.fOutxCtsFlow = FALSE;			// NoFlowControl:
	dcb.fOutxDsrFlow = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	if (!SetCommState(m_handle, (LPDCB)&dcb)) {// Set new state
		// Bluetooth ports cannot always be opened with 2 stopbits
		// Now try to open port with 1 stopbit. 
		dcb.StopBits = ONESTOPBIT;
		if (!SetCommState(m_handle, (LPDCB)&dcb)) {
			CloseHandle(m_handle);
			m_handle = INVALID_HANDLE_VALUE;
			m_portOpen = false;
			return (m_retVal = XBRV_INPUTCANNOTBEOPENED);
		}
	}

	// Set COM timeouts
	COMMTIMEOUTS CommTimeouts;

	GetCommTimeouts(m_handle,&CommTimeouts);	// Fill CommTimeouts structure

	CommTimeouts.ReadTotalTimeoutConstant = 0;	
	CommTimeouts.ReadIntervalTimeout = MAXDWORD;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0; 
	SetCommTimeouts(m_handle, &CommTimeouts);	// Set CommTimeouts structure

	// Other initialization functions
	EscapeCommFunction(m_handle, SETRTS);		// Enable RTS (for Xbus Master use)
	SetupComm(m_handle,inqueueSize,outqueueSize);	// Set queue size

	// Remove any 'old' data in buffer
	PurgeComm(m_handle, PURGE_TXCLEAR | PURGE_RXCLEAR);

	return (m_retVal = XBRV_OK);
#else	
	struct termios options;

	/* Open port */
	
	m_handle = open(portName, O_RDWR | O_NOCTTY);

	// O_RDWR: Read+Write
	// O_NOCTTY: Raw input, no "controlling terminal"
	// O_NDELAY: Don't care about DCD signal

	if (m_handle < 0) {
		// Port not open
		return (m_retVal = XBRV_INPUTCANNOTBEOPENED);
	}
	
	// Once here, port is open
	m_portOpen = true;

	/* Start configuring of port for non-canonical transfer mode */
	// Get current options for the port
	tcgetattr(m_handle, &options);
	
	// Set baudrate. 
	cfsetispeed(&options, baudrate);
	cfsetospeed(&options, baudrate);

	// Enable the receiver and set local mode
	options.c_cflag |= (CLOCAL | CREAD);
	// Set character size to data bits and set no parity Mask the characte size bits
	options.c_cflag &= ~(CSIZE|PARENB);
	options.c_cflag |= CS8;		// Select 8 data bits
	options.c_cflag |= CSTOPB;	// send 2 stop bits
	// Disable hardware flow control
	options.c_cflag &= ~CRTSCTS;
	options.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	// Disable software flow control
	options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	// Set Raw output
	options.c_oflag &= ~OPOST;
	// Timeout 0.005 sec for first byte, read minimum of 0 bytes
	options.c_cc[VMIN]     = 0;
	options.c_cc[VTIME]    = 5;

	// Set the new options for the port
	tcsetattr(m_handle,TCSANOW, &options);
	
	tcflush(m_handle, TCIOFLUSH);

	return (m_retVal = XBRV_OK);
#endif	
}

////////////////////////////////////////////////////////////////////
// openFile
//
// Open file for reading and writing
// Filepos is at start of file
//
// Input
//   fileName	 : file name including path
//	 createAlways: empties the log file, if existing
//
// Output
//   returns XBRV_OK if the file is opened
//   returns XBRV_INPUTCANNOTBEOPENED if the file can not be opened
//   returns XBRV_ANINPUTALREADYOPEN if a serial port / file is already opened
//
short CXbus::openFile(const char *fileName, bool createAlways)
{
	if (m_portOpen || m_portOpen) {
		return (m_retVal = XBRV_ANINPUTALREADYOPEN);
	}
#ifdef WIN32	
	DWORD disposition = OPEN_ALWAYS;
	if (createAlways == true) {
		disposition = CREATE_ALWAYS;
	}
	m_handle = CreateFile(fileName, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ, NULL, disposition, 0, NULL);
	if (m_handle == INVALID_HANDLE_VALUE) {
		return (m_retVal = XBRV_INPUTCANNOTBEOPENED);
	}
#else
	int openMode = O_RDWR | O_CREAT;
	if (createAlways == true) {
		openMode |= O_TRUNC;
	}
	m_handle = open(fileName, openMode, S_IRWXU);
	if (m_handle < 0) {
		return (m_retVal = XBRV_INPUTCANNOTBEOPENED);
	}
#endif
	m_timeOut = 0;	// No timeout in when using file input
	m_fileOpen = true;
	return (m_retVal = XBRV_OK);

}

////////////////////////////////////////////////////////////////////
// isPortOpen
//
// Return if serial port is open or not
//
bool CXbus::isPortOpen()
{
	return m_portOpen;
}

////////////////////////////////////////////////////////////////////
// isFileOpen
//
// Return if serial port is open or not
//
bool CXbus::isFileOpen()
{
	return m_fileOpen;
}

////////////////////////////////////////////////////////////////////
// readData
//
// Reads bytes from serial port or file
//
// Input
//   msgBuffer		: pointer to buffer in which next string will be stored
//   nBytesToRead	: number of buffer bytes to read from serial port
//   retval			: return value, either XBRV_OK, XBRV_ENDOFFILE or XBRV_NOINPUTINITIALIZED
//
// Output
//   number of bytes actually read
int CXbus::readData(unsigned char* msgBuffer, const int nBytesToRead)
{
	if (!m_fileOpen && !m_portOpen) {
		m_retVal = XBRV_NOINPUTINITIALIZED;
		return 0;
	}
#ifdef WIN32
	DWORD nBytesRead;
	BOOL retval = ReadFile(m_handle, msgBuffer, nBytesToRead, &nBytesRead, NULL);
	if (retval && nBytesRead == 0 && m_fileOpen) {
		m_retVal = XBRV_ENDOFFILE;
	}
	else
		m_retVal = XBRV_OK;
	return nBytesRead;
#else
	int nBytesRead = read(m_handle, msgBuffer, nBytesToRead);
	if(nBytesRead == 0 && m_fileOpen) {
		nBytesRead = 0;
		m_retVal = XBRV_ENDOFFILE;
    	}
	else
		m_retVal = XBRV_OK;
	return nBytesRead;

	// In Linux it is sometimes better to read per byte instead of a block of bytes at once
	// Use this block if experiencing problems with the above code
	/*
		int j = 0;	// Index in buffer for read data
		int k = 0;	// Timeout factor
		int nRead = 0;	// Bytes read from port, return by read function

		do {
			nRead = read(m_handle, &msgBuffer[j], 1);
			if (nRead == 1) {	// Byte read
				k = 0;
				j++;
			}
			else {
    			k++;
			}

			if (k == 3)	{ // Timeout, too long no data read from port
    			return nRead;
			}
		}
		while (j < nBytesToRead);
		
		return j;
	*/
#endif
}

////////////////////////////////////////////////////////////////////
// writeData
//
// Writes bytes to serial port (to do: file)
//
// Input
//   msgBuffer		: a pointer to a char buffer containing
//					  the characters to be written to serial port
//   nBytesToWrite	: number of buffer bytes to write to serial port
//
// Output
//   number of bytes actually written
// 
// The Xbus return value is != XBRV_OK if serial port is closed
int CXbus::writeData(const unsigned char* msgBuffer, const int nBytesToWrite)
{
	if (!m_fileOpen && !m_portOpen) {
		m_retVal = XBRV_NOINPUTINITIALIZED;
		return 0;
	}
	
	m_retVal = XBRV_OK;
#ifdef WIN32
	DWORD nBytesWritten;
	WriteFile(m_handle, msgBuffer, nBytesToWrite, &nBytesWritten, NULL);
	return nBytesWritten;
#else
	return write(m_handle, msgBuffer, nBytesToWrite);
#endif
}

////////////////////////////////////////////////////////////////////
// flush
//
// Remove any 'old' data in COM port buffer and flushes internal 
//   temporary buffer
//
void CXbus::flush()
{
	if (m_portOpen) {
#ifdef WIN32
		// Remove any 'old' data in buffer
		PurgeComm(m_handle, PURGE_TXCLEAR | PURGE_RXCLEAR);
#else
		tcflush(m_handle, TCIOFLUSH);
#endif
	}
	m_nTempBufferLen = 0;
	m_retVal = XBRV_OK;
}

////////////////////////////////////////////////////////////////////
// setPortQueueSize
//
// Set input and output buffer size of serial port
//
void CXbus::setPortQueueSize(const unsigned long inqueueSize /* = 4096 */, const unsigned long outqueueSize /* = 1024 */)
{
	if (m_portOpen) {
#ifdef WIN32
		SetupComm(m_handle,inqueueSize,outqueueSize);	// Set queue size
#else
		// Not yet implemented
#endif
	}
	m_retVal = XBRV_OK;
}

////////////////////////////////////////////////////////////////////
// setFilePos
//
// Sets the current position of the file pointer for file input
//
// Input
//	 relPos		: 32-bit value specifying the relative move in bytes
//				    origin is specified in moveMethod
//	 moveMethod	: FILEPOS_BEGIN, FILEPOS_CURRENT or FILEPOS_END
// Output
//	
//   returns XBRV_OK if file pointer is successfully set
//
short CXbus::setFilePos(long relPos, unsigned long moveMethod)
{
#ifdef WIN32
	if (m_fileOpen) {
		if(SetFilePointer(m_handle, relPos, NULL, moveMethod) != INVALID_SET_FILE_POINTER){
			return (m_retVal = XBRV_OK);
		}
	}
#else
	if (m_fileOpen) {
		if (lseek(m_handle, relPos, moveMethod) != -1){
			return (m_retVal = XBRV_OK);
		}
	}
#endif
	return (m_retVal = XBRV_NOTSUCCESSFUL);
}

////////////////////////////////////////////////////////////////////
// getFileSize
//
// Retrieves the file size of the opened file
//
// Input
// Output
//	 fileSize  : Number of bytes of the current file
//	
//   returns XBRV_OK if successful
//
short CXbus::getFileSize(unsigned long &fileSize)
{
#ifdef WIN32
	if(m_fileOpen){
		if ((fileSize = GetFileSize(m_handle, NULL)) != INVALID_FILE_SIZE) {
			return (m_retVal = XBRV_OK);
		}
	}
#else
	if(m_fileOpen){
		struct stat buf;
		if (fstat(m_handle, &buf) == 0) {
			fileSize = buf.st_size;
			return (m_retVal = XBRV_OK);
		}
	}
#endif
	return (m_retVal = XBRV_NOTSUCCESSFUL);
}

////////////////////////////////////////////////////////////////////
// close
//
// Closes handle of file or serial port
//
short CXbus::close()
{
	if (m_portOpen || m_fileOpen) {
#ifdef WIN32
		if (m_portOpen) {		// Because of USB-serial driver bug
			flush();
		}
		CloseHandle(m_handle);
#else
		::close(m_handle);
#endif
	}
	m_fileOpen = m_portOpen = false;
	m_timeOut = TO_DEFAULT; // Restore timeout value (file input)
	m_nTempBufferLen = 0;
	m_deviceError = 0;		// No error
	for(int i=0;i<MAXDEVICES+1;i++){
		m_storedOutputMode[i] = INVALIDSETTINGVALUE;
		m_storedOutputSettings[i] = INVALIDSETTINGVALUE;
		m_storedDataLength[i] = 0;
	}
	return (m_retVal = XBRV_OK);
}

////////////////////////////////////////////////////////////////////
// readMessage
//
// Reads the next message from serial port buffer or file. 
// To be read within current time out period
//
// Input
// Output
//	 mid		: MessageID of message received
//	 data	    : array pointer to data bytes (no header)
//	 dataLen    : number of data bytes read
//   bid		: BID or address of message read (optional)
//	
//   returns XBRV_OK if a message has been read else <>XBRV_OK
//
// Remarks
//   allocate enough memory for message buffer
//   use setTimeOut for different timeout value
short CXbus::readMessage(unsigned char &mid, unsigned char data[], short &dataLen, unsigned char *bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (!(m_fileOpen || m_portOpen)) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}

	if (readMessageRaw(buffer, &msgLen) == XBRV_OK) {
		// Message read
		mid = buffer[IND_MID];
		if (bid != NULL) {
			*bid = buffer[IND_BID];
		}
		if (buffer[IND_LEN] != EXTLENCODE) {
			dataLen = buffer[IND_LEN];
			memcpy(data, &buffer[IND_DATA0], dataLen);
		}
		else{
			dataLen = buffer[IND_LENEXTH]*256 + buffer[IND_LENEXTL];
			memcpy(data, &buffer[IND_DATAEXT0], dataLen);
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// readDataMessage
//
// Read a MTData or XMData message from serial port (using TimeOut val)
//
// Input
//   data		: pointer to buffer in which the DATA field of MTData/XMData is stored
//   dataLen	: number of bytes in buffer (= number bytes in DATA field)
// Output
//   returns XBRV_OK if MTData / XMData message has been read else <>XBRV_OK
//
// Remarks
//   allocate enough memory for data buffer
//   use setTimeOut for different timeout value
short CXbus::readDataMessage(unsigned char data[], short &dataLen)
{
	unsigned char buffer[MAXMSGLEN];
	short buflen;

	if (!(m_fileOpen || m_portOpen)) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}

	if(readMessageRaw(buffer,&buflen) == XBRV_OK){
		if (buffer[IND_MID] == MID_MTDATA) {	// MID_XMDATA is same
			if (buffer[IND_LEN] != EXTLENCODE) {
				dataLen = buffer[IND_LEN];
				memcpy(data, &buffer[IND_DATA0], dataLen);
			}
			else{
				dataLen = buffer[IND_LENEXTH]*256 + buffer[IND_LENEXTL];
				memcpy(data, &buffer[IND_DATAEXT0], dataLen);
			}
			return (m_retVal = XBRV_OK);
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// readMessageRaw
//
// Read a message from serial port for a certain period
//
// Input
//   msgBuffer		: pointer to buffer in which next msg will be stored
//   msgBufferLength: integer to number of bytes written in buffer (header + data + chksum)
// Output
//   returns XBRV_OK if a message has been read else <>XBRV_OK
//
// Remarks
//   allocate enough memory for message buffer
//   use setTimeOut for different timeout value
short CXbus::readMessageRaw(unsigned char *msgBuffer, short *msgBufferLength)
{
	clock_t	clkStart;
	int		state = 0;
	int		nBytesToRead = 1;
	int		nBytesRead = 0;
	int		nOffset = 0;
	int		nMsgDataLen = 0;
	int		nMsgLen;
	unsigned char	chCheckSum;
	bool	Synced = false;

	if (!(m_fileOpen || m_portOpen)) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}

	// Copy previous read bytes if available
	if (m_nTempBufferLen > 0) {
		memcpy(msgBuffer, m_tempBuffer, m_nTempBufferLen);
		nBytesRead = m_nTempBufferLen;
		m_nTempBufferLen = 0;
	}
	clkStart = clockms();		// Get current processor time
	
	do{
		do{
			// First check if we already have some bytes read
			if (nBytesRead > 0 && nBytesToRead > 0) {
				if (nBytesToRead > nBytesRead) {
					nOffset += nBytesRead;
					nBytesToRead -= nBytesRead;
					nBytesRead = 0;
				}
				else{
					nOffset += nBytesToRead;
					nBytesRead -= nBytesToRead;
					nBytesToRead = 0;
				}
			}
			
			// Check if serial port buffer must be read
			if (nBytesToRead > 0) {
				nBytesRead = readData(msgBuffer+nOffset, nBytesToRead);
				if (m_retVal == XBRV_ENDOFFILE) {
					return (m_retVal = XBRV_ENDOFFILE);
				}
				nOffset += nBytesRead;
				nBytesToRead -= nBytesRead;
				nBytesRead = 0;
			}
			
			if(nBytesToRead == 0){
				switch(state){
				case 0:					// Check preamble
					if(msgBuffer[IND_PREAMBLE] == PREAMBLE){
						state++;
						nBytesToRead = 3;
					}
					else{
						nOffset = 0;
						nBytesToRead = 1;
					}
					break;
				case 1:					// Check ADDR, MID, LEN
					if (msgBuffer[IND_LEN] != 0xFF) {
						state = 3;
						nBytesToRead = (nMsgDataLen = msgBuffer[IND_LEN]) + 1; // Read LEN + 1 (chksum)
					}
					else {
						state = 2;
						nBytesToRead = 2;	// Read extended length
					}	
					break;
				case 2:
					state = 3;
					nBytesToRead = (nMsgDataLen = msgBuffer[IND_LENEXTH] * 256 + msgBuffer[IND_LENEXTL]) + 1;	// Read LENEXT + CS
					if (nMsgDataLen > MAXMSGLEN-LEN_MSGEXTHEADERCS) {
						// Not synced - check for preamble in the bytes read
						for (int i = 1; i < LEN_MSGEXTHEADER; i++) {
							if (msgBuffer[i] == PREAMBLE) {
								// Found a maybe preamble - 'fill buffer'
								nBytesRead = LEN_MSGEXTHEADER - i;
								memmove(msgBuffer, msgBuffer+i,nBytesRead);
								break;
							}
						}
						Synced = false;
						nOffset = 0;
						state = 0;
						nBytesToRead = 1;			// Start looking for preamble
					}
					break;
				case 3:					// Check msg
					chCheckSum = 0;
					nMsgLen = nMsgDataLen + 5 + (msgBuffer[IND_LEN] == 0xFF?2:0);

					for(int i = 1; i < nMsgLen; i++){
						chCheckSum += msgBuffer[i];
					}

					if(chCheckSum == 0){				// Checksum ok?
						if (nBytesRead > 0) {			// Store bytes if read too much
							memcpy(m_tempBuffer, msgBuffer+nMsgLen, nBytesRead);
							m_nTempBufferLen = nBytesRead;
						}
						*msgBufferLength = nMsgLen;
						Synced = true;
						return (m_retVal = XBRV_OK);
					}
					else{
						if(!Synced){
							// Not synced - maybe recheck for preamble in this incorrect message
							for (int i = 1; i < nMsgLen; i++) {
								if (msgBuffer[i] == PREAMBLE) {
									// Found a maybe preamble - 'fill buffer'
									nBytesRead = nMsgLen - i;
									memmove(msgBuffer, msgBuffer+i,nBytesRead);
									break;
								}
							}
						}
						Synced = false;
						nOffset = 0;
						state = 0;
						nBytesToRead = 1;			// Start looking for preamble
					}
					break;
				}
			}
		}while(((clockms()-clkStart) < m_timeOut) || m_timeOut == 0 || nBytesRead != 0);
		
		// Check if pending message has a valid message
		if(state > 0){
			int i;
			// Search for preamble
			for (i = 1; i < nOffset; i++) {
				if (msgBuffer[i] == PREAMBLE) {
					// Found a maybe preamble - 'fill buffer'
					nBytesRead = nOffset-i-1; // no preamble
					memmove(msgBuffer+1, msgBuffer+i+1,nBytesRead);
					break;
				}
			}
			if (i < nOffset) {
				// Found preamble in message - recheck
				nOffset = 1;
				state = 1;
				nBytesToRead = 3;			// Start looking for preamble
				continue;
			}
		}
		break;
	}while(true);
	
	return (m_retVal = XBRV_TIMEOUT);
}

////////////////////////////////////////////////////////////////////
// writeMessage (optional: integer value)
//
// Sends a message and in case of an serial port interface it checks
//   if the return message (ack, error or timeout). See return value
//   In case an file is opened the functions returns directly after
//   'sending' the message
//
//   Use this function for GotoConfig, Reset, ResetOrientation etc
//
// Input
//	 mid		  : MessageID of message to send
//	 dataValue	  : A integer value that will be included into the data message field
//				    can be a 1,2 or 4 bytes values
//	 dataValueLen : Size of dataValue in bytes
//   bid		  : BID or address to use in message to send (default = 0xFF)
//
// Return value
//   = XBRV_OK if an Ack message received / or data successfully written to file
//	 = XBRV_RECVERRORMSG if an error message received
//	 = XBRV_TIMEOUT if timeout occurred
//   = XBRV_NOINPUTINITIALIZED
//
short CXbus::writeMessage(const unsigned char mid, const unsigned long dataValue, 
						   const unsigned char dataValueLen, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;
	
	if (!(m_fileOpen || m_portOpen)) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	buffer[IND_LEN] = dataValueLen;
	swapEndian((const unsigned char *)&dataValue,&buffer[IND_DATA0],dataValueLen);
	calcChecksum(buffer,LEN_MSGHEADER + dataValueLen);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + dataValueLen);	

	// Return if file opened
	if (m_fileOpen) {
		return (m_retVal = XBRV_OK);
	}
	
	// Keep reading until an Ack or Error message is received (or timeout)
	clock_t clkStart = clockms();
	short	defTimeOut = m_timeOut;
	do{
		if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
			// Message received
			if(buffer[IND_MID] == (mid+1)) {
				m_timeOut = defTimeOut;
				return (m_retVal = XBRV_OK);				// Acknowledge received
			}
			else if (buffer[IND_MID] == MID_ERROR){
				m_deviceError = buffer[IND_DATA0];
				m_timeOut = defTimeOut;
				return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
			}
		}
		// Calculate remaining time to check
		m_timeOut = defTimeOut - (clockms() - clkStart);
	}while(m_timeOut > 0);

	m_timeOut = defTimeOut;
	return (m_retVal = XBRV_TIMEOUT);
}

////////////////////////////////////////////////////////////////////
// writeMessage (data array)
//
// Sends a message and in case of an serial port interface it checks
//   if the return message (ack, error or timeout). See return value
//   In case an file is opened the functions returns directly after
//   'sending' the message
//
// Input
//	 mid		: MessageID of message to send
//	 data	    : array pointer to data bytes
//	 dataLen    : number of bytes to include in message
//   bid		: BID or address to use in message to send (default = 0xFF)
//
// Output
//   = XBRV_OK if an Ack message received
//	 = XBRV_RECVERRORMSG if an error message received
//	 = XBRV_TIMEOUT if timeout occurred
//   = XBRV_NOINPUTINITIALIZED
//
short CXbus::writeMessage(const unsigned char mid, const unsigned char data[], 
						   const unsigned short &dataLen, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;
	short headerLength;

	if (!(m_fileOpen || m_portOpen)) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}

	// Build message to send
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;

	if (dataLen < EXTLENCODE) {
		buffer[IND_LEN] = (unsigned char)dataLen;
		headerLength = LEN_MSGHEADER;
	}
	else {
		buffer[IND_LEN] = EXTLENCODE;
		buffer[IND_LENEXTH] = (unsigned char)(dataLen >> 8);
		buffer[IND_LENEXTL] = (unsigned char)(dataLen & 0x00FF);
		headerLength = LEN_MSGEXTHEADER;
	}
	memcpy(&buffer[headerLength], data, dataLen);
	calcChecksum(buffer, headerLength + dataLen);

	// Send message
	writeData(buffer, headerLength + dataLen + LEN_CHECKSUM);

	// Return if file opened
	if (m_fileOpen) {
		return (m_retVal = XBRV_OK);
	}

	// Keep reading until an Ack or Error message is received (or timeout)
	clock_t clkStart = clockms();
	short	defTimeOut = m_timeOut;
	do{
		if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
			// Message received
			if(buffer[IND_MID] == (mid+1)) {
				m_timeOut = defTimeOut;
				return (m_retVal = XBRV_OK);				// Acknowledge received
			}
			else if (buffer[IND_MID] == MID_ERROR){
				m_deviceError = buffer[IND_DATA0];
				m_timeOut = defTimeOut;
				return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
			}
		}
		// Calculate remaining time to check
		m_timeOut = defTimeOut - (clockms() - clkStart);
	}while(m_timeOut > 0);

	m_timeOut = defTimeOut;
	return (m_retVal = XBRV_TIMEOUT);
}

////////////////////////////////////////////////////////////////////
// waitForMessage
//
// Read messages from serial port or file using the current timeout period
//  until the received message is equal to a specific message identifier 
// By default the timeout period by file input is set to infinity (= until
//  end of file is reached)
//
// Input/Output
//   mid			: message identifier of message that should be returned
//   data			: pointer to buffer in which the data of the requested msg will be stored
//   dataLen		: integer to number of data bytes
//	 bid			: optional, pointer which holds the bid of the returned message
// Output
//   returns XBRV_OK if the message has been read else != XBRV_OK
//
// Remarks
//   allocate enough memory for data message buffer
//   use setTimeOut for different timeout value
short CXbus::waitForMessage(const unsigned char mid, unsigned char data[], short *dataLen, unsigned char *bid)
{
	unsigned char buffer[MAXMSGLEN];
	short buflen;

	clock_t clkStart;
	short	defTimeOut = m_timeOut;

	if (!(m_fileOpen || m_portOpen)) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}

	clkStart = clockms();		// Get current processor time

	do{
		if(readMessageRaw(buffer, &buflen) == XBRV_OK){
			if(buffer[IND_MID] == mid){
				if (bid != NULL) {
					*bid = buffer[IND_BID];
				}
				if (data != NULL && dataLen != NULL) {
					if (buffer[IND_LEN] != EXTLENCODE) {
						*dataLen = buffer[IND_LEN];
						memcpy(data, &buffer[IND_DATA0], *dataLen);
					}
					else{
						*dataLen = buffer[IND_LENEXTH]*256 + buffer[IND_LENEXTL];
						memcpy(data, &buffer[IND_DATAEXT0], *dataLen);
					}
				}
				else if(dataLen != NULL)
				{
					dataLen = 0;
				}
				m_timeOut = defTimeOut;		// Restore timeout value
				return (m_retVal = XBRV_OK);
			}
		}
		else if (m_retVal == XBRV_ENDOFFILE) {
			m_timeOut = defTimeOut;
			return (m_retVal = XBRV_ENDOFFILE);
		}
		if (m_timeOut != 0) {
			m_timeOut = defTimeOut - (clockms() - clkStart);
		}
	}while(m_timeOut >= 0);

	m_timeOut = defTimeOut;					// Restore timeout value
	return (m_retVal = XBRV_TIMEOUT);
}

////////////////////////////////////////////////////////////////////
// reqSetting (integer & no param variant)
//
// Request a integer setting from the device. This setting
// can be an unsigned 1,2 or 4 bytes setting. Only valid
// for serial port connections
//
// Input
//	 mid		: Message ID of message to send
//   bid		: Bus ID of message to send (def 0xFF)
//
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//	 value contains the integer value of the data field of the ack message
//
short CXbus::reqSetting(const unsigned char mid, unsigned long &value, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	buffer[IND_LEN] = 0;
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	// Read next message or else timeout
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			// Acknowledge received
			value = 0;
			swapEndian(&buffer[IND_DATA0],(unsigned char *)&value, buffer[IND_LEN]);
			return (m_retVal = XBRV_OK);				
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
		else{
			return (m_retVal = XBRV_UNEXPECTEDMSG);// Unexpected message
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// reqSetting (integer & param variant)
//
// Request a integer setting from the device. This setting
// can be an unsigned 1,2 or 4 bytes setting. Only valid
// for serial port connections.
//
// Input
//	 mid		: Message ID of message to send
//	 param		: For messages that need a parameter
//   bid		: Bus ID of message to send (def 0xFF)
//
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//	 value contains the integer value of the data field of the ack message
//
short CXbus::reqSetting(const unsigned char mid, const unsigned char param, unsigned long &value,  
						  const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	if (param != 0xFF) {
		buffer[IND_LEN] = 1;
		buffer[IND_DATA0] = param;
	}
	else{
		buffer[IND_LEN] = 0;
	}
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	// Read next message or else timeout
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			// Acknowledge received
			value = 0;
			if(param == 0xFF){
				swapEndian(&buffer[IND_DATA0],(unsigned char *)&value, buffer[IND_LEN]);
			}
			else{
				swapEndian(&buffer[IND_DATA0]+1,(unsigned char *)&value, buffer[IND_LEN]-1);
			}
			return (m_retVal = XBRV_OK);				
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
		else{
			return (m_retVal = XBRV_UNEXPECTEDMSG);// Unexpected message
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// reqSetting (float & no param variant)
//
// Request a float setting from the device. Only valid
// for serial port connections.
//
// Input
//	 mid		: Message ID of message to send
//   bid		: Bus ID of message to send (def 0xFF)
//
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//	 value contains the float value of the acknowledge data field
//
short CXbus::reqSetting(const unsigned char mid, float &value, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	buffer[IND_LEN] = 0;
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	// Read next message or else timeout
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			// Acknowledge received
			value = 0;
			swapEndian(&buffer[IND_DATA0],(unsigned char *)&value, buffer[IND_LEN]);
			return (m_retVal = XBRV_OK);				
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
		else{
			return (m_retVal = XBRV_UNEXPECTEDMSG);// Unexpected message
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// reqSetting (float & param variant)
//
// Request a float setting from the device. Only valid
// for serial port connections.
//
// Input
//	 mid		: Message ID of message to send
//	 param		: For messages that need a parameter (optional)
//   bid		: Bus ID of message to send (def 0xFF)
//
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//	 value contains the float value of the acknowledge data field
//
short CXbus::reqSetting(const unsigned char mid, const unsigned char param, float &value,
						  const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	if (param != 0xFF) {
		buffer[IND_LEN] = 1;
		buffer[IND_DATA0] = param;
	}
	else{
		buffer[IND_LEN] = 0;
	}
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	// Read next message or else timeout
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			// Acknowledge received
			value = 0;
			if(param == 0xFF){
				swapEndian(&buffer[IND_DATA0],(unsigned char *)&value, buffer[IND_LEN]);
			}
			else{
				swapEndian(&buffer[IND_DATA0]+1,(unsigned char *)&value, buffer[IND_LEN]-1);
			}
			return (m_retVal = XBRV_OK);				
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
		else{
			return (m_retVal = XBRV_UNEXPECTEDMSG);// Unexpected message
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// reqSetting (byte array & no param variant)
//
// Send a message to the device and the data of acknowledge message
// will be returned. Only valid for serial port connections
//
// Input
//	 mid		: Message ID of message to send
//   bid		: Bus ID of message to send (def 0xFF)
//	 
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//	 data[] contains the data of the acknowledge message
//	 dataLen contains the number bytes returned
//
short CXbus::reqSetting(const unsigned char mid, 
						  unsigned char data[], short &dataLen, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	buffer[IND_LEN] = 0;
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	dataLen = 0;
	// Read next message or else timeout
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			// Acknowledge received
			if (buffer[IND_LEN] != EXTLENCODE) {
				dataLen = buffer[IND_LEN];
				memcpy(data, &buffer[IND_DATA0], dataLen);
			}
			else{
				dataLen = buffer[IND_LENEXTH]*256 + buffer[IND_LENEXTL];
				memcpy(data, &buffer[IND_DATAEXT0], dataLen);
			}
			return (m_retVal = XBRV_OK);				
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
		else{
			return (m_retVal = XBRV_UNEXPECTEDMSG);// Unexpected message
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// reqSetting (byte array in + out & no param variant)
//
// Send a message to the device and the data of acknowledge message
// will be returned. Only valid for serial port connections
//
// Input
//	 mid		: Message ID of message to send
//   bid		: Bus ID of message to send (def 0xFF)
//	 dataIn		: Data to be included in request
//	 dataInLen	: Number of bytes in dataIn
//	 
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//	 dataOut[] contains the data of the acknowledge message
//	 dataOutLen contains the number bytes returned
//
short CXbus::reqSetting(const unsigned char mid,
						unsigned char dataIn[], short dataInLen,
						unsigned char dataOut[], short &dataOutLen, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short headerLength;
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	if (dataInLen < EXTLENCODE) {
		buffer[IND_LEN] = (unsigned char)dataInLen;
		headerLength = LEN_MSGHEADER;
	}
	else {
		buffer[IND_LEN] = EXTLENCODE;
		buffer[IND_LENEXTH] = (unsigned char)(dataInLen >> 8);
		buffer[IND_LENEXTL] = (unsigned char)(dataInLen & 0x00FF);
		headerLength = LEN_MSGEXTHEADER;
	}
	memcpy(&buffer[headerLength], dataIn, dataInLen);
	calcChecksum(buffer, headerLength + dataInLen);

	// Send message
	writeData(buffer, headerLength + dataInLen + LEN_CHECKSUM);

	dataOutLen = 0;
	// Read next message or else timeout
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			// Acknowledge received
			if (buffer[IND_LEN] != EXTLENCODE) {
				dataOutLen = buffer[IND_LEN];
				memcpy(dataOut, &buffer[IND_DATA0], dataOutLen);
			}
			else{
				dataOutLen = buffer[IND_LENEXTH]*256 + buffer[IND_LENEXTL];
				memcpy(dataOut, &buffer[IND_DATAEXT0], dataOutLen);
			}
			return (m_retVal = XBRV_OK);				
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
		else{
			return (m_retVal = XBRV_UNEXPECTEDMSG);// Unexpected message
		}
	}
	return m_retVal;
}


////////////////////////////////////////////////////////////////////
// reqSetting (byte array & param variant)
//
// Send a message to the device and the data of acknowledge message
// will be returned. Only valid for serial port connections
//
// Input
//	 mid		: Message ID of message to send
//	 param		: For messages that need a parameter (optional)
//   bid		: Bus ID of message to send (def 0xFF)
//	 
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//	 data[] contains the data of the acknowledge message (including param!!)
//	 dataLen contains the number bytes returned
//
short CXbus::reqSetting(const unsigned char mid, const unsigned char param, 
						  unsigned char data[], short &dataLen, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	if (param != 0xFF) {
		buffer[IND_LEN] = 1;
		buffer[IND_DATA0] = param;
	}
	else{
		buffer[IND_LEN] = 0;
	}
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	dataLen = 0;
	// Read next message or else timeout
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			// Acknowledge received
			if (buffer[IND_LEN] != EXTLENCODE) {
				dataLen = buffer[IND_LEN];
				memcpy(data, &buffer[IND_DATA0], dataLen);
			}
			else{
				dataLen = buffer[IND_LENEXTH]*256 + buffer[IND_LENEXTL];
				memcpy(data, &buffer[IND_DATAEXT0], dataLen);
			}
			return (m_retVal = XBRV_OK);				
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
		else{
			return (m_retVal = XBRV_UNEXPECTEDMSG);// Unexpected message
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// setSetting (integer & no param variant)
//
// Sets a integer setting of the device. This setting
// can be an unsigned 1,2 or 4 bytes setting. Only valid
// for serial port connections.
//
// Input
//	 mid		: Message ID of message to send
//   bid		: Bus ID of message to send (def 0xFF)
//	 value		: Contains the integer value to be used
//	 valuelen	: Length in bytes of the value
//
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//
short CXbus::setSetting(const unsigned char mid,
						  const unsigned long value, const unsigned short valuelen,
						  const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	msgLen = LEN_MSGHEADER;
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	buffer[IND_LEN] = valuelen;
	swapEndian((unsigned char *)&value, &buffer[msgLen], valuelen);
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	// Read next received message
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			return (m_retVal = XBRV_OK);				// Acknowledge received
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// setSetting (integer & param variant)
//
// Sets a integer setting of the device. This setting
// can be an unsigned 1,2 or 4 bytes setting. Only valid
// for serial port connections.
//
// Input
//	 mid		: Message ID of message to send
//	 param		: For messages that need a parameter (optional)
//   bid		: Bus ID of message to send (def 0xFF)
//	 value		: Contains the integer value to be used
//	 valuelen	: Length in bytes of the value
//
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//
short CXbus::setSetting(const unsigned char mid, const unsigned char param,
						  const unsigned long value, const unsigned short valuelen,
						  const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	msgLen = LEN_MSGHEADER;
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	if (param != 0xFF) {
		msgLen++;
		buffer[IND_LEN] = valuelen+1;
		buffer[IND_DATA0] = param;
	}
	else{
		buffer[IND_LEN] = valuelen;
	}
	swapEndian((unsigned char *)&value, &buffer[msgLen], valuelen);
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	// Read next received message
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			return (m_retVal = XBRV_OK);				// Acknowledge received
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// setSetting (float & no param variant)
//
// Sets a float setting of the device. Only valid
// for serial port connections.
//
// Input
//	 mid		: Message ID of message to send
//   bid		: Bus ID of message to send (def 0xFF)
//	 value		: Contains the float value to be used
//
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
short CXbus::setSetting(const unsigned char mid, const float value, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	msgLen = LEN_MSGHEADER;
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	buffer[IND_LEN] = LEN_FLOAT;
	swapEndian((unsigned char *)&value, &buffer[msgLen], LEN_FLOAT);
	calcChecksum(buffer,LEN_MSGHEADER + LEN_FLOAT);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + LEN_FLOAT);

	// Read next received message
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			return (m_retVal = XBRV_OK);				// Acknowledge received
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// setSetting (float & param variant)
//
// Sets a float setting of the device. Only valid
// for serial port connections.
//
// Input
//	 mid		: Message ID of message to send
//	 param		: For messages that need a parameter (optional)
//   bid		: Bus ID of message to send (def 0xFF)
//	 value		: Contains the float value to be used
//
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//
short CXbus::setSetting(const unsigned char mid, const unsigned char param,
						  const float value, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	msgLen = LEN_MSGHEADER;
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	if (param != 0xFF) {
		msgLen++;
		buffer[IND_LEN] = LEN_FLOAT+1;
		buffer[IND_DATA0] = param;
	}
	else{
		buffer[IND_LEN] = LEN_FLOAT;
	}
	swapEndian((unsigned char *)&value, &buffer[msgLen], LEN_FLOAT);
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	// Read next received message
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			return (m_retVal = XBRV_OK);				// Acknowledge received
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// setSetting (float & param & store variant)
//
// Sets a float setting of the device and with the Store field.
// Only valid for serial port connections
//
// Input
//	 mid		: Message ID of message to send
//	 param		: For messages that need a parameter (optional)
//	 value		: Contains the float value to be used
//	 store		; Store in non-volatile memory (1) or not (0)
//   bid		: Bus ID of message to send (def 0xFF)
//
// Output
//   = XBRV_OK if an Ack message is received
//	 = XBRV_RECVERRORMSG if an error message is received
//	 = XBRV_TIMEOUT if timeout occurred
//
//
short CXbus::setSetting(const unsigned char mid, const unsigned char param,
						  const float value, const bool store, const unsigned char bid)
{
	unsigned char buffer[MAXMSGLEN];
	short msgLen;

	if (m_fileOpen) {
		return (m_retVal = XBRV_INVALIDFORFILEINPUT);
	}
	if (!m_portOpen) {
		return (m_retVal = XBRV_NOINPUTINITIALIZED);
	}
	msgLen = LEN_MSGHEADER;
	buffer[IND_PREAMBLE] = PREAMBLE;
	buffer[IND_BID] = bid;
	buffer[IND_MID] = mid;
	if (param != 0xFF) {
		msgLen++;
		buffer[IND_LEN] = LEN_FLOAT+2;
		buffer[IND_DATA0] = param;
	}
	else{
		buffer[IND_LEN] = LEN_FLOAT+1;
	}
	swapEndian((unsigned char *)&value, &buffer[msgLen], LEN_FLOAT);
	buffer[msgLen+LEN_FLOAT] = store;
	calcChecksum(buffer,LEN_MSGHEADER + buffer[IND_LEN]);

	// Send message
	writeData(buffer, LEN_MSGHEADERCS + buffer[IND_LEN]);

	// Read next received message
	if(readMessageRaw(buffer, &msgLen) == XBRV_OK){
		// Message received
		if(buffer[IND_MID] == (mid+1)) {
			return (m_retVal = XBRV_OK);			// Acknowledge received
		}
		else if (buffer[IND_MID] == MID_ERROR){
			m_deviceError = buffer[IND_DATA0];
			return (m_retVal = XBRV_RECVERRORMSG);	// Error message received
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// getDeviceMode
//
// Requests the current output mode/setting of input (file or serialport)
//  the Outputmode, Outputsettings, DataLength & number of devices
//  are stored in member variables of the Xbus class. These values 
//  are needed for the GetValue functions.
//  The function optionally returns the number of devices
//
// File: expects the Configuration message at the start of the file
//       which holds the OutputMode & OutputSettings. File position 
//       is after the first message
//
// Input
// Output
//	 numDevices : [optional] number of devices connected to port or
//                found in configuration file
//	
//   returns XBRV_OK if the mode & settings are read
//
short CXbus::getDeviceMode(unsigned short *numDevices)
{
	unsigned char mid, data[MAXMSGLEN];
	short datalen;
	
	if (numDevices != NULL) {
		*numDevices = 0;
	}
	
	// In case serial port is used (live device / XM or MT)
	if (m_portOpen) {
		if (reqSetting(MID_INITBUS,data,datalen) != XBRV_OK) {
			return m_retVal;
		}
		
		// Retrieve outputmode + outputsettings
		for (int i = 0; i < datalen / LEN_DEVICEID; i++) {
			if (reqSetting(MID_REQOUTPUTMODE,m_storedOutputMode[BID_MT+i],BID_MT+i) != XBRV_OK) {
				return m_retVal;
			}
			
			if (reqSetting(MID_REQOUTPUTSETTINGS,m_storedOutputSettings[BID_MT+i],BID_MT+i) != XBRV_OK) {
				return m_retVal;
			}
			
			if (reqSetting(MID_REQDATALENGTH,m_storedDataLength[BID_MT+i],BID_MT+i) != XBRV_OK) {
				return m_retVal;
			}
		}

		if (numDevices != NULL) {
			*numDevices = datalen / LEN_DEVICEID;
		}
				
		unsigned char masterDID[4];
		short DIDlen;

		if(reqSetting(MID_REQDID,masterDID,DIDlen) != XBRV_OK) {
			return m_retVal;
		}
		
		if (memcmp(masterDID,data,LEN_DEVICEID) != 0) {
			// Using an XbusMaster
			m_storedOutputMode[0] = OUTPUTMODE_XM;
			m_storedOutputSettings[0] = OUTPUTSETTINGS_XM;
			m_storedDataLength[0] = LEN_SAMPLECNT;
		}
		else{
			m_storedOutputMode[0] = m_storedOutputMode[BID_MT];
			m_storedOutputSettings[0] = m_storedOutputSettings[BID_MT];
			m_storedDataLength[0] = m_storedDataLength[BID_MT];
		}
		return (m_retVal = XBRV_OK);
	}
	else if(m_fileOpen){
		// Configuration message should be the first message in the file
		setFilePos(0);
		if (readMessage(mid,data,datalen) == XBRV_OK) {
			if (mid == MID_CONFIGURATION) {
				unsigned long _numDevices = 0;
				swapEndian(data+CONF_NUMDEVICES, (unsigned char *)&_numDevices,CONF_NUMDEVICESLEN);
				for(unsigned int i = 0; i < _numDevices; i++){
					m_storedOutputMode[BID_MT+i] = 0;
					swapEndian(data+CONF_OUTPUTMODE+i*CONF_BLOCKLEN, (unsigned char *)(m_storedOutputMode+BID_MT+i), 
							   CONF_OUTPUTMODELEN);
					m_storedOutputSettings[BID_MT+i] = 0;
					swapEndian(data+CONF_OUTPUTSETTINGS+i*CONF_BLOCKLEN, (unsigned char *)(m_storedOutputSettings+BID_MT+i), 
							   CONF_OUTPUTSETTINGSLEN);
					m_storedDataLength[BID_MT+i] = 0;
					swapEndian(data+CONF_DATALENGTH+i*CONF_BLOCKLEN, (unsigned char *)(m_storedDataLength+BID_MT+i), 
							   CONF_DATALENGTHLEN);
				}
				if (numDevices != NULL) {
					*numDevices = _numDevices;
				}
				if (memcmp(data+CONF_MASTERDID, data+CONF_DID, LEN_DEVICEID) != 0) {
					// Using an XbusMaster
					m_storedOutputMode[0] = OUTPUTMODE_XM;
					m_storedOutputSettings[0] = OUTPUTSETTINGS_XM;
					m_storedDataLength[0] = LEN_SAMPLECNT;
				}
				else{
					m_storedOutputMode[0] = m_storedOutputMode[BID_MT];
					m_storedOutputSettings[0] = m_storedOutputSettings[BID_MT];
					m_storedDataLength[0] = m_storedDataLength[BID_MT];
				}
				return (m_retVal = XBRV_OK);
			}
		}
		return (m_retVal = XBRV_NOTSUCCESSFUL);
	}
	return (m_retVal = XBRV_NOINPUTINITIALIZED);
}

////////////////////////////////////////////////////////////////////
// setDeviceMode
//
// Sets the current output mode/setting of input (not for file-based 
//   inputs)
//
// Input
//	 OutputMode		: OutputMode to be set in device & stored in Xbus 
//						class member variable
//	 OutputSettings : OutputSettings to be set in device & stored in 
//						Xbus class member variable
// Output
//	
//   returns XBRV_OK if the mode & settings are read
//
short CXbus::setDeviceMode(unsigned long OutputMode, unsigned long OutputSettings, const unsigned char bid)
{
	// In case serial port is used (live XM / MT)
	if (m_portOpen) {
		// Set OutputMode
		if (setSetting(MID_SETOUTPUTMODE, OutputMode, LEN_OUTPUTMODE, bid) != XBRV_OK) {
			return m_retVal;
		}
		if (bid == BID_MASTER) {
			m_storedOutputMode[0] = m_storedOutputMode[BID_MT] = OutputMode;
		}
		else{
			m_storedOutputMode[bid] = OutputMode;
		}
		// Set OutputSettings
		if (setSetting(MID_SETOUTPUTSETTINGS, OutputSettings, LEN_OUTPUTSETTINGS, bid) != XBRV_OK) {
			return m_retVal;
		}
		if (bid == BID_MASTER) {
			m_storedOutputSettings[0] = m_storedOutputSettings[BID_MT] = OutputSettings;
		}
		else{
			m_storedOutputSettings[bid] = OutputSettings;
		}
		// Get DataLength from device
		if (OutputMode != OUTPUTMODE_XM) {
			unsigned long value;
			if (reqSetting(MID_REQDATALENGTH, value, bid) == XBRV_OK) {
				if (bid == BID_MASTER) {
					m_storedDataLength[0] = m_storedDataLength[BID_MT] = value;
				}
				else{
					m_storedDataLength[bid] = value;
				}
			}
		}else{
			m_storedDataLength[0] = LEN_SAMPLECNT;
		}
		return (m_retVal = XBRV_OK);
	}
	return (m_retVal = XBRV_INVALIDFORFILEINPUT);
}

////////////////////////////////////////////////////////////////////
// getMode
//
// Gets the output mode/setting used in Xbus class and the corresponding
//  datalength. These variables are set by the functions GetDeviceMode, 
//  SetDeviceMode or SetMode
//
// Input
// Output
//	 OutputMode		: OutputMode stored in Xbus class member variable
//	 OutputSettings : OutputSettings stored in Xbus class member variable
//	
//   returns always XBRV_OK
//
short CXbus::getMode(unsigned long &OutputMode, unsigned long &OutputSettings, 
					   unsigned short &dataLength, const unsigned char bid)
{
	unsigned char nbid = (bid == BID_MASTER)?0:bid;
	OutputMode = m_storedOutputMode[nbid];
	OutputSettings = m_storedOutputSettings[nbid];
	dataLength = m_storedDataLength[nbid];
	return (m_retVal = XBRV_OK);
}

////////////////////////////////////////////////////////////////////
// setMode
//
// Sets the output mode/setting used in Xbus class. Use the function
//  GetDeviceMode to retrieve the current values of file/device.
// This function will also calculate the data length field
//
// Input
//	 OutputMode		: OutputMode to be stored in Xbus class member variable
//	 OutputSettings : OutputSettings to be stored in Xbus class member variable
// Output
//	
//   returns always XBRV_OK
//
short CXbus::setMode(unsigned long OutputMode, unsigned long OutputSettings, const unsigned char bid)
{
	unsigned char nbid = bid;

	if (nbid == BID_MASTER){
		nbid = 0;
	}
	m_storedOutputMode[nbid] = OutputMode;
	m_storedOutputSettings[nbid] = OutputSettings;
	if (OutputMode == INVALIDSETTINGVALUE || OutputSettings == INVALIDSETTINGVALUE) {
		m_storedDataLength[nbid] = 0;
	}
	else{
		unsigned short dataLength = 0;
		if (OutputMode & OUTPUTMODE_MT9) {
			dataLength = ((OutputSettings & OUTPUTSETTINGS_TIMESTAMP_MASK) == OUTPUTSETTINGS_SAMPLECNT)?LEN_SAMPLECNT:0 + LEN_RAWDATA;
		}
		else if (OutputMode == OUTPUTMODE_XM){
			// XbusMaster concatenates sample counter
			dataLength = LEN_SAMPLECNT;
		}
		else{
			if (OutputMode & OUTPUTMODE_RAW) {
				dataLength = LEN_RAWDATA;
			}
			else{
				if (OutputMode & OUTPUTMODE_CALIB) {
					dataLength = LEN_CALIBDATA;
				}
				if (OutputMode & OUTPUTMODE_ORIENT) {
					switch(OutputSettings & OUTPUTSETTINGS_ORIENTMODE_MASK) {
					case OUTPUTSETTINGS_QUATERNION:
						dataLength += LEN_ORIENT_QUATDATA;
						break;
					case OUTPUTSETTINGS_EULER:
						dataLength += LEN_ORIENT_EULERDATA;
						break;
					case OUTPUTSETTINGS_MATRIX:
						dataLength += LEN_ORIENT_MATRIXDATA;
						break;
					default:
						break;
					}
				}
			}
			switch(OutputSettings & OUTPUTSETTINGS_TIMESTAMP_MASK) {
			case OUTPUTSETTINGS_SAMPLECNT:
				dataLength += LEN_SAMPLECNT;
				break;
			default:
				break;
			}
		}
		m_storedDataLength[nbid] = dataLength;
	}
	// If not XbusMaster store also in BID_MT
	if (bid == BID_MASTER && OutputMode != OUTPUTMODE_XM) {
		m_storedOutputMode[BID_MT] = m_storedOutputMode[0];
		m_storedOutputSettings[BID_MT] = m_storedOutputSettings[0];
		m_storedDataLength[BID_MT] = m_storedDataLength[0];
	}
	return (m_retVal = XBRV_OK);
}

////////////////////////////////////////////////////////////////////
// getValue (unsigned short variant)
//
// Retrieves a unsigned short value from the data input parameter
// This function is valid for the following value specifiers:
//		VALUE_RAW_TEMP
//		VALUE_SAMPLECNT		
//
// Use getDeviceMode or setMode to initialize the Outputmode
// and Outputsettings member variables used to retrieve the correct
// value
//
// Input
//	 valueSpec		: Specifier of the value to be retrieved
//	 data[]			: Data field of a MTData / BusData message
//	 bid			: bus identifier of the device of which the
//						value should be returned (default = BID_MT)
// Output
//	 value			: reference to unsigned short in which the retrieved
//						value will be returned
//	
//	Return value
//    XBRV_OK		: value is successfully retrieved
//	  != XBRV_OK	: not successful
//
short CXbus::getValue(const unsigned long valueSpec, unsigned short &value, const unsigned char data[], 
						const unsigned char bid)
{
	short offset = 0;
	unsigned char nbid = bid;

	if(nbid == BID_MASTER){
		nbid = 0;
	}

	// Check for invalid mode/settings
	if (m_storedOutputMode[nbid] == INVALIDSETTINGVALUE || m_storedOutputSettings[nbid] == INVALIDSETTINGVALUE) {
		return (m_retVal = XBRV_NOVALIDMODESPECIFIED);
	}

	// Calculate offset for XM input
	if (m_storedOutputMode[0] == OUTPUTMODE_XM) {
		int i = 0;
		while (i < nbid) {
			offset += m_storedDataLength[i++];
		}
	}
	
	// Check if data is unsigned short & available in data
	m_retVal = XBRV_INVALIDVALUESPEC;
	if (valueSpec == VALUE_RAW_TEMP) {
		if (m_storedOutputMode[nbid] & (OUTPUTMODE_RAW | OUTPUTMODE_MT9)) {
			offset += (m_storedOutputMode[nbid] == OUTPUTMODE_MT9 && \
					   m_storedOutputSettings[nbid] == OUTPUTSETTINGS_SAMPLECNT)?LEN_SAMPLECNT:0;
			swapEndian(data + offset + valueSpec*LEN_UNSIGSHORT*3, (unsigned char *)&value, LEN_RAW_TEMP);
			m_retVal = XBRV_OK;
		}
	}
	else if (valueSpec == VALUE_SAMPLECNT) {
		if ((m_storedOutputSettings[nbid] & OUTPUTSETTINGS_TIMESTAMP_MASK) == OUTPUTSETTINGS_SAMPLECNT) {
			if (!(m_storedOutputMode[nbid] == OUTPUTMODE_MT9)) {
				offset += m_storedDataLength[nbid] - LEN_SAMPLECNT;
			}
			swapEndian(data + offset, (unsigned char *)&value, LEN_SAMPLECNT);
			m_retVal = XBRV_OK;
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// getValue (array of unsigned short variant)
//
// Retrieves an array of unsigned short values from the data input 
// parameter. This function is valid for the following value specifiers:
//		VALUE_RAW_ACC
//		VALUE_RAW_GYR
//		VALUE_RAW_MAG
//
// Use getDeviceMode or setMode to initialize the Outputmode
// and Outputsettings member variables used to retrieve the correct
// value
//
// Input
//	 valueSpec		: Specifier of the value to be retrieved
//	 data[]			: Data field of a MTData / BusData message
//	 bid			: bus identifier of the device of which the
//						value should be returned (default = BID_MT)
// Output
//	 value[]		: pointer to array of unsigned shorts in which the 
//						retrieved values will be returned
//	
//	Return value
//    XBRV_OK		: value is successfully retrieved
//	  != XBRV_OK	: not successful
//
short CXbus::getValue(const unsigned long valueSpec, unsigned short value[], const unsigned char data[], 
						const unsigned char bid)
{
	short offset = 0;
	unsigned char nbid = bid;

	if(nbid == BID_MASTER){
		nbid = 0;
	}
	// Check for invalid mode/settings
	if (m_storedOutputMode[nbid] == INVALIDSETTINGVALUE || m_storedOutputSettings[nbid] == INVALIDSETTINGVALUE) {
		return (m_retVal = XBRV_NOVALIDMODESPECIFIED);
	}

	// Calculate offset for XM input
	if (m_storedOutputMode[0] == OUTPUTMODE_XM) {
		int i = 0;
		while (i < nbid) {
			offset += m_storedDataLength[i++];
		}
	}

	// Check if data is unsigned short, available in data & retrieve data
	m_retVal = XBRV_INVALIDVALUESPEC;
	if (valueSpec >= VALUE_RAW_ACC && valueSpec <= VALUE_RAW_MAG) {
		if (m_storedOutputMode[nbid] & (OUTPUTMODE_RAW | OUTPUTMODE_MT9)) {
			offset += valueSpec*LEN_UNSIGSHORT*3;
			offset += (m_storedOutputMode[nbid] == OUTPUTMODE_MT9 && \
				       m_storedOutputSettings[nbid] == OUTPUTSETTINGS_SAMPLECNT)?LEN_SAMPLECNT:0;
			for (int i = 0; i < 3; i++) {
				swapEndian(data+offset+i*LEN_UNSIGSHORT, (unsigned char *)value+i*LEN_UNSIGSHORT, LEN_UNSIGSHORT);
			}
			m_retVal = XBRV_OK;
		}
	}
	return m_retVal;
}

////////////////////////////////////////////////////////////////////
// getValue (array of floats variant)
//
// Retrieves an array of float values from the data input parameter. 
// This function is valid for the following value specifiers:
//		VALUE_CALIB_ACC
//		VALUE_CALIB_GYR
//		VALUE_CALIB_MAG
//		VALUE_ORIENT_QUAT
//		VALUE_ORIENT_EULER
//		VALUE_ORIENT_MATRIX
//
// Use getDeviceMode or setMode to initialize the Outputmode
// and Outputsettings member variables used to retrieve the correct
// value
//
// Input
//	 valueSpec		: Specifier of the value to be retrieved
//	 data[]			: Data field of a MTData / BusData message
//	 bid			: bus identifier of the device of which the
//						value should be returned (default = BID_MT)
// Output
//	 value[]		: pointer to array of floats in which the 
//						retrieved values will be returned
//	
//	Return value
//    XBRV_OK		: value is successfully retrieved
//	  != XBRV_OK	: not successful
//
short CXbus::getValue(const unsigned long valueSpec, float value[], const unsigned char data[], 
						const unsigned char bid)
{
	short offset = 0;
	unsigned char nbid = bid;

	if(nbid == BID_MASTER){
		nbid = 0;
	}

	// Check for invalid mode/settings
	if (m_storedOutputMode[nbid] == INVALIDSETTINGVALUE || m_storedOutputSettings[nbid] == INVALIDSETTINGVALUE) {
		return (m_retVal = XBRV_NOVALIDMODESPECIFIED);
	}

	// Calculate offset for XM input
	if (m_storedOutputMode[0] == OUTPUTMODE_XM) {
		int i = 0;
		while (i < nbid) {
			offset += m_storedDataLength[i++];
		}
	}

	// Check if data is float & available in data
	m_retVal = XBRV_INVALIDVALUESPEC;
	if (valueSpec >= VALUE_CALIB_ACC && valueSpec <= VALUE_CALIB_MAG) {
		if (m_storedOutputMode[nbid] & OUTPUTMODE_CALIB) {
			offset += (valueSpec-VALUE_CALIB_ACC)*LEN_CALIB_ACCX*3;
			for (int i = 0; i < 3; i++) {
				swapEndian(data+offset+i*LEN_FLOAT, (unsigned char *)value+i*LEN_FLOAT, LEN_FLOAT);
			}
			m_retVal = XBRV_OK;
		}
	}
	else if (valueSpec >= VALUE_ORIENT_QUAT && valueSpec <= VALUE_ORIENT_MATRIX) {
		offset += ((m_storedOutputMode[nbid] & OUTPUTMODE_CALIB) != 0)?LEN_CALIBDATA:0;
		if (m_storedOutputMode[nbid] & OUTPUTMODE_ORIENT) {
			unsigned long orientmode = m_storedOutputSettings[nbid] & OUTPUTSETTINGS_ORIENTMODE_MASK;
			int nElements = 0;
			switch(valueSpec) {
			case VALUE_ORIENT_QUAT:
				if (orientmode == OUTPUTSETTINGS_QUATERNION) {
					nElements = LEN_ORIENT_QUATDATA / LEN_FLOAT;
					m_retVal = XBRV_OK;
				}
				break;
			case VALUE_ORIENT_EULER:
				if (orientmode == OUTPUTSETTINGS_EULER) {
					nElements = LEN_ORIENT_EULERDATA / LEN_FLOAT;
					m_retVal = XBRV_OK;
				}
				break;
			case VALUE_ORIENT_MATRIX:
				if (orientmode == OUTPUTSETTINGS_MATRIX) {
					nElements = LEN_ORIENT_MATRIXDATA / LEN_FLOAT;
					m_retVal = XBRV_OK;
				}
				break;
			default:
				break;
			}
			for (int i = 0; i < nElements; i++) {
				swapEndian(data+offset+i*LEN_FLOAT, (unsigned char *)value+i*LEN_FLOAT, LEN_FLOAT);
			}
		}
	}
	return m_retVal;
}

//////////////////////////////////////////////////////////////////////
// getLastDeviceError
//
// Returns the last reported device error of the latest received Error 
//	message
//
// Output
//   Error code
short CXbus::getLastDeviceError()
{
	return m_deviceError;
}

//////////////////////////////////////////////////////////////////////
// getLastRetVal
//
// Returns the returned value of the last called function
//
// Output
//   Return value
short CXbus::getLastRetVal()
{
	return m_retVal;
}

//////////////////////////////////////////////////////////////////////
// setTimeOut
//
// Sets the time out value in milliseconds used by the functions
// Use 0 for infinite timeout
//
// Output
//   XBRV_OK is set, XBRV_INVALIDTIMEOUT if time value < 0
short CXbus::setTimeOut(short timeOutMs)
{
	if(timeOutMs >= 0){
		m_timeOut = timeOutMs;
		return (m_retVal = XBRV_OK);
	}
	else
		return (m_retVal = XBRV_INVALIDTIMEOUT);
}

//////////////////////////////////////////////////////////////////////
// swapEndian
//
// Convert 2 or 4 bytes data from little to big endian or back
//
// Input
//	 input	: Pointer to data to be converted
//   output	: Pointer where converted data is stored
//   length	: Length of setting (0,2 & 4)
//
// Remarks:
//	 Allocate enough bytes for output buffer
void CXbus::swapEndian(const unsigned char input[], unsigned char output[], const short length)
{
	switch(length) {
	case 0:
		break;
	case 1:
		output[0] = input[0];
		break;
	case 2:
		output[0] = input[1];
		output[1] = input[0];
		break;
	case 4:
		output[0] = input[3];
		output[1] = input[2];
		output[2] = input[1];
		output[3] = input[0];
		break;
	default:
		break;
	}
}

//////////////////////////////////////////////////////////////////////
// calcChecksum
//
// Calculate and append checksum to msgBuffer
//
void CXbus::calcChecksum(unsigned char *msgBuffer, const int msgBufferLength)
{
	unsigned char checkSum = 0;
	int i;

	for(i = 1; i < msgBufferLength; i++)		
		checkSum += msgBuffer[i];

	msgBuffer[msgBufferLength] = -checkSum;	// Store chksum
}

//////////////////////////////////////////////////////////////////////
// checkChecksum
//
// Checks if message checksum is valid
//
// Output
//   returns true checksum is OK
bool CXbus::checkChecksum(const unsigned char *msgBuffer, const int msgBufferLength)
{
	unsigned char checkSum = 0;
	int i;

	for (i = 1; i < msgBufferLength; i++)
		checkSum += msgBuffer[i];

	if (checkSum == 0) {
		return true;
	}
	else {
		return false;
	}
}


