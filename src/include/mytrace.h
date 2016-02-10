////////////////////////////////////////////////
// MyTrace is small and simple TRACE utility. //
// Based on ATLTRACE                          //
// -=MaGGuS=- (maggus@mail.ru)   2006         //
////////////////////////////////////////////////

#ifndef __LOG_H__
#define __LOG_H__

#pragma once

/////// MT defines start ///////

//#define MT_USE_ATLTRACE				// forward trace messages to ATLTRACE
//#define MT_KEEP_ATLTRACE_LEVELS	// convert LOG levels to ATLTRACE levels, requres DO_ATLTRACE
//#define MT_REDIRECT_STDOUT		// redirect all output to stdout into log file. Use this carefully!

/////// MT defines end ///////

//#ifdef MT_USE_ATLTRACE	// for ATLTRACE support
//#include <atldef.h>
//#include <atlconv.h>
//#endif // MT_USE_ATLTRACE
enum Level {LEVEL_ERROR, LEVEL_WARNING, LEVEL_INFO, LEVEL_DEBUG};		// LOG message severity levels

namespace MT
{


#ifndef _DEBUG	/*empty macroses in release mode*/

#define LOG_INIT(x)			{}
#define LOG_INIT_EX(x, y)	{}
#define LOG_LEVEL(x)		{}
#define LOG_DEINIT			{}
#define LOG					{}

#else	// _DEBUG

	class CMyTrace;	// forward declaration

#define LOG_INIT(x)			MT::CMyTrace::Init(x);
#define LOG_INIT_EX(x, y)	MT::CMyTrace::InitEx((x), (y));
#define LOG_LEVEL(x)		MT::CMyTrace::SetLevel(x);
#define LOG_DEINIT			MT::CMyTrace::Deinit();
#define LOG					MT::CMyTrace(__FILE__, __LINE__)

	/* MyTrace class */
	class CMyTrace
	{
	public:
		CMyTrace(const char *pszFileName, int nLineNo)
			: m_pszFileName(pszFileName), m_nLineNo(nLineNo)
			//m_callTime(time(NULL))
		{
		}

		/* Add trace message to log with desired severity level */
		void __cdecl operator()(UINT nLevel, LPCTSTR pszFmt, ...) const
		{
			va_list ptr; 
			va_start(ptr, pszFmt);
			if(nLevel <= (UINT)traceLevel)
			{
				//CString strFormat(pszFmt);
				char strFormat[1024];
				char strHead[512];

				// compute time
				time_t rawtime;
				struct tm * timeinfo;
				time ( &rawtime );
				timeinfo = localtime ( &rawtime );

				sprintf(strHead, "L%d,%d/%d/%d %d:%d:%d,%s,%d",nLevel,timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year - 100, 
					timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, m_pszFileName, m_nLineNo );

				printf("%s: ",strHead);
				vsprintf(strFormat,pszFmt,ptr);
				printf("%s\n",strFormat);

				//decorateMessage(strFormat, nLevel);
				if(pLOG_FILE != NULL)
					printMessageV(pLOG_FILE, strHead, strFormat);
			}
			va_end(ptr);
		};

		/* Add trace messge to log with default INFO severity level */
		void __cdecl operator()( LPCTSTR pszFmt, ...) const
		{
			va_list ptr; va_start(ptr, pszFmt);
			//CString strFormat(pszFmt);
			char strFormat[512];
			decorateMessage(strFormat);
			
			char strHead[512];
			
			// compute time
			time_t rawtime;
			struct tm * timeinfo;
			time ( &rawtime );
			timeinfo = localtime ( &rawtime );
			
			sprintf(strHead, "%d/%d/%d %d:%d:%d %s [%d]",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year - 100, 
				timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, m_pszFileName, m_nLineNo );
			
			if(pLOG_FILE != NULL)
				printMessageV(pLOG_FILE, strHead, strFormat);
			//#ifdef MT_USE_ATLTRACE
//			ATL::CTrace::s_trace.TraceV(m_pszFileName, m_nLineNo, atlTraceGeneral, 0, strFormat, ptr);
//#endif	// MT_USE_ATLTRACE
			va_end(ptr);
		};
		
		/* Open main log file */
		static void Noop()
		{
		}

		static bool Init(LPCTSTR fName)
		{
			bool bRes1= false;
			if( (pLOG_FILE = fopen( fName, "w" )) == NULL )
			{
				::MessageBox(NULL, "Failed to open LOG file", "Notice", 0x10000);
				//LOG(_T("\t\t\t FAILED TO OPEN LOG FILE: %s"), fName);
				bRes1 = false;
			}
			else
			{
//#ifdef MT_REDIRECT_STDOUT
				// redirect stdout to enable simple "printf" output to file
//				*stdout = *pLOG_FILE;
//				setvbuf(stdout,NULL,_IONBF,0);	// turn off cashing
				//sync_with_stdio();	// check
//#endif	// MT_REDIRECT_STDOUT
				//LOG(_T("\t\t*** Log Started at %s ***\n\n"), CTime(time(NULL)).Format("%c"));
				bRes1 = true;
			}
			return bRes1;
		};

		/* Open main log file and extra log file for errors report only */
		static bool InitEx(LPCTSTR fName, LPCTSTR fNameEx)
		{
			bool bRes1 = Init(fName), bRes2 = false;
			// open extra log file for errors only
			if( (pLOG_FILE_ERRORS = fopen( fNameEx, "w" )) == NULL )
			{
				::MessageBox(NULL, "Failed to open Extra LOG file", "Notice", 0x10000);
				//LOG(_T("\t\t\t FAILED TO OPEN EXTRA LOG FILE: %s"), fNameEx);
				bRes2 = false;
			}
			else
			{
				//LOG(_T("\t\t Extra Log file opened"));
				bRes2 = true;
			}
			return (bRes1 && bRes2);
		};

		/* Close all log files */
		static void Deinit()
		{
			if(pLOG_FILE_ERRORS != NULL)	// close extra log file
			{
				//LOG(_T("\t\t Extra Log file closed"));
				fflush(pLOG_FILE_ERRORS);
				fclose(pLOG_FILE_ERRORS);
				pLOG_FILE_ERRORS = NULL;
			}
			if(pLOG_FILE != NULL)	// close main log file
			{
				//LOG(_T("\n\t\t*** Log Finished at %s ***"), CTime(time(NULL)).Format("%c"));
				fflush(pLOG_FILE);
				fclose(pLOG_FILE);
				pLOG_FILE = NULL;
			}
		};

		static void SetLevel(Level level)
		{
			traceLevel = level;
		};

	private:
		/* Change message format into desired form */
		void __cdecl decorateMessage(char *strFmt, int nLevel = LEVEL_INFO) const
		{
			//CString strLevel;
			char strLevel[10];

			switch(nLevel)
			{
			case LEVEL_ERROR:
				sprintf(strLevel,"%s","ERROR");
				//strLevel = _T("ERROR");
				break;
			case LEVEL_WARNING:
				sprintf(strLevel,"%s","WARNING");
				//strLevel = _T("WARNING");
				break;
			case LEVEL_INFO:
				sprintf(strLevel,"%s","INFO");
				//strLevel = _T("INFO");
				break;
			case LEVEL_DEBUG:
				sprintf(strLevel,"%s","DEBUG");
				//strLevel = _T("DEBUG");
				break;
			default:
				sprintf(strLevel,"%s","UNKNOWN");
				//strLevel = _T("UNKNOWN");
				break;
			}
			//CString decMsg;
			//decMsg.Format("%s\t%s(%d)\t%s:\n%s%s", strLevel, m_pszFileName, m_nLineNo, m_callTime.Format("%c"), 
			//	strFmt, strFmt.Right(1) != _T("\n") ? _T("\n") : "" );	// add line end if requred
			//strFmt = decMsg;
			sprintf(strFmt,"%s (%d): %s\n",m_pszFileName,m_nLineNo,strFmt);
		};
		
		/* Printout message into file */
		void __cdecl printMessageV(FILE *pFile,  char *strHead, char *strFmt) const
		{
			if(!pFile) { assert( false ); return; }
//			fprintf(pFile,"%s [%d]: ",m_pszFileName,m_nLineNo);
			fprintf(pFile,"%s: ",strHead);
			fprintf(pFile,strFmt);
			fprintf(pFile,"\n");
			
		}

		const char *const m_pszFileName;	// curent source file name
		const int m_nLineNo;		// curent source line number
		//const CTime m_callTime;	// time of call
		static FILE *pLOG_FILE;	// trace log file
		static FILE *pLOG_FILE_ERRORS;	// trace errors file
		static Level traceLevel;	// minimum severity level to report
	};

	__declspec( selectany ) FILE *CMyTrace::pLOG_FILE = NULL;
	__declspec( selectany ) FILE *CMyTrace::pLOG_FILE_ERRORS = NULL;
	__declspec( selectany ) Level CMyTrace::traceLevel = LEVEL_DEBUG;

#endif	// _DEBUG

};	// namespace MT

#endif  // __LOG_H__