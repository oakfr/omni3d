#include "util/util.h"
#include <windows.h>

void printFileInfo (const char *filename)
{
	struct _stat buf;
	int result;
	
	/* Get data associated with "crt_stat.c": */
	result = _stat( filename, &buf );
	
	/* Check if statistics are valid: */
	if (result != -1) {
		printf("warning: file %s does not exist.\n",filename);
		return;
	}

	/* Output some of the statistics: */
	printf( "File size     : %ld\n", buf.st_size );
	printf( "Drive         : %c:\n", buf.st_dev + 'A' );
	printf( "Time modified : %s", ctime( &buf.st_mtime ) );
	
}

time_t getModificationTime (const char *filename)
{
	struct _stat buf;
	int result;

	char fn[256];
	sprintf(fn,"%s",filename);

	/* Get data associated with "crt_stat.c": */
	result = _stat( fn, &buf );
	
	/* Check if statistics are valid: */
	if (result == -1) {
		printf("warning: file %s does not exist.\n",filename);
		return -1;
	}
	
	return buf.st_mtime;
	
}

bool isFileOlder (const char *filename1, const char *filename2)
{
	time_t t1 = getModificationTime (filename1);
	time_t t2 = getModificationTime (filename2);

	if (t1 == -1) 
		return false;

	if (t2 == -1)
		return true;

	return (t1 > t2);
}

FILE *fileOpen (const char *filename, const char *mode)
{
	char fn[256];
	sprintf(fn,"%s",filename);
	FILE *f = fopen (fn, mode);
	
	if (f == NULL) {
		f = fopen (filename, mode);

		if (f == NULL) {
			fprintf(stderr,"failed to open file %s\n",filename);
		}
	}

	return f;
}

void fileClose (FILE *f)
{
	fclose (f);
}

void fileTouch (const char *filename)
{
	FILE *f = fopen(filename,"w");
	if (f == NULL)
		return;

	fclose(f);
}

void fileErase (const char *filename)
{
	char fn[256];
	sprintf(fn,"%s",filename);
	FILE *f = fopen (fn, "w");
	fclose(f);
}

bool fileExist (const char *filename)
{
	struct _stat buf;
	int result;

	char fn[256];
	sprintf(fn,"%s",filename);

	/* Get data associated with "crt_stat.c": */
	result = _stat( fn, &buf );

	return (result != -1);
}

void fileRemove (const char *filename)
{
	if (!fileExist(filename))
		return;

	char fn[256];
	sprintf(fn,"%s",filename);

	int res = remove( fn );
	assert (res != -1);
}

bool fileScan (FILE *f, char *str, int n)
{
	char ch[256];
	bool ok = false;
	//printf("FILE SCAN:\n");

	while (!ok && !feof(f)) {
		fscanf(f,"%s",ch);
		//printf("%s\n",ch);
		if (strncmp(ch,str,n) == 0)
			ok = true;
	}

	//printf("********* %d\n",ok);
	return ok;
}

bool fileOffset (FILE *f, int n)
{
	char c;
	for (int i=0;i<n;i++)
		fscanf(f,"%c",&c);
	return (!feof(f));
}

bool isEmptyDirectory (char *dirname)
{
	WIN32_FIND_DATA data;
	HANDLE h = FindFirstFile (dirname, &data);
	if (h == INVALID_HANDLE_VALUE)
		return true;
	else {
		FindClose(h);
		return false;
	}
}

void emptyDirectory (char *dirname)
{
	WIN32_FIND_DATA FindFileData;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	char DirSpec[MAX_PATH + 1];  // directory specification
	DWORD dwError;
	TCHAR DirPath[MAX_PATH + 1]; 

	strncpy (DirSpec, dirname, strlen(dirname)+1);
	strncat (DirSpec, "\\*", 3);
	
	hFind = FindFirstFile(DirSpec, &FindFileData);
	
	if (hFind == INVALID_HANDLE_VALUE) 
	{
		return;
	} 
	else 
	{
		//CloseHandle(hFind);

		strncpy (DirPath, dirname, strlen(dirname)+1);
		strncat (DirPath, "\\", 2);
		strncat (DirPath, FindFileData.cFileName, strlen(FindFileData.cFileName)+1);

		DeleteFile(DirPath);

		while (FindNextFile(hFind, &FindFileData) != 0) 
		{
			//CloseHandle(hFind);
		strncpy (DirPath, dirname, strlen(dirname)+1);
		strncat (DirPath, "\\", 2);
		strncat (DirPath, FindFileData.cFileName, strlen(FindFileData.cFileName)+1);

		DeleteFile(DirPath);
			//printf ("Next file name is %s\n", FindFileData.cFileName);
		}
		
		dwError = GetLastError();
		FindClose(hFind);
		if (dwError != ERROR_NO_MORE_FILES) 
		{
			printf ("FindNextFile error. Error is %u\n", dwError);
			return;
		}
	}
}


// create a data info file in the database
void createDataInfoFile (char *dirname, int w, int h, int nimages, double framerate, int synthetic)
{
	MyTimer timer;
	std::string dataname = std::string(dirname) + "/data.ini";
	FILE *file = fopen(dataname.c_str(),"w");
	if (file == NULL) {
		printf("ERROR creating file %s.\n",dataname.c_str());
		assert(false);
	}
	fprintf(file,"DATE= %04d%02d%02d\n",timer.tyear(),timer.tmonth(),timer.tday());
	fprintf(file,"WIDTH= %d\nHEIGHT= %d\n",w,h);
	fprintf(file,"FRAME RATE= %f\n", framerate);
	fprintf(file,"CAMERA= LADYBUG\n");
	fprintf(file,"N_IMAGES= %d\n",nimages);
	fprintf(file,"SYNTHETIC = %d\n", synthetic);
	fprintf(file,"FILETYPE = bmp\n");  // bmp images by default
	fprintf(file,"LADYBUG_ID = %d\n", DEFAULT_LADYBUG_ID); // default ladybug ID
	fclose(file);
}