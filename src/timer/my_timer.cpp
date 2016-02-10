#include "my_timer.h"

int MyTimer::thour()
{
	time_t rawtime;
	struct tm * timeinfo;
	
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	return timeinfo->tm_hour;
}

int MyTimer::tmin()
{
	time_t rawtime;
	struct tm * timeinfo;
	
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	return timeinfo->tm_min;
}

int MyTimer::tsec()
{
	time_t rawtime;
	struct tm * timeinfo;
	
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	return timeinfo->tm_sec;
}

int MyTimer::tyear()
{
	time_t rawtime;
	struct tm * timeinfo;
	
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	return timeinfo->tm_year + 1900;
}

int MyTimer::tmonth()
{
	time_t rawtime;
	struct tm * timeinfo;
	
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	return timeinfo->tm_mon + 1;
}

int MyTimer::tday()
{
	time_t rawtime;
	struct tm * timeinfo;
	
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	return timeinfo->tm_mday;
}

int MyTimer::lifetime()
{
	time_t rawtime;
	time ( &rawtime );
	return difftime(rawtime,_ctime);
}

// timestamp = ddhhmmss (unique over a month and at the second granularity)
long int MyTimer::timestamp()
{
	long int timestamp = 0;

	//timestamp += tmonth() * 100000000;
	timestamp += tday() * 1000000;
	timestamp += thour() * 10000;
	timestamp += tmin() * 100;
	timestamp += tsec();

	return timestamp;
}

std::string MyTimer::timestring()
{
	int hour = thour();
	int min = tmin();
	int sec = tsec();

	char *str = (char*)malloc(10*sizeof(char));
	sprintf(str,"%d:%d:%d",hour,min,sec);
	std::string res = std::string(str);
	delete str;
	return res;
}

PerfTimer::PerfTimer()
{
	QueryPerformanceFrequency(&_ticksPerSecond); 
	QueryPerformanceCounter(&_start);
}

void PerfTimer::reset()
{
	QueryPerformanceCounter(&_start);
}

double PerfTimer::elapsed()
{
	LARGE_INTEGER end;
	QueryPerformanceCounter(&end);

	LONGLONG diff = end.QuadPart - _start.QuadPart;
	return (double)(diff)/_ticksPerSecond.QuadPart;
}

void PerfTimer::print()
{
	printf("elapsed time (sec): %f\n",elapsed());
}

void PerfTimer::print(const char *string)
{
	printf(string);
	printf(" - elapsed time (sec): %f\n",elapsed());
}