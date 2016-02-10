#ifndef _MY_TIMER_H
#define _MY_TIMER_H

#include <stdlib.h>
#include <stdio.h>
#include "highgui.h"
#include <time.h>
#include <string>

// a rough-grain timer (1 second resolution)
class MyTimer {
public:

	//** attributes **//
	time_t _ctime;

	//** constructors **//
	MyTimer() { time(&_ctime); };
	~MyTimer() {};

	//** methods **//
	int tday();
	int tmonth();
	int tyear();
	int thour();
	int tmin();
	int tsec();
	int lifetime();
	std::string timestring();
	long int timestamp();
};

// a fine grain timer (1 nano-second resolution)
class PerfTimer {
public:

	//** attributes **//
	LARGE_INTEGER _ticksPerSecond;
	LARGE_INTEGER _start;

	PerfTimer();
	~PerfTimer() {};

	double elapsed();
	void print();
	void print(const char *string);
	void reset();
};

#endif
