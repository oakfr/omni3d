#ifndef _HISTOGRAM_H__
#define _HISTOGRAM_H__

#include "basic.h"

////////////////////////////////////////////////
// a basic histogram class
//

class Histogram {
public:

	Histogram();
	~Histogram();
	Histogram( int k );

	void normalize(); // normalize volume to 1.0
	double Volume(); // compute the volume
	void Increment( int k ); // increment bucket
	double Diff( Histogram &histogram); // diff two histograms
	double getVal ( int k ) {  assert( k < n ); return val[k]; }
	int getSize() { return n;}
	void reset();
	bool valid() { return ( val != NULL ); }
	double maxValue();
	bool Histogram::Write( const char *file, long int &pos );
	bool Histogram::Read ( const char *file, long int pos );
	void Histogram::gaussianFilter ();

protected:

	int n; // number of buckets
	double *val; // values
	double _max; // max value
};

#endif
