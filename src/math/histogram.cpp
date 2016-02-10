#include "histogram.h"
#include "numerical.h"

Histogram::Histogram()
{
	n = 0;
	val = NULL;
	_max = 0.0;
}

Histogram::Histogram( int k )
{
	n = k;
	val = (double*)malloc( n*sizeof( double ) );
	for (int i=0;i<n;i++)
		val[i] = 0.0;
	_max = 0.0;
}

Histogram::~Histogram ()
{
	if ( val != NULL )
		delete val;
}

double Histogram::Volume ()
{
	if ( val == NULL )
		return 0.0;

	double volume = 0.0;

	for (int i=0;i<n;i++)
		volume += val[i];

	return volume;
}

void Histogram::normalize ()
{
	// flatten the first and the last bucket	
	val[n-1] = val[n-2];
	val[0] = val[1];

	// apply a gaussian filter
	gaussianFilter();
}

void Histogram::gaussianFilter ()
{
	if ( !valid() )
		return;

	double *temp;
	temp = (double*)malloc(n*sizeof(double));

	temp[0] = val[0];
	temp[n-1] = val[n-1];

	for ( int i=1;i<n-1;i++ ) {
		temp[i] = ( val[i-1] + 2 * val[i] + val[i+1] ) / 4.0;
	}

	for ( i=0;i<n;i++ )
		val[i] = temp[i];

	delete temp;
}

void Histogram::Increment( int k )
{
	assert( val != NULL );
	assert( k < n );

	val[k] += 1.0;

	if ( val[k] > _max )
		_max = val[k];
}

// difference in the sense of chi square
double Histogram::Diff( Histogram &hist )
{
	if ( val == NULL )
		return -1.0;

	if ( hist.val == NULL )
		return -1.0;

	if ( n != hist.n )
		return -1.0;

	
	double R = Volume();
	double S = hist.Volume();
	double r = ( S/R );

	double temp;
	int knstrn = 0;
	double df = n - knstrn;
	double chsq = 0.0;

	for (int j=0;j<n;j++) {
		if (val[j] == 0.0 && hist.val[j] == 0.0) {
			--df;
			//--(*df); No data means one less degree of freeelse
		} else { 
		temp = fabs( r * val[j] - hist.val[j] ) / ( r * val[j] + hist.val[j] );;
		//temp = fabs( val[j] - hist.val[j] ) / ( val[j] + hist.val[j] );
		chsq += temp;// * temp / ( val[j] + hist.val[j]);
		}
	}
	
	//double prob=gammq(0.5*(df),0.5*(chsq)); //Chi-square probability function. See §6.2.

	return 1.0 / chsq;
}

void Histogram::reset ()
{
	for ( int i=0;i<n;i++ )
		val[i] = 0.0;
}

double Histogram::maxValue ()
{
	return _max;
}

bool Histogram::Write( const char *file, long int &pos )
{
	// open the file in append mode
	FILE *f = fopen( file, "a" );

	if ( f == NULL )
		return false;

	// get the position in the file
	setbuf(f,NULL);
	fprintf(f," ");
	fflush( f );
	pos = ftell ( f );

	fprintf(f, "%d ", n );

	fprintf(f, "%f ", _max );

	for (int i=0;i<n;i++)
		fprintf( f, "%f ", val[i] );

	fprintf( f, "\n" );

	fclose( f );

	return true;
}

bool Histogram::Read ( const char *file, long int pos )
{
	FILE *f = fopen( file, "r" );

	if ( f == NULL )
		return false;

	if ( fseek( f, pos-1, SEEK_SET ) != 0 ) {
		fclose( f );
		return false;
	}

	int p = 0;
	if ( fscanf( f, "%d", &p ) != 1 ) {
		fclose( f );
		return false;
	}

	n = p;

	if ( val == NULL )
		val = (double*)malloc(n*sizeof(double));

	double v = 0.0;

	fscanf( f, "%lf", &v );
	_max = v;

	for (int i=0;i<n;i++) {
		if ( fscanf( f, "%lf", &v ) != 1 ) {
			fclose( f );
			return false;
		}

		val[i] = v;
	}

	fclose( f );

	return true;
}


	


