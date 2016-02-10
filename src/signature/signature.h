#ifndef _SIGNATURE_H__
#define _SIGNATURE_H__


#include "basic.h"
#include "viewer/viewer.h"

class Signature;
#define HISTOGRAM_SIZE_DIST 100
#define HISTOGRAM_SIZE_ANG 10

#define MAX_LINES_MATCHING 2000
#define SCALE_FACTOR_RESOLUTION 8
#define MIN_LINES_DISTANCE 10.0 // minimum distance between two lines to be considered for line matching, in inches.

class SignaturePair {
public:
	int _i,_j;
	double _skewness;
	double _dist;
	
	SignaturePair(){};
	SignaturePair(int i, int j, double skewness, double dist);
	bool match (SignaturePair);
	void print() {printf(":: %d %d %f %f\n",_i,_j,_skewness,_dist);};
};

typedef std::vector<SignaturePair> signaturePairVector;

class LineMatcher {
public:
	
	int **_matchTable;
	signaturePairVector _bestMatches; // best matches
	int _N,_M; // number of lines in each signature

	LineMatcher() {_M=_N=0;};
	void LineMatcher::insertMatch (SignaturePair p1, SignaturePair p2);
	void LineMatcher::insertMatch (int i, int j);
	void clear();
	void LineMatcher::process ();
	void LineMatcher::init (int N, int M);
};


class Signature {
public:
	
	/** attributes **/
	edgePlaneVector _edgeplanes;
	edgeVector _edges;
	Viewer _viewer;

	int _histogram_dist[HISTOGRAM_SIZE_DIST];
	int _histogram_ang[HISTOGRAM_SIZE_ANG];
	intVector _histogram_scale;
	
	int _histogram_dist_max,_histogram_ang_max, _histogram_scale_max;
	Vec3d _position;
	bool _computed;
	signaturePairVector _signatureVector;
	double _scaleFactor, _minScaleFactor, _maxScaleFactor;
	LineMatcher _lineMatcher;

	doubleVector _fft_dist_real, _fft_dist_im;
	
	/** methods **/
	Signature(){_computed = false; };
	~Signature(){};
	
	void set (std::vector<edgePlaneVector> &vector);
	void computeMap();
	void draw(Viewer &viewer);

	/// old stuff
	void print();
	void Signature::display(Viewer viewer, int x0, int y0, int w, int h);
	void Signature::displayMatch (Viewer viewer, double idealScaleFactor, int x0, int y0, int w, int h);
	void Signature::write(const char *filename, const char *file_list);
	bool Signature::read (Vec3d position, const char *file, const char *file_list);
	double Signature::compare(Signature);
	void Signature::histogram(doubleVector vector, int *output, int size, double &max_hist_x, int &max_hist_y, bool angle);
	void Signature::disturb (double scale, double removed_percent, double modified_percent, double angle);
	double histogramMatching (int *h1, int *h2, int n, int w);
	void Signature::computeSignatureVector(int maxsize);
	double Signature::findScalingFactor (Signature signature);
	int Signature::maxEdgeId ();
	
};

#endif
