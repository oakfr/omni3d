#include "signature.h"


// initialize the signature
void Signature::set (std::vector<edgePlaneVector> &vector)
{
	_edgeplanes.clear();
	_edges.clear();
	for (int i=0;i<vector.size();i++) {
		for (int j=0;j<vector[i].size();j++) {
			_edgeplanes.push_back(vector[i][j]);
		}
	}

	computeMap();
}

// compute a 2D map from the edge planes, assuming a vertical position of the camera
// vertical edgeplanes are transformed into vertical lines
// other edgeplanes are assumed horizontal and transformed into horizontal lines

void Signature::computeMap ()
{
	double d = 50.0; // typical distance, 50 inches, only used for display
	double lambda;

	for (int i=0;i<_edgeplanes.size();i++) {

		EdgePlane edgeplane = _edgeplanes[i];

		// vertical ones
		if (fabs(edgeplane._normal[2]) < COS_85) {
			Vec3d a = d*edgeplane._a;
			Vec3d axis = Vec3d(a[0],a[1],0.0);
			Vec3d b = intersectRayPlane(Vec3d(0,0,0),edgeplane._b,a,axis,lambda);
			Edge *edge = new Edge(a,b);
			edge->_keep = true;
			_edges.push_back(edge);
		} else {
			// edge assumed horizontal
			// special case: image edge is also horizontal
			if ((fabs(edgeplane._normal[0]) < COS_85) && (fabs(edgeplane._normal[1]) < COS_85)) {
				Vec3d a = d*edgeplane._a;
				Vec3d b = d*edgeplane._b;
				Edge *edge = new Edge (a,b);
				edge->_keep = true;
				//_edges.push_back(edge); skip these ones
			} else {
				Vec3d a = edgeplane._a * d;
				Vec3d b = intersectRayPlane(Vec3d(0,0,0),edgeplane._b,a,Vec3d(0,0,1),lambda);
				if (lambda > 0) {
					Edge *edge = new Edge(a,b);
					edge->_keep = true;
					_edges.push_back(edge);
				}
			}
		}
	}
}

void Signature::draw(Viewer &viewer)
{
	// draw the computed map
	viewer.drawEdges(_edges,GREEN,3,true);

	// draw the camera center
	viewer.drawSphere(Vec3d(0,0,0),10.0,YELLOW,10,10);
}


void Signature::histogram(doubleVector vector, int *output, int size, double &max_hist_x, int &max_hist_y, bool angle)
{
	if (vector.empty())
		return;
	
	int i;
	double min = vector[0];
	double max = vector[0];
	
	if (angle) {
		min = 0.0;
		max = M_PI/2;
	} else {
		// find min and max
		for (int i=1;i<vector.size();i++) {
			min = MIN(min,vector[i]);
			max = MAX(max,vector[i]);
		}
	}
	
	double bucket_size = (max-min)/size;
	
	// reset the buckets
	for ( i=0;i<size;i++) {
		output[i]=0;
	}
	
	// fill in the buckets
	for ( i=1;i<vector.size();i++) {
		if (vector[i] == max)
			output[size-1]++;
		else
			output[int((vector[i]-min)/bucket_size)]++;
	}
	
	// find the max size of histogram
	max_hist_y = 0;
	max_hist_x = 0.0;

	for ( i=0;i<size;i++) {
		if (output[i]>max_hist_y) {
			max_hist_y = output[i];
			max_hist_x = i*bucket_size + min;
		}
	}
}

double Signature::histogramMatching (int *h1, int *h2, int n, int w)
{
	double score = 0;
	int i;

	// find maximum value in h1
	int max_h1 = 0;
	for (i=1;i<n;i++) {
		//max_h1 = MAX(max_h1,h1[i]);
		max_h1 += h1[i];
	}

	// find maximum value in h1
	int max_h2 = 0;//h2[0];
	for (i=1;i<n;i++) {
		//max_h2 = MAX(max_h2,h2[i]);
		max_h2 += h2[i];
	}

	double ratio = (double)max_h1/MAX(1,max_h2);

	for (i=0;i<n;i++) {
		int val1 = h1[i];
		int val2 = int(ratio*h2[i]);

		double subscore = ((val2==0) ? (val1) : (fabs(val2-val1)/val2));

		for (int j=0;j<w;j++) {
			int idx = MAX(0,MIN(int(i+j-(double)w/2),n-1));
			val2 = int(ratio*h2[idx]);
			subscore = MIN(subscore,((val2==0) ? (val1) : (fabs(val2-val1)/val2)));
		}
		
		score += subscore;
	}
	
	return score/n;
}

void Signature::print()
{
}

void Signature::displayMatch (Viewer viewer, double idealScaleFactor, int x0, int y0, int w, int h)
{
	if (!_computed)
		return;

	// display the scaling factor
	viewer.displayMsg(x0,5,YELLOW,1,"Scale factor: %f",_scaleFactor);

	// draw the scaling factor histogram
	int i;
	int N = _histogram_scale.size();
	double x = x0;
	double y = y0;
	double dx = w/(1.1*N);
	printf("N: %d   dx = %f\n",N,dx);
	
	for ( i=0;i<N;i++) {
		double dy = (double)h/3*_histogram_scale[i]/_histogram_scale_max;
		viewer.drawBox(x,y,x+dx,y+dy,1,BLUE);
		x += dx;
	}

	// draw the pointer to the detected max
	double bucket_size = (_maxScaleFactor-_minScaleFactor)/N;
	i = x0 + dx*(_scaleFactor-_minScaleFactor)/bucket_size;
	viewer.drawLine(Vec2d(i,y0-20),Vec2d(i,y0-5),YELLOW,1);

	// draw the pointer to the ideal max
	i = x0 + dx*(idealScaleFactor-_minScaleFactor)/bucket_size;
	viewer.drawLine(Vec2d(i,y0-35),Vec2d(i,y0-20),RED,1);

	// display the ideal scaling factor
	viewer.displayMsg(x0+200,5,RED,1,"Ideal: ",idealScaleFactor);

}


void Signature::display(Viewer viewer, int x0, int y0, int w, int h)
{
	if (!_computed)
		return;

	// draw the distance histogram	
	int i;
	int x = x0;
	int y = y0;
	for ( i=0;i<HISTOGRAM_SIZE_DIST;i++) {
		double dx = w/(1.1*HISTOGRAM_SIZE_DIST);
		double dy = h/3*_histogram_dist[i]/_histogram_dist_max;
		viewer.drawBox(x,y,x+dx,y+dy,1,BLUE);
		x += dx;
	}
	
	// draw the angle histogram	
	x = x0;
	y = y0;	
	y+= h/2;

	for ( i=0;i<HISTOGRAM_SIZE_ANG;i++) {
		double dx = w/(1.1*HISTOGRAM_SIZE_ANG);
		double dy = h/3*_histogram_ang[i]/_histogram_ang_max;
		viewer.drawBox(x,y,x+dx,y+dy,1,RED);
		x += dx;
	}

}

void Signature::write(const char *filename, const char *file_list)
{
	FILE *f = fopen(filename,"a");
	assert(f!=NULL);

	int i;
	fprintf(f,"%f %f %f\n",_position[0],_position[1],_position[2]);
	
	setbuf(f,NULL);
	fflush(f);
	long fpos = ftell(f);

	fprintf(f,"%d\n",HISTOGRAM_SIZE_ANG);

	for ( i=0;i<HISTOGRAM_SIZE_ANG;i++) 
		fprintf(f,"%d ",_histogram_ang[i]);
	fprintf(f,"\n");

	fprintf(f,"%d\n",HISTOGRAM_SIZE_DIST);

	for ( i=0;i<HISTOGRAM_SIZE_DIST;i++) 
		fprintf(f,"%d ",_histogram_dist[i]);
	fprintf(f,"\n");

	fclose(f);

	f = fopen(file_list,"a");
	fprintf(f,"%f %f %f %d\n",_position[0],_position[1],_position[2],fpos);
	fclose(f);
}

bool Signature::read (Vec3d position, const char *file, const char *file_list)
{
	FILE *f = fopen(file_list,"r");
	if (f == NULL)
		return false;

	double x,y,z;
	int value;
	int counter;
	int i;
	long int fpos;

	while (fscanf(f,"%lf%lf%lf%d",&x,&y,&z,&fpos) == 4) {
		if (len(Vec3d(x,y,z)-position) < EPSILON) {
			
			// read the signature
			FILE *f2 = fopen(file,"r");
			assert(f2 != NULL);

			fseek(f2,fpos-1,SEEK_SET);
			
			fscanf(f2,"%d",&counter);
			
			_histogram_ang_max = 0;
			for (i=0;i<counter;i++) {
				fscanf(f2,"%d",&value);
				_histogram_ang[i] = value;
				_histogram_ang_max = MAX(_histogram_ang_max,value);
			}
			
			fscanf(f2,"%d",&counter);
			
			_histogram_dist_max = 0;
			for (i=0;i<counter;i++) {
				fscanf(f2,"%d",&value);
				_histogram_dist[i] = value;
				_histogram_dist_max = MAX(_histogram_dist_max,value);
			}
			
			fclose(f2);
			fclose(f);
			_computed = true;
			return true;
		}
	}

	_computed = false;
	fclose(f);
	return false;
}

double Signature::compare (Signature signature)
{
	assert(_computed);

	double score_dist = (histogramMatching(_histogram_dist,signature._histogram_dist,HISTOGRAM_SIZE_DIST,6) + \
		histogramMatching(signature._histogram_dist,_histogram_dist,HISTOGRAM_SIZE_DIST,6))/2;
	double score_ang  = (histogramMatching(_histogram_ang,signature._histogram_ang,HISTOGRAM_SIZE_ANG,2) + \
		histogramMatching(signature._histogram_ang,_histogram_ang,HISTOGRAM_SIZE_ANG,2))/2;
	double score = score_dist + score_ang;
	printf("score = %f\n",score);

	findScalingFactor (signature);

	return score;
}

int Signature::maxEdgeId ()
{
	return 0;
}

//find the scaling factor assuming that the two signatures match
double Signature::findScalingFactor (Signature signature)
{
	int i,j;
	doubleVector scales;

	// fill in the signature vector of both signatures
	computeSignatureVector(MAX_LINES_MATCHING);
	signature.computeSignatureVector(MAX_LINES_MATCHING);
	
	_minScaleFactor = 1E10;
	_maxScaleFactor = 0.0;

	printf("[%d %d]\n",_signatureVector.size(),signature._signatureVector.size());

	//_lineMatcher.clear();
	//_lineMatcher.init(maxEdgeId(),signature.maxEdgeId());

	// find matches
	for (i=0;i<_signatureVector.size();i++) {
		for (j=0;j<signature._signatureVector.size();j++) {

			//_signatureVector[i].print();
			//signature._signatureVector[j].print();

			if (_signatureVector[i].match(signature._signatureVector[j])) {
				if (_signatureVector[i]._dist > EPSILON) {
					double scale = _signatureVector[i]._dist / signature._signatureVector[j]._dist;
					scales.push_back(scale);
					_minScaleFactor = MIN(scale,_minScaleFactor);
					_maxScaleFactor = MAX(scale,_maxScaleFactor);
					//_lineMatcher.insertMatch(_signatureVector[i],signature._signatureVector[j]);
				}
			}
		}
	}

	// print histogram
	int N = int(_maxScaleFactor)*SCALE_FACTOR_RESOLUTION;
	printf("histograming (%d buckets)...\n",N);

	int *histogram_scale = (int*)malloc(N*sizeof(int));

	int max_y;
	histogram(scales,histogram_scale,N,_scaleFactor,max_y,false);

	_histogram_scale.clear();
	_histogram_scale_max = 0;
	for (i=0;i<N;i++) {
		_histogram_scale.push_back(histogram_scale[i]);
		_histogram_scale_max = MAX(_histogram_scale_max,histogram_scale[i]);
	}
	
	printf("scale factor: %f\n",_scaleFactor);

	delete histogram_scale;

	// process the line matcher
	//_lineMatcher.process();

	return _scaleFactor;
}

void Signature::computeSignatureVector(int maxsize)
{
}


void Signature::disturb (double s, double removed_percent, double modified_percent, double angle)
{
	// randomly disturb the signature
	

}

SignaturePair::SignaturePair(int i, int j, double skewness, double dist)
{
	_i=i;
	_j=j;
	_skewness = skewness;
	_dist = dist;
}

bool SignaturePair::match (SignaturePair pair)
{
	return (fabs(_skewness - pair._skewness) < M_PI/(2*HISTOGRAM_SIZE_ANG));
}

void LineMatcher::insertMatch (SignaturePair p1, SignaturePair p2)
{
	insertMatch(p1._i,p2._i);
	insertMatch(p1._i,p2._j);
	insertMatch(p1._j,p2._i);
	insertMatch(p1._j,p2._j);
}

void LineMatcher::insertMatch (int i, int j)
{
	assert(i<_N);
	assert(j<_M);

	_matchTable[i][j] += 1;
}

void LineMatcher::process ()
{
	int i,j,best_j;
	_bestMatches.clear();

	printf("processing %d,%d lines\n",_N,_M);

	for (i=0;i<_N;i++) {
		
		bool found = false;
		int best_w = 0;

		for (j=0;j<_M;j++) {

			if (_matchTable[i][j] > best_w) {
				found = true;
				best_j = j;
				best_w = _matchTable[i][j];
			}
		}

		if (found) {

			// best match: i, vector[j]._j
			_bestMatches.push_back(SignaturePair(i,best_j,0.0,_matchTable[i][best_j]));
			printf("match: %d %d %d\n",i,best_j,_matchTable[i][best_j]);

		}
	}

	// dump line matcher into file
	for (i=0;i<_N;i++) {
		bool found = false;
		for (j=0;j<_M;j++) {
			if (_matchTable[i][j] != 0)
			{
				found = true;
				printf("%d ",i,_matchTable[i][j]);
			}
		}
		if (found)
			printf("(%d)\n",i);
	}
}

void LineMatcher::init (int N, int M)
{
	printf("line matcher init: %d,%d\n",N,M);
	_N = N+1;
	_M = M+1;

	_matchTable = (int **)malloc(_N*sizeof(int*));

	int i;
	for (i=0;i<_N;i++)
		_matchTable[i] = (int*)malloc(_M*sizeof(int));

	clear();
}

void LineMatcher::clear()
{
	for (int i=0;i<_N;i++)
		for (int j=0;j<_M;j++)
			_matchTable[i][j] = 0;
}

