#include "basic.h"
#include "camera/camera.h"
#include "model/model.h"

class Correspondence;
class CorrespondenceTable;
class CorrespondenceManager;

typedef std::vector<Correspondence> CorrespondenceVector;

////////////////////////////////////
// one correspondence per image edge
class Correspondence {
public:
	Correspondence() {score_alignment = score_overlap = score =0.0; valid = false;};
	~Correspondence() {};

	Correspondence(Edge *line, EdgePlane plane, ExtrinsicParameters came, int frameId, int uid);
	void set( const Correspondence &correspondence );

	// accessors
	double getScore() {return score;}
	double getScoreAlignment() {return score_alignment;}
	double getScoreOverlap() {return score_overlap;}
	double getScoreDistance() {return score_distance;}
	Edge *getLine() {return l;}
	void setLine (Edge *edge) {l = edge;}
	int getTokenId() {return _token;}
	EdgePlane getPlane() {return p;}
	EdgePlane getUntransformedPlane() {return p0;}

	// methods
	void setPlane(EdgePlane pn) {p = pn; computeScore();}
	void setUntransformedPlane(EdgePlane pn) {p0 = pn; computeScore();}
	void draw(Viewer &viewer, double radius);
	bool isValid() {return valid;}
	bool operator<(Correspondence &p) {return (score>p.score);} // sort by decreasing score
	void drawInfo (Viewer &viewer, int x, int y, const float color[3]);
	void drawScoreInfo (Edge *line, Viewer &viewer, int x, int y, const float color[3]);
	void print();

	int _frameId; // frame ID where this edge was observed

protected:
	void computeScore();
	EdgePlane p; // transformed edge plane
	EdgePlane p0; // original un-transformed edge plane
	Edge *l; // 3D line in the model -- may be NULL
	bool valid; // true if the line is in front of the edgeplane after camera transformation
	double score_alignment, score_overlap, score_distance, score;
	int _token; // unique ID, used to track a correspondence over frames
};

////////////////////////////////////
// one correspondence table per frame
/*
class CorrespondenceTable {
public:

	void init (edgeVector &lines, edgePlaneVector &planes, ExtrinsicParameters c);
	void populate (edgeVector &lines, edgePlaneVector &planes);
	void populate (edgeVector &linesVector, std::vector<edgePlaneVector> &planesVector);
	void clear() {_m.clear(); _score = 0.0;}
	int size() {return _m.size();}
	bool empty() {return _m.empty();}
	Correspondence getCorrespondence (int n) {assert(n<size()); return _m[n];}
	double getScore() {return _score;}
	int getSelectedCorrespondence() {return _selected_correspondence;}
	void setSelectedCorrespondence(int n) {_selected_correspondence = MIN(n,size()-1);}
	ExtrinsicParameters getPose() {return came;}
	void computeScore(); // called by the manager once the table has been populated
	void Insert (Correspondence correspondence);
	void Sort();
	void print();
	void resetLines();

	CorrespondenceVector _m;

protected:
	double _score;
	int _selected_correspondence; // for display
	ExtrinsicParameters came;
	//Model *_model;
};
*/
//////////////////////////////////////////////
// one correspondence manager for the whole system
/*
class CorrespondenceManager {

public:

	CorrespondenceManager();
	//CorrespondenceManager(Model*);
	~CorrespondenceManager();
	void clear () {queue->clear();}
	void init (edgeVector &lines, edgePlaneVector &planes, ExtrinsicParameters c);
	void populate (edgeVector &linesVector, std::vector<edgePlaneVector> &planesVector);
	int getSelectedCorrespondence() {return queue->getSelectedCorrespondence();}
	void setSelectedCorrespondence(int n) { queue->setSelectedCorrespondence(n);}
	void Insert (Correspondence correspondence);
	Correspondence getCorrespondence (int n) {return queue->getCorrespondence(n);}

	int size() {return queue->size();}
	bool empty () {return queue->empty();}
	int getValidCorrespondences (int frameId, CorrespondenceVector &vector); // those for which line != NULL
	void cleanup(int frameId);
	void print();

	CorrespondenceTable *queue;

	// this 3D line cliques are used for localization
	int BACKUP_SIZE, VALIDATION_SIZE;

	//Clique PRIMARY;
	//cliqueVector VALIDATION, BACKUP;


protected:
	//Model *_model;
	//correspondenceTableVector tables;
	//int _nextToken;
	int MAX_FRAME_DEPTH; // depth of search for correspondences
	double SCORE_THRESHOLD; // score threshold when matching correspondences
	int TIME_TO_LIVE; // time to live for a correspondence in the queue
};
*/

