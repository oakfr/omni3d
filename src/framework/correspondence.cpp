#include "correspondence.h"

Correspondence::Correspondence(Edge *line, EdgePlane plane, ExtrinsicParameters came, int frameId, int id)
{
	score_alignment = score_overlap = score =0.0;
	p0 = plane;
	p=plane; 
	p.fromCameraFrameToWorldFrame(came.getTranslation(),came.getRotation()); // do not invert rotation matrix here since it has been done in ::init()
	l=line;
	valid = false;
	_frameId = frameId;
	_token = id;

	computeScore();
}

void Correspondence::print()
{
	if (l == NULL)
		LOG(LEVEL_DEBUG,"[%3d] [------] [%d][%.3f] %.2f %.2f %.2f",_frameId,valid,score,p._normal[0],p._normal[1],p._normal[2]);
	else  {
		LOG(LEVEL_DEBUG,"[%3d] [%6d->%4d]  [%d][%.3f] %.2f %.2f %.2f",_frameId,l->_id,l->_correspondenceId,valid,score,p._normal[0],p._normal[1],p._normal[2]);
	}
}

// find the best possible match for an edgeplane and return the pointer to the line
// the returned pointer is NULL if no match was found
bool FindMatch (EdgePlane edgeplane, edgeVector &lines, ExtrinsicParameters came, int frameId, int tokenId, \
				Correspondence &correspondence)
{
	double score = 0.0;
	correspondence = Correspondence(NULL,edgeplane,came,frameId,tokenId);
	
	edgeVector::iterator iter = lines.begin();
	bool found = false;
	
	Correspondence corr;
	
	for (;iter!=lines.end();iter++) {
		
		corr = Correspondence(*iter,edgeplane,came,frameId,tokenId);
		
		if (!corr.isValid()) {
			continue;
		}
		
		if (corr.getScore() > score) {
			if (corr.getScoreAlignment() < 0.98) // threshold degrees 
				continue;
			correspondence = corr;
			score = corr.getScore();
			found = true;
		}
	}
	return found;
}

void CorrespondenceTable::populate (edgeVector &lines, std::vector<edgePlaneVector> &planesVector)
{
	// second populate the table
	for (int i=0;i<planesVector.size();i++) {
		populate(lines,planesVector[i]);
	}

	//Sort();

}

void CorrespondenceTable::Sort()
{
	// sort
	 sort(_m.begin(),_m.end());

	 // update the line indices
	 int counter = 0;
	 for (CorrespondenceVector::iterator iter = _m.begin(); iter != _m.end(); iter++,counter++) {
		 if (iter->getLine() != NULL)
			iter->getLine()->_correspondenceId = counter;
	 }
}


void CorrespondenceTable::populate (edgeVector &lines, edgePlaneVector &planes)
{
	// for each plane in <planes>, find the best scoring 3D line in <lines>
	int n = lines.size();
	int m = planes.size();

	edgePlaneVectorIter iter_edge = planes.begin();

	for (;iter_edge!=planes.end();iter_edge++) {
		Correspondence best_correspondence;
		FindMatch (*iter_edge,lines,came,0,size(),best_correspondence);
		Insert(best_correspondence);
	}
}

void CorrespondenceTable::init (edgeVector &lines, edgePlaneVector &planes, ExtrinsicParameters c)
{
	// initialize the table with some correspondences
	if (lines.size() != planes.size())
		return;

	_m.clear();
	came = c;
	//came.setRotation (came.getRotation()); // perform the inversion right now to save some time later

	for (int i=0;i<lines.size();i++) {
		Insert(Correspondence(lines[i],planes[i],came,0,size()));
	}

	_selected_correspondence = 0;
}

void Correspondence::computeScore ()
{
	// the score is based on (1) the alignment and (2) the overlap
	score = score_alignment = score_overlap = score_distance =  0.0;

	if (l == NULL) {
		score = -p.length();
		valid = false;
		return;
	}

	valid = false;

	// if frontality test fails, score is zero
	if (!p.testFrontality(l)) {
		score = -p.length();
		return;
	}

	valid = true;

	// alignment
	score_alignment = p.score_alignement(l); // 1-fabs(dot(p._normal,l->dir)); // 0: bad => 1: good

	if (score_alignment < COS_80) { // threshold 10 degrees
		valid = false;
		return;
	}

	// overlap
	score_overlap = p.overlap(l);

	if (score_overlap < EPSILON) {
		valid = false;
		return;
	}

	// distance 
	score_distance = 1/(0.5+p.distance(l));

	// final score
	score = p.length() * score_alignment * score_overlap * score_distance;
}

void Correspondence::draw(Viewer &viewer, double radius)
{
	// draw the edge plane
	viewer.drawEdgePlaneDetails(p,Vec3d(0,0,0),Quaternion(),radius,2,6); // color by camera ID

	// draw the 3D line
	if (l == NULL)
		return;

	viewer.drawLine(l->_a->getPosition(),l->_b->getPosition(),PINK,6); // color by camera ID

	// for debug info, draw the projection line in yellow
	double lambda;
	Vec3d m = intersectRayPlane(l->_a->getPosition(),p._normal,p._s,p._normal,lambda);
	viewer.drawLine(l->_a->getPosition(),m,YELLOW,3);
	m = intersectRayPlane(l->_b->getPosition(),p._normal,p._s,p._normal,lambda);
	viewer.drawLine(l->_b->getPosition(),m,YELLOW,3);
	double distance = p.distance(l);
}

void Correspondence::drawInfo (Viewer &viewer, int x, int y, const float color[3])
{
	if (l == NULL)
		viewer.displayMsg(x,y,color,0,"Line: NULL");
	else
		viewer.displayMsg(x,y,color,0,"Line: %d",l->_id);

	y -= 15;

	viewer.displayMsg(x,y,color,0,"[valid: %d]",valid);
	y -= 15;
	viewer.displayMsg(x,y,color,0,"[frameId: %d]",_frameId);
	y -= 15;
	viewer.displayMsg(x,y,color,0,"Alignment: %.2f",score_alignment);
	y -= 15;
	viewer.displayMsg(x,y,color,0,"Overlap: %.2f",score_overlap);
	y -= 15;
	viewer.displayMsg(x,y,color,0,"Distance: %.2f",score_distance);
	y -= 15;
	viewer.displayMsg(x,y,color,0,"Score: %.2f",score);
	y -= 15;
}

// compute the score with a given line and display it
// useful for debugging info
void Correspondence::drawScoreInfo (Edge *line, Viewer &viewer, int x, int y, const float color[3])
{
	double cscore = 0.0;
	double cscore_alignment = 0.0;
	double cscore_overlap = 0.0;
	double cscore_distance =  0.0;

	if (line == NULL) {
		return;
	}

	bool cvalid = false;

	// if frontality test fails, score is zero
	if (!p.testFrontality(line)) {
		cscore = -p.length();
	} else {
		
		cvalid = true;
		
		// alignment
		cscore_alignment = p.score_alignement(line); // 0: bad => 1: good
				
		// overlap
		cscore_overlap = p.overlap(line);
		
		// distance 
		cscore_distance = 1/(0.5+p.distance(line));
		
		// final score
		cscore = p.length() * cscore_alignment * cscore_overlap * cscore_distance;
	}

	if (line == NULL)
		viewer.displayMsg(x,y,color,0,"Line: NULL");
	else
		viewer.displayMsg(x,y,color,0,"Line: %d",line->_id);

	y -= 15;

	viewer.displayMsg(x,y,color,0,"[valid: %d]",cvalid);
	y -= 15;
	viewer.displayMsg(x,y,color,0,"Alignment: %.2f",cscore_alignment);
	y -= 15;
	viewer.displayMsg(x,y,color,0,"Overlap: %.2f",cscore_overlap);
	y -= 15;
	viewer.displayMsg(x,y,color,0,"Distance: %.2f",cscore_distance);
	y -= 15;
	viewer.displayMsg(x,y,color,0,"Score: %.2f",cscore);
	y -= 15;

}

/*
void CorrespondenceTable::computeScore()
{
	_score = 0.0;

	for (int i=0;i<size();i++)
		_score += getCorrespondence(i).getScore();
}

void CorrespondenceTable::print()
{	
	LOG(LEVEL_DEBUG,"******* Correspondence Table (%d items) ***********",size());
	came.getRotation().print();
	LOG(LEVEL_DEBUG,"-----------------------------");

	for (int i=0;i<size();i++) {
		getCorrespondence(i).print();
	}

	LOG(LEVEL_DEBUG,"******************");
}

void CorrespondenceTable::Insert (Correspondence correspondence)
{
	Edge *line = correspondence.getLine();

	if (line == NULL) {
		_m.push_back(correspondence);
		return;
	}
	
	if (line->_id == 1332)
		LOG(LEVEL_DEBUG,"warning");

	if (line->_correspondenceId == -1) {
		line->_correspondenceId = size();
		_m.push_back(correspondence);
	} else {
		Correspondence other_corr = _m[line->_correspondenceId];
		if (correspondence._frameId > other_corr._frameId) {
			//LOG(LEVEL_DEBUG,"replacing correspondence for line %d",line->_id);
			_m[line->_correspondenceId] = correspondence;
			return;
		}
		if (correspondence._frameId == other_corr._frameId) {
			if (correspondence.getScore() > other_corr.getScore()) {
				LOG(LEVEL_DEBUG,"replacing correspondence for line %d",line->_id);
				_m[line->_correspondenceId] = correspondence;
				return;
			}
		}
	}
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

CorrespondenceManager::~CorrespondenceManager()
{
}

CorrespondenceManager::CorrespondenceManager()
{
	MAX_FRAME_DEPTH = 5;
	SCORE_THRESHOLD = .95; // 18 degrees
	TIME_TO_LIVE = 5; // time to live for a correspondence in the queue
	BACKUP_SIZE = 10;
	VALIDATION_SIZE = 5;
	queue = new CorrespondenceTable ( );
}

// init the queue with a set of user-provided correspondences
// warning: no check is made!
void CorrespondenceManager::init (edgeVector &lines, edgePlaneVector &planes, ExtrinsicParameters c)
{
	queue->init (lines,planes,c);
}

// look for correspondences between 3D lines and image edge planes
// and populate the table for a (supposedly) new frame ID
void CorrespondenceManager::populate (edgeVector &linesVector, std::vector<edgePlaneVector> &planesVector)
{
	queue->populate(linesVector,planesVector);
}

int CorrespondenceManager::getValidCorrespondences (int frameId, CorrespondenceVector &cs)
{
	for (CorrespondenceVector::iterator iter = queue->_m.begin(); iter != queue->_m.end(); iter++) {
		if ((iter->getLine() != NULL) && (iter->_frameId == frameId))
			cs.push_back(*iter);
	}

	return cs.size();
}

void CorrespondenceManager::cleanup (int frameId)
{
	// kill deprecated correspondences
	bool done = false;
	return;

	while (!done) {

		done = true;

		for (CorrespondenceVector::iterator iter = queue->_m.begin(); iter != queue->_m.end(); iter++) {
			if (frameId > iter->_frameId + TIME_TO_LIVE) {
				if (iter->getLine() != NULL) {
					iter->getLine()->_correspondenceId = NULL;
				}
				queue->_m.erase(iter);
				done = false;
				break;
			}
		}
	}
}

void CorrespondenceManager::Insert (Correspondence correspondence)
{
	queue->Insert(correspondence);
}

// print out the three pools 
void CorrespondenceManager::print()
{
	LOG( LEVEL_DEBUG, "Correspondence manager [ok][valid] three line IDs");

	Clique clique = PRIMARY;
	LOG( LEVEL_DEBUG, "PRIMARY: [%d][%d] %d %d %d",clique.ok(),clique.valid(),clique.a->_id,clique.b->_id,clique.c->_id);
	LOG( LEVEL_DEBUG, "************************************");
	
	cliqueVector::iterator iter;
	for ( iter = VALIDATION.begin(); iter != VALIDATION.end(); iter++) {
		clique = *iter;
		LOG( LEVEL_DEBUG, "VALIDAT: [%d][%d] %d %d %d",clique.ok(),clique.valid(),clique.a->_id,clique.b->_id,clique.c->_id);
	}
	LOG( LEVEL_DEBUG, "************************************");
	
	int counter = 0;
	for ( iter = BACKUP.begin(); iter != BACKUP.end(); iter++,counter++) {
		if (counter == 10)
			continue;
		clique = *iter;
		LOG( LEVEL_DEBUG, "BACKUP : [%d][%d] %d %d %d",clique.ok(),clique.valid(),clique.a->_id,clique.b->_id,clique.c->_id);
	}
	LOG( LEVEL_DEBUG, "...\n************************************");
}

///////////////////////////////////////////////////////////////////////////
//     POOL MANAGEMENT
//
*/