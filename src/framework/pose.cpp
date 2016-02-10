#include "main.h"

// filter camera pose
/*int MyGlWindow::filterCameraPoses()
{
	std::vector<PoseSolution> poses;
	for (int i=0;i<_poses.size();i++)
		poses.push_back(_poses[i]);
	
	_poses.clear();
	
	for (i=0;i<poses.size();i++) {
		
		checkPose(poses[i]);
		
		if (!poses[i].isValid())
			continue;
		
		bool found = false;
		for (int j=0;j<_poses.size();j++) {
			if (_poses[j].equal(_poses[i])){
				found = true;
				break;
			}
		}
		
		if (found)
			continue;
		
		_poses.push_back(poses[i]);
		_save_good_poses.push_back(poses[i]);
	}
	
	return _poses.size();
}*/

// compute the camera pose from 3 line correspondences
// all solutions are computed and stored in _poses (if valid)
/*int MyGlWindow::computeCameraPoseFrom3Correspondences()
{
	
	_poses.clear();

	computeCameraPoseFrom3Correspondences(true,true,_poses);
	computeCameraPoseFrom3Correspondences(true,false,_poses);
	computeCameraPoseFrom3Correspondences(false,true,_poses);
	computeCameraPoseFrom3Correspondences(false,false,_poses);

	return _poses.size();

}*/

/*
// compute the camera pose from 3 correspondences and two boolean parameters
bool MyGlWindow::computeCameraPoseFrom3Correspondences(bool flip1, bool flip2, std::vector<PoseSolution> &poses)
{	
	if (_LUT._selected_lines.size() != 3)
		return false;

	if (_camera->_frame._selected_edges.size() != 3)
		return false;

	Edge *p = _LUT._selected_lines[0];
	Edge *q = _LUT._selected_lines[1];
	Edge *r = _LUT._selected_lines[2];
	EdgePlane P = _camera->_frame._selected_edges[0];
	EdgePlane Q = _camera->_frame._selected_edges[1];
	EdgePlane R = _camera->_frame._selected_edges[2];

	return computeCameraPoseFrom3Correspondences( flip1, flip2, p, q, r, P, Q, R, poses );
}

// compute the camera pose from 3 correspondences and update the pose passed as input
//
bool MyGlWindow::computeCameraPoseFrom3Correspondences(Edge *p, Edge *q, Edge *r, EdgePlane &P, EdgePlane &Q, \
													   EdgePlane &R, ExtrinsicParameters &pose)
{

	for (int i=0;i<2;i++) {
		for (int j=0; j<2; j++) {
			std::vector< PoseSolution > poses;
			_camera->resetPose();
			if ( !computeCameraPoseFrom3Correspondences( (i==0), (j==0), p, q, r, P, Q, R, poses) )
				continue;

			for (int k=0;k<poses.size(); k++) {
				checkPose( poses[k] );
				if ( !poses[k].isValid() )
					continue;

				ExtrinsicParameters cpose = poses[k].getPose();
				pose = cpose;
				return true;
			}
		}
	}

	return false;

}
*/

// this methods computes a set of camera poses bringing the three edge planes in alignment
// with the three lines. The edge planes are expressed in the camera coord frame.
// The vector <poses> contains the set of poses.
// Sanity check: after alignment, each pair <line,edge> must pass the frontality test
// This is the latest version to use
//
bool MyGlWindow::cameraPoseFromThreeCorrespondences( EdgePlane P, EdgePlane Q, EdgePlane R, Edge *p, Edge *q, Edge *r, 
													ExtrinsicParameters &pose )
{

	double angle, check;

	// reset camera pose
	_camera->resetPose();
	
	//pose_history.clear();

	//LOG( LEVEL_INFO, "lines: %d %d %d", p->_id, q->_id, r->_id );

	// debug display
	//mline1_angle = P.angle ( Q );
	//middleLine( p->getA(), p->dir, q->getA(), q->dir, mline1_p, mline1_dir );

	// create a triplet of correspondences for camera pose refinement
	//Correspondence c1, c2, c3;
	//c1.first = p;
	//c1.second = P;
	//c2.first = q;
	//c2.second = Q;
	//c3.first = r;
	//c3.second = R;
	//CorrespondenceVector correspondences;
	//correspondences.push_back( c1 );
	//correspondences.push_back( c2 );
	//correspondences.push_back( c3 );

//	SM_Clear();				// insert them in the queue for display
//	SM_InsertItem( c1 );
//	SM_InsertItem( c2 );
//	SM_InsertItem( c3 );

	// save the original edge planes
	EdgePlane P0 = P, Q0 = Q, R0 = R;

	// compute the intersection of P and Q : <y>
	Vec3d y = cross( P._normal, Q._normal );

	if ( len(y) < EPS ) {
		return false;
	}

	y = norm( y );

	// bring <y> on p
	Quaternion q1;
	Vec3d z = cross( y, p->dir );

	angle = Acos(dot(y, p->dir));

	if ( len( z ) > EPS )
		q1 = Quaternion( norm(z), angle );

	P.rotate( q1 );

	check = dot( P._normal, p->dir );

	/*Q = Q0;
	Q.rotate( q1 );
	y = q1.rotate( y );

	// rotate along y so that q becomes parallel to Q

	double lambda;
	Quaternion q2;
	Vec3d m = intersectRayPlane( q->getB(), y, q->getA(), y, lambda ) - q->getA();
	if ( len(m) > EPS ) {
		angle = Asin(dot(Q._normal,m));
		q2 = Quaternion( y, angle );
	}

	Q.rotate( q2 );

	check = fabs(dot( Q._normal, q->dir )); // check that q is perpendicular to the normal to Q
	if ( check > 1E-3 ) // we picked the wrong angle
		q2 = q2.bar();

	double dotu = dot( (q2*q1).rotate(P0._normal), p->dir );
	double dotv = dot( (q2*q1).rotate(Q0._normal), q->dir );
	*/

	// the four possible rotations
	//std::vector<Quaternion> qs;
	//qs.push_back( q2 * q1 );
	//qs.push_back( q2.plusPi() * q1 );
	//qs.push_back( q2 * q1.plusPi() );
	//qs.push_back( q2.plusPi() * q1.plusPi() );

	double best_score = -1.0;

	//for (int k=0;k<qs.size();k++) {

		//Quaternion q3 = q1; //qs[k];
		
		// at this point, p is aligned with P and q is aligned with Q	
		// now, we are going to align r on R
		
		// find the best rotations that bring (P,Q,R) in alignment with (p,q,r)
		doubleVector angles;
		
		P = P0; Q = Q0; R = R0;
		P.rotate(q1); Q.rotate(q1); R.rotate(q1);
		findBestRotations(P,Q,R,p,q,r,angles);

		// for each rotation, compute the corresponding translation and score the overlap
		for (int i=0;i<angles.size()/2;i++) {
			double alpha = angles[2*i];
			double beta = angles[2*i+1];
			P = P0;
			P.rotate( q1/*qs[k]*/ );
			Quaternion q3 = Quaternion (p->dir,beta) * Quaternion (P._normal,alpha) * q1; //qs[k];	
			_camera->setRotation(q3);
			P = P0; Q = Q0; R = R0;
			if ( !computeTranslation (p,q,r,P,Q,R,true) )
				continue;
			double score = P.overlap( p ) + Q.overlap( q ) + R.overlap( r );

			//pose_history.push_back( _camera->getPose() );

			if ( score > best_score ) {
				best_score = score;
				pose =_camera->getPose();
			}
		}
	//}

	_camera->setPose( pose );

	// debug: print the orientation vectors
	/*for (k=0;k<pose_history.size();k++) {
		Vec3d axis;
		axis = pose_history[k].getRotation().rotate( Vec3d(0,0,1) );
		LOG(LEVEL_INFO, "axis %d: %f %f %f", k, axis[0], axis[1], axis[2] );
	}*/

	return ( best_score > 0.0 ) ;
}

/*
// compute the camera pose from 3 correspondences and two boolean markers
bool MyGlWindow::computeCameraPoseFrom3Correspondences(bool flip1, bool flip2, Edge *p, Edge *q, Edge *r, EdgePlane &P0, EdgePlane &Q0, \
													   EdgePlane &R0, std::vector<PoseSolution> &poses)
{	
	//_LUT._selected_lines  => 3D lines
	//_camera->_frame._selected_edges   => 2D edges

	// save the camera pose
	ExtrinsicParameters pose = _camera->getPose();

	double lambda,alpha,beta,check;
	EdgePlane P = P0, Q = Q0, R = R0;

	Quaternion Rcam = Quaternion();
	Quaternion Rt   = Quaternion();
	Vec3d m,x,y,z;
		
	// check that P and Q are not parallel
	alpha = len(cross(P._normal,Q._normal));
	if (alpha < EPSILON)
		return false;
	
	// compute the intersection of P and Q
	y = norm(cross(P._normal,Q._normal));
	Vec3d origin = intersectRayPlane(P._s,P._a,Q._s,Q._normal,lambda);
	
	// bring p on <y>
	Rcam = Quaternion(); //Identity();
	z = cross(p->dir,y);
	if (len(z)>EPSILON) { // need to rotate as well
		z = norm(z);
		alpha = Acos(dot(p->dir,y))+M_PI;
		if (flip1)
			alpha += M_PI;
		Rcam = Quaternion(z,-alpha);
	}
	y = Rcam.rotate(y);
	Rt = Rcam * Rt;
	P = P0; Q = Q0; R = R0;
	P.rotate(Rt);
	Q.rotate(Rt);
	R.rotate(Rt);
	check = len(cross(y,p->dir));
	assert(check < EPSILON);
	check = fabs(dot(P._normal,p->dir));
	assert(check < EPSILON);
	
	// rotate around <y> so that Q is parallel to q
	Rcam = Quaternion();
	bool test = (fabs(dot(Q._normal,q->dir))>EPSILON);
	if (test) { // need to rotate around <y>
		m = intersectRayPlane(q->_b->getPosition(),y,q->_a->getPosition(),y,lambda);
		alpha = -(M_PI/2.0 - Acos(dot(Q._normal,norm(m-q->_a->getPosition()))));
		if (flip2)
			alpha += M_PI;
		Rcam = Quaternion(y,-alpha);
	}
	y = Rcam.rotate(y);
	Rt = Rcam * Rt;
	P = P0; Q = Q0; R = R0;
	P.rotate(Rt);
	Q.rotate(Rt);
	R.rotate(Rt);
	check = fabs(dot(P._normal,p->dir));
	assert(check < EPSILON);
	check = fabs(dot(Q._normal,q->dir));
	if ((check>EPSILON) && test){
		// we picked the wrong angle
		y = Rcam.inv().rotate(y);
		Rt = Rcam.inv()*Rt;
		if (flip2)
			alpha -= M_PI;
		alpha = -alpha;
		if (flip2)
			alpha += M_PI;
		Rcam = Quaternion(y,-alpha);
		y = Rcam.rotate(y);
		Rt = Rcam * Rt;
		P = P0; Q = Q0; R = R0;
		P.rotate(Rt);
		Q.rotate(Rt);
		R.rotate(Rt);
		check = fabs(dot(Q._normal,q->dir));
		assert(check < EPSILON);
	}
	
	// at this point, p is aligned with P and q is aligned with Q	
	// now, we are going to align r on R
	
	Quaternion R1 = Rt;

	//findBestRotationExhaustiveSearch(R0); // this is for debug only.

	doubleVector angles;
	//LOG(LEVEL_DEBUG,"********************");
	findBestRotations(P0,Q0,R0,p,q,r,angles);
	
	P = P0; Q = Q0; R = R0;
	P.rotate(R1);
	Q.rotate(R1);
	R.rotate(R1);
	double error_trans;
	
	for (int i=0;i<angles.size()/2;i++) {
		alpha = angles[2*i];
		beta = angles[2*i+1];
		Rt = Quaternion (p->dir,beta) * Quaternion (P._normal,alpha) * R1;	
		_camera->setRotation(Rt);
		//error_trans = computeTranslation (lines,edges);	
		//poses.push_back(PoseSolution(lines,edges,_camera->came));
		error_trans = computeTranslation (p,q,r,P0,Q0,R0,true);	
		poses.push_back(PoseSolution(p,q,r,P0,Q0,R0,_camera->came));
	}
	
	//reset pose
	_camera->setPose( pose );

	return true;
}*/

// this is an exhaustive search -- can be used to generate nice graph
void MyGlWindow::findBestRotationExhaustiveSearch (Quaternion Rcam)
{
	edgeVector lines; // = _LUT._selected_lines;
	edgePlaneVector planes; // = _camera->_frame._selected_edges;
	int i,j;
	
	for (i=0;i<_LUT._selected_lines.size();i++)
		lines.push_back(_LUT._selected_lines[i]);
	
	for (i=0;i<_camera->_frame._selected_edges.size();i++)
		planes.push_back(_camera->_frame._selected_edges[i]);
	
	Edge *p = lines[0], *q = lines[1], *r = lines[2];
	EdgePlane P = planes[0], Q = planes[1], R = planes[2];
	EdgePlane P0 = planes[0], Q0 = planes[1], R0 = planes[2];
	P0.rotate(Rcam);
	Q0.rotate(Rcam);
	R0.rotate(Rcam);
	Quaternion R1,R2;
	
	double score;
	double _alpha,_beta;

	FILE *f;
	f = fopen("data.dat","w");
	fprintf(f,"X = [\n");
	
	// populate the table
	for (i=0;i<360;i+=2) {
		_alpha = i*M_PI/180.0;
		
		for (j=0;j<360;j+=2) {
			_beta = j*M_PI/360;
			
			// initialize the camera pose
			P = P0, Q = Q0, R = R0;
			
			// rotate along P._normal by alpha
			R1 = Quaternion(P._normal,_alpha);
			P._normal = R1.rotate(P._normal);
			Q._normal = R1.rotate(Q._normal);
			R._normal = R1.rotate(R._normal);
			
			// rotate along <p> by beta
			R2 = Quaternion(p->dir,_beta);
			P._normal = R2.rotate(P._normal);
			Q._normal = R2.rotate(Q._normal);
			R._normal = R2.rotate(R._normal);
			
			// score
			score = fabs(dot(P._normal,p->dir)) + fabs(dot(Q._normal,q->dir)) + \
				fabs(dot(R._normal,r->dir));
			
			fprintf(f,"%f ",score);
		}
		
		fprintf(f,";\n");
	}
	
	fprintf(f,"];\n");
	fclose(f);
}

// compute the score for three correspondences (based on pure alignment)
// v is a 2D-vector containing angles alpha and beta
double computescore_gsl (const gsl_vector *v, void *params)
{
	double alpha = gsl_vector_get(v, 0);
	double beta  = gsl_vector_get(v, 1);
	
	double *dp = (double *)params;
	Vec3d Pnormal = Vec3d(dp[0],dp[1],dp[2]);
	Vec3d Qnormal = Vec3d(dp[3],dp[4],dp[5]);
	Vec3d Rnormal = Vec3d(dp[6],dp[7],dp[8]);
	Vec3d pn = Vec3d(dp[9],dp[10],dp[11]);
	Vec3d qn = Vec3d(dp[12],dp[13],dp[14]);
	Vec3d rn = Vec3d(dp[15],dp[16],dp[17]);
	
	// rotate along P._normal by alpha
	Quaternion R1 = Quaternion (Pnormal,alpha);
	Pnormal = R1.rotate(Pnormal);
	Qnormal = R1.rotate(Qnormal);
	Rnormal = R1.rotate(Rnormal);
	
	// rotate along <p> by beta
	Quaternion R2 = Quaternion(pn,beta);
	Pnormal = R2.rotate(Pnormal);
	Qnormal = R2.rotate(Qnormal);
	Rnormal = R2.rotate(Rnormal);
	
	// score
	return fabs(dot(Pnormal,norm(pn))) + fabs(dot(Qnormal,norm(qn))) + \
		fabs(dot(Rnormal,norm(rn)));
	
}

// compute the score for N edges and M model lines
// v is a 6-DOF vector encoding the camera rotation and translation
// each model line is encoded in a 6-DOF <p1,p2> vector expressed in the world coord frame
// each edge is encoded in a 9-DOF <s,a,b> vector expressed in the camera coord frame
//
double computescore_init_gsl( const gsl_vector *v, void *params )
{
	Vec3d rotation_vector = Vec3d(gsl_vector_get(v, 0),gsl_vector_get(v, 1),gsl_vector_get(v, 2));
	Vec3d translation = Vec3d(gsl_vector_get(v, 3),gsl_vector_get(v, 4),gsl_vector_get(v, 5)); 

	Quaternion rotation = Quaternion( rotation_vector, len(rotation_vector) );

	double *dp = (double *)params;
	double total_score = 0.0;
	int offset = 2, i,j;
	
	int N = int(dp[0]); // number of edges
	int M = int(dp[1]); // number of lines

	std::vector< Vec3d > planes; // each plane has 4 Vec3d: s, a, b, normal
	std::vector< Vec3d > lines;  // each line has 2 Vec3d: p1, p2

	// read the lines

	for (i=0;i<M;i++) {
		Vec3d Lp1 = Vec3d(dp[offset+0],dp[offset+1],dp[offset+2]); // read the line
		Vec3d Lp2 = Vec3d(dp[offset+3],dp[offset+4],dp[offset+5]);
		offset += 6;

		lines.push_back( Lp1 );
		lines.push_back( Lp2 );
	}

	// read the edges

	for (i=0;i<N;i++) {
		Vec3d Pcenter = Vec3d(dp[offset+0],dp[offset+1],dp[offset+2]); // read the edge
		Vec3d Pa = Vec3d(dp[offset+3],dp[offset+4],dp[offset+5]);
		Vec3d Pb = Vec3d(dp[offset+6],dp[offset+7],dp[offset+8]);

		Pa = rotation.rotate( Pa );
		Pb = rotation.rotate( Pb );
		Vec3d Pnormal = norm(cross(Pa-Pcenter, Pb-Pcenter));
		Pcenter = translation + Pcenter;
		offset += 9;

		planes.push_back( Pcenter);
		planes.push_back( Pa);
		planes.push_back( Pb);
		planes.push_back( Pnormal);

	}

	// for each line, try to find a good match and update the score

	intVector score_js; // list of edges that found a match

	for (i=0;i<M;i++) {
		Vec3d Lp1 = lines[2*i];
		Vec3d Lp2 = lines[2*i+1];

		// for each edge, compute the score

		double best_score = 1.0;
		int score_j = -1;

		for (j=0;j<N;j++) {
			Vec3d Pcenter = planes[4*j];
			Vec3d Pa = planes[4*j+1];
			Vec3d Pb = planes[4*j+2];
			Vec3d Pnormal = planes[4*j+3];

			double overlap = OverlapPlaneLine( Lp1 - Pcenter, Lp2 - Pcenter, Pa, Pb );

			if ( overlap < .5 )
				continue;

			double score =  1.0 - fabs( dot( Pnormal, norm(cross(Lp1 - Pcenter, Lp2 - Pcenter)) ));

			if ( best_score > score ) {
				best_score = score;
				score_j = j;
			}
		}

		// update the total score with the best score for the current line

		total_score += best_score;

		if ( score_j != -1 )
			score_js.push_back( score_j );
	}

	// malus for each unmatched edge
	for (i=0;i<N;i++) {

		bool matched = false;

		for (j=0;j<score_js.size();j++) {
			if ( score_js[j] == i ) {
				matched = true;
				break;
			}
		}

		if ( !matched ) {
			total_score -= Acos(dot(norm(planes[4*j+1] - planes[4*j]), norm(planes[4*j+2] - planes[4*j]) ) );
		}
	}

	return total_score;
}

// compute the score for N edges and M model lines
// the edges are edges observed by the cameras
// the lines are the visible lines for the given camera position
// small score = good news
//
double MyGlWindow::computescore_init ()
{
	// first lookup the db for the current position
	_LUT.lookup( _camera->came.getTranslation(), 0.0, 0.0, 0, 0 );

	// setup a vector of lines and edges
	int N = 0, M = _LUT._lines.size();
	int i,j;

	edgePlaneVector edgeplanes;
	for (i=0;i<_camera->_frame._edgeplanes_chained.size();i++) {
		for (j=0;j<_camera->_frame._edgeplanes_chained[i].size();j++) {
			edgeplanes.push_back( _camera->_frame._edgeplanes_chained[i][j] );
		}
	}
	
	N = edgeplanes.size();

	double *par = new double[ 6*M + 9*N + 2];
	par[0] = N;
	par[1] = M;

	// write the lines
	int offset = 2;
	for (i=0;i<M;i++) {
		Vec3d a = _LUT._lines[i]->getA();
		Vec3d b = _LUT._lines[i]->getB();

		par[offset+0] = a[0];
		par[offset+1] = a[1];
		par[offset+2] = a[2];
		par[offset+3] = b[0];
		par[offset+4] = b[1];
		par[offset+5] = b[2];

		offset += 6;
	}

	// write the edges
	for(i=0;i<N;i++) {
		Vec3d a = edgeplanes[i]._a;
		Vec3d b = edgeplanes[i]._b;
		Vec3d s = edgeplanes[i]._s;

		par[offset+0] = s[0];
		par[offset+1] = s[1];
		par[offset+2] = s[2];
		par[offset+3] = a[0];
		par[offset+4] = a[1];
		par[offset+5] = a[2];
		par[offset+6] = b[0];
		par[offset+7] = b[1];
		par[offset+8] = b[2];

		offset += 9;
	}

	// setup the camera pose vector
	gsl_vector *x;
	x = gsl_vector_alloc (6);

	Vec3d rotation_vector;
	_camera->came.getRotation().toAxisAngle(rotation_vector);

	gsl_vector_set (x, 0, rotation_vector[0]);
	gsl_vector_set (x, 1, rotation_vector[1]);
	gsl_vector_set (x, 2, rotation_vector[2]);
	
	gsl_vector_set (x, 3, _camera->getTranslation()[0]);
	gsl_vector_set (x, 4, _camera->getTranslation()[1]);
	gsl_vector_set (x, 5, _camera->getTranslation()[2]);

	// compute the score
	return computescore_init_gsl( x, par );

}


// compute the score for N correspondences (based on pure alignment)
// v is a 3-DOF vector encoding the camera rotation
// params contains the N correspondence normal vectors and line directions (6-DOF each)
// small score = good news
//
double computescore_nrot_gsl (const gsl_vector *v, void *params)
{
	Vec3d rotation_vector = Vec3d(gsl_vector_get(v, 0),gsl_vector_get(v, 1),gsl_vector_get(v, 2));

	double theta = len(rotation_vector);
	Quaternion rotation = Quaternion(norm(rotation_vector),theta);

	double *dp = (double *)params;
	double score = 0.0;

	int n = int(dp[0]);

	for (int i=0;i<n;i++) {
		Vec3d Pnormal = Vec3d(dp[6*i+1],dp[6*i+2],dp[6*i+3]);
		Vec3d linedir = norm(Vec3d(dp[6*i+4],dp[6*i+5],dp[6*i+6]));

		Pnormal = rotation.rotate(Pnormal);

		score += fabs(dot(Pnormal,linedir));

	}

	return score;
}


// compute the score for N correspondences (based on pure translation error)
// v is a 3-DOF vector encoding the camera translation
// params contains the N correspondence center+normal vectors and line end points (12-DOF each)

double computescore_ntrans_gsl (const gsl_vector *v, void *params)
{
	Vec3d translation = Vec3d(gsl_vector_get(v, 0),gsl_vector_get(v, 1),gsl_vector_get(v, 2));

	double *dp = (double *)params;
	double score = 0.0;
	//double lambda;

	int n = int(dp[0]);

	for (int i=0;i<n;i++) {
		Vec3d Pcenter = translation + Vec3d(dp[12*i+1],dp[12*i+2],dp[12*i+3]);
		Vec3d Pnormal =Vec3d(dp[12*i+4],dp[12*i+5],dp[12*i+6]);
		Vec3d line_a = Vec3d(dp[12*i+7],dp[12*i+8],dp[12*i+9]);
		Vec3d line_b = Vec3d(dp[12*i+10],dp[12*i+11],dp[12*i+12]);

		// important: assume the Pnormal has been rotated already!
		score += 1.0 - fabs( dot( Pnormal, norm(cross(line_a - Pcenter, line_b - Pcenter)) ));
		//Pcenter += translation;
		//Vec3d a = intersectRayPlane(line_a,Pnormal,Pcenter,Pnormal,lambda);
		//score += fabs(lambda)/2.0;
		//Vec3d b = intersectRayPlane(line_b,Pnormal,Pcenter,Pnormal,lambda);
		//score += fabs(lambda)/2.0;

	}

	return score;
}

// find the best rotations that bring (P0,Q0,R0) onto (p,q,r)
//
void MyGlWindow::findBestRotations (EdgePlane P0, EdgePlane Q0, EdgePlane R0, 
									Edge *p, Edge *q, Edge *r, doubleVector &angles)
{
	double alpha, beta;
	int step = 4;
	for (int i=0; i<step; i++) {
		alpha = i * 2*M_PI/step;
		
		for (int j=0;j<step;j++) {
			beta = j * 2*M_PI/step;

			if (!findBestRotation (P0, Q0, R0, p, q, r, alpha, beta))
				continue;
	
			bool found = false;
			for (int k=0;k<angles.size()/2;k++) {
				if ((fabs(alpha - angles[2*k]) < 0.1) && (fabs(beta - angles[2*k+1]) < 0.1)) {
					found = true;
					break;
				}
			}

			if (!found) {
				//LOG(LEVEL_DEBUG, "alpha = %f, beta = %f",alpha,beta);
				normAngle(alpha);
				normAngle(beta);
				angles.push_back(alpha);
				angles.push_back(beta);
			}

			if (angles.size() == 16) // found all possible solutions
				return;
		}
	}
}


// this version is GSL-based
bool MyGlWindow::findBestRotation ( EdgePlane _P0, EdgePlane _Q0, EdgePlane _R0, Edge *p, Edge *q, Edge *r, 
								   double &alpha, double &beta)
{
	
	//edgePlaneVector planes; // = _camera->_frame._selected_edges;
	
	//for (int i=0;i<_camera->_frame._selected_edges.size();i++)
	//	planes.push_back(_camera->_frame._selected_edges[i]);
	
	
	//EdgePlane _P0,_Q0,_R0;
	
	//_P0 = planes[0];
	//_Q0 = planes[1];
	//_R0 = planes[2];
	//_P0.rotate(Rcam);
	//_Q0.rotate(Rcam);
	//_R0.rotate(Rcam);
		
	/* gsl code starting here */
	size_t np = 2;
	size_t iter = 0;
	int status;
	double size;
	Vec3d pn = norm(p->dir);
	Vec3d qn = norm(q->dir);
	Vec3d rn = norm(r->dir);
	double par[18] = {_P0._normal[0], _P0._normal[1], _P0._normal[2], _Q0._normal[0], _Q0._normal[1], _Q0._normal[2], \
		_R0._normal[0], _R0._normal[1], _R0._normal[2], pn[0], pn[1], pn[2], qn[0], qn[1], qn[2],\
		rn[0], rn[1], rn[2]};
	
	const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex;
	gsl_multimin_fminimizer *s;
	
	gsl_multimin_function minex_func;
	
	gsl_vector *ss, *x;
	
	/* Initial vertex size vector */
	ss = gsl_vector_alloc (np);
	
	/* Set all step sizes to 1 */
	gsl_vector_set_all (ss, 1.0);
	
	   /* Starting point */
	x = gsl_vector_alloc (np);
	
	gsl_vector_set (x, 0, alpha);
	gsl_vector_set (x, 1, beta);
	
	/* Initialize method and iterate */
	minex_func.f = &computescore_gsl;
	minex_func.n = np;
	minex_func.params = (void *)&par;
	
	s = gsl_multimin_fminimizer_alloc (T, np);
    gsl_multimin_fminimizer_set (s, &minex_func, x, ss);
	do
	{
		iter++;
		status = gsl_multimin_fminimizer_iterate(s);
		
		if (status)
			break;
		
		size = gsl_multimin_fminimizer_size (s);
		status = gsl_multimin_test_size (size, 1e-3);
		
		/*if (status == GSL_SUCCESS)
		{
		printf ("converged to minimum at\n");
		}
		
		printf ("%5d ", iter);
		for (i = 0; i < np; i++)
		{
			printf ("%10.3e ", gsl_vector_get (s->x, i));
		}
		printf ("f() = %7.3f size = %.3f\n", s->fval, size);
		*/
	}
	while (status == GSL_CONTINUE && iter < 100);
	
	alpha = gsl_vector_get (s->x, 0);
	beta = gsl_vector_get (s->x, 1);
	double fval = s->fval;
	normAngle(alpha);
	normAngle(beta);
	
	gsl_vector_free(x);
	gsl_vector_free(ss);
	gsl_multimin_fminimizer_free (s);
	
	return (fabs(fval) < 0.2/*0.01*/);
	
}

// assuming that the correct rotation has been found which bring the 3D lines p,q,r
// in alignment with the image planes P,Q,R, find the translation that embedds p into P,
// q into Q and r into R
bool MyGlWindow::computeTranslation( Edge *p, Edge *q, Edge *r, EdgePlane &P, EdgePlane &Q, EdgePlane &R, bool transform )
{
	//Edge *p = lines[0], *q = lines[1], *r = lines[2];
	//EdgePlane P = planes[0], Q = planes[1], R = planes[2];
	Vec3d tcam = _camera->getTranslation();
	Quaternion Rcam = _camera->getRotation();
	if (transform) {
		P.fromCameraFrameToWorldFrame(tcam,Rcam);
		Q.fromCameraFrameToWorldFrame(tcam,Rcam);
		R.fromCameraFrameToWorldFrame(tcam,Rcam);
	}
	Vec3d t,x,y;
	double lambda;
	
	// first embedd p into P
	t = p->_a->getPosition()-intersectRayPlane(p->_a->getPosition(),P._normal,P._s,P._normal,lambda);
	_camera->translate(t);
	P._s = t + P._s;
	Q._s = t + Q._s;
	R._s = t + R._s;
	
	// second embedd q into Q
	y = norm(cross(P._normal,Q._normal));
	x = norm(cross(y,P._normal));
	
	if ( fabs(dot( x, Q._normal )) < 1E-3 )
		return false;

	// <x> is parallel to P and not parallel to <y> so it is a good direction for translating q
	t = q->_a->getPosition()-intersectRayPlane(q->_a->getPosition(),x,Q._s,Q._normal,lambda);
	_camera->translate(t);
	P._s = t + P._s;
	Q._s = t + Q._s;
	R._s = t + R._s;
	
	// finally, translate r along <y> to embedd it into R
	if ( fabs(dot( y, R._normal )) < 1E-3 )
		return false;

	t = r->_a->getPosition() - intersectRayPlane(r->_a->getPosition(),y,R._s,R._normal,lambda);
	//return 0.0;
	_camera->translate(t);
	P._s = t + P._s;
	Q._s = t + Q._s;
	R._s = t + R._s;
	
	return true;
}

/*void MyGlWindow::checkPose (PoseSolution &pose)
{
	if (!pose.isValid())
		return;
	
	// check that the solution is in the model
	if (_LUT._model->isOutside(pose.getPose().getTranslation())) {
		pose.setValid(false);
		return;
	}
}*/

// assuming correspondence between edge1 and line1 and between edge2 and a set of lines2
// remove lines that don't satisfy the azimuthal constraint
// (line1 and lines2 are assumed horizontal)
void MyGlWindow::filterLinesOnAzimuth (EdgePlane edge2, edgeVector &lines2)
{
	
	if (fabs(edge2._normal[2]) < COS_25) // if the edge is vertical, do nothing
		return;
	
	if ((fabs(edge2._normal[0]) < COS_25) && (fabs(edge2._normal[1]) < COS_25)) // if the edge is horizontal, do nothing
		return;
	
	// otherwise, assume that edge2 corresponds to a horizontal edge in the world
	// and compute its relative azimuth to edge1
	Vec3d dir2 = norm(cross(edge2._normal,Vec3d(0,0,1)));
	
	// now, filter out lines from <lines2> if they don't make an angle <alpha> with line1 (+/- 10 degrees)
	edgeVector new_lines;
	
	for (int i=0;i<lines2.size();i++) {
		Edge *line = lines2[i];
		if (line->_vertical) {
			printf("reject: line vertical for non-vertical image edge\n");
			continue;
		}
		if (fabs(dot(line->dir,dir2)) > COS_25)
			new_lines.push_back(line);
	}
	
	lines2.clear();
	for (i=0;i<new_lines.size();i++)
		lines2.push_back(new_lines[i]);
}


// assuming that the correspondence table has been filled, 
// refine the camera rotation and translation from n correspondences
int MyGlWindow::refineCameraPoseFromNCorrespondences( CorrespondenceVector &correspondences )
{
	//PerfTimer timer;

	// small perturbation
	//_camera->rotate(Vec3d(1,1,0),15.0*M_PI/180.0);
	//_camera->translate(Vec3d(12.0,15.0,13.0));


	// refine camera pose
	//LOG(LEVEL_INFO,"Camera pose (before refinement):\n");
	//_camera->printPose();
	//LOG(LEVEL_DEBUG,"corrTable score = %f",_corrManager->getScore(frameId));

	refineCameraRotationFromNCorrespondences( correspondences );
	
	refineCameraTranslationFromNCorrespondences( correspondences );

	//refineCameraRotationFromNCorrespondences( correspondences );
	
	//refineCameraTranslationFromNCorrespondences( correspondences );
	
	//LOG(LEVEL_INFO,"Final pose (%f sec):\n",timer.elapsed());
	//_camera->printPose();
	//LOG(LEVEL_DEBUG,"corrTable score = %f",_corrManager->getScore());

	return 1;
}

// assuming that the correspondence table has been filled, 
// refine the camera rotation from n correspondences
int MyGlWindow::refineCameraRotationFromNCorrespondences( CorrespondenceVector &correspondences )
{
	/* gsl code starting here */
	size_t np = 3;
	size_t iter = 0;
	int status;
	double size;

	int n = correspondences.size();

	double par[6*MAX_CORRESPONDENCES_FOR_REFINEMENT+1]; // use at most MAX_CORRESPONDENCES_FOR_REFINEMENT correspondences
	par[0] = MIN(MAX_CORRESPONDENCES_FOR_REFINEMENT,n);
	for (int i=0;i<MIN(MAX_CORRESPONDENCES_FOR_REFINEMENT,n);i++) {
		EdgePlane plane = correspondences[i].second;
		Edge *line = correspondences[i].first;

		par[6*i+1] = plane._normal[0];
		par[6*i+2] = plane._normal[1];
		par[6*i+3] = plane._normal[2];
		par[6*i+4] = line->dir[0];
		par[6*i+5] = line->dir[1];
		par[6*i+6] = line->dir[2];
	}
	
	const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex;
	gsl_multimin_fminimizer *s;
	
	gsl_multimin_function minex_func;
	
	gsl_vector *ss, *x;
	
	/* Initial vertex size vector */
	ss = gsl_vector_alloc (np);
	
	/* Set all step sizes to 1 */
	gsl_vector_set_all (ss, 1.0);
	
	   /* Starting point */
	x = gsl_vector_alloc (np);
	
	Vec3d rotation_vector;
	_camera->came.getRotation().toAxisAngle(rotation_vector);

	gsl_vector_set (x, 0, rotation_vector[0]);
	gsl_vector_set (x, 1, rotation_vector[1]);
	gsl_vector_set (x, 2, rotation_vector[2]);
	
	/* Initialize method and iterate */
	minex_func.f = &computescore_nrot_gsl;
	minex_func.n = np;
	minex_func.params = (void *)&par;
	
	s = gsl_multimin_fminimizer_alloc (T, np);
    gsl_multimin_fminimizer_set (s, &minex_func, x, ss);
	do
	{
		iter++;
		status = gsl_multimin_fminimizer_iterate(s);
		
		if (status)
			break;
		
		size = gsl_multimin_fminimizer_size (s);
		status = gsl_multimin_test_size (size, 1e-4);
		
		/*if (status == GSL_SUCCESS)
		{
		printf ("converged to minimum at\n");
		}
		
		printf ("%5d ", iter);
		for (i = 0; i < np; i++)
		{
			printf ("%10.3e ", gsl_vector_get (s->x, i));
		}
		printf ("f() = %7.6f size = %.6f\n", s->fval, size);
		*/
		
	}
	while (status == GSL_CONTINUE && iter < 100);
	
	rotation_vector = Vec3d(gsl_vector_get (s->x, 0),gsl_vector_get (s->x, 1),gsl_vector_get (s->x, 2));

	double theta = len(rotation_vector);
	if ( theta < EPS ) {
		_camera->setRotation( Quaternion( Vec3d(1,0,0), 0.0 ) );
	} else {
		_camera->setRotation(Quaternion(norm(rotation_vector),theta));
	}

	gsl_vector_free(x);
	gsl_vector_free(ss);
	gsl_multimin_fminimizer_free (s);
	
	//_corrManager->setPose(frameId,_camera->getPose());

	return 1;
}

// assuming that the correspondence table has been filled, 
// refine the camera translation from n correspondences
int MyGlWindow::refineCameraTranslationFromNCorrespondences( CorrespondenceVector &correspondences )
{
	/* gsl code starting here */
	size_t np = 3;
	size_t iter = 0;
	int status;
	double size;

	int n = correspondences.size();

	Quaternion qq = _camera->getRotation();

	double par[12*MAX_CORRESPONDENCES_FOR_REFINEMENT+1]; // use at most MAX_CORRESPONDENCES_FOR_REFINEMENT correspondences
	par[0] = MIN(MAX_CORRESPONDENCES_FOR_REFINEMENT,n);
	for (int i=0;i<MIN(MAX_CORRESPONDENCES_FOR_REFINEMENT,n);i++) {
		EdgePlane plane = correspondences[i].second;
		plane.rotate( qq );
		Edge *line = correspondences[i].first;
	
		if (line == NULL) {
			LOG(LEVEL_DEBUG,"failed to find line for id = %d (frameId = %d)",i,frameId);
			assert(false);
		}

		par[12*i+1] = plane._s[0];
		par[12*i+2] = plane._s[1];
		par[12*i+3] = plane._s[2];
		par[12*i+4] = plane._normal[0];
		par[12*i+5] = plane._normal[1];
		par[12*i+6] = plane._normal[2];
		par[12*i+7] = line->_a->getX();
		par[12*i+8] = line->_a->getY();
		par[12*i+9] = line->_a->getZ();
		par[12*i+10] = line->_b->getX();
		par[12*i+11] = line->_b->getY();
		par[12*i+12] = line->_b->getZ();
	}
	
	const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex;
	gsl_multimin_fminimizer *s;
	
	gsl_multimin_function minex_func;
	
	gsl_vector *ss, *x;
	
	/* Initial vertex size vector */
	ss = gsl_vector_alloc (np);
	
	/* Set all step sizes to 1 */
	gsl_vector_set_all (ss, 1.0);
	
	   /* Starting point */
	x = gsl_vector_alloc (np);
	
	Vec3d translation = _camera->came.getTranslation();

	gsl_vector_set (x, 0, translation[0]);
	gsl_vector_set (x, 1, translation[1]);
	gsl_vector_set (x, 2, translation[2]);
	
	/* Initialize method and iterate */
	minex_func.f = &computescore_ntrans_gsl;
	minex_func.n = np;
	minex_func.params = (void *)&par;
	
	s = gsl_multimin_fminimizer_alloc (T, np);
    gsl_multimin_fminimizer_set (s, &minex_func, x, ss);
	do
	{
		iter++;
		status = gsl_multimin_fminimizer_iterate(s);
		
		if (status)
			break;
		
		size = gsl_multimin_fminimizer_size (s);
		status = gsl_multimin_test_size (size, 1e-5);
		
		/*if (status == GSL_SUCCESS)
		{
		printf ("converged to minimum at\n");
		}
		
		printf ("%5d ", iter);
		for (i = 0; i < np; i++)
		{
			printf ("%10.3e ", gsl_vector_get (s->x, i));
		}
		printf ("f() = %7.6f size = %.6f\n", s->fval, size);
		*/
		
	}
	while (status == GSL_CONTINUE && iter < 100);
	
	translation = Vec3d(gsl_vector_get (s->x, 0),gsl_vector_get (s->x, 1),gsl_vector_get (s->x, 2));

	_camera->setTranslation(translation);

	gsl_vector_free(x);
	gsl_vector_free(ss);
	gsl_multimin_fminimizer_free (s);
	
	//_corrManager->setPose(frameId,_camera->getPose());

	return 1;
}

// compute the ECP (expected camera pose) from pose history
//
bool MyGlWindow::computeECP ( ExtrinsicParameters &pose )
{
	if ( pose_history.empty() ) {
		pose = _camera->getPose();
		return false;
	}

	int RANSAC_ECP_NITEMS = 10;

	if ( pose_history.size() < RANSAC_ECP_NITEMS ) {
		pose = pose_history[0];
		return false;
	}

	// set rotation (we will work only on translation)
	pose = _camera->getPose();

	// ransac on last 10 frames
	int RANSAC_ECP_N_TRIALS = 10;
	double RANSAC_ECP_SUCCESS_THRESHOLD = .70;
	double RANSAC_ECP_ACCEPTANCE_ERROR = 5.0; // 5 inches

	for ( int trial = 0; trial < RANSAC_ECP_N_TRIALS; trial++ ) {

		// pick two random positions in the past
		intVector indices;
		selectNRandomInt( 2, RANSAC_ECP_NITEMS, indices);

		// compute linear model
		int n = pose_history.size() - 1;
		Vec3d p0 = pose_history[n-indices[0]].getTranslation();
		Vec3d p1 = pose_history[n-indices[1]].getTranslation();

		// p  = p0 + ( t - t0 ) * ( p1 - p0 ) / dt
		double t0 = indices[0];
		double dt = (double)(indices[1] - indices[0]);

		// compute acceptance rate
		double rate = 0.0;
		for (int i=0;i<RANSAC_ECP_NITEMS;i++) {
			Vec3d p = pose_history[ n - i ].getTranslation();
			Vec3d estimated_p = p0 + ( i - t0 ) * ( p1 - p0 ) / dt;
			if ( len( p - estimated_p ) < RANSAC_ECP_ACCEPTANCE_ERROR )
				rate += 1.0 / RANSAC_ECP_NITEMS;
		}

		if ( rate > RANSAC_ECP_SUCCESS_THRESHOLD ) {
			pose.setTranslation( p0 + ( -1.0 - t0 ) * ( p1 - p0 ) / dt) ;
			return true;
		}
	}

	return false;

		// p = x0 * t^2 + x1 * t + x2 (for t = 0,1,2 at positions p0,p1,p2)
		//Vec3d x0 = p2 / 2 - p1 + p0 / 2;
		//Vec3d x1 = 2 * p1 - p2 / 2 - 3 * p0 / 2;
		//Vec3d x2 = p0;

		//compute acceptance rate
		//pose.setTranslation( x0 * 9 + x1 * 3 + x2 );
		
}


// same as PROJ_TEST, but synthetic!
bool MyGlWindow::PROJ_TEST_SYNTHETIC( corres &c, bool use_image )
{
	// save camera pose
	ExtrinsicParameters original_pose = _camera->getPose();

	// set camera pose at the next frame
	ExtrinsicParameters pose = GetSyntheticPose( frameId );
	_camera->setPose( pose );

	// generate the synthetic edgeplane
	intVector cameras;
	assert( c.line != NULL );
	_camera->findSensors( c.line, cameras );

	EdgePlane best_ep = EdgePlane();

	if ( cameras.empty() )
		return false;

	for (int i=0;i<cameras.size();i++) {
		int camera = cameras[i];
		
		Vec3d center = _camera->fromCameraFrameToWorldFrame( _camera->getUnitCameraCenter(camera) );
		Vec3d a = c.line->getA() - center;
		Vec3d b = c.line->getB() - center;
		EdgePlane edgeplane = EdgePlane( a, b, center, camera, 0, 0 );
		edgeplane.fromWorldFrameToCameraFrame( _camera->getPose() );
		if ( edgeplane.length() < best_ep.length() )
			continue;
		best_ep = edgeplane;
	}

	c.clear();
	c.eps.push_back( best_ep );
	c.computeScore( _camera->getPose() );
	_camera->setPose( original_pose );
	return true;
}

// return true if the line finds one or several matches on the image (and updates edgeplane if so)
// if use_image = true, <edgeplane> is used as a reference for clustering
// otherwise, a synthetic edge plane is generated and used instead
//
bool MyGlWindow::PROJ_TEST( corres &c, bool clear_mask, bool save_image, double angle_thresehold )
{
	int i,j;

	printf("correspondence %d: ", c._id);

	// if synthetic mode 2, read a synthetic edge
	if ( _synthetic_mode == 2 ) {
		return PROJ_TEST_SYNTHETIC( c, save_image );
	}
	
	if ( c.line == NULL )
		return false;

	// reference edge
	EdgePlane ref_edge;
	bool ref_edge_valid = false;
	
	Vec3d center = _camera->getTranslation();
	Vec3d a = c.line->getA()-center;
	Vec3d b = c.line->getB()-center;
	
	// make a synthetic edge plane to be used later in clustering
	EdgePlane line_edge = EdgePlane( a, b, center, 0, 0, 0 );
	line_edge.fromWorldFrameToCameraFrame( _camera->getPose() );
	
	if ( c.eid != -1 && c.eid < c.eps.size() ) {
		ref_edge = c.eps[c.eid];
		ref_edge_valid = true;
		//LOG(LEVEL_INFO, "using color information.");
	} else {
		ref_edge = line_edge;
	}

	// find the bucket ID of the edge
	intVector bucket_ids;
	get_edgeplane_buckets(ref_edge, bucket_ids);

	c.eps.clear();
	c.eid = -1;
	c.length = 0.0;

	std::vector< std::pair< double, int> > book;

	// find the corresponding matches on the image
	for (i=0;i<bucket_ids.size();i++) {

		int bucket_id = bucket_ids[i];

		if ( bucket_id > _edges_buckets.size() ) {
			LOG(LEVEL_ERROR, "pointer error (%d / %d)", bucket_id, _edges_buckets.size());
			continue;
		}

		for (j=0;j<_edges_buckets[bucket_id].size();j++) {

			int edge_id = _edges_buckets[bucket_id][j];

			EdgePlane edge;
			if (!_camera->_frame.get_edgeplane_chained( edge_id, edge ) ) {
				LOG(LEVEL_ERROR, "error accessing edgeplane %d out of %d edgeplanes.", edge_id, _camera->_frame.n_edgeplanes_chained );
				continue;
			}

			if ( edge.length() < 1E-7 )
				continue;

			if ( edge.overlap( line_edge ) < .1 && line_edge.overlap( edge ) < .1 )
				continue;

			if ( ref_edge_valid && edge.colorDistance( ref_edge ) > COLOR_THRESHOLD )
				continue;

			if ( edge.length() > 2.0 * line_edge.length() )
				continue;

			//if ( fabs(edge.angle( line_edge )) > toRadians( 5.0 ) )
			//	continue;

			std::pair< double, int > element;

			//printf("******* book element %d (%f, %f) *******\n", c.eps.size(), edge.length(), ref_edge.length());

			//edge.print();
			//ref_edge.print();

			element.first = fabs(edge.angle(ref_edge)) /* fabs( edge.length() - ref_edge.length() ) / ref_edge.length()*/;
			element.second = c.eps.size();

			book.push_back( element );

			c.eps.push_back( edge );
		}
	}

	if ( c.eps.empty() ) {
		printf("correspondence is empty...\n");
	} else {
		printf("OK\n");
	}

	if ( c.valid() ) {

	//for (i=0;i<book.size();i++) {
	//	printf("[%d] %f %d\n", i, book[i].first, book[i].second);
	//}

	std::sort( book.begin(), book.end() );

	c.eid = book[0].second;

	c.age++;

	c.length = c.eps[c.eid].length();

	}

	return c.valid();
}

// return true if the line finds one or several matches on the image (and updates edgeplane if so)
// if use_image = true, <edgeplane> is used as a reference for clustering
// otherwise, a synthetic edge plane is generated and used instead
//
bool MyGlWindow::PROJ_TEST_DISPLAY( corres &c, bool clear_mask, bool save_image, double angle_thresehold, edgePlaneVector &edges_display )
{
	int i,j;

	// if synthetic mode 2, read a synthetic edge
	if ( _synthetic_mode == 2 ) {
		return PROJ_TEST_SYNTHETIC( c, save_image );
	}
	
	if ( c.line == NULL )
		return false;

	// reference edge
	EdgePlane ref_edge;
	bool ref_edge_valid = false;
	
	Vec3d center = _camera->getTranslation();
	Vec3d a = c.line->getA()-center;
	Vec3d b = c.line->getB()-center;
	
	// make a synthetic edge plane to be used later in clustering
	EdgePlane line_edge = EdgePlane( a, b, center, 0, 0, 0 );
	line_edge.fromWorldFrameToCameraFrame( _camera->getPose() );
	
	if ( c.eid != -1 && c.eid < c.eps.size() ) {
		ref_edge = c.eps[c.eid];
		ref_edge_valid = true;
		//LOG(LEVEL_INFO, "using color information.");
	} else {
		ref_edge = line_edge;
	}

	// find the bucket ID of the edge
	intVector bucket_ids;
	get_edgeplane_buckets(ref_edge, bucket_ids);

	c.eps.clear();
	c.eid = -1;
	c.length = 0.0;

	std::vector< std::pair< double, int> > book;

	edges_display.clear();

	// find the corresponding matches on the image
	for (i=0;i<bucket_ids.size();i++) {

		int bucket_id = bucket_ids[i];

		if ( bucket_id > _edges_buckets.size() ) {
			LOG(LEVEL_ERROR, "pointer error (%d / %d)", bucket_id, _edges_buckets.size());
			continue;
		}

		for (j=0;j<_edges_buckets[bucket_id].size();j++) {

			int edge_id = _edges_buckets[bucket_id][j];

			EdgePlane edge;
			if (!_camera->_frame.get_edgeplane_chained( edge_id, edge ) ) {
				LOG(LEVEL_ERROR, "error accessing edgeplane %d out of %d edgeplanes.", edge_id, _camera->_frame.n_edgeplanes_chained );
				continue;
			}

			edges_display.push_back( edge );

			if ( edge.length() < 1E-7 )
				continue;

			if ( edge.overlap( line_edge ) < .1 && line_edge.overlap( edge ) < .1 )
				continue;

			if ( ref_edge_valid && edge.colorDistance( ref_edge ) > COLOR_THRESHOLD )
				continue;

			if ( edge.length() > 2.0 * line_edge.length() )
				continue;

			//if ( fabs(edge.angle( line_edge )) > toRadians( 5.0 ) )
			//	continue;

			std::pair< double, int > element;

			//printf("******* book element %d (%f, %f) *******\n", c.eps.size(), edge.length(), ref_edge.length());

			//edge.print();
			//ref_edge.print();

			element.first = fabs(edge.angle(ref_edge)) /* fabs( edge.length() - ref_edge.length() ) / ref_edge.length()*/;
			element.second = c.eps.size();

			book.push_back( element );

			c.eps.push_back( edge );
		}
	}

	if ( c.valid() ) {

	//for (i=0;i<book.size();i++) {
	//	printf("[%d] %f %d\n", i, book[i].first, book[i].second);
	//}

	std::sort( book.begin(), book.end() );

	c.eid = book[0].second;

	c.age++;

	c.length = c.eps[c.eid].length();

	}

	return c.valid();
}




// return true if the line finds one or several matches on the image (and updates edgeplane if so)
// if use_image = true, <edgeplane> is used as a reference for clustering
// otherwise, a synthetic edge plane is generated and used instead
//
/*bool MyGlWindow::PROJ_TEST( corres &c, bool clear_mask, bool save_image, double angle_thresehold )
{
	// if synthetic mode 2, read a synthetic edge
	if ( _synthetic_mode == 2 ) {
		return PROJ_TEST_SYNTHETIC( c, save_image );
	}
	
	if ( c.line == NULL )
		return false;
	
	// reference edge
	EdgePlane ref_edge;
	bool ref_edge_valid = false;

	if ( c.eid != -1 && c.eid < c.eps.size() ) {
		ref_edge = c.eps[c.eid];
		ref_edge_valid = true;
		LOG(LEVEL_INFO, "using color information.");
	}

	//PerfTimer timer;

	// compute the mask for the given line
	if (clear_mask)
		_camera->clearMasks();
	
	computeMask( c.line, DISTORTION_EDGE_STEP );
	
	//timer.print("compute mask");

	// find which sensor is able to see the 3D line
	intVector cameras;
	_camera->findSensors( c.line, cameras );
		
	printf("detecting lines\n");

	// query the edge detector
	_camera->detectLines( false,save_image,edgePixels->value(),edgeThresh->value(),edgeSigma->value());
	
	//timer.print("detect lines");

	// convert the (distorted) 2D edges into rectified edgeplanes
	_camera->convertLinesToPlanes();
	
	printf("done.\n");

	//timer.print("convert to planes");

	// if no edge is found, reject
	if (_camera->_frame.nedgeplanes() == 0) {
		c.eps.clear();
		c.eid = -1;
		c.length = 0.0;
		return false;
	}
	
	// otherwise, merge and keep *all* matches for each camera
	// RANSAC pose computation will later decide whether one of them is "correct"
		
	c.eps.clear();
	c.eid = -1;
	c.length = 0.0;

	doubleVector angles;

	for ( int cameraId=0;cameraId<_camera->_frame._nImages;cameraId++ ) {
		
		//PerfTimer timer3;

		Vec3d center = _camera->fromCameraFrameToWorldFrame( _camera->getUnitCameraCenter(cameraId) );//_camera->getTranslation();//
		
		Vec3d a = c.line->getA()-center;
		Vec3d b = c.line->getB()-center;
		
		// make a synthetic edge plane to be used later in clustering
		EdgePlane edgeplane = EdgePlane( a, b, center, cameraId, 0, 0 );
		edgeplane.fromWorldFrameToCameraFrame( _camera->getPose() );
		
		//timer3.print("\tmake");

		edgePlaneVector edgeplanes;
		clusterEdges ( cameraId, edgeplane, edgeplanes, angle_thresehold );
		
		//timer3.print("\tcluster");

		for (int i=0;i<edgeplanes.size();i++) {

			EdgePlane edge = edgeplanes[i];

			// flip edge so that it has the same left and right as the reference edge
			if ( dot(edge._normal,ref_edge._normal) < 0 )
				edge.flip();

			//PerfTimer timer2;

			printf("update color\n");
			_camera->updateColor( edge, false );
			printf("done\n");

			//timer2.print("\t\tcolor");

			// filter out using color
			printf("post-prcessing\n");

			if ( ref_edge_valid ) {
				double color_distance = edge.colorDistance( ref_edge );
				if ( color_distance < 0.02 ) {
					c.eps.push_back( edge );
					double alpha = fabs(edge.angle( edgeplane ) );
					angles.push_back( alpha );
				}
			} else { // don't filter out if color info is not available for previous frame
				c.eps.push_back( edge );
				double alpha = fabs((edge.length() - edgeplane.length()) / edgeplane.length()); //fabs(edge.angle( edgeplane ) );
				angles.push_back( alpha );
			}
		}

		//timer3.print("color");

	}
	
	//timer.print("clustering");
	int i;

	if ( c.valid() ) {
		
		// if the line was not visible before, only keep the top three edges
		// based on subtended angle
		if ( c.eid == -1 && c.eps.size() > 3) {
			
			std::vector< std::pair< double, int > > book;
			for (i=0;i<angles.size();i++) {
				std::pair< double, int > element;
				element.first = angles[i];
				element.second = i;
				book.push_back( element );
			}
			
			std::sort( book.begin(), book.end() );

			edgePlaneVector edges = c.eps;
			
			c.eps.clear();
			
			for (i=0;i<3;i++) {
				c.eps.push_back( edges[book[i].second] );
			}

			if ( angles.size() > 3 )
				angles.resize(3);
		}
		
		int best_j = 0;
		for (int j=1;j<angles.size();j++) {
			if ( angles[j] < angles[best_j] ) 
				best_j = j;
		}
		
		c.eid = best_j;
		c.age++;
		c.length = c.eps[best_j].length();
	}
	

	return c.valid();
}
*/

// once edges have been detected in the region, cluster them into a set of edges
// the idea is to first cluster edges according to overlap (each bucket contains non-overlapping edges)
// then inside each bucket, RANSAC to find inliers (orientation-wise) and merge inliers
// <edgeplane> is used to filter out irrelevant clusters
bool MyGlWindow::clusterEdges( int cameraId, EdgePlane &edgeplane, edgePlaneVector &edgeplanes, double angle_thresehold )
{
	// 0. reset the clusters
	resetClusters( cameraId );

	// 1. cluster edges by overlap
	int n = clusterEdgesOverlap( cameraId );

	//LOG( LEVEL_INFO, "# clusters [1]: %d", n );

	// 2. filter outliers in each cluster
	filterClustersOutliers ( cameraId );

	// 3. redistribute edges across clusters by affinity
	redistributeEdges( cameraId, n );

	//LOG( LEVEL_INFO, "# clusters [2]: %d", n );

	// 4. merge inliers in each cluster
	mergeClusterInliers ( cameraId, n, edgeplanes );

	// 5. filter out irrelevant clusters (orientation)
	filterIrrelevantClusters( edgeplane, edgeplanes, angle_thresehold );

	n = edgeplanes.size();

	//LOG( LEVEL_INFO, "camera %d --  clusters : %d", cameraId, n );

	return true;
}

// cluster edges into buckets according to overlap (buckets contain non-overlapping edges)
//
int MyGlWindow::clusterEdgesOverlap( int cameraId )
{
	int nclusters = 0;
	
	for (int i=0;i<_camera->_frame._edgeplanes[cameraId].size();i++) {
		EdgePlane ep = _camera->_frame._edgeplanes[cameraId][i]; // for each edgeplane...
		
		bool accepted = false;
		double max_angle = M_PI;
		int best_cluster = -1;

		for ( int c=0; c<nclusters; c++ ) {
			// check whether this cluster accepts the edge
			
			accepted = true;
			double angle = 0.0; // maximum angle with the element of the cluster
			
			for (int k=0;k<_camera->_frame._edgeplanes[cameraId].size();k++) {
				EdgePlane ep2 = _camera->_frame._edgeplanes[cameraId][k];
				if ( ep2._chaineId != c ) // for each edgeplane in this cluster
					continue;
				if ( ep2.overlap( ep ) > EPS || ep.overlap( ep2 ) > EPS ) { // if overlap, edge is not accepted
					accepted = false;
					break;
				}
				
				// if accepted by this edge, memorize the angle
				angle = MAX( angle, ep.angle( ep2 ) );
				
				if ( !accepted )
					break;
			}
			
			// if accepted, memorize the max angle
			if ( accepted ) {
				if ( angle < max_angle ) {
					best_cluster = c;
					max_angle = angle;
				}
			}
		}
		
		if ( accepted ) {
			assert( best_cluster != -1 );
			_camera->_frame._edgeplanes[cameraId][i]._chaineId = best_cluster;
		} else { // if didn't find a spot in a cluster, make a new cluster
			_camera->_frame._edgeplanes[cameraId][i]._chaineId = nclusters;
			nclusters++;
		}
	}
	
	return nclusters;
}

// redistribute edges that have been ejected across clusters
// for each edge, measure the average angle distance with each cluster
// and keep the best one
//
void MyGlWindow::redistributeEdges ( int cameraId, int &nclusters )
{
	doubleVector angles;
	intVector counters;
	int p;
	
	for (int i=0;i<_camera->_frame._edgeplanes[cameraId].size();i++) {
		EdgePlane ep = _camera->_frame._edgeplanes[cameraId][i];
		
		if ( ep._chaineId != -1 ) // if already has a cluster, continue
			continue;

		// initialize counters
		angles.clear();
		counters.clear();
		for ( p=0; p<nclusters; p++ ) {
			angles.push_back( 0.0 );
			counters.push_back( 0 );
		}
		
		// fill in tables
		for (int k=0;k<_camera->_frame._edgeplanes[cameraId].size();k++) {
			EdgePlane dep = _camera->_frame._edgeplanes[cameraId][k];
			
			int c = dep._chaineId;
			
			if ( c == -1 )
				continue;
			
			counters[dep._chaineId]++;
			angles[dep._chaineId] += ep.angle( dep );
		}
		
		// normalize angle table and find best cluster
		for ( p=0; p<nclusters; p++ ) {
			if ( counters[p] > 0 )
				angles[p] = angles[p] / counters[p];
			else
				angles[p] = 2 * M_PI; // worst possible average angle
		}
		
		double best_angle = angles[0];
		for ( p=1; p<nclusters; p++ ) {
			if ( angles[p] < best_angle ) {
				best_angle = angles[p];
				_camera->_frame._edgeplanes[cameraId][i]._chaineId = p;
			}
		}
		
		// if best angle is larger than threshold, create a new cluster
		if ( best_angle > CLUSTER_ANGLE_THRESHOLD ) {
			_camera->_frame._edgeplanes[cameraId][i]._chaineId = nclusters;
			nclusters++;
		}
	}		
}

// merge edges with each cluster
int MyGlWindow::mergeClusterInliers ( int cameraId, int nclusters, edgePlaneVector &edgeplanes )
{
	// create a temporary output and initialize it with empty edge planes
	edgePlaneVector planes;
	intVector counters;
	for ( int c=0; c<nclusters; c++ ) {
		planes.push_back ( EdgePlane() );
		counters.push_back( 0 );
	}
	
	// merge each edge into the corresponding bucket in the temporary output
	for (int i=0;i<_camera->_frame._edgeplanes[cameraId].size();i++) {
		
		c = _camera->_frame._edgeplanes[cameraId][i]._chaineId;
		
		if ( c == -1 )
			continue;
		
		if ( counters[c] == 0 )
			planes[c] = _camera->_frame._edgeplanes[cameraId][i];
		else {
			if ( planes[c].angle( _camera->_frame._edgeplanes[cameraId][i] ) < CLUSTER_ANGLE_THRESHOLD )
				planes[c].merge( &_camera->_frame._edgeplanes[cameraId][i] );
		}
		
		counters[c]++;
	}
	
	// filter out edges corresponding to empty buckets
	edgeplanes.clear();
	
	for ( c=0; c<nclusters; c++ ) {
		if ( counters[c] != 0 ) 
			edgeplanes.push_back( planes[c] );
	}
	
	return edgeplanes.size();
}

// find the inliers in each cluster:
// if an edge disagrees with more than half of its cluster mates, it is an outlier
void MyGlWindow::filterClustersOutliers ( int cameraId )
{
	for (int i=0;i<_camera->_frame._edgeplanes[cameraId].size();i++) {
		EdgePlane ep = _camera->_frame._edgeplanes[cameraId][i];
		int size = 0;
		int counter = 0;
		
		// for each edge, compute the <size> of its cluster and the number <counter>
		// of its mates for which the angle is larger than some threshold
			for (int k=0;k<_camera->_frame._edgeplanes[cameraId].size();k++) {
				
				if  ( k == i ) 
					continue;
				
				EdgePlane dep = _camera->_frame._edgeplanes[cameraId][k];
				
				if ( dep._chaineId != ep._chaineId )
					continue;
				
				size++;
				if ( dep.angle( ep ) > CLUSTER_ANGLE_THRESHOLD )
					counter++;
			}
		
		// if the ratio is larger than 50%, the edge is an outlier
		if ( (double)counter / size > 0.5 - EPS )
			_camera->_frame._edgeplanes[cameraId][i]._chaineId = -1;
	}
}

// use <edgeplane> to filter out irrelevant clusters
//
void MyGlWindow::filterIrrelevantClusters( EdgePlane &edgeplane, edgePlaneVector &edgeplanes, double angle_thresehold )
{
	edgePlaneVector temp;

	// first filter out the clusters which are not in the right orientation, or too small
	for (int i=0;i<edgeplanes.size();i++) {
		if ( edgeplane.angle( edgeplanes[i] ) > angle_thresehold/*CLUSTER_MAX_ANGLE_THRESHOLD*/ ) {
			//LOG(LEVEL_DEBUG, "filter out: reference angle = %f deg.", toDegrees(edgeplane.angle( edgeplanes[i] )) );
			continue;
		}
		if ( edgeplanes[i].length() < CLUSTER_MIN_SUBTENDED_ANGLE ) {
			//LOG(LEVEL_DEBUG, "filter out: subtended angle = %f deg.", toDegrees(edgeplanes[i].length()) );
			continue;
		}
		temp.push_back( edgeplanes[i] );
	}

	// copy temp into edgeplanes
	edgeplanes.clear();
	for ( i=0;i<temp.size();i++ ) {
		edgeplanes.push_back( temp[i] );
	}
}


// select the best match for <edgeplane> among <edgeplanes>
// we use angle distance primarily
// but we might use color etc. later on
// mode = 0: reject if list has more than one element
// mode = 1: keep the longest element
// mode = 2: keep the best orientation
//
bool MyGlWindow::selectBestEdge( EdgePlane &edgeplane, edgePlaneVector &edgeplanes, int mode = 0 )
{
	// if empty, return failure
	if ( edgeplanes.empty() )
		return false;

	if ( mode == 0 ) {
		if ( (_database != NULL) && (_database->_synthetic == 1) && (edgeplanes.size() != 1) ) { // this filter is only applicable to synthetic data...
			return false;
		}
		else {
			edgeplane = edgeplanes[0];
			return true;
		}
	}

	if ( mode == 1 ) {
		// select the first edge plane
		EdgePlane output = edgeplanes[0];
		double angle = output.length();
		
		// check whether the other ones are better
		for ( int i=1; i<edgeplanes.size(); i++ ) {
			double a = edgeplanes[i].length();
			if ( a > angle ) {
				angle = a;
				output = edgeplanes[i];
			}
		}
		
		// update the output edge
		edgeplane = output;
		
		return true;
	}

	if ( mode == 2 ) {
		// select the first edge plane
		EdgePlane output = edgeplanes[0];
		double angle = edgeplane.angle( output );
		
		// check whether the other ones are better
		for ( int i=1; i<edgeplanes.size(); i++ ) {
			double a = edgeplanes[i].angle( edgeplane );
			if ( a < angle ) {
				angle = a;
				output = edgeplanes[i];
			}
		}
		
		// update the output edge
		edgeplane = output;
		
		return true;
	}

	return false;
}

// reset the cluster IDs
//
void MyGlWindow::resetClusters ( int cameraId )
{
	for (int i=0;i<_camera->_frame._edgeplanes[cameraId].size();i++) {
		_camera->_frame._edgeplanes[cameraId][i]._chaineId = -1;
	}
}

// score the camera pose given as argument
// using the visible lines (_LUT._lines) and the observed edges (_camera._frame._edgeplanes_chained)
//
double MyGlWindow::score_camera_pose ( ExtrinsicParameters pose, std::vector< intVector > &edges_buckets )
{
	int i,j;

	int N = _LUT._lines.size();
	int M = _camera->_frame.nedgeplanes_chained();

	// convert the model lines into edgeplanes in the camera coordinate frame
	edgePlaneVector lines_edgeplanes;
	Vec3d center = pose.getTranslation();
	for (i=0;i<N;i++) {
		EdgePlane ep( _LUT._lines[i]->getA() - center, _LUT._lines[i]->getB() - center, center, -1, _LUT._lines[i]->_id, i );
		ep.fromWorldFrameToCameraFrame( pose );
		lines_edgeplanes.push_back( ep );
		//Vec3d a = _camera->fromWorldFrameToCameraFrame( _LUT._lines[i]->getA() );
		//Vec3d b = _camera->fromWorldFrameToCameraFrame( _LUT._lines[i]->getB() );
		//Vec3d s = Vec3d(0,0,0);
		//lines_edgeplanes.push_back( EdgePlane( a, b, s, -1, _LUT._lines[i]->_id, i ) );
	}

	// init a list of lines to match (lines will be pulled from the list and pushed again if another match was better)
	intVector lines;
	for (i=0;i<N;i++)
		lines.push_back( i );

	double **scores = new double*[N]; // keep a table of scores
	for (i=0;i<N;i++)
		scores[i] = new double[M];
	
	for (i=0;i<N;i++) {		// init the scores to -1.0
		for (j=0;j<M;j++) {
			scores[i][j] = -1.0;
		}
	}

	// find the best match for each line
	while ( !lines.empty() ) {

		int line_id = lines.back();
		lines.pop_back();
		
		EdgePlane line = lines_edgeplanes[line_id];
		
		intVector bucket_ids; 
		get_edgeplane_buckets( line, bucket_ids );
				
		int best_edge_id = -1;
		double best_score = -1.0;
		
		for (i=0;i<bucket_ids.size();i++) {
			
			int bucket_id = bucket_ids[i];
			
			for (int j=0;j<edges_buckets[bucket_id].size();j++) {
				
				int edge_id = edges_buckets[bucket_id][j];
				
				EdgePlane edge;
				if (!_camera->_frame.get_edgeplane_chained( edge_id, edge ) ) {
					LOG(LEVEL_ERROR, "error accessing edgeplane %d out of %d edgeplanes.", edge_id, _camera->_frame.n_edgeplanes_chained );
					continue;
				}
				
				if ( edge.length() > line.length() * 1.5 )
					continue;
				
				if ( edge.overlap( line ) < SCORE_MIN_OVERLAP )
					continue;
				
				double angle = line.angle( edge );
				
				if ( angle > SCORE_MAX_DIHEDRAL_ANGLE ) 
					continue;
				
				double score = ( 1.0 - ( angle / M_PI ) ) * MIN(1.0, edge.length() / line.length());
				
				assert ( score >= 0.0 );
				
				// either the score has increased and the edge had no match
				// or the edge had already a match but we are beating it
				if ( score > best_score ) {
					
					best_edge_id = edge_id;
					best_score = score;
				}
			}
		}
		
		
		if ( best_edge_id == -1 ) {
			continue;
		}
		
		scores[line_id][best_edge_id] = best_score; // update the score table
	}

	// line with no match: -1
	// edge with no match: -1
	// edge-line match: +1
	double final_score = 0.0;

	double *row = new double[M];
	for (i=0;i<M;i++)
		row[i] = -1.0;

	for (i=0;i<N;i++) {

		double best_score = -1.0;

		for (j=0;j<M;j++) {
		
			best_score = MAX(best_score, scores[i][j]);

			if ( scores[i][j] >= 0.0 )
				row[j] = 0.0;

		}

		final_score += best_score;
	}

	for (i=0;i<M;i++)
		final_score += row[i];

	// scale score between 0 and 1
	final_score = ( ( final_score / (N+M) ) + 1.0 ) / 2.0;

	// free memory
	for (i=0;i<N;i++)
		delete[] scores[i];
	delete[] scores;

	delete[] row;

	return final_score;
}

// refine camera translation assuming camera rotation given as input is correct
// and that camera height is between h1 and h2
// return the best score and update the pose
//
double MyGlWindow::refine_camera_translation( ExtrinsicParameters &pose, double h1, double h2, std::vector< intVector > &edges_buckets )
{
	Vec3d t0 = pose.getTranslation();

	ExtrinsicParameters best_pose = pose;

	double best_score = -1.0;

	for (double alpha = 0.0; alpha < 2*M_PI; alpha += 2*M_PI / 10.0) {

		for ( double d = 0.0; d < _LUT.GRID_STEP; d += _LUT.GRID_STEP / 10.0) {

			for (double h = h1; h < h2; h += (h2-h1)/10.0) {

				Vec3d t = t0 + Vec3d( d * cos(alpha), d * sin(alpha), 0.0 );
				t[2] = h;
				pose.setTranslation( t );

				double score = score_camera_pose( pose, edges_buckets );

				if ( score > best_score ) {

					best_score = score;

					best_pose = pose;
				}
			}
		}
	}

	pose = best_pose;

	return best_score;
}

// insert a pose in the list of INIT poses
//
void MyGlWindow::insert_init_pose( ExtrinsicParameters &pose )
{

	if ( pose.getTranslation()[2] > 0.0 && pose.getTranslation()[2] < INIT_MAX_CAMERA_HEIGHT && _LUT.valid_pose( pose ) ) {

		pose.update_time();

		init_poses.push_back( pose );

		init_poses_changed = true;
	}
}

// determine the neighboring buckets for that edgeplane
//
int MyGlWindow::get_edgeplane_buckets( EdgePlane ep, intVector &bucket_ids )
{
	// determine the highest end point
	Vec3d n = norm(cross(ep._b, ep._a));

	//if ( n[2] < 0 )
	//	n = -n;

	spheretsn->closest_cells( n, bucket_ids );
	spheretsn->closest_cells( -n, bucket_ids );

	return 0;
}

// return the number of buckets
//
int MyGlWindow::get_n_buckets()
{
	return spheretsn->_faces.size();
}


// distribute edge planes into buckets
//
void MyGlWindow::distribute_edgeplanes_into_buckets ( edgePlaneVector &edgeplanes, std::vector< intVector > &buckets )
{
	int counter = 0;
	
	int n_buckets = get_n_buckets();
	
	buckets.clear();
	
	for (int k=0;k<n_buckets;k++) {
		intVector v;
		buckets.push_back( v );
	}
	
	for (int i=0;i<edgeplanes.size();i++) {
		
		EdgePlane ep = edgeplanes[i];
		
		intVector bucket_ids;
		get_edgeplane_buckets( ep, bucket_ids );
		
		for (int k=0;k<bucket_ids.size();k++) {
			buckets[bucket_ids[k]].push_back( counter );
		}
		
		counter++;
	}
}

// distribute edge planes into buckets
//
void MyGlWindow::distribute_edgeplanes_into_buckets ( std::vector< intVector > &buckets )
{
	int counter = 0;

	int n_buckets = get_n_buckets();

	buckets.clear();

	for (int k=0;k<n_buckets;k++) {
		intVector v;
		buckets.push_back( v );
	}

	for (int i=0;i<_camera->_frame._edgeplanes_chained.size();i++) {
		for (int j=0;j<_camera->_frame._edgeplanes_chained[i].size();j++) {

			EdgePlane ep = _camera->_frame._edgeplanes_chained[i][j];

			intVector bucket_ids;
			get_edgeplane_buckets( ep, bucket_ids );

			for (int k=0;k<bucket_ids.size();k++) {
				buckets[bucket_ids[k]].push_back( counter );
			}

			counter++;
		}
	}
}

// compute the topk vanishing points from a list of edgeplanes
void MyGlWindow::compute_vanishing_points( int topk, edgePlaneVector &edgeplanes, std::vector< Vec3d > &vps) 
{
	vps.clear();
	
	int i,j,k;
	
	// use cells to find high density of VPs
	std::vector< intVector > cells;
	
	for (i=0;i<get_n_buckets();i++) {
		intVector v;
		cells.push_back( v );
	}
	
	std::vector< Vec3d > select_vps;
	
	for ( i=0;i<edgeplanes.size();i++) {
		
		EdgePlane epi = edgeplanes[i];
		
		Vec3d ai = epi._a;
		Vec3d bi = epi._b;
		
		Vec3d ni = norm(cross(ai,bi));
		
		for (j=i+1;j<edgeplanes.size();j++) {
			
			
			EdgePlane epj = edgeplanes[j];
			
			Vec3d aj = epj._a;
			Vec3d bj = epj._b;
			
			Vec3d nj = norm(cross(aj,bj));
			
			Vec3d n = cross( ni, nj ); // compute the vanishing point
			
			if ( len(n) < toRadians( 5.0 ) ) // skip if the two planes are almost parallel
				continue;
			
			n = norm(n); // normalize the vanishing point
			
			// check that the planes do not intersect at the vanishing point
			
			if ( dot(ni, cross(ai,n)) > 0 && dot(ai,n) > dot(ai,bi) )
				continue;
			
			if ( dot(nj, cross(aj,n)) > 0 && dot(aj,n) > dot(aj,bj) )
				continue;
			
			if ( dot(ni, cross(ai,-n)) > 0 && dot(ai,-n) > dot(ai,bi) )
				continue;
			
			if ( dot(nj, cross(aj,-n)) > 0 && dot(aj,-n) > dot(aj,bj) )
				continue;
			
			if ( n[2] < 0 ) // vps must belong to the northern hemisphere
				n = -n;
			
			if ( n[2] < EPS && n[0] < 0)
				n = -n;
			
			intVector buckets;
			
			spheretsn->closest_cells( n, buckets );
			
			for (k=0;k<buckets.size();k++) {
				cells[buckets[k]].push_back( vps.size() );
				vps.push_back( n );
			}
			
		}
	}
	
	// look for the top-k VPs
	
	for (i=0;i<topk;i++) {
		
		int best_cell = 0;
		
		for (j=0;j<cells.size();j++) {
			
			if ( cells[j].size() > cells[best_cell].size() )
				best_cell = j;
		}
		
		int ni = cells[best_cell].size();
		
		if ( ni < 1 )
			continue;
		
		Vec3d average_vp = Vec3d(0,0,0);
		
		for (j=0;j<ni;j++) 
			average_vp = average_vp + vps[cells[best_cell][j]];
		
		average_vp = norm(average_vp / ni);
		
		select_vps.push_back( average_vp );
		
		// cleanup the density in the area
		intVector buckets;
		
		spheretsn->closest_cells( average_vp, buckets );
		
		for (j=0;j<buckets.size();j++)
			cells[buckets[j]].clear();
	}

	vps.clear();

	for (i=0;i<select_vps.size();i++)
		vps.push_back( select_vps[i] );
}

void MyGlWindow::sort_init_poses( )
{
	std::sort( init_poses.begin(), init_poses.end() );
}

// compute the color hits for the current image
//
void MyGlWindow::computeColorHits( int level_of_details )
{
	int i;

	// lookup LUT
	_LUT.lookup( _camera->came.getTranslation(), 0.0/*c->MIN_SUBTENDED_ANGLE*/, 0.0, 0, 0);

	// compute the color hits for the current frame
	for (i=0;i<_camera->_frame._nImages;i++) {

		std::vector < color_hit > hits;

		LOG(LEVEL_INFO, "processing sensor %d", i);
		_camera->computeColorHits( i, level_of_details, _LUT._model->_color_resolution, _LUT._faces, hits );
		LOG(LEVEL_INFO, "%d hits", hits.size());

		for (int j=0;j<hits.size();j++) {
			_LUT._model->insertColorHit( hits[j] );
		}

		Fl::check();
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
// add an anchor given the point selected on the screen (x,y) and the camera ID
//
void MyGlWindow::addAnchor( int x, int y, int cameraId ) 
{

	// lookup the visible faces (for intersection)
	_LUT.lookup( _camera->came.getTranslation(), 0.0, 0.0, 0, 0 );

	//

	int rx,ry;

	cameraId = fromSelectedPointToImagePoint( x, y, cameraId, rx, ry );

	if ( cameraId == -1 )
		return;

	// back project the ray
	_camera->setActiveSensor( cameraId );
	
	Vec3d center = _camera->getTranslation();
	
	Vec3d ray = _camera->rectifyAndUnproject( Vec2d(rx,ry) ) - center;
	
	// intersect it with the current faces
	double lambda = -1.0;
	
	Vec3d target = Vec3d(0,0,0);
	
	int face_id = intersectRayFaces( center, ray, _LUT._faces, target );
	
	// insert a new anchor
	if ( face_id != -1 ) {
		
		Face *face = _LUT._faces[face_id];
		
		// define a local coordinate frame for the anchor (u,v,n)
		Vec3d n = face->_norm;
		Vec3d v = cross(n,Vec3d(0,0,1));
		if ( len(v) < EPS ) 
			v = cross(n,Vec3d(1,0,0));
		v = norm(v);
		
		Vec3d u = norm(cross(v,n));
		
		// define the face vertices of the anchor target
		
		anchor anc;
		anc.cc = center;
		anc.a = target + ANCHOR_SIZE /2 * u + ANCHOR_SIZE /2 * v;
		anc.b = target - ANCHOR_SIZE /2 * u + ANCHOR_SIZE /2 * v;
		anc.c = target - ANCHOR_SIZE /2 * u - ANCHOR_SIZE /2 * v;
		anc.d = target + ANCHOR_SIZE /2 * u - ANCHOR_SIZE /2 * v;
		anc.o = (anc.a + anc.b + anc.c + anc.d) / 4.0;
		anc.c1 = Vec3d(0,0,0);
		anc.c2 = Vec3d(0,0,0);

		anc.n = n;
		
		_anchors.push_back( anc );
		
		LOG(LEVEL_INFO, "Anchor added on face %d", _LUT._faces[face_id]->_id);
		
	}
}

void MyGlWindow::removeAnchor( int x, int y, int cameraId ) 
{

	if ( _anchors.empty() )
		return;

	// lookup the visible faces (for intersection)
	_LUT.lookup( _camera->came.getTranslation(), 0.0, 0.0, 0, 0 );

	//
	int rx,ry;
	cameraId = fromSelectedPointToImagePoint( x, y, cameraId, rx, ry );

	if ( cameraId == -1 )
		return;

	// back project the ray
	_camera->setActiveSensor( cameraId );
	
	Vec3d center = _camera->getTranslation();
	
	Vec3d ray = _camera->rectifyAndUnproject( Vec2d(rx,ry) ) - center;

	// look for the closest anchor

	std::vector< anchor >::iterator iter = _anchors.begin();
	std::vector< anchor >::iterator rm_iter = NULL;
	double distance = 1E10;

	// for each anchor...
	for (iter = _anchors.begin(); iter != _anchors.end(); iter++) {

		// determine the closest distance between the ray and the anchor
		Vec3d p1, p2;
		closest_segment_approach( center, center + 100.0 * ray, iter->cc, (iter->a + iter->b + iter->c + iter->d) / 4.0, p1, p2 );
		
		double d = len(p2-p1);
		
		if ( dot(ray,p2-center) < 0 ) {
			continue;
		}

		//iter->c1 = p1;
		//iter->c2 = p2;

		if ( d < distance ) {

			distance = d;
			rm_iter = iter;
		}

	}

	if ( rm_iter != NULL ) 
		_anchors.erase( rm_iter );
	
}

void MyGlWindow::updateAnchor( int x, int y, int cameraId ) 
{

	if ( _anchors.empty() )
		return;

	// lookup the visible faces (for intersection)
	_LUT.lookup( _camera->came.getTranslation(), 0.0, 0.0, 0, 0 );

	//
	int rx,ry;
	cameraId = fromSelectedPointToImagePoint( x, y, cameraId, rx, ry );

	if ( cameraId == -1 )
		return;

	// back project the ray
	_camera->setActiveSensor( cameraId );
	
	Vec3d center = _camera->getTranslation();
	
	Vec3d ray = _camera->rectifyAndUnproject( Vec2d(rx,ry) ) - center;

	// look for the closest anchor

	std::vector< anchor >::iterator iter = _anchors.begin();
	std::vector< anchor >::iterator up_iter = NULL;
	double distance = 1E10;
	Vec3d new_o;

	// for each anchor...
	for (iter = _anchors.begin(); iter != _anchors.end(); iter++) {

		// determine the closest distance between the ray and the anchor
		Vec3d p1, p2;
		closest_segment_approach( center, center + 100.0 * ray, iter->cc, (iter->a + iter->b + iter->c + iter->d) / 4.0, p1, p2 );
		
		double d = len(p2-p1);
		
		if ( dot(ray,p2-center) < 0 ) {
			continue;
		}

		//iter->c1 = p1;
		//iter->c2 = p2;

		if ( d < distance ) {

			distance = d;
			up_iter = iter;
			new_o = p1;
		}

	}

	if ( up_iter != NULL ) {
		Vec3d t = new_o - up_iter->o;
		Vec3d new_n = norm(cross(norm(up_iter->o - up_iter->cc), norm(ray)));

		up_iter->o = up_iter->o + t;
		up_iter->a = up_iter->a + t;
		up_iter->b = up_iter->b + t;
		up_iter->c = up_iter->c + t;
		up_iter->d = up_iter->d + t;
		up_iter->n = new_n;
	}

}


// return the camera ID or -1
//
int MyGlWindow::fromSelectedPointToImagePoint( int x, int y,  int cameraId, int &rx, int &ry )
{
	rx=-1, ry=-1;
	int w = _camera->_frame._rotated_img[cameraId]->width;
	int h = _camera->_frame._rotated_img[cameraId]->height;

	if ( _windowMode == MODE_CALIBRATION ) {

		// determine the true point on the image
		rx = y - (WINDOW_HEIGHT - CALIBRATION_DRAW_Y - h);
		ry = x - (CALIBRATION_DRAW_X + 1.2 * w);

	}

	if ( _windowMode == MODE_VIDEO )
	{
		int x1 = VIDEO_DRAW_X;
		int x2 = VIDEO_DRAW_X + 6 * w / _zoomFactor;
		int x3 = VIDEO_DRAW_X + 4 * w / _zoomFactor;
		int x4 = VIDEO_DRAW_X + 5 * w / _zoomFactor;

		if ( x < x1 || x > x2 )
			return -1;

		int y1 = WINDOW_HEIGHT-MENUBAR_HEIGHT-2*int(h/_zoomFactor);
		int y2 = y1 + h / _zoomFactor;
		int y3 = y2 + h / _zoomFactor;

		if ( WINDOW_HEIGHT - y > y1 && WINDOW_HEIGHT - y < y2 ) {
			int counter = int( (x - VIDEO_DRAW_X) / ( w / _zoomFactor ) );
			cameraId = (counter + 1) % 5;
			rx = _zoomFactor * (y2 -  (WINDOW_HEIGHT - y));
			ry = _zoomFactor * (x - VIDEO_DRAW_X - counter * w / _zoomFactor);
		} else if ( WINDOW_HEIGHT - y > y2 && WINDOW_HEIGHT - y < y3 && x > x3 && x < x4 ) {
			cameraId = 5;
			rx = _zoomFactor * (y3 -  (WINDOW_HEIGHT - y));
			ry = _zoomFactor * (x - x3);
		}

	}
	
	// test whether the point is in the image
	if ( rx < 0 || rx > h )
		return -1;
	
	if ( ry < 0 || ry > w )
		return -1;

	return cameraId;
}
