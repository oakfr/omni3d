#ifndef _RECOVERY_H__
#define _RECOVERY_H__

#include "basic.h"

// structures for edge-line matching in the initialization step
// linetriplet is a triplet of line IDs (l_k,l_l, l_m) along with three min and max dihedral angles (b_min*,b_max*) 
// edgetriplet is a triplet of edgeplane IDs along with a triplet of dihedral angles <a*> 

typedef struct { int l_k, l_l, l_m; double b_min0, b_max0, b_min1, b_max1, b_min2, b_max2; } linetriplet;
typedef struct { int e_i, e_j, e_k; double a0, a1, a2; } edgetriplet;

class corner_line { 
	
public:
	int camera_id, corner_id, vertex_id, edge_1_id, edge_2_id, line_1_id, line_2_id; double score;

	corner_line() {camera_id=corner_id=vertex_id=edge_1_id=edge_2_id=line_1_id=line_2_id=-1; score=0.0;}
	~corner_line() {};

	bool operator<(const corner_line &cl) const { return score > cl.score; }
};

// a line pair and an edge pair form a valid match iff one of the following is true:
//
// (1) b_min0 < a0 < b_max0  AND  b_min1 < a1 < b_max1   AND   b_min2 < a2 < b_max2
//
// (2) b_min0 < a0 < b_max0  AND  b_min1 < a2 < b_max1   AND   b_min2 < a1 < b_max2
//
// (3) b_min0 < a1 < b_max0  AND  b_min1 < a0 < b_max1   AND   b_min2 < a2 < b_max2
//
// (4) b_min0 < a1 < b_max0  AND  b_min1 < a2 < b_max1   AND   b_min2 < a0 < b_max2
//
// (5) b_min0 < a2 < b_max0  AND  b_min1 < a0 < b_max1   AND   b_min2 < a1 < b_max2
//
// (6) b_min0 < a2 < b_max0  AND  b_min1 < a1 < b_max1   AND   b_min2 < a0 < b_max2


static bool valid_match( linetriplet &lp, edgetriplet &ep, int &index ) {
	if ( lp.b_min0 < ep.a0 && ep.a0 < lp.b_max0 && lp.b_min1 < ep.a1 && ep.a1 < lp.b_max1 && lp.b_min2 < ep.a2 && ep.a2 < lp.b_max2 ) {
		index = 0;
		return true;
	}
	if ( lp.b_min0 < ep.a0 && ep.a0 < lp.b_max0 && lp.b_min1 < ep.a2 && ep.a2 < lp.b_max1 && lp.b_min2 < ep.a1 && ep.a1 < lp.b_max2 ) {
		index = 1;
		return true;
	}
	if ( lp.b_min0 < ep.a1 && ep.a1 < lp.b_max0 && lp.b_min1 < ep.a0 && ep.a0 < lp.b_max1 && lp.b_min2 < ep.a2 && ep.a2 < lp.b_max2 ) {
		index = 2;
		return true;
	}
	if ( lp.b_min0 < ep.a1 && ep.a1 < lp.b_max0 && lp.b_min1 < ep.a2 && ep.a2 < lp.b_max1 && lp.b_min2 < ep.a0 && ep.a0 < lp.b_max2 ) {
		index = 3;
		return true;
	}
	if ( lp.b_min0 < ep.a2 && ep.a2 < lp.b_max0 && lp.b_min1 < ep.a0 && ep.a0 < lp.b_max1 && lp.b_min2 < ep.a1 && ep.a1 < lp.b_max2 ) {
		index = 4;
		return true;
	}
	if ( lp.b_min0 < ep.a2 && ep.a2 < lp.b_max0 && lp.b_min1 < ep.a1 && ep.a1 < lp.b_max1 && lp.b_min2 < ep.a0 && ep.a0 < lp.b_max2 ) {
		index = 5;
		return true;
	}
	//if ( ep.a < lp.b_min || lp.b_max < ep.a )
	//	return false;
	index = -1;
	return false;
}

#endif

