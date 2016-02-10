#pragma warning(disable: 4786)

#ifndef _TRACKER_H__
#define _TRACKER_H__

#include <stdlib.h>																			
#include <iostream>
#include "fast.h"
#include <algorithm>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <math.h>
#include <assert.h>
#include <functional>

#include "util/util.h"

// a class for fast feature tracking

typedef struct { int frameid, id, x, y; float a,b; float c; int positive; float n[16];} xyt; 

static bool featureIsNotValid( const xyt &ret) { return ( ret.id == -1 ); }
static bool featureIsValid( const xyt &ret) { return ( ret.id != -1 ); }

class fasttracker {
public:
	
	fasttracker();
	fasttracker ( int _xsize, int _ysize, int _window_size, int _max_features, int _min_dist, int _time_to_live );
	
	void bags_features ( xy*, int n );
	xyt convert_feature ( const xy &ret, byte *im, int frameid );
	double normalized_correlation ( const xyt &ret1, const xyt &ret2 );
	double xyt_sq_dist ( const xyt &ret1, const xyt &ret2 ) { return ( ret1.x - ret2.x ) * ( ret1.x - ret2.x ) + ( ret1.y - ret2.y ) * ( ret1.y - ret2.y );  }
	void bag_features ( std::vector<xyt> &features, std::vector< std::vector< int > > &bags, bool test_bag_size, bool test_min_dist );
	bool insert_features ( const xy *set, int n, byte *im, int frameid );

	void update_tracker_table ( std::vector< xyt > &old_features, std::vector< xyt > &new_features, int frameid );
	void update_tracker_bag( std::vector< xyt > &features, std::vector< std::vector< int > > &bag );
	void init_tracker( const xy *set, int n, byte *im, int frameid );
	void match_features( std::vector< std::vector< int > > &bag1, std::vector< xyt > &v1,
							 std::vector< std::vector< int > > &bag2, std::vector< xyt > &v2);

	void clear();
	void print_feature ( xyt &ret );
	void print_tracker ();

	bool get_feature( int frameid, int featureid, xyt &ret, int &age );
	bool get_feature_box ( int frameid, int featureid, std::vector< int > &xbox );
	bool get_feature_box_neighbor ( int frameid, int featureid, std::vector< int > &xbox );

	void round_robin_features( std::vector<xyt> &v, std::vector< std::vector< int > > bags, std::vector< std::vector< xyt > > &m );

	int dbg_id; // ID of the feature being tracked for debug

private:
	int window_size; // window size in pixels (size of a bag)
	int max_features; // max number of features tracked by the tracker
	int sq_min_dist; // square min distance between two features (in pixels^2)
	int time_to_live; // max number of frames after which a feature is declared deprecated

	int _frameid; // current frameId
	
	int xsize, ysize; // width and height of the image
	
	int pixel[16]; // pointers to the pixel neighborhood
	int size_x; // size of the grid (x)
	int size_y; // size of the grid (y)
	std::vector< xyt > v_t; // features at frame t
	std::vector< xyt > v_tt; // features at frame t+1
	std::vector< std::vector< xyt> > m; // the list of all features over time
	std::vector< std::vector< int > > bag_t; // grid of features at frame t
	std::vector< std::vector< int > > bag_tt; // grid of features at frame t+1
};


// first frame: populate v_t, bag_t and m

// for any new frame: populate v_tt, bag_tt, find matches between bag_t and bag_tt, update v_t and m

// note that each <xyt> has an ID that specifies the correspondence between v_t and <m>
// namely: for each k, v_t[k] = m[v_t[k].id].back()

#endif
