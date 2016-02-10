#include "tracker.h"

#pragma warning(disable: 4786)

fasttracker::fasttracker ()
{
	_frameid = -1;
	dbg_id = -1;
}

// init the tracker
//
// _xsize and _ysize are the image width and height in pixels (e.g 512 x 384)
// _window_size is the size of a bag in pixels (e.g 20 pixels)
// _max_features is the max number of features tracked by the tracker (e.g 300)
// _min_dist is the minimum distance between any two features in pixels (e.g 5 pixels)
//
fasttracker::fasttracker ( int _xsize, int _ysize, int _window_size, int _max_features, int _min_dist, int _time_to_live )
{
	xsize = _xsize;
	ysize = _ysize;
	window_size = _window_size;
	max_features = _max_features;
	sq_min_dist = _min_dist * _min_dist;

	size_x = int( xsize / window_size ) + 1;
	size_y = int( ysize / window_size ) + 1;

	pixel[0] = 0 + 3 * xsize;		
	pixel[1] = 1 + 3 * xsize;		
	pixel[2] = 2 + 2 * xsize;		
	pixel[3] = 3 + 1 * xsize;		
	pixel[4] = 3 + 0 * xsize;		
	pixel[5] = 3 + -1 * xsize;		
	pixel[6] = 2 + -2 * xsize;		
	pixel[7] = 1 + -3 * xsize;		
	pixel[8] = 0 + -3 * xsize;		
	pixel[9] = -1 + -3 * xsize;		
	pixel[10] = -2 + -2 * xsize;		
	pixel[11] = -3 + -1 * xsize;		
	pixel[12] = -3 + 0 * xsize;		
	pixel[13] = -3 + 1 * xsize;		
	pixel[14] = -2 + 2 * xsize;		
	pixel[15] = -1 + 3 * xsize;	

	_frameid = -1;
	dbg_id = -1;
	time_to_live = _time_to_live;

	printf("init tracker: window size: %d max features: %d  min distance: %d  time to live: %d\n", window_size, max_features, _min_dist, _time_to_live );

}

// convert a 2d point into a feature
// see David Nister, Oleg Naroditsky and James Bergen, Visual Odometry for Ground Vehicle Applications, 
// inaugural issue of Journal of Field Robotics, Volume 23, Number 1, January 2006.

xyt fasttracker::convert_feature ( const xy &ret, byte *im, int frameid )
{
	const byte* cache_0;
	const byte* offset = im + ret.y * xsize + ret.x;

	float a = 0, b = 0;
	float c = 0.0;
	int i;
	xyt feature;

	for (i=0;i<16;i++) {
		cache_0 = offset + pixel[i];
		float val = (float)*cache_0 / 255;
		feature.n[i] = val;
		a += val;
		b += val * val;
	}

	c = 1.0 / sqrt( 16 * b - a*a );

	feature.x = ret.x;
	feature.y = ret.y;
	feature.a = a;
	feature.b = b;
	feature.c = c;
	feature.frameid = frameid;
	feature.id = -1;
	feature.positive = ret.positive;

	return feature;
}

// compute normalized correlation between two features
//
double fasttracker::normalized_correlation ( const xyt &ret1, const xyt &ret2 )
{
	// compute scalar product
	float d = 0.0;

	for (int i=0;i<16;i++) {
		d += ret1.n[i] * ret2.n[i];
	}

	// normalized correlation
	return ( 16 * d - ret1.a * ret2.a ) * ret1.c * ret2.c;

}

// convert a set of features into bags
//
void fasttracker::bag_features ( std::vector<xyt> &features, std::vector< std::vector< int > > &bags, bool test_bag_size, bool test_min_dist )
{
	// first clear the bags
	bags.clear();

	// init the bags with empty vectors
	int i,j,k;
	for (i=0;i<size_y;i++) {
		for (j=0;j<size_x;j++) {
			std::vector< int > v;
			bags.push_back( v );
		}
	}

	int max_n = max_features < features.size() ? max_features : features.size();
	int n = test_bag_size ? max_n : features.size();

	// for each feature, find corresponding bag and insert (if min_dist check passed)
	for (i=0;i<n;i++) {
		xyt feature = features[i];
		//assert( feature.id != -1 ); // simple check
		bool up = false, left = false;
		std::vector< int > bag_ids; // bag_ids contains the IDs of the bag which will contain the feature + the neighboring bags
		bag_ids.push_back( which_bag( feature.x, feature.y, window_size, size_x, size_y, up, left ) ); // which_bag specifies whether the feature is on the left/up side of the bucket

		bool check_min_dist = true;

		if ( test_min_dist ) { // check the min distance over the bags
			which_bags_neighbor( bag_ids[0], bag_ids, size_x, size_y, up, left );
						
			for (j=0;j<bag_ids.size();j++) {
				int bag_id = bag_ids[j];
				for (k=0;k<bags[bag_id].size();k++) {
					if ( xyt_sq_dist( feature, features[bags[bag_id][k]] ) < sq_min_dist ) {
						check_min_dist = false;
						break;
					}
				}
				if ( !check_min_dist )
					break;
			}
		}

		if ( !check_min_dist ) // if the feature did not pass the test, discard it!
			continue;

		// otherwise, add it to the bag
		bags[bag_ids[0]].push_back( i );

	}
}

// search a feature in the table <m>
// return true if found, false otherwise
// 
bool fasttracker::get_feature( int frameid, int featureid, xyt &ret, int &age )
{
	for (int i=0;i<m.size();i++) {
		if ( m[i].empty() )
			continue;

		if ( m[i][0].id == featureid ) {
			for (int j=0;j<m[i].size();j++) {
				if ( m[i][j].frameid == frameid ) {
					ret = m[i][j];
					age = m[i].size();
					return true;
				}
			}

			return false;
		}
	}

	return false;
}

// insert the features in the tracker, presumably for a new frame
// if this is the first frame, the tracker simply populates the table <m> and "bag" the features for the next frame
// otherwise, the tracker updates the table <m> by looking at the best match for each feature, "bag" the features and
// clean up deprecated tracked features (those that haven't been observed for some time)
//
bool fasttracker::insert_features ( const xy *set, int n, byte *im, int frameid )
{

	if ( frameid <= _frameid )
		return false;

	_frameid = frameid;

	int i;
	bool first_frame = m.empty(); // if <m> is empty, this is the first frame

	// if the tracker is empty, just populate vector <v_t> table <m>
	if ( first_frame ) {
		
		init_tracker( set, n, im, frameid );

	} else { // otherwise, updates the table <m> by looking at the best match for each feature
		
		// populate vector v_tt
		v_tt.clear();

		for (i=0;i<n;i++) {
			xyt feature = convert_feature( set[i], im, frameid ); // here, keep feature id = -1 since it will be used as a match flag
			//print_feature( feature );
			v_tt.push_back( feature );
		}

		// split features into bags
		bag_features( v_tt, bag_tt, false, false );

		//printf( "size: %d %d\n", v_t.size(), v_tt.size() );

		// match features
		printf("matching features...\n");
		match_features( bag_t, v_t, bag_tt, v_tt );

		// update table <m> (this is where you check min_dist)
		printf("updating table...\n");
		update_tracker_table( v_t, v_tt, frameid );

		// update v_t and bag_t from <m>
		printf("updating new features...\n");
		update_tracker_bag (v_t, bag_t );

		printf("done.\n");
	}

	return true;
}

// updates the table <m> with the new correspondences
// in <old_features>, ids correspond to the position of the feature in table <m>
// in <new_features>, ids correspond to the feature match in <old_features> (-1 if no match was found)
// the idea is to select those features in <new_features> which have a valid ID, make a vector out of it and run bag_features(...,true,true) on it
// then parse the bag and for each feature, update <m>
// once this is done, go through <m> once again to remove deprecated features and replace them with features from <new_features> having an invalid ID
// in the process, features coming from <new_features> see their ID being updated (i.e == the ID of the match in <old_features>)
//
void fasttracker::update_tracker_table ( std::vector< xyt > &old_features, std::vector< xyt > &new_features, int frameid )
{
	// remove deprecated features
	int i,j;
	for (i=0;i<m.size();i++) {
		if ( !m[i].empty() && m[i].back().frameid + time_to_live < frameid )
			m[i].clear();
	}

	// select features that are valid
	std::vector< xyt > matched_features, unmatched_features;
	std::remove_copy_if(new_features.begin(), new_features.end(),std::back_inserter(matched_features),featureIsNotValid);
	std::remove_copy_if(new_features.begin(), new_features.end(),std::back_inserter(unmatched_features),featureIsValid);

	//printf(" %d unmatched features\n", unmatched_features.size() );

	// split the valid features into bag, paying attention to min_dist and max_size
	std::vector< std::vector< int > > bag;
	bag_features( matched_features, bag, true, true );

	// for each new feature, update the table <m> and *then* the vector <old_features>
	// warning: the IDs in <old_features> are useful since they point to the right location in <m>
	for (i=0;i<bag.size();i++) {
		for (j=0;j<bag[i].size();j++) {
			//printf(" selecting %d out of %d\n", bag[i][j], matched_features.size() );
			
			xyt feature = matched_features[bag[i][j]]; // the new feature
			int m_id = old_features[feature.id].id; // the position to target in table <m>
			//printf(" feature.id: %d / %d  m_id: %d / %d\n", feature.id, old_features.size(), m_id, m.size());
			feature.id = m_id;
			m[m_id].push_back( feature ); // update table <m>
		}
	}

	// re-generate the old features from table <m>
	// and clear deprecated features at the same time
	old_features.clear();

	//printf("*** unmatched features ***\n");

	//for (i=0;i<unmatched_features.size();i++)
	//	print_feature( unmatched_features[i] );

	//std::vector< xyt >::iterator iter = unmatched_features.begin(); // pointer to new features used to insert new features in the table
	for (i=0;i<m.size();i++) {
		if ( m[i].empty() && !unmatched_features.empty() ) { // if the feature is deprecated, try to find a new one
			xyt feature = unmatched_features.back();
			feature.id = i;
			unmatched_features.pop_back();

			//printf("adding following feature:\n");
			//print_feature( feature );

			// check for min_dist
			bool accept = true;

			for (j=0;j<m.size();j++) {
				if ( !m[j].empty() && xyt_sq_dist( m[j].back(), feature ) < sq_min_dist ) {
					accept = false;
					break;
				}
			}

			if ( !accept )
				continue;

			// insert feature if accepted
			old_features.push_back( feature );
			m[i].push_back( feature );

		} else { // update the <old_features> with <m>
			old_features.push_back( m[i].back() );
		}
	}

	// here: in case the table <m> is still missing some elements, we add some new ones
	bool stop = false;

	while ( (m.size() < max_features) && !stop ) {
		stop = false;
		//while ( iter != unmatched_features.end() )
		//		iter++;
		if ( !unmatched_features.empty() ) {
			xyt feature = unmatched_features.back();
			feature.id = m.size();
			unmatched_features.pop_back();

			//printf("adding following feature:\n");
			//print_feature( feature );
			// check for min_dist
			bool accept = true;

			for (j=0;j<m.size();j++) {
				if ( !m[j].empty() && xyt_sq_dist( m[j].back(), feature ) < sq_min_dist ) {
					accept = false;
					break;
				}
			}

			if ( !accept )
				continue;

			old_features.push_back( feature );
			std::vector< xyt > v;
			v.push_back( feature );
			m.push_back( v );
		} else {
			stop = true;
		}
	}
}

// print out a feature
//
void fasttracker::print_feature ( xyt &ret )
{
	printf (" [%d] x: %d y: %d    frameid: %d  a = %f  b = %f  c = %f\n", ret.id, ret.x, ret.y, ret.frameid, ret.a, ret.b, ret.c );
}

// print out the whole tracker
void fasttracker::print_tracker ()
{
	printf(" **** fast tracker ****** \n");

	for (int i=0;i<m.size();i++) {
		if ( m[i].empty() ) 
			printf(" [%d] \n", i );
		else
			print_feature( m[i].back() );
	}
}

// update the vector <features> and the corresponding bag from <m>
// update <features> from <m>.back(), warning some spots might be empty
// generate bags
void fasttracker::update_tracker_bag( std::vector< xyt > &features, std::vector< std::vector< int > > &bag )
{
	features.clear();

	// update <features> vector from table <m>
	for (int i=0;i<m.size();i++) {
		if ( m[i].empty() )
			continue;

		features.push_back( m[i].back() );
	}

	// split features into bags
	bag_features( features, bag, false, false );
}


// init the tracker by populating vector <v_t>, table <m> and bag_t
void fasttracker::init_tracker( const xy *set, int n, byte *im, int frameid )
{
	// clear the vector
	v_t.clear();
	int i;

	// convert features and put them into <v_t> and <m>
	for (i=0;i<n;i++) {
		xyt feature = convert_feature( set[i], im, frameid );

		v_t.push_back( feature );
	}

	// split features into bags
	bag_features( v_t, bag_t, true, true );

	// round-robin distribute features into <m>
	round_robin_features( v_t, bag_t, m );
}

// round-robin distribute features from vector <v> into vector of vector <m>
// bags contains the distribution of features into cells
//
void fasttracker::round_robin_features( std::vector<xyt> &v, std::vector< std::vector< int > > bags, std::vector< std::vector< xyt > > &m )
{
	std::vector< int > counters; // list of counters for each bag

	int i;
	for (i=0;i<bags.size();i++) { // init the counters
		counters.push_back ( bags[i].size()-1 );
	}

	bool done = false;

	while ( !done ) {

		done = true;

		for (i=0;i<bags.size();i++) {
			if ( counters[i] >= 0 ) {

				v[bags[i][counters[i]]].id = m.size();

				std::vector< xyt > vv;
				vv.push_back( v[bags[i][counters[i]]] );
				m.push_back( vv );
				counters[i]--;
				done = false;
			}
		}
	}
}


// match_features takes as input two bags and the corresponding features vectors and find the best correspondences between the bags
// this function assumes that the bags have been properly filled
// at the end of the function, each feature in bag 2 has an ID of either -1 (no match was found) or > 0 pointing to 
// the corresponding match in v1
//
void fasttracker::match_features( std::vector< std::vector< int > > &bag1, std::vector< xyt > &v1, \
								 std::vector< std::vector< int > > &bag2, std::vector< xyt > &v2)
{
	std::map<std::pair<int,int>, double> book; //keep a book of the feature correlations
	std::map<std::pair<int,int>, double>::iterator iter;
	
	// for each feature in <bag1>, try to find the best match in bag2
	int i,j,k,p;
	bool up, left;
	xyt feature1, feature2;
	
	for (i=0;i<bag1.size();i++) { // i is the ID of the bag
		for (j=0;j<bag1[i].size();j++) {

			int ret1_id = bag1[i][j]; // ret1_id is the ID of the j-th feature in bag 1 (== position in v1)
			assert(  ret1_id < v1.size() );
			feature1 = v1[ret1_id]; // this is the feature in bag1
			
			std::vector< int > bag_ids; // these are the IDs of the neighbor bags on the image
			bag_ids.push_back( which_bag( feature1.x, feature1.y, window_size, size_x, size_y, up, left ) );
			//assert( ret1_id == bag_ids[0] );
			which_bags_neighbor( i, bag_ids, size_x, size_y, up, left );
			
			if ( ret1_id == dbg_id ) {
				printf("searching in neighbor bags: ");
				for (k=0;k<bag_ids.size();k++) {
					printf(" %d", bag_ids[k]);
				}
				printf("\n");
			}

			// look for best match in all the neighbor bags
			double best_correlation = 0.0;
			int best_match = -1;
			
			for (k=0;k<bag_ids.size();k++) {
				int n_id = bag_ids[k]; // n_id is the ID of the neighbor bag
				
				for (p=0;p<bag2[n_id].size();p++) {

					int ret2_id = bag2[n_id][p]; // ret2_id is the ID of the feature in the neighbor bag
					//printf("element %d in [%d]\n", ret2_id, v2.size() );

					assert(  ret2_id < v2.size() );
					feature2 = v2[ret2_id]; // this is the feature in bag2

					//print_feature( feature2 );

					if ( feature2.positive != feature1.positive ) // if features of different type, skip
						continue;

					if ( feature2.id != -1 ) // if this feature has already been matched, skip it
						continue;

					iter = book.find( std::pair<int,int>( ret1_id, ret2_id )); // otherwise, look for the correlation in the book
					double corr = 0.0;
					if ( iter == book.end() ) {	// if not found, compute and insert in the book
						corr = normalized_correlation( feature1, feature2 ); // compute the correlation
						std::pair<int,int> q;
						q.first = ret1_id;
						q.second = ret2_id;
						std::pair<std::pair<int,int>,double> element;
						element.first = q;
						element.second = corr;
						book.insert( element ); // store the value in the book
					} else { // read in the book
						corr = iter->second;
					}
					
					if ( corr > best_correlation ) { // keep if it is a potential candidate
						best_correlation = corr;
						best_match = ret2_id;
					}
				}
			}
			
			if ( ret1_id == dbg_id && best_match == -1 ) 
				printf("found no match\n");

			// if not match has been found, continue
			if ( best_match == -1 )
				continue;
			
			feature2 = v2[best_match]; // this is the best match in bag2
			
			// run the mutual consistency check
			// in other words, check that the best match considers the current feature as its best possible match as well
			bool consistency_check = true; //  a priori, the test is passed
			
			bag_ids.clear(); // find the neighborhood of feature2 -- should be roughly the same as for feature1 but might be a bit different
			bag_ids.push_back( which_bag( feature2.x, feature2.y, window_size, size_x, size_y, up, left ) );
			which_bags_neighbor( bag_ids[0], bag_ids, size_x, size_y, up, left );
			

			if ( ret1_id == dbg_id ) {
				printf("cons. check: searching in neighbor bags: ");
				for (k=0;k<bag_ids.size();k++) {
					printf(" %d", bag_ids[k]);
				}
				printf("\n");
			}

			// look for best match in all the neighbor bags
			for (k=0;k<bag_ids.size();k++) {
				int n_id = bag_ids[k]; // n_id is the ID of the neighbor bag
				
				for (p=0;p<bag1[n_id].size();p++) {
					int r_id = bag1[n_id][p]; // r_id is the ID of the feature in the neighbor bag
					
					feature1 = v1[r_id]; // this is the feature in bag1

					if ( feature1.positive != feature2.positive ) // if not same type, continue
						continue;

					double corr = 0.0;
					iter = book.find( std::pair<int,int>( r_id, best_match )); // look for the correlation in the book
					if ( iter == book.end() ) { // if not found, compute and insert in the book
						corr = normalized_correlation( v1[r_id], v2[best_match] ); // compute the correlation
						std::pair<int,int> q;
						q.first = r_id;
						q.second = best_match;
						std::pair<std::pair<int,int>,double> element;
						element.first = q;
						element.second = corr;
						book.insert( element ); // store the value in the book
					} else { // read in the book
						corr = iter->second;
					}
					
					if ( corr > best_correlation ) { // test whether the correlation is higher
						consistency_check = false;
						break;
					}
				}
				
				if ( consistency_check ) // skip next bag if we already found a better correlation (in which case the match is already inconsistent)
					break;
			}
			
			if ( ret1_id == dbg_id )
				printf("cons. check: %d\n", consistency_check );

			if ( !consistency_check ) // if the match is not consistent, continue
				continue;
			
			// otherwise, we found a nice match, good news!
			v2[best_match].id = ret1_id;
		}
	}
}

// return the bounding box of the bag containing the feature
// into a vector of int {x1,y1,x2,y2}
//
bool fasttracker::get_feature_box ( int frameid, int featureid, std::vector< int > &xbox )
{
	xyt ret;
	int age;

	xbox.clear();

	if ( !get_feature( frameid, featureid, ret, age ) )
		return false;

	bool up, left;
	int bag_id = which_bag( ret.x, ret.y, window_size, size_x, size_y, up, left );

	int bag_i = bag_id / size_x;
	int bag_j = bag_id - bag_i * size_x;

	int bag_x = window_size * bag_j;
	int bag_y = window_size * bag_i;

	xbox.clear();
	xbox.push_back( bag_x );
	xbox.push_back( bag_y );
	xbox.push_back( bag_x + window_size );
	xbox.push_back( bag_y + window_size );

	return true;
}

// return the bounding box of the neighbor bags containing the feature
// into a vector of int {x1,y1,x2,y2, .... }
//
bool fasttracker::get_feature_box_neighbor ( int frameid, int featureid, std::vector< int > &xbox )
{
	xyt ret;
	int age;

	xbox.clear();

	if ( !get_feature( frameid, featureid, ret, age ) )
		return false;

	bool up, left;
	int bag_id = which_bag( ret.x, ret.y, window_size, size_x, size_y, up, left );

	std::vector< int > bag_ids;
	bag_ids.push_back( bag_id );
	which_bags_neighbor( bag_id, bag_ids, size_x, size_y, up, left );

	xbox.clear();

	for (int i=0;i<bag_ids.size();i++) {

		int bag_i = bag_ids[i] / size_x;
		int bag_j = bag_ids[i] - bag_i * size_x;

		int bag_x = window_size * bag_j;
		int bag_y = window_size * bag_i;

		xbox.push_back( bag_x );
		xbox.push_back( bag_y );
		xbox.push_back( bag_x + window_size );
		xbox.push_back( bag_y + window_size );
	}

	return true;
}
	
// clear the tracker
//
void fasttracker::clear()
{
	m.clear();
	v_t.clear();
	v_tt.clear();

	bag_t.clear();
	bag_tt.clear();
}

