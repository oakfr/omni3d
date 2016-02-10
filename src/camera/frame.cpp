#include "camera/camera.h"
#include "fast/fast.h"

#pragma warning(disable: 4786)

/* Frame constructor 
 */
Frame::Frame()
{
	_selected_edges.clear();
}

/* Convert an image to grayscale
 */
void Frame::convertToGrayscale()
{
	return;

	for (int p=0;p<_nImages;p++) {
		IplImage *ip = _display_img[p];
		char *data = ip->imageData;

		for (int i=0;i<ip->height;i++) {
			for (int j=0;j<ip->width;j++) {
				int pos = 3*(i*ip->width+j);
				double val = ((double)data[pos]+(double)data[pos+1]+(double)data[pos+2])/3.0;
				data[pos] = (unsigned char)val;
				data[pos+1] = (unsigned char)val;
				data[pos+2] = (unsigned char)val;
			}
		}
	}
}

/* Initializes a frame
 */
void Frame::set (CameraType type, int w, int h, int nImages, double zoomFactor)
{
	_cameraType = type;
	_width = w;
	_height = h;
	_nImages = nImages;
	_display_zoom_factor = zoomFactor;
	_sift_computed = false;

	for (int i=0;i<nImages;i++) {

		_grab_img.push_back(cvCreateImage(cvSize(int(_height), int(_width)),  IPL_DEPTH_8U, 4));
		_tmp_img.push_back(cvCreateImage(cvSize(int(_height), int(_width)),  IPL_DEPTH_8U, 3)); // used for edge detection
		_rotated_img.push_back(cvCreateImage(cvSize(int(_width), int(_height)),  IPL_DEPTH_8U, 3));
		_copy_img.push_back(cvCreateImage(cvSize(int(_width), int(_height)),  IPL_DEPTH_8U, 3));
		_copy_img.push_back(cvCreateImage(cvSize(int(_width), int(_height)),  IPL_DEPTH_8U, 3));
		_rectified_img.push_back(cvCreateImage(cvSize(int(_width), int(_height)),  IPL_DEPTH_8U, 3));
		_display_img.push_back(cvCreateImage(cvSize(int(_width/zoomFactor), int(_height/zoomFactor)),IPL_DEPTH_8U, 3));
		_grayscale_img.push_back(cvCreateImage(cvSize(int(_height), int(_width)),IPL_DEPTH_8U, 1));
	    temp_img = cvCreateImage(cvSize(int(_width/zoomFactor), int(_height/zoomFactor)),IPL_DEPTH_8U, 3);
		_flipped_img = cvCreateImage(cvSize(int(_width/zoomFactor), int(_height/zoomFactor)),IPL_DEPTH_8U, 3);
		_hough_img.push_back(cvCreateImage(cvGetSize(_rotated_img[0]), 8, 1));
		_temp_img.push_back(cvCreateImage(cvGetSize(_rotated_img[0]), 8, 1));
		
		edge2DVector vector;
		_edges.push_back(vector);
		edgePlaneVector vector3;
		_edgeplanes.push_back(vector3);
		edgePlaneVector vector4;
		_edgeplanes_chained.push_back(vector4);
		std::vector<Vec2d> vector5;
		_points.push_back(vector5);
		edge2DVector vector6;
		_edges_reprojected.push_back(vector6);
		edge2DVector vector7;
		_save_edges.push_back(vector7);

		std::vector< Vec4d > vector8;
		_sift.push_back( vector8 );

		SiftImage im = CreateImage(w, h, PERM_POOL);
		_sift_images.push_back( im );

	}
}

/* Scale edges for display
 */
void Frame::rescaleEdges()
{
	int img;

	for (img=0;img<_edges.size();img++) {
		edge2DVectorIter iter = _edges[img].begin();

		for(;iter!=_edges[img].end();iter++) {
			iter->_a = _display_zoom_factor*iter->_a;
			iter->_b = _display_zoom_factor*iter->_b;
		}
	}
}

/* Scale edges for display
 */
void Frame::scaleEdges()
{
	int img;

	for (img=0;img<_edges.size();img++) {
		edge2DVectorIter iter = _edges[img].begin();

		for(;iter!=_edges[img].end();iter++) {
			iter->_a = iter->_a/_display_zoom_factor;
			iter->_b = iter->_b/_display_zoom_factor;
		}
	}
}

/* Save frame image in a file
 */
void Frame::save(const char *dirname, int id)
{
	for (int img=0;img<_grab_img.size();img++) {
		char *name;
		name = (char*)malloc(256*sizeof(char));
		sprintf(name,"%s/cam%d-%d.jpg",dirname,img,id);
		printf("%s\n",name);
		cvSaveImage(name,_rotated_img[img]);
		delete name;
	}
}

/* Save frame image in a file
 */
void Frame::save(const char *dirname, int id, int cameraId)
{
		char *name;
		name = (char*)malloc(256*sizeof(char));
		sprintf(name,"%s/cam%d-%d.jpg",dirname,cameraId,id);
		printf("%s\n",name);
		cvSaveImage(name,_tmp_img[cameraId]);
		delete name;
}

/* Process a frame
 */
void Frame::processFrame( long int run_id )
{

	for (int i=0;i<_nImages;i++) {
		cvConvertImage(_grab_img[i],_tmp_img[i],0);
		cvCopyImage( _rotated_img[i], _copy_img[i] );
		rotateCW(_tmp_img[i],_rotated_img[i]);

		// resize and convert from 4-channel to 3-channel
		// (Ladybug outputs 4-channel images by default)
		cvResize(_rotated_img[i],_display_img[i]);
		cvConvertImage(_tmp_img[i],_grayscale_img[i],CV_RGB2GRAY);

		// save the image for debug
		if ( run_id > 0 ) {
			LOG(LEVEL_INFO, "saving image in %d_%d.bmp", run_id, i );
			char filename[256];
			sprintf( filename, "synthetic/%d_%d.bmp", run_id, i );
			cvSaveImage( filename, _tmp_img[i] );
		}
	}

	flipHorizontal(_display_img[5],temp_img); // flip image horizontally
	cvConvertImage(temp_img,_flipped_img,1); // flip image vertically
}

/* return the id-th edgeplane_chained
 */
bool Frame::get_edgeplane_chained( int id, EdgePlane &ep )
{
	if ( id >= n_edgeplanes_chained ) {
		return false;
	}

	int sensor_id = 0;

	while ( _edgeplanes_chained[sensor_id].size() < id + 1 ) {

		id -= _edgeplanes_chained[sensor_id].size();
		sensor_id++;
		if ( sensor_id > 5 )
			break;
	}

	if ( sensor_id > 5 )
		return false;

	if ( id < _edgeplanes_chained[sensor_id].size() ) {
		ep = _edgeplanes_chained[sensor_id][id];
		return true;
	}

	return false;
}

/* detect lines with Hough transform (OpenCV)
   this method is deprecated
 */
void Frame::detectLines()
{
	int i,j;

	for (i=0;i<_nImages;i++) {
		detectHoughLines(_grab_img[i], _hough_img[i], _temp_img[i], _edges[i]);
	}

	//resize lines
	if (_grab_img.size() > 0) {
		double factor = (double)_width/_grab_img[0]->width;
		for (i=0;i<_nImages;i++) {
			for (j=0;j<_edges[i].size();j++) {
				_edges[i][j]._a = factor * _edges[i][j]._a;
				_edges[i][j]._b = factor * _edges[i][j]._b;
			}
		}
	}
}	

/* Frame destructor
 */
Frame::~Frame()
{
}

/* return the i-th edge plane
 */
EdgePlane Frame::getEdgePlane (int id)
{
	assert (id < n_edgeplanes_chained);

	for (int i=0;i<_edgeplanes_chained.size();i++) {
		for (int j=0;j<_edgeplanes_chained[i].size();j++) {
			if (_edgeplanes_chained[i][j]._uid == id)
				return _edgeplanes_chained[i][j];
		}
	}

	printf("edgeplane not found for ID %d. Max ID = %d\n",id,n_edgeplanes_chained);
	assert (false);
	return EdgePlane();
}

/* convert a list of 3D edges into edge planes
 */
void Frame::convert3DEdgesToEdgePlanes (int sensorId, edgePlaneVector edges, edgePlaneVector &edgeplanes, Vec3d center)
{
	return;

	// read the 3D edges and convert them into edge planes
	int counter = 0;
		for (int edgeId=0;edgeId<edges.size();edgeId++) {
			EdgePlane edge = edges[edgeId];
			EdgePlane edgeplane = EdgePlane(edge._a - center,edge._b - center,center,sensorId,edgeId,counter);
			if (edgeplane.length() > 0) {
				edgeplanes.push_back(edgeplane);
				counter++;
			}
		}
}

/* merge edgeplanes using an edgeplane as reference
 * and write the result in edgeplane_res
 * if there are no edgeplanes to merge, return false
 */
bool Frame::merge ( std::vector<edgePlaneVector> edgeplanes, EdgePlane edgeplane_ref, EdgePlane &edgeplane_res)
{
	edgePlaneVector vector;

	for (int i=0;i<edgeplanes.size();i++) {
		for (int j=0;j<edgeplanes[i].size();j++) {
			if (edgeplanes[i][j].score_alignement( edgeplane_ref ) > 0.1/*.98*/)
				vector.push_back( edgeplanes[i][j] );
		}
	}

	if (vector.empty())
		return false;

	edgeplane_res = vector[0];

	return true;
	// merge each new edgeplane by looking for the longest outcome
	for (edgePlaneVector::iterator iter = vector.begin()+1; iter != vector.end(); iter++) {
		Vec3d a,b;

		if (len(edgeplane_res._a - iter->_a) < len(edgeplane_res._b - iter->_a))
			a = edgeplane_res._b;
		else
			a = edgeplane_res._a;

		if (len(iter->_a - edgeplane_res._a) < len(iter->_b - edgeplane_res._a))
			b = iter->_b;
		else
			b = iter->_a;

		edgeplane_res._a = a;
		edgeplane_res._b = b;
	}

	edgeplane_res._normal = norm(cross(edgeplane_res._a,edgeplane_res._b));

	return true;
}

/* chain the edge planes for each camera
 */
void Frame::chainEdges2 (int topK)
{
	double angle_normal_threshold = 0.04;
	double thickness_threshold = 0.04;
	double separation_criteria = 0.5;
	int img;
	double score=0,score_alignment=0;
	int i,j,k;
	
	if ( topK == -1 ) {
		
		for (img=0;img<_nImages;img++) {
			_edgeplanes_chained[img] = _edgeplanes[img];
		}
		
		
	} else {
		
		for (img=0;img<_nImages;img++) {
			
			edgePlaneVector new_edges;
			edgePlaneVector edges;
			for (k=0;k<_edgeplanes[img].size();k++)
				edges.push_back( _edgeplanes[img][k] );
			edgePlaneVectorIter iter;			
			
			// reset bucket IDs
			for (iter=edges.begin();iter!=edges.end();iter++)
				iter->_chaineId = -1;
			
			std::vector<edgePlaneVector> buckets;
			
			bool done = edges.empty();
			
			// step 1: fill-in the buckets
			
			while (1) {
				
				// find next edge without bucket
				for (iter=edges.begin();iter!=edges.end();iter++) {
					if (iter->_chaineId < 0)
						break;
				}
				
				if (iter == edges.end())
					break;
				
				// try to find a host bucket
				int bucket_id = -1;
				for (i=0;i<buckets.size();i++) {
					
					bucket_id = i;
					for (j=0;j<buckets[i].size();j++) {
						if ((iter->scoreAlignment(&buckets[i][j]) > angle_normal_threshold) || \
							(iter->thickness(&buckets[i][j]) > thickness_threshold)) {
							bucket_id = -1;
							break;
						}
					}
					
					if (bucket_id >= 0) // found a host bucket
						break;
				}
				
				if (bucket_id >= 0) {
					// update host bucket
					iter->_chaineId = bucket_id;
					buckets[bucket_id].push_back(*iter);
				} else {
					// create a new bucket
					iter->_chaineId = buckets.size();
					edgePlaneVector vector;
					vector.push_back(*iter);
					buckets.push_back(vector);
				}
			}
			
			// step 2: process each bucket
			
			for ( i=0;i<buckets.size();i++) {
				edges = buckets[i];
				
				// filter out duplicates
				bool done = (edges.size() < 1);
				
				while (!done) {
					
					done = true;
					edgePlaneVectorIter iter1,iter2;
					
					for (iter1=edges.begin();iter1!=edges.end();iter1++) {
						for (iter2=edges.begin();iter2!=edges.end();iter2++) {
							if (iter1->_uid==iter2->_uid) continue;
							if (iter1->overlap(iter2)) {
								if (iter1->length()>iter2->length())
									edges.erase(iter2);
								else
									edges.erase(iter1);
								done = false;
								break;
							}
						}
						if (!done)
							break;
					}
				}
				
				done = (edges.size()<2);
				
				// merge edges
				while (!done) {
					
					done = true;
					edgePlaneVectorIter iter1,iter2;
					
					for (iter1=edges.begin();iter1!=edges.end();iter1++) {
						for (iter2=edges.begin();iter2!=edges.end();iter2++) {
							if (iter1->_uid==iter2->_uid) continue;
							if ((iter1->separation(iter2) < separation_criteria) && (!iter1->overlap(iter2))) {
								iter1->merge(iter2);
								done = false;
								edges.erase(iter2);
								break;
							}
						}
						if (!done)
							break;
					}
				}
				
				for ( j=0;j<edges.size();j++) {
					new_edges.push_back(edges[j]);
				}
			}
			
			
			// keep the topK edges for each camera
			//	printf("keep the topK edges for each camera\n");
			
			int max = new_edges.size();
			if (topK > 1) {
				std::sort(new_edges.begin(),new_edges.end());
				max = MIN(new_edges.size(),topK);
			}
			
			_edgeplanes_chained[img].clear();
			for (j=0;j<max;j++) {
				new_edges[j]._uid = j;
				_edgeplanes_chained[img].push_back(new_edges[j]);
				
				// update edges chaineId
				for (k=0;k<new_edges[j]._edgeIds.size();k++) {
					int id = new_edges[j]._edgeIds[k];
					if ( id < _edges[img].size() )
						_edges[img][id]._edgeplaneId = j;
				}
			}
		}
	}
	
	// sort edge planes by decreasing length
	for (i=0;i<_edgeplanes_chained.size();i++) {
		std::sort(_edgeplanes_chained[i].begin(),_edgeplanes_chained[i].end());
	}
	
	// reset edge planes IDs
	n_edgeplanes_chained = 0;
	
	for (i=0;i<_edgeplanes_chained.size();i++) {
		for (j=0;j<_edgeplanes_chained[i].size();j++) {
			_edgeplanes_chained[i][j]._uid = n_edgeplanes_chained;
			n_edgeplanes_chained++;
		}
	}
}

/* chain the edge planes for each camera
 */
void Frame::chainEdges (int topK)
{
	double angle_normal_threshold = toRadians( 2.0 );
	double angle_normal_threshold2 = toRadians( 4.0 );
	double separation_criteria = toRadians( 5.0 );
	double max_color_distance = 0.25;

	int img;
	double score=0,score_alignment=0;
	int i,j,k;
	
	if ( topK == -1 ) {
		
		for (img=0;img<_nImages;img++) {
			_edgeplanes_chained[img] = _edgeplanes[img];
		}
	} else {
		
		for (img=0;img<_nImages;img++) {

			_edgeplanes_chained[img].clear();

			edgePlaneVector edges = _edgeplanes[img];

			std::vector< intVector > buckets;

			// first split edges into buckets
			for (i=0;i<_edgeplanes[img].size();i++) {

				EdgePlane edge = _edgeplanes[img][i];

				if ( buckets.empty() ) {
					intVector vs;
					vs.push_back( i );
					buckets.push_back( vs );
					continue;
				}

				// check each bucket
				bool matched = false;

				for (j=0;j<buckets.size();j++) {

					bool accept = true;

					for (k=0;k<buckets[j].size();k++) {

						if ( _edgeplanes[img][buckets[j][k]].angle( edge ) > angle_normal_threshold || _edgeplanes[img][buckets[j][k]].colorDistance( edge ) > max_color_distance) {
							accept = false;
							break;
						}

						if ( _edgeplanes[img][buckets[j][k]].overlap( edge ) > EPS || edge.overlap( _edgeplanes[img][buckets[j][k]] ) > EPS ) {
							accept = false;
							break;
						}
					}

					if ( accept ) {
						buckets[j]. push_back( i );
						matched = true;
						break;
					}
				}

				if ( matched ) 
					continue;

				// if did not find any bucket, create a new one
				intVector vs;
				vs.push_back( i );
				buckets.push_back( vs );
			}

			// second, merge edges within each bucket
			for (i=0;i<buckets.size();i++) {

				bool done = buckets[i].size() < 2;

				while ( !done ) {

					done = true;

					for (j=0;j<buckets[i].size();j++) {

						if ( buckets[i][j] == -1 )
							continue;

						for (k=j+1;k<buckets[i].size();k++) {

							if ( buckets[i][k] == -1 )
								continue;

							double d1 = Acos(dot( edges[buckets[i][j]]._a, edges[buckets[i][k]]._a ));
							double d2 = Acos(dot( edges[buckets[i][j]]._a, edges[buckets[i][k]]._b ));
							double d3 = Acos(dot( edges[buckets[i][j]]._b, edges[buckets[i][k]]._a ));
							double d4 = Acos(dot( edges[buckets[i][j]]._b, edges[buckets[i][k]]._b ));

							if ( d1 < separation_criteria ) {

								edges[buckets[i][j]]._a = edges[buckets[i][k]]._b;
								buckets[i][k] = -1;
								done = false;
								break;
							}

							if ( d2 < separation_criteria ) {

								edges[buckets[i][j]]._a = edges[buckets[i][k]]._a;
								buckets[i][k] = -1;
								done = false;
								break;
							}


							if ( d3 < separation_criteria ) {

								edges[buckets[i][j]]._b = edges[buckets[i][k]]._b;
								buckets[i][k] = -1;
								done = false;
								break;
							}

							if ( d4 < separation_criteria ) {

								edges[buckets[i][j]]._b = edges[buckets[i][k]]._a;
								buckets[i][k] = -1;
								done = false;
								break;
							}
						}

						if ( !done ) {
							edges[buckets[i][j]]._normal = norm(cross(edges[buckets[i][j]]._a, edges[buckets[i][j]]._b));
							break;
						}
					}
				}
			}

			// create a new edge for each bucket
			for (i=0;i<buckets.size();i++) {

				for (j=0;j<buckets[i].size();j++) {

					if ( buckets[i][j] == -1 )
						continue;

					edges[buckets[i][j]]._chaineId = i;

					if ( edges[buckets[i][j]].length() > 1E-7 ) 
						_edgeplanes_chained[img].push_back( edges[buckets[i][j]] );

				}
			}

			std::sort(_edgeplanes_chained[img].begin(),_edgeplanes_chained[img].end());

			if ( topK > 0 && _edgeplanes_chained[img].size() > topK )
				_edgeplanes_chained[img].resize( topK );

		}
	}
		
	// reset edge planes IDs
	n_edgeplanes_chained = 0;
	
	for (i=0;i<_edgeplanes_chained.size();i++) {
		for (j=0;j<_edgeplanes_chained[i].size();j++) {
			_edgeplanes_chained[i][j]._uid = n_edgeplanes_chained;
			n_edgeplanes_chained++;
		}
	}
}

/* filter the vector of edgeplanes given as input based on three criterias:
 * * minimum subtended angle
 * * minimum dihedral angle between any two edgeplanes
 * * topk based on length
 */
void Frame::filterEdges( edgePlaneVector &edgeplanes, double min_subtended_angle, double min_dihedral_angle, int topk ) 
{

	edgePlaneVector temp;

	// filter on minimum subtended angle
	for (edgePlaneVector::iterator iter = edgeplanes.begin(); iter != edgeplanes.end(); iter++) {
		if ( iter->length() > min_subtended_angle )
			temp.push_back( *iter );
	}

	// clear the output
	edgeplanes.clear();

	// sort on length
	std::sort( temp.begin(), temp.end() );

	// filter on minimum dihedral angle between any two edgeplanes
	for (int i=0;i<temp.size();i++) {
		bool similar = false;

		for (int j=0;j<edgeplanes.size();j++) {
			if ( temp[i].angle( edgeplanes[j] ) < min_dihedral_angle ) { // if dihedral angle is small
				if ( temp[i].overlap( &edgeplanes[j] ) ) { // and the two edge planes overlap
					similar = true;
					break;
				}
			}
		}

		if ( !similar ) {
			edgeplanes.push_back( temp[i] );   // if not similar, add to the list
		} 
	}

	// sort on length
	std::sort( edgeplanes.begin(), edgeplanes.end() );

	// keep the topk elements
	if ( edgeplanes.size() > topk ) 
		edgeplanes.resize( topk );


}

/* Return the number of edge planes
 */
int Frame::nedgeplanes ()
{
	int n=0;

	for (int i=0;i<_edgeplanes.size(); i++)
		n += _edgeplanes[i].size();

	return n;
}

/* Return the number of chained edge planes
 */
int Frame::nedgeplanes_chained ()
{
	int n = 0;

	for (int i=0;i<_edgeplanes_chained.size();i++)
		n += _edgeplanes_chained[i].size();

	return n;
}

/* Clear elements in the frame
 */
void Frame::clear()
{
	int i;

	for (i=0;i<_edges.size();i++)
		_edges[i].clear();

	for (i=0;i<_edges_reprojected.size();i++)
		_edges_reprojected[i].clear();


	for (i=0;i<_edges_3d.size();i++)
		_edges_3d[i].clear();
	

	_edges.clear();

	_edges_3d.clear();
	for (i=0;i<_edgeplanes.size();i++)
		_edgeplanes[i].clear();
	for (i=0;i<_edgeplanes_chained.size();i++)
		_edgeplanes_chained[i].clear();
	for (i=0;i<_points.size();i++)
		_points[i].clear();

}

/* Select N random edge planes in the frame (used for RANSAC)
 */
void Frame::selectRandomEdgePlanes(int N, int cameraId)
{
	int n = 0, k;

	if (_edgeplanes_chained.size() <= cameraId)
		return;

	n = _edgeplanes_chained[cameraId].size();

	intVector indexes;
	selectNRandomInt(N,n,indexes);
	
	for (k=0;k<N;k++) {
		_selected_edges.push_back(_edgeplanes_chained[cameraId][indexes[k]]);
	}
}

/* Select N random edge planes in the frame (used for RANSAC)
 */
void Frame::selectRandomEdgePlanes(int N)
{
	int n = 0, i, j, k;

	if (_edgeplanes_chained.empty())
		return;

	for (i=0;i<_edgeplanes_chained.size();i++)
		n += _edgeplanes_chained[i].size();

	intVector indexes;
	selectNRandomInt(N,n,indexes);
	
	for (k=0;k<N;k++) {

		std::vector<edgePlaneVector>::iterator lg_iter = _edgeplanes_chained.begin();
		edgePlaneVectorIter iter = lg_iter->begin();

		for (j=0;j<indexes[k];j++) {
			iter++;
			if (iter == lg_iter->end()) {
				lg_iter++;
				iter = lg_iter->begin();
			}
		}

		assert(iter != NULL);

		_selected_edges.push_back(*iter);
	}
}

/* Search for the bounding box of a feature in the tracker
 */
bool Frame::getFeaturePointBox( int cameraid, int frameid, int featureid, std::vector< Vec2d > &boxes )
{
	if ( ftracker[cameraid] == NULL )
		return false;

	intVector xbox;

	bool res = ftracker[cameraid]->get_feature_box_neighbor( frameid, featureid, xbox );

	if ( !res )
		return false;

	for (int i=0;i<xbox.size() / 4;i++) {
		Vec2d a = Vec2d( xbox[4*i+0], xbox[4*i+1] );
		Vec2d b = Vec2d( xbox[4*i+2], xbox[4*i+3] );
		boxes.push_back( a );
		boxes.push_back( b );
	}

	return true;
}

/* Search for a feature in the tracker
 */
bool Frame::getFeaturePoint( int cameraid, int frameid, int featureid, Vec2d &point, int &age )
{
	xyt feature;
	if ( ftracker[cameraid] == NULL )
		return false;

	bool res = ftracker[cameraid]->get_feature( frameid, featureid, feature, age );
	if ( !res )
		return false;

	point[0] = feature.x;
	point[1] = feature.y;

	return true;
}

/* compute the SIFT features on the image
 */
int Frame::computeSiftFeatures () 
{
	if ( _sift_computed )
		return 1;

	for (int img_id = 0; img_id < 6; img_id++ ) {
		int q = computeSiftFeatures( img_id, _sift[img_id] );
		LOG(LEVEL_INFO, "SIFT [%d] : %d", img_id, q );
	}

	_sift_computed = true;

	return 1;
}

/* compute the SIFT features on the image
 */
int Frame::computeSiftFeatures( int img_id, std::vector<Vec4d> &points)
{
	int w = _grayscale_img[img_id]->width;
	int h = _grayscale_img[img_id]->height;
	int ws = _grayscale_img[img_id]->widthStep;

	for (int r = 0; r < h; r++) {
	  for (int c = 0; c < w; c++) {
		  double val = (double)(unsigned int)(unsigned char)(_grayscale_img[img_id]->imageData[c+r*ws]);//((double) fgetc(fp)) / 255.0;
			_sift_images[img_id]->pixels[r][c] = val / 255.0;
	  }
	}

	points.clear();

	double *features = new double[4*MAX_SIFT_FEATURES]; //each feature has 4 parameters: x,y,scale,orientation 
	int n = compute_sift_features( _sift_images[img_id], features );

	for (int c = 0; c < MIN(n,4000); c++ ) {

		Vec4d point;
		point[0] = features[4*c+0];
		point[1] = w-1-features[4*c+1];
		point[2] = features[4*c+2];
		point[3] = features[4*c+3];

		points.push_back( point );
	}

	delete [] features;

	return n;
}

/* compute the FAST features on the image and update the tracker if the <tracker> flag is set to true
 */
int Frame::computeFeaturePoints (int img_id, int frameId, std::vector<Vec2d> &points)
{
	IplImage *img = _grayscale_img[img_id];
	points.clear();

	int w = img->width;
	int h = img->height;

	int n;
	xy *xy_points = fast_corner_detect_9 ((unsigned char*)img->imageData,w,h,20,&n);

	xy *xy_points_nonmax = fast_nonmax ((unsigned char*)img->imageData,w,h,xy_points,n,8,&n);

	for (int i=0;i<n;i++) {
		xy xy_point = xy_points_nonmax[i];
		assert (xy_point.x < w);
		assert (xy_point.y < h);

		points.push_back(Vec2d(xy_point.x,xy_point.y));

		_points[img_id].push_back(Vec2d(xy_point.x,xy_point.y));
	}
	
	delete xy_points;
	delete xy_points_nonmax;

	return points.size();
}

/* Clear the feature points
 */
void Frame::clearFeaturePoints()
{
	for (int i=0;i<_points.size();i++)
		_points[i].clear();

	_corners.clear();
}

/* compute the corners (see skeleton.h) i.e feature points connected to image edges (max_dist is in radians)
 * min_angle: min angle (in radians) between two edges to validate a corner
 * max_dist: max distance between a point and a line to make a connection (in radians)
 */
void Frame::computeCorners( int cameraid, std::vector< Vec3d > points, std::vector< EdgePlane > &edges, double max_dist, double min_angle, Model *spherets )
{
	int i;

	int n = points.size();

	std::vector< corner > corners;
	std::vector < intVector > cells;
	
	// split edges into cells
	split_edges_into_cells( edges, cells, spherets );	

	// create a new corner each time two edges intersect
	create_corners_2_lines( edges, cells, corners, max_dist );          

	// create a new corner for each feature point
	for (i=0;i<n;i++) {	

		Vec3d ret = norm(points[i]);

		corner c( ret[0], ret[1], ret[2] ); // initialize a corner

		corners.push_back( c );
	}

	// split corners into cells
	std::vector < intVector > cells_corners;
	split_corners_into_cells( corners, cells_corners, spherets );	

	// update the corners with their connected edges
	update_corners_connected_edges( edges, cells, cells_corners, corners, max_dist, min_angle, spherets );

	// eliminate corner duplicates
	filter_corner_duplicates( corners, max_dist );

	// assign the camera id to each corner
	for (i=0;i<corners.size();i++)
		corners[i].set_cameraid( cameraid );
	
	// keep all valid corners
	for (i=0;i<corners.size();i++) {
		if ( corners[i].valid() )
			_corners.push_back( corners[i] );
	}
}

/* Split edges into cells
 * Each cell contains a list of pairs < edge ID, start_end > where start_end = 0 if start point and 1 if end point
 */
void Frame::split_edges_into_cells ( std::vector< EdgePlane > &edges, std::vector< intVector > &cells, Model *spherets )
{
	cells.clear();
	int i,j;

	int n = spherets->_faces.size();

	for (i=0;i<n;i++) { // init the cells empty
		intVector vect;
		cells.push_back( vect );
	}

	for (i=0;i<edges.size();i++) {
		
		Vec3d a = edges[i]._a;
		Vec3d b = edges[i]._b;

		intVector cell_ids;
		spherets->closest_cells( a, cell_ids ); // update the neighbor cells with start and end points

		for (j=0;j<cell_ids.size();j++) {

			cells[cell_ids[j]].push_back( i );
			cells[cell_ids[j]].push_back( 0 );
		}

		spherets->closest_cells( b, cell_ids );

		for (j=0;j<cell_ids.size();j++) {

			cells[cell_ids[j]].push_back( i );
			cells[cell_ids[j]].push_back( 1 );
		}
	}
}

/* Split the corners into cells
 * each corner is assigned a cell id
 */
void Frame::split_corners_into_cells( std::vector< corner > &corners,  std::vector < intVector > &cells, Model *spherets )
{
	int n = spherets->_faces.size();
	int i,j;

	for (i=0;i<n;i++) { // init the cells empty
		intVector vect;
		cells.push_back( vect );
	}

	for (i=0;i<corners.size();i++) {	

		Vec3d p = corners[i].getpoint();
		intVector cell_ids;
		spherets->closest_cells( p, cell_ids );
		for (j=0;j<cell_ids.size();j++)	
			cells[cell_ids[j]].push_back( i );
	}
}

/* Create new corners where lines intersect (max_dist is in radians)
 */
void Frame::create_corners_2_lines( std::vector< EdgePlane > &edges, std::vector < intVector > &cells, std::vector< corner > &corners, double max_dist )
{
	int i,j,k;

	std::map < std::pair<int, int>, int > book; // book keeping of pairs of end points
	std::map < std::pair<int, int>, int >::iterator iter; // iterator in the book

	for (i=0;i<cells.size();i++) { // within each cell

		for (j=0;j<cells[i].size() / 2;i++) { // for each end point

			int marker = cells[i][2*j+1]; // 0 if start point, 1 if end point
			int edgeid = cells[i][2*j];

			double x,y,z; // end point (x,y,z)
			if ( marker == 0 ) {
				x = edges[edgeid]._a[0];
				y = edges[edgeid]._a[1];
				z = edges[edgeid]._a[2];
			} else {
				x = edges[edgeid]._b[0];
				y = edges[edgeid]._b[1];
				z = edges[edgeid]._b[2];
			}
			
			int closest_id = -1;
			int closest_marker = -1;
			double min_dist = 1.0;

			for (k=0;k<cells[i].size()/2;k++) { // for each other end point in the neighborhood

				if ( j == k ) // don't compare to itself
					continue;

				int marker2 = cells[i][2*k+1];
				int edgeid2 = cells[i][2*k];

				iter = book.find( std::pair<int,int>( edgeid, edgeid2 )); // don't compare if already processed
				if ( iter != book.end() )
					continue;

				iter = book.find( std::pair<int,int>( edgeid2, edgeid ));
				if ( iter != book.end() )
					continue;

				double x2,y2,z2; // end point (x2,y2)
				if ( marker2 == 0 ) {
					x2 = edges[edgeid2]._a[0];
					y2 = edges[edgeid2]._a[1];
					z2 = edges[edgeid2]._a[2];
				} else {
					x2 = edges[edgeid2]._b[0];
					y2 = edges[edgeid2]._b[1];
					z2 = edges[edgeid2]._b[2];
				}

				double dist = Acos( dot( Vec3d(x,y,z), Vec3d(x2,y2,z2) ) );

				if ( dist < min_dist ) { // keep the closest end point
					min_dist = dist;
					closest_id = edgeid2;
					closest_marker = marker2;
				}
			}

			if ( closest_id != -1 && min_dist < max_dist ) {

				double x2,y2,z2;
				if ( closest_marker == 0 ) {
					x2 = edges[closest_id]._a[0];
					y2 = edges[closest_id]._a[1];
					z2 = edges[closest_id]._a[2];
				} else {
					x2 = edges[closest_id]._b[0];
					y2 = edges[closest_id]._b[1];
					z2 = edges[closest_id]._b[2];
				}
				
				std::pair<int,int> p; // update the book
				p.first = edgeid;
				p.second = closest_id;
				std::pair< std::pair<int, int>, int> element;
				element.first = p;
				element.second = 0;
				book.insert( element );

				// compute the two intersections (poles) of the two edgeplanes and test each of them
				Vec3d pole_1, pole_2;
				if ( ! edges[edgeid].poles( edges[closest_id], pole_1, pole_2 ) )
					continue;

				double l1 = Acos( dot( pole_1, Vec3d(x2,y2,z2) ) );
				double l2 = Acos( dot( pole_1, Vec3d(x, y, z ) ) );
				if ( l1 < max_dist && l2 < max_dist ) {
					corner c( pole_1[0], pole_1[1], pole_1[2] ); // create a new corner
					corners.push_back( c );
					continue;
				}

				l1 = Acos( dot( pole_2, Vec3d(x2,y2,z2) ) );
				l2 = Acos( dot( pole_2, Vec3d(x, y, z ) ) );
				if ( l1 < max_dist && l2 < max_dist ) {
					corner c( pole_2[0], pole_2[1], pole_2[2] ); // create a new corner
					corners.push_back( c );
					continue;
				}
			}
		}
	}
}

/* Update corners with connected edges (double max_dist is in radians)
 * for each edge end point, look for close corners and attach them together if distance < max_dist
 * min_angle: min angle (in radians) between two edges to validate a corner
 * max_dist: max distance between a point and a line to make a connection (in radians)
 */
void Frame::update_corners_connected_edges( std::vector< EdgePlane > &edges, std::vector < intVector > &cells, std::vector < intVector > &cells_corners,
										   std::vector< corner > &corners, double max_dist, double min_angle, Model *spherets )
{
	int i,j,k;

	std::map < std::pair<int, int>, int > book; // book keeping of pairs of end points
	std::map < std::pair<int, int>, int >::iterator iter; // iterator in the book

	for (i=0;i<cells.size();i++) {

		for (j=0;j<cells[i].size()/2;j++) {

			int marker = cells[i][2*j+1]; // 0 if start point, 1 if end point
			int edgeid = cells[i][2*j];

			double x,y,z; // edge end point (x,y,z)
			double x2,y2,z2; // other edge end point (x2,y2,z2)
			if ( marker == 0 ) {
				x = edges[edgeid]._a[0];
				y = edges[edgeid]._a[1];
				z = edges[edgeid]._a[2];
				x2 = edges[edgeid]._b[0];
				y2 = edges[edgeid]._b[1];
				z2 = edges[edgeid]._b[2];
			} else {
				x = edges[edgeid]._b[0];
				y = edges[edgeid]._b[1];
				z = edges[edgeid]._b[2];
				x2 = edges[edgeid]._a[0];
				y2 = edges[edgeid]._a[1];
				z2 = edges[edgeid]._a[2];
			}

			double min_dist = 1E10;
			int closest_corner = -1;

			// for each corner in the neighborhood
			for (k=0;k<cells_corners[i].size();k++) {

				int corner_id = cells_corners[i][k];

				iter = book.find( std::pair<int,int>( edgeid, corner_id )); // don't compare if already processed
				if ( iter != book.end() )
					continue;

				corner c = corners[corner_id];

				Vec3d p = c.getpoint();

				double d = Acos( dot( p, Vec3d(x,y,z) ) );
				
				if ( d < min_dist ) {

					min_dist = d;
					closest_corner = corner_id;
				}
			}

			// if we found a point within the range
			// we add the edge to that corner
			if ( closest_corner != -1 && min_dist < max_dist ) {
				corners[closest_corner].insert_edge( x, y, z, x2, y2, z2, min_angle );

				std::pair<int,int> p; // update the book
				p.first = edgeid;
				p.second = closest_corner;
				std::pair< std::pair<int, int>, int> element;
				element.first = p;
				element.second = 0;
				book.insert( element );

			}
		}
	}
}

/* eliminate corner duplicates
 * when two corners are close to each other, keep the one with heighest weight
 * max_dist: max distance between two corners (in radians)
 */
void Frame::filter_corner_duplicates( std::vector< corner > &corners, double max_dist )
{
	for (int i=0;i<corners.size();i++) {

		for (int j=0;j<corners.size();j++) {

			if ( i == j )
				continue;

			if ( corners[i].getweight() < corners[j].getweight() ) {

				if ( len(corners[i].getpoint() - corners[j].getpoint()) < max_dist ) 

					corners[i].clear_edges();
			}
		}
	}
}


