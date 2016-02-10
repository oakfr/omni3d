#ifndef _CAMERA_H__
#define _CAMERA_H__

#include "basic.h"
#include "geom/geom3D.h"
#include "impmath.hpp"
#include "util/util.h"
#include "viewer/viewer.h"
#include "ImageEdge.h" // Mike's edge detector
#include "ladybug/my_ladybug.h"
#include "fast/tracker.h"
#include "camera/skeleton.h"
#include "model/model.h"
#include "sift/key.h"

#pragma warning(disable: 4786)

#define CALIBRATION_FILE "C:\\omni3d\\config\\calib_distortion.dat"

#define USER_MASK_FILE "mask.dat"

#define EDGEMASK_WIDTH 10
#define MAINTENANCE_DISTANCE_POSE 20.0 // in inches, maximum distance walked by the camera between two frames

#define EDGE_SIGMA_INIT_REAL 1.0  // edge detector sigma (the higher, the more edges)
#define EDGE_SIGMA_INIT_SYNT 1.5
#define EDGE_SIGMA_MAIN_REAL 1.0
#define EDGE_SIGMA_MAIN_SYNT 1.5

#define EDGE_SIZE_INIT_REAL 20 // edge detector minimum edge length ( in pixels)
#define EDGE_SIZE_INIT_SYNT	10
#define EDGE_SIZE_MAIN_REAL	20
#define EDGE_SIZE_MAIN_SYNT	10

#define EDGE_THRESHOLD_INIT_REAL 8.0 // edge detector threshold (the lower, the more edges)
#define EDGE_THRESHOLD_INIT_SYNT 10.0
#define EDGE_THRESHOLD_MAIN_REAL 20.0
#define EDGE_THRESHOLD_MAIN_SYNT 10.0

#define CAMERA_EDGES_REF_FILE "edges_ref.txt"

typedef std::vector<ExtrinsicParameters> extrinsicParametersVector;

struct IntrinsicParameters {
	Mat3d cameraMatrix;
	double kc[5];		// distorsion
};

struct camera {
	double ax, ay;
	double x0, y0;
};

enum CameraType {LADYBUG, CANON_ELURA, SONY_DFW_V500};
enum DistortionModel {SPHERICAL_DISTORTION, POLYNOMIAL_DISTORTION};
 
class Intrinsic {
public:
	

	Intrinsic::Intrinsic ();
	Intrinsic::Intrinsic (int w, int h, double focal, double c1, double c2, double k1, double k2, double k3, double k4, double k5, double w_mm, double h_mm) ;
	Intrinsic::Intrinsic (int id, int w, int h, double focal, double c1, double c2, double v, double pixel_w_mm, double pixel_h_mm);

	void print(); 
	Mat3d toKMatrix();
	Vec2d normalize(Vec2d);
	Vec2d denormalize(Vec2d);
	Vec2d Intrinsic::normalize_focal (Vec2d x);
	Vec2d Intrinsic::denormalize_focal (Vec2d x);

	Vec2d rectify(Vec2d); // input and output in normalized coordinates
	Vec2d distort(Vec2d); // input and output in normalized coordinates
	Edge2D rectify ( Edge2D &edge );
	Edge2D distort ( Edge2D &edge );

	double getFocalX() {return focal_x;}
	bool setFocalX(double val) {bool res = (val != focal_x); focal_x = val; return res;}
	double getFocalY() {return focal_y;}
	bool setFocalY(double val) {bool res = (val != focal_y); focal_y = val; return res;}

	double getCenterX() {return cc[0];}
	bool setCenterX(double val) {bool res = (val != cc[0]); cc[0] = val; return res;}
	double getCenterY() {return cc[1];}
	bool setCenterY(double val) {bool res = (val != cc[1]); cc[1] = val; return res;}

	double getKC0() {return kc[0];}
	double getKC1() {return kc[1];}
	double getKC2() {return kc[2];}
	double getKC3() {return kc[3];}
	double getKC4() {return kc[4];}
	DistortionModel getDistortionModel() {return dist_model;}
	double getDistortionValue() {return value;}
	double getFocal() {return f;}
	int getWidth() {return width;}
	int getHeight() {return height;}

	double get_pixel_angle() { return pixel_angle; }
	double getKC(int i) {return kc[i];}
	bool setKC(int i,double val) {bool res = (val != kc[i]); kc[i] = val; return res;}

	double getPixelWidth() {return pixel_width_mm;}
	double getPixelHeight() {return pixel_height_mm;}

	double from_pixels_to_radians( int v );
	
	void Intrinsic::computeEdgeBoundingBox (Edge2D edge, double angle, double translation, double depth, 
		Vec2d &a, Vec2d &b, Vec2d &c, Vec2d &d, int min_mask_width, int max_mask_width);

	Edge2D Intrinsic::clip (Edge2D edge);
	bool Intrinsic::visible (Vec2d point);
	bool Intrinsic::visible_normalized (Vec2d point);
	Vec2d Intrinsic::flipHorizontal (Vec2d point);
	void Intrinsic::distorsionChecker();
		
protected:
	int _id; // camera ID, 0-6
	double f; // focal length, in normalized coordinates
	double focal_x,focal_y; // focal length, in pixels
	Vec2d cc; // projection center, in normalized coordinates (also distortion center)
	Vec2d 	CC; // projection center, in pixel coordinates (also distortion center)
	DistortionModel dist_model; // distortion model (spherical, polynomial)
	double kc[5]; // polynomial distorsion parameters
	double value; // spherical distortion parameter
	int width, height;
	double pixel_width_mm, pixel_height_mm; // pixel width & height in mm
	Edge2D TOP,BOTTOM,LEFT,RIGHT;
	double pixel_angle;

};

class Camera;

//** ====================================== **//


class Frame
{
public:

	CameraType _cameraType;
	std::vector<IplImage*> _img;
	std::vector<IplImage*> _grab_img;
	std::vector<IplImage*> _display_img;
	std::vector<IplImage*> _grayscale_img;
	std::vector<IplImage*> _rotated_img;
	std::vector<IplImage*> _copy_img;
	std::vector<IplImage*> _hough_img;
	std::vector<IplImage*> _temp_img;
	std::vector<IplImage*> _tmp_img;
	std::vector<IplImage*> _rectified_img;
	IplImage *temp_img,*_flipped_img; // we flip the top image (camera 5) for display only
	std::vector<edge2DVector> _edges; // 1 for each image on the frame (ex ladybug: 6 vectors)
	std::vector<edge2DVector> _save_edges; // 1 for each image on the frame (ex ladybug: 6 vectors)
	std::vector<edge2DVector> _edges_reprojected; // reprojected edges
	std::vector<edgePlaneVector> _edges_3d;
	std::vector<edgePlaneVector> _edgeplanes; // spherical representation of observed edges
	std::vector<edgePlaneVector> _edgeplanes_chained; // chained edges
	int n_edgeplanes_chained;
	edgePlaneVector _edgeplanes_frustum; // spherical representation of the camera frustum

	std::vector< std::vector<Vec2d> > _points; // tracked 2D feature points
	std::vector< SiftImage > _sift_images;

	fasttracker *ftracker[6];  // one tracker per sensor

	int _nImages;
	int _width, _height;
	double _display_zoom_factor;
	edgePlaneVector _selected_edges;
	
	std::vector< corner > _corners; // a list of corners (see skeleton.h)

	Frame ();
	void set (CameraType,int,int,int,double);
	void clear();
	void convert3DEdgesToEdgePlanes (int sensorId, edgePlaneVector edges, edgePlaneVector &edgeplanes, Vec3d center);
	void convertToGrayscale();
	void processFrame() { processFrame( -1 ); }
	void processFrame( long int run_id );

	void detectLines(); // using openCV algorithm - deprecated.
	void rescaleEdges();
	void scaleEdges();
	void chainEdges (int topK);
	void chainEdges2 (int topK);
	void filterEdges( edgePlaneVector &edgeplanes, double min_subtended_angle, double min_dihedral_angle, int topk );
	bool merge ( std::vector<edgePlaneVector> edgeplanes, EdgePlane edgeplane_ref, EdgePlane &edgeplane_res );

	EdgePlane getEdgePlane (int id);
	int nedgeplanes();
	int nedgeplanes_chained ();
	bool get_edgeplane_chained ( int id, EdgePlane &ep );

	void save(const char *dirname, int id);
	void save(const char *dirname, int id, int cameraId);
	void selectRandomEdgePlanes(int N);
	void selectRandomEdgePlanes(int N, int cameraId);

	int computeFeaturePoints (int img_id, int frameId, std::vector<Vec2d> &points);
	void computeCorners( int cameraid, std::vector< Vec3d > points, std::vector< EdgePlane > &edges, double max_dist, double min_angle, Model *spherets);
	void split_edges_into_cells ( std::vector< EdgePlane > &edges, std::vector< intVector > &cells, Model *spherets );
	void split_corners_into_cells( std::vector< corner > &corners,  std::vector < intVector > &cells, Model *spherets );
	void create_corners_2_lines( std::vector< EdgePlane > &edges, std::vector < intVector > &cells, std::vector< corner > &corners, double max_dist );
	void update_corners_connected_edges( std::vector< EdgePlane > &edges, std::vector < intVector > &cells, std::vector < intVector > &cells_corners, std::vector< corner > &corners, 
										   double max_dist, double min_angle, Model *spherets );
	void filter_corner_duplicates( std::vector< corner > &corners, double max_dist );

	void clearFeaturePoints();
	
	bool getFeaturePoint( int cameraid, int frameid, int featureid, Vec2d &point, int &age );
	bool getFeaturePointBox( int cameraid, int frameid, int featureid, std::vector< Vec2d > &boxes );

	int Frame::computeSiftFeatures () ;
	int Frame::computeSiftFeatures( int img_id, std::vector<Vec4d> &points);
	std::vector< std::vector<Vec4d> > _sift;
	bool _sift_computed;

	~Frame();
};

/*
class PoseSolution 
{
public:
	PoseSolution() {valid=false;};
	PoseSolution(edgeVector, edgePlaneVector,ExtrinsicParameters); // computes errors and valid
	PoseSolution(Edge*, Edge*, Edge* , EdgePlane, EdgePlane, EdgePlane, ExtrinsicParameters); // computes errors and valid
	void draw(Viewer &viewer);
	bool isValid() {return valid;}
	void setValid (bool v) {valid = v;}
	edgeVector getLines() {return lines;}
	edgePlaneVector getPlanes() {return planes;}
	ExtrinsicParameters getPose() {return came;}
	bool testValidity ();
	void print();
	bool equal (PoseSolution pose);
	double getErrorRotation() {return error_rot;}
	double getErrorTranslation() {return error_trans;}
	void setTranslation (Vec3d t) {came.setTranslation(t);}


protected:

	int n; // # of correspondences
	ExtrinsicParameters came;
	edgeVector lines;
	edgePlaneVector planes;
	double error, error_rot, error_trans;
	bool valid; // a pose is valid if all correspondences satisfy the frontality test
	bool test_overlap, test_frontality;
};
*/

class Camera {
public:
	
	// ----------- camera parameters -------------------
	//const double aspect = (double)width_image/(double)height_image;
	Eye_pose eyec; // camera pose in Video Stream window
	Eye_pose eyep; // eye pose in 3D model window
	Eye_pose _pose;
	camera kcam;
	IntrinsicParameters cami;
	ExtrinsicParameters came, _backupPose;

	Matd _extr; // the current unit extrinsic matrix
	std::vector<Matd> _extrinsic; // list of six unit extrinsic matrices
	std::vector<Matd> _extrinsic_inv; // list of six inverse unit extrinsic matrices (for projection)
	std::vector<Matd> _extrinsic_dft; // list of six unit extrinsic matrices before user adjustment

	SphericalCoord orientation_eye, orientation_cam;
	double zoomFactorEye;
	CameraType _cameraType;
	int _activeSensor; // active sensor in the ladybug, [0-5]
	int _saveActiveSensor; //
	int _width,_height;
	Frame _frame;
	int _anchorx,_anchory;
	int _glutModifier, _button;
	Matd _cameraMatrix,_cameraMatrix_inv;
	Ladybug *_ladybug; // the Ladybug

	CEdgeDetector* _m_EdgeDetector[6];
	ColorImage *_edge_image;
	ByteImage *_edge_mask_image;
	std::vector<CEdgeDetectorEdgeMaskVector> _edge_masks;
	std::vector<CEdgeDetectorEdgeMaskVector> _save_edge_masks;
	
	int _min_mask_width, _max_mask_width; // min and max mask width for edge detection

	Model *spherets; // sphere tessellation for point features

	void Camera::computeColorHits( int cameraid, int level_of_detail, double color_resolution, faceVector &faces, std::vector< color_hit > &hits );

	std::string	fname_edges;
	std::string fname_edges_filtered;
	std::string fname_edges_ref; 
	bool Camera::writeEdgesToFile( int frameid );
	void Camera::initEdgesFiles( int ladybug_id, int w, int h );
	bool Camera::readEdgesFromFile( int frameid );

	std::vector<IplImage*> _mask_imgs; // masks, for display
	void AddMask(int cameraId, Edge *line, Edge2D edge, bool debug_save);
	void clearMasks();
	void setMasks(int cameraId);
	void drawMasks (int cameraId, int x, int y, double scale, bool flip);

	double _max_rotation_speed; // in radian per frame
	double _max_translation_speed; // in mm per frame
	double _average_edge_depth; // in inches

	double _ladybug_extrinsic[6][6];

	void initializeEdgeDetectors();

	Vec2d _rectification_display_trans;
	double _rectification_display_scale;
	void writePose ( int frameId, FILE *pose_file );

	////////////////////////////////////////////////////////
	// user mask
	std::vector< UserMask > user_masks;
	//void clearUserMasks();
	//void addUserMask( UserMask );
	//bool writeUserMasks( std::string dirname );
	//bool readUserMasks( std::string dirname );
	//void deleteUserMask( int cameraId );
	void drawUserMask( Viewer &viewer, int cameraId, int x, int y, double zoom, bool flipVertical );

	void computeCorners( int frameid, int max_dist, double min_angle );

	// Ladybug extrinsic parameters refinement
	double _drx[6],_dry[6],_drz[6]; // rotation
	double _dtx[6],_dty[6],_dtz[6]; // translation
	//void printExtrinsicOffset();
	void printUnitExtrinsic();
	void printPose();
	ExtrinsicParameters getPose() {return came;}

	//Vec3d getCameraTarget( SphericalCoord orientation_eye);
	//Vec3d getCameraUp (SphericalCoord orientation_eye);
	Vec3d getCenterPoint(int sensorId);
	Vec3d getTranslation () {return came.getTranslation();}
	void setTranslation (Vec3d t) { came.setTranslation(t);}
	Quaternion getRotation() {return came.getRotation();}
	void setRotation(Quaternion r) { came.setRotation(r);}
	Mat3d getRotationMatrix() {return came.getRotationMatrix();}
	double getHeight() { return came.getHeight();}

	void perturbation ( double angle, double translation ) ;

	void setPose (ExtrinsicParameters c) {came = c;}
	void setHeight( double h ) { came.setHeight( h );}

	void resetPose() { _backupPose = getPose(); setTranslation (Vec3d(0,0,0)); setRotation(Quaternion());}
	void restorePose() { setPose( _backupPose );}
	//void resetEye(int mode, ModelBoundingBox modelbbox);
	void init (CameraType type, Ladybug *ladybug, int w, int h, double zoomFactor,Vec3d position, Mat3d rotation, 
		double SPHERE_TESSELLATION_RESOLUTION);
	void drawFrustum(const float color[3], bool fill);
	Vec3d fromWorldFrameToCameraFrame (Vec3d point);
	Edge fromWorldFrameToCameraFrame (Edge edge);	
	Vec3d fromCameraFrameToWorldFrame (Vec3d point);
	Edge fromCameraFrameToWorldFrame (Edge edge);
	EdgePlane Camera::fromWorldFrameToCameraFrame( Edge *edge ) ;

	bool Camera::getColor( int cameraid, Vec2d point, double *r, double *g, double *b );
	bool Camera::updateColor( EdgePlane &ep, bool debug );
	std::vector< Vec3d > _color_left, _color_right;

	Vec3d unproject (Vec2d object) ;	
	void unproject( Edge2D edge_in, Vec3d &a3d, Vec3d &b3d );
	void unproject (edge2DVector edges2d, edgePlaneVector &edges, Vec3d center);
	Edge unproject (Edge2D edge);
	Vec2d project (Vec3d point);
	bool project( int cameraId, Vec3d a, Vec3d b, Edge2D &edge2D);
	bool project (int cameraId, Vec3d point, Vec2d &point_2d);
	bool projectAndSubdivide( int cameraId, Vec3d a, Vec3d b, int edge_step, edge2DVector &edges);
	bool Camera::projectLocalPoint (int cameraId, Vec3d point, Vec2d &point_2d);
	//bool project (Vec3d point, Vec2d &p);
	Edge2D project (Edge edge3d);
	void computeSphericalFrustum();
	int reprojectEdgePlane (int cameraId, int id, EdgePlane edgeplane, int edge_step, edge2DVector &edges_2d);
	
	void drawEdgeplanes (Viewer &viewer, int cameraId, edgePlaneVector edgeplanes, int edge_step,
							int x, int y, double zoom, bool flipVert, bool flipHoriz,
							int h, int w, int linewidth);

	void drawEdgeplanesSelectMode (Viewer &viewer, int cameraId, edgePlaneVector edgeplanes, int edge_step,
							int x, int y, double zoom, bool flipVert, bool flipHoriz,
							int h, int w);


	void drawEdgeplane (Viewer &viewer, EdgePlane edgeplane, int edge_step,
							int x, int y, double zoom, const float color[3], bool flipVert, bool flipHoriz,
							int h, int w, int cameraId, int linewidth);

	void translate (Vec3d v);
	void rotate (Vec3d w, double theta);
	void manual_rotate (int x, int y, int anchorx, int anchory, Vec3d xaxis, Vec3d yaxis);
	Vec3d getZDirection(int cameraId);
	
	void clearEdges();
	void clearReprojectedEdges();
	void clearSelectedEdges();
	bool testCameraMotion( ExtrinsicParameters &pose );

	void printCameraPose (const char *filename);
	//void setPosition (Vec3d position, Vec3d target, Vec3d up, double pitch);
	
	void setActiveSensor (int id);
	void resetActiveSensor ();
	void readUnitExtrinsic (const char *filename);
	Matd getUnitExtrinsic (int sensor_id);
	Matd getUnitExtrinsicInv (int sensor_id);
	Matd computeUnitExtrinsic (int sensor_id);
	void Camera::computeUnitExtrinsic( int sensor_id, double ext1, double ext2, double ext3, double ext4, double ext5, double ext6, Matd &M, Matd &Minv);
	void computeUnitExtrinsic(char *filename);
	void computeUnitExtrinsic (int sensor_id, Matd &M, Matd &Minv);
	Vec3d getUnitCameraCenter (int sensor_id);

	Matd getExtrinsic (int sensor_id);
	//void resetPixels(unsigned char *pixels);
	//bool checkPixel (int x, int y);
	//int getPixelIndex (int x, int y);
	//void applyPixel (unsigned char *pixels, unsigned char *pixels_new, Vec2d x, Vec2d xd);
	//void applyPixelColor (unsigned char *pixels, unsigned char *pixels_new, double error, Vec2d xd);
	Vec2d rectifyPixel (Vec2d x);
	void undistortion (unsigned char *pixels, unsigned char *pixels_new);
	Edge2D rectifyEdge (Edge2D edge);

	void convert3DEdgesToEdgePlanes (int n, std::vector<edgePlaneVector> edges3D, std::vector<edgePlaneVector> &edgeplanes);

	void detectLines(bool first_time, bool save_debug, int cameraId, int m_nMinEdgePixels, double m_dfEdgeThreshold, double m_dfEdgePixelSigma); // using Mike's code - active as of Feb 2006
	void detectLines( bool first_time, bool save_debug, int m_nMinEdgePixels, double m_dfEdgeThreshold, double m_dfEdgePixelSigma); // using Mike's code - active as of Feb 2006

	//void rectifyImages(std::vector<IplImage*> images, std::vector<IplImage*> output_images);
	void rectifyImage (IplImage *image, IplImage *output_image);
	void displayRectificationError (Vec2d x, IplImage *output_image);
	void displayRectificationError (IplImage *image, IplImage *output_image);
	Vec3d rectifyAndUnproject (Vec2d point);
	Vec2d distort (int cameraId, Vec2d point);
	Vec2d denormalize (int cameraId, Vec2d point);
	Edge2D denormalize (int cameraId, Edge2D edge);

	void computeAngleHistogram ( Histogram &hist );

	void findSensors( Edge *line, intVector &ids );

	void convertLinesToPlanes ();

	void draw (Viewer &viewer, double radius, const float color[3]);
	Matd getCameraMatrix();
	void computeCameraMatrix();
	void computeInverseCameraMatrix();

	bool makeSyntheticImage (int sensorId, edgeVector lines, IplImage *image);
	bool makeSyntheticImage (int sensorId, faceVector &faces, int n, IplImage *image);
	void makeSyntheticImage (edgeVector lines, intVector &cameras);
	void makeSyntheticImage (faceVector &faces, int n, intVector &cameras);
	bool makeIdealEdgeplane( Edge *line, EdgePlane &edgeplane );

	void readIntrinsic(int nImages);
	void readIntrinsic(int nImages, int sensor);
	Intrinsic getIntrinsic () {return _intr[_activeSensor];}
	Intrinsic getIntrinsic (int id) {return _intr[id];}

	double getFocalX() {return getIntrinsic().getFocalX();}
	double getFocalY() {return getIntrinsic().getFocalY();}
	bool setFocalX(double val) {return _intr[_activeSensor].setFocalX(val);}
	bool setFocalY(double val) {return _intr[_activeSensor].setFocalY(val);}

	double getCenterX() {return getIntrinsic().getCenterX();}
	bool setCenterX(double val) {return _intr[_activeSensor].setCenterX(val);}
	double getCenterY() {return getIntrinsic().getCenterY();}
	bool setCenterY(double val) {return _intr[_activeSensor].setCenterY(val);}

	double getCenterXDiff() {return getCenterX() - _intr_dft[_activeSensor].getCenterX();}
	double getCenterYDiff() {return getCenterY() - _intr_dft[_activeSensor].getCenterY();}

	double getKC0() {return getIntrinsic().getKC0();}
	double getKC1() {return getIntrinsic().getKC1();}
	double getKC2() {return getIntrinsic().getKC2();}
	double getKC3() {return getIntrinsic().getKC3();}
	double getKC4() {return getIntrinsic().getKC4();}
	double getKC(int i) {return getIntrinsic().getKC(i);}
	bool setKC(int i,double val) {return _intr[_activeSensor].setKC(i,val);}

	DistortionModel getDistortionModel() {return getIntrinsic().getDistortionModel();}
	double getDistortionValue() {return getIntrinsic().getDistortionValue();}

	double getFocal() {return getIntrinsic().getFocal();}
	double getPixelWidth() {return getIntrinsic().getPixelWidth();}
	double getPixelHeight() {return getIntrinsic().getPixelHeight();}

	bool visible (int cameraId, Vec2d point);
bool visible (int cameraId, Edge2D edge);
Edge2D clip (int cameraId, Edge2D edge);
bool clipLine( int cameraId, Vec3d &a, Vec3d &b );

protected:
	std::vector<Intrinsic> _intr;
	std::vector<Intrinsic> _intr_dft; // default values

};



#endif

