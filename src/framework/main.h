#ifndef _FRAMEWORK_H__
#define _FRAMEWORK_H__

#pragma warning(disable : 4786)

#include <FL/Fl_Gl_Window.h>
#include <Fl/Fl.h>
#include <FL/Fl_Menu_Bar.H>
#include <Fl/Fl_Double_Window.h>
#include <Fl/Fl_Window.h>
#include <FL/Fl_Button.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Clock.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Input_Choice.H>
#include <FL/Fl_Light_Button.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Roller.H>
#include <FL/Fl_Progress.H>
#include <FL/Fl_Int_Input.H>
#include <FL/Fl_Counter.H>
#include <FL/Fl_Scrollbar.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Tabs.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_BMP_Image.H>

//#include <windows.h>
#include <GL/gl.h>
#include "signature/signatureviewer.h"
#include "signature/signature.h"
#include "timer/my_timer.h"
#include "ladybug/my_ladybug.h"
#include "camera/camera.h"
#include "util/util.h"
#include "database/database.h"
#include "state.h"
#include "recovery.h"

#include <gsl/gsl_math.h>
#include <gsl/gsl_multimin.h>

#define WINDOW_WIDTH 1380
#define WINDOW_HEIGHT 1000
#define MENUBAR_HEIGHT 30
#define CONTROLBAR_WIDTH 200
#define CONTROLBAR_HEIGHT 200
#define VISIBILITY_WIDTH 800
#define VISIBILITY_HEIGHT 800

#define REPROJECTION_ERROR_MAX 0.01
#define MAX_TRIALS 1000
#define LOCK_RECOVERY_MAX_TRANSLATION 25.0
#define LOCK_RECOVERY_MAX_ROTATION COS_10
#define LOCK_RECOVERY_RANSAC_MAX_ROTATION COS_15
#define LOCK_RECOVERY_RANSAC_THRESHOLD 0.5

#define SCORE_THRESHOLD COS_20
#define MIN_CORRESPONDENCES_FOR_REFINEMENT 20
#define MAX_CORRESPONDENCES_FOR_REFINEMENT 100

#define CLUSTER_ANGLE_THRESHOLD 0.0087266462599716478846184538424431 // angle threshold for clustering in radians, 0.5 degrees
#define CLUSTER_MAX_ANGLE_THRESHOLD 0.087266462599716478846184538424431 // maximum angle allowed with reference edgeplane during clustering, 10 degrees
#define POSE_MAX_ANGLE_THRESHOLD 0.0872664625997 // maximum angle allowed with reference edgeplane during pose computation, 5 degrees
#define CLUSTER_MIN_SUBTENDED_ANGLE 0.1745329251994 // min subtended angle to keep correspondence

#define CALIBRATION_DRAW_X 50
#define CALIBRATION_DRAW_Y 300
#define VIDEO_DRAW_X 10

#define NOISE_SYNTHETIC 0
#define NOISE_SYNTHETIC_ANGLE_AVERAGE 0.0
#define NOISE_SYNTHETIC_ANGLE_STDEV 0.1

#define POSE_HISTORY_FILE "poses.dat"
#define XSENS_LOG_FILE "xsens.dat"

#define POSE_MAX_ELEMENTS 15 // maximum number of correspondences taken into account during pose computation
#define POSE_MIN_ELEMENTS 8  // minimum number of correspondences needed to run a pose computation
#define POSE_THRESHOLD_SCORE 0.5 // score threshold for acceptance. the score function 0 < 1
#define POSE_MAX_TRIALS 30

enum WindowMode {MODE_3D_MAP, MODE_2D_MAP, MODE_VIDEO, MODE_SPHERE, MODE_CALIBRATION, MODE_ROTATIONS};

class MyControlBarWindow;

// make a Gl window that draws something:
class MyGlWindow : public Fl_Gl_Window {
public:
	MyGlWindow(int x, int y, int w, int h) :
	  Fl_Gl_Window(x,y,w,h,"My GL Window")
	  {
		  _windowMode = MODE_VIDEO;
		  frameId = 0;
	  }
	WindowMode _windowMode, _historyMode;
	LUT _LUT; // visibility lookup table
	int _mousex, _mousey;
	Ladybug *_ladybug; // the Ladybug
	Camera *_camera; // the camera model
	int _zoomFactor; 
	int frameId; // frameId

	void MyGlWindow::init_pose();

	bool _pauseVideo, _display_edge_boxes; // options for video
	bool _wireframe, _depth_test,_display_3d_edges; // options for 3D model
	Viewer _sphere_viewer,_video_viewer, _calibration_viewer, _rotations_viewer; // viewers
	int _calibrated_camera; // tracked camera (for the GUI only)
	bool _rectified_done;
	IplImage *_calib_img,*_calib_rectified_img; // images for display
	MyControlBarWindow *_control_bar; //pointer to the control bar
	Database *_database; // database of images when Ladybug is out
	//std::vector<PoseSolution> _poses, _save_good_poses, _save_bad_poses; // list of solutions
	std::vector<ExtrinsicParameters> _solutions;

	bool _locked; // true when the init phase was successful
	extrinsicParametersVector _recorded_poses;
	bool _initGraphics;
	UserMask user_mask; // let the user add a mask

	intVector correspondence_distribution; // histogram of correspondence statuses
	Edge2D _oedge;
	long _idum; // random number generator seed

	std::vector< ExtrinsicParameters > init_poses; // list of poses found by the system during initialization
	bool init_poses_changed;
	void insert_init_pose( ExtrinsicParameters &pose );

	void MyGlWindow::sort_init_poses( );
	
	void MyGlWindow::detect_edges();

	rude::Config _config;

	bool _draw_texture;

	bool _line_color_coding;

	/////////////////////////////////////////////////////////////
	// edge tracker
	std::vector< std::pair< int, std::vector< std::pair< int, EdgePlane > > > > _tracked_edges; // < number of keyframes, vector of <frame ID, EdgePlane > >

	int MIN_KEYFRAMES_PER_TRACK, MAX_TRACK_AGE;
	double KEYFRAME_DISTANCE;
	double MIN_EDGE_SIZE_TRACKING ;

	std::vector< std::pair < int, ExtrinsicParameters > > _keyframes;  // <frame ID, pose>

	void MyGlWindow::initEdgeTracker ();
	void MyGlWindow::updateEdgeTracker () ;
	bool MyGlWindow::updateKeyFrames ();
	void MyGlWindow::updateTrackerCounters ();
	void MyGlWindow::cleanupEdgeTracker();
	void MyGlWindow::processTrackedEdges ();

	void MyGlWindow::WriteEdgeTracker ();
	void MyGlWindow::ReadEdgeTracker ();

	/////////////////////////////////////////////////////////////
	
	Fl_Progress *progress_bar; // a multi-purpose progress bar
	Fl_Light_Button *pause_video;
	Fl_Button *edge_clearmask, *detect_edges_button, *clear_edges, *clear_selected_edges, *clear_sm, *add_sm, *compute_pose;
	Fl_Button *icon1, *icon2, *icon3, *icon4, *icon5;
	Fl_Light_Button *view_mask, *view_reprojected_edges, *view_correspondences, *run_maintenance;
	Fl_Value_Slider *edgePixels, *edgeSigma, *edgeThresh;
	Fl_Value_Slider *calib_focal_x, *calib_focal_y;
	Fl_Input_Choice *calib_camera, *sphere_correspondence_mode;
	Fl_Button *calib_img_rectification,*grab_img, *next_frame;
	Fl_Roller *calib_center_x, *calib_center_y;
	std::vector<Fl_Value_Slider*> calib_distortion;
	Fl_Light_Button *view_flow,*calib_load_img,*draw_axis,*sphere_draw_frustum, *run_analysis;
	Fl_Roller *sphere_calib_yaw, *sphere_calib_pitch, *sphere_calib_roll, *sphere_calib_z;
	Fl_Scrollbar *frame_slider;
	Fl_Button *switch_view, *lock_recovery, *set_camera_pose;
	Fl_Light_Button *model_show_grid, *model_show_pixels, *calib_img_mask, *init;
	Fl_Counter *select_pose,*select_generic;
	Fl_Check_Button *vanishing_points, *rotation_scores;
	Fl_Light_Button *snapshot;
	
	double computescore_init ();

	double MyGlWindow::refine_camera_pose( std::vector< intVector > &edges_buckets );
	double MyGlWindow::refine_camera_pose( corresVector &correspondences, std::vector< intVector > &edges_buckets );
	void MyGlWindow::init_correspondences( corresVector &correspondences, std::vector< intVector > &edges_buckets, double max_dihedral_angle, double min_overlap, bool store );
	bool MyGlWindow::update_correspondences();
	void MyGlWindow::refine_from_accepted_correspondences ();
	void MyGlWindow::update_init_pose( std::vector< intVector > &edges_buckets );

	int refineCameraPoseFromNCorrespondences( CorrespondenceVector &correspondences );
	int refineCameraRotationFromNCorrespondences( CorrespondenceVector &correspondences );
	int refineCameraTranslationFromNCorrespondences( CorrespondenceVector &correspondences );
	//int computeCameraPoseFrom3Correspondences();
	//bool computeCameraPoseFrom3Correspondences(Edge *p, Edge *q, Edge *r, EdgePlane &P, EdgePlane &Q, \
	//	EdgePlane &R, ExtrinsicParameters &pose);
	//bool computeCameraPoseFrom1Correspondence();
	void computeMask (edgeVector lines,int edge_step);

	vp_vector _vps; // vanishing points
	vp_vector _vpc; // clusters of vanishing points

	std::vector< Vec3d > _vps_edges;
	std::vector< Vec3d > _vps_lines;

	bool _display_vps;

	std::vector<corner_line> _corner_lines;

	int _n_correspondences;

	/////////////////////////////////////////////////////////////////////
	// synthetic world
	void makeSyntheticSequence();
	void generateSyntheticEdgeplanes ();
	void generateSyntheticCorrespondences ( corresVector &cs );
	bool initSynthetic ();
	bool syntheticNextFrame () ;
	ExtrinsicParameters GetSyntheticPose ( int frameId );
	void generateSyntheticFrame ();
	void edgeDetectionErrorAnalysis();
	void writeError( const char *filename, int frameId, ExtrinsicParameters &pose, ExtrinsicParameters &ref_pose );

	int _synthetic_mode;
	ExtrinsicParameters _synthetic_start_pose;
	/////////////////////////////////////////////////////////////////////

	std::vector< InitMode > initModes;

	/////////////////////////////////////////////////////////////////////
	// ladybug recording
	bool _ladybug_recording;
	int  _ladybug_record_nimages;
	double _ladybug_record_framerate;

	/////////////////////////////////////////////////////////////////////
	// clustering of edges (maintenance phase)
	void resetClusters ( int cameraId );
	bool selectBestEdge( EdgePlane &edgeplane, edgePlaneVector &edgeplanes, int mode );
	void filterClustersOutliers ( int cameraId );
	int mergeClusterInliers (  int cameraId, int nclusters, edgePlaneVector &edgeplanes );
	void redistributeEdges (  int cameraId, int &nclusters );
	int clusterEdgesOverlap( int cameraId );
	bool clusterEdges(  int cameraId, EdgePlane &edgeplane, edgePlaneVector &edgeplanes, double angle_threshold );
	void filterIrrelevantClusters( EdgePlane &edgeplane, edgePlaneVector &edgeplanes, double angle_threshold );

	double MyGlWindow::score_camera_pose ( ExtrinsicParameters pose, std::vector< intVector > &edges_buckets );
	double MyGlWindow::refine_camera_translation( ExtrinsicParameters &pose, double h1, double h2, std::vector< intVector > &edges_buckets );
	int MyGlWindow::get_edgeplane_buckets( EdgePlane ep, intVector &bucket_ids );
	int MyGlWindow::get_n_buckets();
	void MyGlWindow::distribute_edgeplanes_into_buckets ( std::vector< intVector > &buckets );
	void MyGlWindow::distribute_edgeplanes_into_buckets ( edgePlaneVector &edgeplanes, std::vector< intVector > &buckets );
	void MyGlWindow::define_adjacency_rules( std::vector< std::pair< std::pair<int,int>, std::pair<int,int> > > &rules );

	/////////////////////////////////////////////////////////////////////

	void checkReprojection (int edge_step);
	//void updateCorrespondencesFromModelLines ( int topK, int edge_step, CorrespondenceVector &correspondences);
	//void updateCorrespondencesFromCorrespondences ( int edge_step, CorrespondenceVector &correspondences);
	void computeCameraPoseFromPools();
	bool computeECP( ExtrinsicParameters &pose );

	//////////////////////////////////////////////////////////////////////
	// 
	// Camera pose history
	std::vector< ExtrinsicParameters > pose_history, pose_history_backup;
	int POSE_HISTORY_N_DISPLAY;  // how many poses are drawn on the map
	void drawPoseHistory( Viewer &viewer, int radius, const float color[3] ); // draw a sphere at every position in the history
	Vec3d MyGlWindow::get_average_position( int n );
	Vec3d MyGlWindow::get_expected_position ( int n );
	Quaternion MyGlWindow::get_expected_rotation( ) ;
	
	corresVector cs;
	CorrespondenceVector _inliers;
	int _last_recovery;

	double RANSAC_SCORE_THRESHOLD, RANSAC_MIN_SCORE_INLIER;
	int INIT_ANGLE_STEPS;
	bool MyGlWindow::PROJ_TEST( corres &c, bool clear_mask, bool use_image, double angle_thresehold );
	bool MyGlWindow::PROJ_TEST_DISPLAY( corres &c, bool clear_mask, bool save_image, double angle_thresehold, edgePlaneVector &edges_display );
	bool MyGlWindow::PROJ_TEST_SYNTHETIC( corres &c, bool use_image );
	int BACKGROUND_WHITE;
	int STATE_COLOR_CODED;
	double ANCHOR_SIZE ;

	double COLOR_THRESHOLD;

	int    RANSAC_MAX_TRIALS;
	double ransacLockRecovery();
	void   fillInStateMachine (  CorrespondenceVector &csv );
	void   refineFromInliers ( corresVector &csv, CorrespondenceVector &inliers, double score_threshold );
	void   findLineMatches ( corresVector &csv );
	void   initCorrespondences ( corresVector &csv );
	bool   ransacInitFromNode( double &best_score );
	void   computeRegions( regionVector &regions );
	double MyGlWindow::compute_pose_vertical (  std::vector< intVector > &edges_buckets );
	double MyGlWindow::compute_pose_from_rotation_voting( double h1, double h2, std::vector< intVector > &edges_buckets );
	double MyGlWindow::compute_pose_from_line_matching( double h1, double h2, std::vector< intVector > &edges_buckets );
	void MyGlWindow::initInitialCorrespondences( edgePlaneVector &edgeplanes );
	void MyGlWindow::initIdealCorrespondences( edgePlaneVector &edgeplanes );
	void MyGlWindow::compute_initial_correspondences_voting ( edgePlaneVector &edgeplanes, edgeVector &lines,
		double h1, double h2, int topk, corresVector &correspondences, std::vector<Vec4d> &rotations );
	double MyGlWindow::compute_initial_correspondences_matching ( edgePlaneVector &edgeplanes, edgeVector &lines, double h1, double h2, int topk,
												std::vector< intVector > &edges_buckets );


	std::vector< std::pair< int, std::vector< std::pair< int, int > > > > _lines_history;

	void MyGlWindow::clearLineHistory(int frameid);
	void MyGlWindow::readLineHistory ();
	void MyGlWindow::writeLineHistory();
	bool MyGlWindow::getLineHistory();
	void MyGlWindow::updateLineHistory( );

	std::vector < std::pair< intVector, edgePlaneVector > > _line_matching_history;

	std::vector< ExtrinsicParameters > _line_matching_pose_history;

	int hist_min_rot[180]; // do not change 180 to anything else!
	
	std::vector< intVector > _edges_buckets;

	void run_synthetic_init( int nruns );

	void computeBestRotations( std::vector<Vec4d> rotations, int topk, double search_radius, std::vector<Vec3d> &best_rotations );

	bool _adding_anchor, _updating_anchor, _removing_anchor;
	bool _show_anchor;

	std::vector< anchor > _anchors; // see definition of anchor in geom3d.h

	void MyGlWindow::drawAnchors (int x, int y, double zoom, bool flip1, bool flip2, int cameraid, int h, int w) ;
	void MyGlWindow::addAnchor( int x, int y, int cameraId );
	void MyGlWindow::updateAnchor( int x, int y, int cameraId ) ;
	void MyGlWindow::removeAnchor( int x, int y, int cameraId ) ;

	int MyGlWindow::fromSelectedPointToImagePoint( int x, int y,  int cameraId, int &rx, int &ry );

	std::vector<Vec4d> _rotations, _best_rotations;
	Vec3d _ideal_rotation;

	void MyGlWindow::compute_vanishing_points( int topk, edgePlaneVector &edgeplanes, std::vector< Vec3d > &vps) ;
	double MyGlWindow::compute_pose_from_vps( double h1, double h2, std::vector< intVector > &edges_buckets );

	void MyGlWindow::relock ( double max_dihedral_angle, int nruns );
	void
	MyGlWindow::write_correspondences ( int frameid );
	bool MyGlWindow::read_correspondence ( int frameid, int lineid, EdgePlane *ep );

	//bool ransacCameraPose( int max_trials, double thres );
	void MyGlWindow::cameraPoseFromCorrespondences( corresVector correspondences );
	bool MyGlWindow::cameraPoseFromThreeCorrespondences( EdgePlane P, EdgePlane Q, EdgePlane R, Edge *p, Edge *q, Edge *r, 
													ExtrinsicParameters &pose );

	double MyGlWindow::compute_pose_from_corners( std::vector< corner > &corners, std::vector< corner_line > &corner_lines );
	double MyGlWindow::maximize_score( std::vector< intVector > &edges_buckets );
	void MyGlWindow::find_inliers_and_refine_camera_pose( corresVector &correspondences );

	double MIN_SUBTENDED_ANGLE;	// if lines have a subtended angle smaller than this limit, they are removed from the visibility set
	double MIN_DIHEDRAL_ANGLE;		// if two lines differ by less than this angle viewed from a position, 
								//only one of them is kept in the visibility set
	double MAINTENANCE_DIHEDRAL_ANGLE;
	double MAINTENANCE_MIN_OVERLAP;
	int MAINTENANCE_MAX_CORRESPONDENCES;
	int MAINTENANCE_HISTORY_SIZE;

	char *LADYBUG_CALIBRATION_FILE;

	double INIT_DIAMETER;
	bool INIT_OCCLUSION;
	bool INIT_CLUTTER;
	bool INIT_NOISE;
	int INIT_SCORING_MODE; // the scoring mode during init phase: 0 = binary, 1 = uniform prob., 2 = gaussian-pdf prob.
	int INIT_TOPK; // top-k matches for each model line
	int INIT_TOPK_VPS;
	int INIT_MAX_VOTES;
	double INIT_MAX_VPS_ANGLE;

	int INIT_RANSAC_TRIALS ; 
	double INIT_MIN_DIHEDRAL_ANGLE ;
	double INIT_MAX_SCORE_ANGLE ;
	double INIT_REFINEMENT_ANGLE ;
	double INIT_REFINEMENT_OVERLAP;

	int N_CHAIN_EDGES  ;

	int LADYBUG_ID;

	double INIT_MAX_CAMERA_HEIGHT;
	int INIT_HEIGHT_STEPS;
	double SCORE_MIN_OVERLAP;
	double SCORE_MAX_DIHEDRAL_ANGLE;
	int INIT_CORNERS_TOPK;
	int INIT_MIN_CORRESPONDENCES;

	Model *spheretsn; // sphere tessellation for edgeplanes normal vectors
	int _init_search_depth; // search depth for initialization phase
	double _init_score;

	std::vector<Vec3d> _point_cloud, _point_cloud_centroids;

	void MyGlWindow::computeColorHits(int level_of_detail );

	std::vector< std::pair<Vec3d, double> > rotation_scoresheet;

	int INIT_TOPK_ROTATIONS;
	double INIT_SEARCH_RADIUS_ROTATION;

	int FTRACKER_WINDOW_SIZE, FTRACKER_MAX_FEATURES, FTRACKER_MIN_DIST, FTRACKER_TIME_TO_LIVE; // FAST tracker parameters
	double FTRACKER_MIN_CORNER_ANGLE ;

	double SPHERE_TESSELLATION_RESOLUTION ; // tessellation resolution in radians

	// debug display
	Vec3d mline1_p, mline1_dir;
	double mline1_angle;

	bool readConfig();

	bool _processing_frame_colors;

	Fl_Menu_Bar *menu_bar;

	// backup is used for camera pose computation
	corresVector backup;

	//////////////////////////////////////////////////////////////////////

	void snapshotVideo();
	int _timestamp; // for snapshot generation
	IplImage *_screenshot_image;

	int SNAPSHOT_X;
	int SNAPSHOT_Y;
	int SNAPSHOT_W;
	int SNAPSHOT_H;

	void setEdgeDetectorParameters( bool init, bool synthetic ) ;

	void drawVideo(bool snap);
	void reprojectEdges ( edgeVector &lines );
	void reprojectEdges( edgeVector &lines, ExtrinsicParameters &pose );

	Fl_Menu_Bar *_menu_bar;
	void draw3DMap();
	void draw2DMap();
	void drawSphere();
	void drawCalibration();
	void drawRotations();

private:
	void draw();
	void hideAll();
	void showAndHide();
	int handle (int event);
	int keyboard(int event, int key);

	void restoreMode();
	void initMainWindow();
	void sphere_picking_3d (int x, int y);
	void video_picking_2d (int x, int y);
	void model_picking_3d (int x, int y);
	void drawVideoImages();
	void drawVideoEdges ();
	void drawVideoEdges (int cameraId, int _x, int _y, bool flip);
	void drawVideoInfo();
	void drawVideoPoints ();
	void drawSnapshotScreen();


	//bool computeCameraPoseFrom3Correspondences(bool flip1, bool flip2, Edge *p, Edge *q, Edge *r, EdgePlane &P, EdgePlane &Q, \
	//	EdgePlane &R, std::vector<PoseSolution> &poses);
	//bool computeCameraPoseFrom3Correspondences(bool flip1, bool flip2, std::vector<PoseSolution> &poses);
	bool computeTranslation( Edge *p, Edge *q, Edge *r, EdgePlane &P, EdgePlane &Q, EdgePlane &R, bool transform );
	void findBestRotations ( EdgePlane _P0, EdgePlane _Q0, EdgePlane _R0, Edge *p, Edge *q, Edge *r, doubleVector &angles);
	bool findBestRotation ( EdgePlane _P0, EdgePlane _Q0, EdgePlane _R0, Edge *p, Edge *q, Edge *r, double &alpha, double &beta);
	void findBestRotationExhaustiveSearch (Quaternion Rcam);
	//void checkPose (PoseSolution &pose);
	//int filterCameraPoses();
	void filterLinesOnAzimuth ( EdgePlane edge2, edgeVector &lines2);

	void computeMask (Edge *line, int edge_step);
	void computeMask (EdgePlane edgeplane, int edge_step);
	void computeMask (EdgePlane edgeplane, Edge *line, int edge_step);
};

// make a Gl window that draws something:
class MyControlBarWindow : public Fl_Gl_Window {
public:
	MyControlBarWindow(int x, int y, int w, int h) :
	  Fl_Gl_Window(x,y,w,h,"My GL Window")
	  {
	  }

	  MyGlWindow *_main_w_ptr;
	  double _perf_time;
	  Viewer _controlbar_viewer;

private:
	void draw(); 
};

void init(MyGlWindow *gl_main);
void initMainWindow(MyGlWindow *gl_main, MyControlBarWindow *gl_control/*, Fl_Menu_Bar *menu_bar*/) ;

// callbacks
void redraw_cb(Fl_Widget *w, void*);
void open_model_cb (Fl_Widget*, void*);
void ladybug_hd_cb(Fl_Widget *w, void*);
void quit_cb(Fl_Widget*, void*);
void set_cb(Fl_Widget*, void*);
void options_cb (Fl_Widget *w, void *);
void compute_sequence_edges_cb (Fl_Widget *w, void *);
void file_cb (Fl_Widget*, void*);
void rectify_img_cb (Fl_Widget *w, void*);
void grab_img_cb (Fl_Widget *w, void*);
void calib_load_img_cb (Fl_Widget *w, void*);
void edge_clearmasks_cb(Fl_Widget *w, void*);
void sphere_calib_apply_cb(Fl_Widget *w, void*);
void load_database_hd_cb(Fl_Widget *w, void*);
void exit_database_hd_cb(Fl_Widget *w, void*);
void detect_edges_cb(Fl_Widget *w, void*);
void clear_edges_cb(Fl_Widget *w, void*);
void clear_selected_edges_cb (Fl_Widget *w, void*);
void run_analysis_cb (Fl_Widget *w, void*);
void lock_recovery_cb (Fl_Widget *w, void*);
void init_cb (Fl_Widget *w, void*);
void select_pose_cb (Fl_Widget *w, void*);
void switch_view_cb (Fl_Widget *w, void*);
void make_synthetic_cb (Fl_Widget *w, void *);
void next_frame_cb (Fl_Widget *w, void*);
void run_maintenance_cb (Fl_Widget *w, void*);
void write_history_cb (Fl_Widget *w, void*);
void read_history_cb (Fl_Widget *w, void*);
void screenshot_cb ( Fl_Widget *w, void*);
void clear_sm_cb ( Fl_Widget *w, void*);
void add_sm_cb ( Fl_Widget *w, void*);
void set_camera_pose_cb( Fl_Widget *w, void*);
void ladybug_record_cb( Fl_Widget *w, void*);
void ladybug_framerate_cb( Fl_Widget *w, void*);
void refine_camera_pose_cb( Fl_Widget *w, void*);
void init_set_search_area_cb( Fl_Widget *w, void*);
void init_set_mode_cb(Fl_Widget *w, void*);
void process_init_mode_checks( Fl_Widget *w, void*);
void compute_pose_cb( Fl_Widget *w, void*);
void switch_bgd_color_cb( Fl_Widget *w, void*);
void init_synthetic_cb ( Fl_Widget *w, void*);
void init_correspondences_cb ( Fl_Widget *w, void*);
void sift_features_cb ( Fl_Widget *w, void*);
void relock_cb ( Fl_Widget *w, void*);
void set_video_mode_cb( Fl_Widget *w, void*);
void set_3dmap_mode_cb( Fl_Widget *w, void*);
void set_2dmap_mode_cb( Fl_Widget *w, void*);
void set_sphere_mode_cb( Fl_Widget *w, void*);
void set_calibration_mode_cb( Fl_Widget *w, void*);
void select_sensor_cb (Fl_Widget*w, void*);
void reproject_edges_cb (Fl_Widget*w, void*);
void frame_slider_changed_cb (Fl_Widget *w, void*);
void clear_history_cb (Fl_Widget *w, void*);
void select_generic_changed_cb( Fl_Widget *w, void*);
void select_correspondence_mode_changed_cb (Fl_Widget *w, void*);
void option_texture_cb(Fl_Widget *w, void*);
void average_pose_history_cb ( Fl_Widget *w, void*);

void read_config_cb(Fl_Widget *w, void*);

void color_read_cb(Fl_Widget *w, void*);
void color_write_cb(Fl_Widget *w, void*);
void color_process_frame_cb(Fl_Widget *w, void*);
void color_process_frame_series_cb(Fl_Widget *w, void*);
void color_clear_cb(Fl_Widget *w, void*);

void option_line_color_coding_cb (Fl_Widget *w, void*);
void make_movie_cb (Fl_Widget *w, void*);

void anchor_add_cb(Fl_Widget *w, void*);
void anchor_remove_cb(Fl_Widget *w, void*);
void anchor_update_cb(Fl_Widget *w, void*);
void anchor_clear_all_cb(Fl_Widget *w, void*);
void anchor_hide_show_cb(Fl_Widget *w, void*);
void option_vps_display_cb(Fl_Widget *w, void*);

void tracker_write_cb(Fl_Widget *w, void*);
void tracker_read_cb(Fl_Widget *w, void*);
void tracker_clear_cb(Fl_Widget *w, void*);
void tracker_compute_cb(Fl_Widget *w, void*);

// menu bar
static Fl_Menu_Item mainMenu[] =
{
	{ "&File",              0, 0, 0, FL_SUBMENU },
		{ "&Open Model", FL_CTRL + 'O', (Fl_Callback *)open_model_cb, 0 },
		{ "&Read Config", ' ',  (Fl_Callback *)read_config_cb, 0},
		{ "&Save snapshot", ' ',  (Fl_Callback *)file_cb, 0},
		{ "&Save from Ladybug HD", ' ', (Fl_Callback *)ladybug_hd_cb, 0},
		{ "&Save history", ' ', (Fl_Callback *)write_history_cb, 0},
		{ "&Clear history", ' ', (Fl_Callback *)clear_history_cb, 0},
		{"&Load database", ' ', (Fl_Callback *)load_database_hd_cb, 0},
		{"&Exit database", ' ', (Fl_Callback *)exit_database_hd_cb, 0},
		{ "E&xit", FL_CTRL + 'q', (Fl_Callback *)quit_cb, 0 },
		{ 0 },

	{ "&Window", 0, 0, 0, FL_SUBMENU },
		{ "Video", FL_CTRL + 'v', (Fl_Callback *)set_cb, (void*)0 },
		{ "3D Map", FL_CTRL + 'd', (Fl_Callback *)set_cb, (void*)0 },
		{ "2D Map", FL_CTRL + 'm', (Fl_Callback *)set_cb, (void*)0 },
		{ "Sphere", FL_CTRL + 's', (Fl_Callback *)set_cb, (void*)0 },
		{ "Camera calibration", FL_CTRL + 's', (Fl_Callback *)set_cb, (void*)0 },
		{ "Rotations", FL_CTRL + 's', (Fl_Callback *)set_cb, (void*)0 },
		{0},

	{ "&Synthetic", 0, 0, 0, FL_SUBMENU },
		{ "Real", FL_CTRL + 'x', (Fl_Callback *)options_cb, (void*)0 },
		{ "Synthetic 1", FL_CTRL + 'x', (Fl_Callback *)options_cb, (void*)0 },
		{ "Synthetic 2", FL_CTRL + 'x', (Fl_Callback *)options_cb, (void*)0 },
		{ "Synthetic 3", FL_CTRL + 'x', (Fl_Callback *)options_cb, (void*)0 },
		{ "Error analysis", FL_CTRL + 'x', (Fl_Callback *)options_cb, (void*)0 },
		{"&Make synthetic sequence", ' ', (Fl_Callback *)make_synthetic_cb, 0},
		{"&Synthetic init", ' ', (Fl_Callback *)init_synthetic_cb, 0},
		{"&Smooth pose history", ' ', (Fl_Callback *)average_pose_history_cb, 0},
		{ 0 },

		// do not change the names in this section, nor the ordering! (see callback.cpp)
	{ "&Init", 0, 0, 0, FL_SUBMENU },
		{ "Set init search area", FL_CTRL + 'x', (Fl_Callback *)init_set_search_area_cb, (void*)0 },
		{"Vertical pose",    FL_ALT+'i', 0, 0, FL_MENU_TOGGLE|FL_MENU_VALUE},
		{"Corners matching",    FL_ALT+'i', 0, 0, FL_MENU_TOGGLE|FL_MENU_VALUE},
		{"Vanishing points",    FL_ALT+'i', 0, 0, FL_MENU_TOGGLE|FL_MENU_VALUE},
		{"Rotation voting",    FL_ALT+'i', 0, 0, FL_MENU_TOGGLE|FL_MENU_VALUE},
		{"Line-line matching",    FL_ALT+'i', 0, 0, FL_MENU_TOGGLE|FL_MENU_VALUE},
		{"Exhaustive Search",    FL_ALT+'i', 0, 0, FL_MENU_TOGGLE|FL_MENU_VALUE},
		{ "Refine camera pose", FL_CTRL + 'x', (Fl_Callback *)refine_camera_pose_cb, (void*)0 },
		{ "Init correspondences", FL_CTRL + 'x', (Fl_Callback *)init_correspondences_cb, (void*)0 },
		{ "Relock", FL_CTRL + 'x', (Fl_Callback *)relock_cb, (void*)0 },
		{ 0 },

		{ "&Color", 0, 0, 0, FL_SUBMENU },
		{ "Read colors", FL_CTRL + 'x', (Fl_Callback *)color_read_cb, (void*)0 },
		{ "Write colors",  FL_CTRL + 'w', (Fl_Callback *)color_write_cb, (void*)0 },
		{ "Clear colors", FL_CTRL + 'z', (Fl_Callback *)color_clear_cb, (void*)0 },
		{ "Process frame", FL_CTRL + 'z', (Fl_Callback *)color_process_frame_cb, (void*)0 },
		{ "Process frame series", FL_CTRL + 'z', (Fl_Callback *)color_process_frame_series_cb, (void*)0, FL_MENU_TOGGLE },
		{ 0 },

		{ "&Options", 0, 0, 0, FL_SUBMENU },
		{ "Wireframe", FL_CTRL + 'x', (Fl_Callback *)options_cb, (void*)0 },
		{ "Depth test", FL_CTRL + 'x', (Fl_Callback *)options_cb, (void*)0 },
		{ "Compute edges for sequence", FL_CTRL + 'x', (Fl_Callback *)compute_sequence_edges_cb, (void*)0 },
		{ "Switch Background color", 0, (Fl_Callback *)switch_bgd_color_cb, (void*)0 },
		{ "Hide/show VPs", 0, (Fl_Callback *)option_vps_display_cb, (void*)0 },
		{ "Line Color Coding", FL_CTRL + 'z', (Fl_Callback *)option_line_color_coding_cb, (void*)0, FL_MENU_TOGGLE|FL_MENU_VALUE },
		{ "Toggle textures",    FL_ALT+'i', (Fl_Callback *)option_texture_cb, (void*)0},
		{"SIFT",    FL_ALT+'i', 0, (Fl_Callback *)sift_features_cb, FL_MENU_TOGGLE|FL_MENU_VALUE},
		{ "Make movie", FL_CTRL + 'x', (Fl_Callback *)make_movie_cb, (void*)0 },
		{ 0 },

		{ "&Tracker", 0, 0, 0, FL_SUBMENU },
		{ "Write tracker", FL_CTRL + 'x', (Fl_Callback *)tracker_write_cb, (void*)0 },
		{ "Read tracker", FL_CTRL + 'x', (Fl_Callback *)tracker_read_cb, (void*)0 },
		{ "Clear tracker", FL_CTRL + 'x', (Fl_Callback *)tracker_clear_cb, (void*)0 },
		{ "Compute tracker", FL_CTRL + 'x', (Fl_Callback *)tracker_compute_cb, (void*)0 },
		{ 0 },

		{ "&Ladybug", 0, 0, 0, FL_SUBMENU },
		{ "Start/Stop Record", FL_CTRL + 'x', (Fl_Callback *)ladybug_record_cb, (void*)0 },
		{ 0 },
		
		{ "&Anchor", 0, 0, 0, FL_SUBMENU },
		{ "Add", FL_CTRL + 'x', (Fl_Callback *)anchor_add_cb, (void*)0 },
		{ "Remove", FL_CTRL + 'x', (Fl_Callback *)anchor_remove_cb, (void*)0 },
		{ "Update", FL_CTRL + 'x', (Fl_Callback *)anchor_update_cb, (void*)0 },
		{ "Hide/Show", FL_CTRL + 'x', (Fl_Callback *)anchor_hide_show_cb, (void*)0 },
		{ "Clear All", FL_CTRL + 'x', (Fl_Callback *)anchor_clear_all_cb, (void*)0 },
		{ 0 },

	{0},


};

static Fl_Double_Window* _wind;
static MyGlWindow* _gl_main;
static MyControlBarWindow* _gl_control;
static double _tempo;

#endif
