#ifndef _STATE_H__
#define _STATE_H__

#include "main.h"

class MyGlWindow;

#define BLACKLISTED_TIMEOUT 20
#define EXPECTED_TIMEOUT 20
#define OBSERVED_TIMEOUT 3
#define PAUSED_TIMEOUT 20
#define SILENT_TIMEOUT 5
#define CONNECTED_MIN_AGE_FOR_POSE 3
#define CONNECTED_MIN_AGE_FOR_MODEL_UPDATE 5

#define ALIGNMENT_THRESHOLD_TO_CONNECTED COS_7
#define ALIGNMENT_THRESHOLD_TO_OBSERVED COS_10
#define ALIGNMENT_THRESHOLD_TO_EXPECTED COS_20

static MyGlWindow *w;
static std::vector< corres > cq;
static std::vector< std::vector< corres > > history;
static intVector empty_seats;

static int index;
static int _max_n_correspondences; // maximum number of correspondences to keep track of
static int _max_n_frames; // maximum number of frames to keep track of

void SM_Init(MyGlWindow*, int max_n_correspondences, int max_n_frames);
void SM_ProcessItems( );
void SM_ProcessItem( corres &c );
void SM_Blacklisted( corres &c );
void SM_Expected( corres &c );
void SM_Observed( corres &c );
void SM_Connected( corres &c );
void SM_Paused( corres &c );
void SM_Alternate( corres &c );
void SM_Silent( corres &c );
void SM_RemoveItem ( corres &c );
void SM_RemoveItem( Edge *line );
void SM_InsertItem( corresStatus status, EdgePlane &ep );
void SM_InsertItem( corresStatus status, Edge *line );
void SM_InsertItem( corresStatus status, EdgePlane ep, Edge *line, int age );
void SM_InsertItem( corres &c );
void SM_InsertItem( Correspondence &c );
void SM_Clear();
void SM_ClearHistory();
void SM_UpdateHistory();
void SM_UpdateHistory2( ExtrinsicParameters &pose );
void SM_Statistics( int frameid, ExtrinsicParameters &pose );

bool SM_GetHistory( int line_id, corres &c ) ;
void SM_VerifyCorrespondences( ExtrinsicParameters &pose );

void SM_Clear(int i);

void SM_Print();
bool SM_ComputePose(int pose_max_elements, int pose_min_elements, int pose_max_trials, int pose_threshold_score, corresVector &cs, CorrespondenceVector &inliers);
bool SM_ComputePose(int pose_max_elements, int pose_min_elements, int pose_max_trials, int pose_threshold_score, CorrespondenceVector &inliers);
void SM_UpdateModel();
void SM_NodeChanged( edgeVector &lines );
void SM_ComputeDistribution( intVector &dist );
int SM_Size();
corres SM_Get(int p);
void SM_SetId( int i, int eid );
void SM_UpdateCorrespondences( );

bool SM_get_correspondence( int p, Correspondence &c );

#endif
