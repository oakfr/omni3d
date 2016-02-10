#include "basic.h"
#include "util/util.h"
#include "camera/camera.h"
#include "timer/my_timer.h"

#define POSE_HISTORY_FILE "poses.dat" // list of poses for synthetic data

class Database;

class Database
{
public:
	int _date;
	std::string _dirname;
	int _width, _height;
	double _framerate;
	CameraType _cameraType;
	int _nImages;
	int _frameId;
	IplImage *temp;
	int _synthetic; // 1 if synthetic dataset, 0 otherwise
	std::string _filetype; // bmp, jpg, etc.
	int _ladybug_id; // Ladybug ID, default 5040012

	Database() {_frameId=0;_nImages=0;};
	~Database() {if (temp != NULL ) {cvReleaseImage(&temp);} temp=NULL;};

	void Database::init (std::string configfile);
	void Database::initNewDatabase ( std::string dirname, double framerate );
	void Database::createDataFile ();
	void Database::print();
	std::string Database::getLadybugImageFilename (int sensorId, int frameId);
void Database::prevFrame();
void Database::nextFrame();
void Database::loadFrame(Frame &frame);
void Database::createDataFile(int resolution_code, char *dirname, int nimages, int synthetic);
void setFrameId (int id) {assert(id < _nImages); _frameId = id;}

bool readPose( int frameId, ExtrinsicParameters &pose );

};
