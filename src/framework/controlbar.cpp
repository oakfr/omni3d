#include "main.h"

void MyControlBarWindow::draw()
{

	// the draw method must be private
	_controlbar_viewer.set(CONTROLBAR_WIDTH,CONTROLBAR_HEIGHT,0);

	_controlbar_viewer.setup2D();
	_controlbar_viewer.clearWindow(true);
	glDisable(GL_LIGHTING);

	int x = 10;
	int y = CONTROLBAR_HEIGHT-20;

	_controlbar_viewer.drawBox(0,0,CONTROLBAR_WIDTH,CONTROLBAR_HEIGHT,2,RED);
	
	_controlbar_viewer.displayMsg(x,y,"Control Bar",RED,0);
	y -= 20;

	// print the time
	MyTimer timer;
	_controlbar_viewer.displayMsg(x,y,(char*)timer.timestring().c_str(),RED,0);
	y -= 20;

	// print info

	// print performance time
	_controlbar_viewer.displayMsg(x,y,RED,0,"Frame ID: %d",_main_w_ptr->frameId);
	y -= 20;

	// print synthetic mode
	_controlbar_viewer.displayMsg(x,y,RED,0,"# correspondences: %d",_main_w_ptr->_n_correspondences);
	y -= 20;

	/*if ( !_main_w_ptr->correspondence_distribution.empty() ) {
		// print the distribution
		_controlbar_viewer.displayMsg(x, y, BLUE, 0, "CONNECTED: %d", _main_w_ptr->correspondence_distribution[0] );
		y -= 20;
		_controlbar_viewer.displayMsg(x, y, BLUE, 0, "EXPECTED: %d", _main_w_ptr->correspondence_distribution[1] );
		y -= 20;
		_controlbar_viewer.displayMsg(x, y, BLUE, 0, "BLACKLISTED: %d", _main_w_ptr->correspondence_distribution[2] );
		y -= 20;
	}
	*/

	// print VP scores
	_controlbar_viewer.displayMsg(x,y,RED,0,"VP Score: %.4f", scoreSetsOfVanishingPoints(_main_w_ptr->_LUT._vps, _main_w_ptr->_vpc, toRadians(10.0)));

};


