#ifndef KEYBOARD_CONTROLS_H
#define KEYBOARD_CONTROLS_H

#include "Controls.h"

//OpenCV
#include "opencv/cv.h"

class KeyboardController: public Controller 
{
public:
	KeyboardController(bt_ARMM_world *m_world) : Controller(m_world){};

	int check_input(boost::shared_ptr<osg_Root> osgRoot);
	int TransmitInput(const int & input);

private:
	void PressB();
	void PressN();
	inline bool getKey(int key);
};
#endif