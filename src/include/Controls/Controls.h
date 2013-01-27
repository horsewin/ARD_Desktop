#ifndef CONTROLS_H
#define CONTROLS_H

#include <boost\shared_ptr.hpp>

class bt_ARMM_world;
class osg_Root;

class Controller 
{
	public:
		Controller(bt_ARMM_world *m_world) { world = m_world; }
		void check_input(boost::shared_ptr<osg_Root> osgRoot);

	protected:
		bt_ARMM_world *world;
};

#endif