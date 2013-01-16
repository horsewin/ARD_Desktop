/*
 * osg_Object.cpp
 *
 *  Created on: 2012/12/20
 *      Author: umakatsu
 */
#include "osg_Object.h"

osg_Object::~osg_Object()
{
	obj_node_array.clear();
	obj_transform_array.clear();

#if CAR_SIMULATION == 1
	car_transform.clear();
	REP(i,NUMBER_CAR)
	{
		wheel_transform[i].clear();
	}
#endif /* CAR_SIMULATION == 1 */

	hand_object_array.clear();
	hand_object_global_array.clear();
	REP(i,MAX_NUM_HANDS)
	{
		hand_object_transform_array[i];
	}
	hand_object_shape_array.clear();
}

void osg_Object::osg_resetNodes()
{
	obj_node_array.clear();
	obj_transform_array.clear();
}
