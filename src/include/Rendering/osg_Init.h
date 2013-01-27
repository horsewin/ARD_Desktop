/*
 * osg_Init.h
 *
 *  Created on: 2012/12/20
 *      Author: umakatsu
 */

#ifndef OSG_INIT_H_
#define OSG_INIT_H_

#include <boost\shared_ptr.hpp>

class osg_Object;

class osg_Init
{
public:
	osg_Init(){};
	~osg_Init(){};

	//Init of Height field
	void CreateHeightfield(boost::shared_ptr<osg_Object> osgObject);
	void SetHeightfield(boost::shared_ptr<osg_Object> osgObject);

	//Init of Cars
	void CreateCarUnit(boost::shared_ptr<osg_Object> osgObject);
	void SetCarRenderBin(boost::shared_ptr<osg_Object> osgObject);
};

#endif