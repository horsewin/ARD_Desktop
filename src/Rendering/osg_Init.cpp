/*
 * osg_Init.cpp
 *
 *  Created on: 2012/12/20
 *      Author: umakatsu
 */

#include "Rendering\osg_Init.h"
#include "Rendering\osg_Object.h"

#include "UserConstant.h"

#include <osgDB/ReadFile>


void osg_Init::CreateHeightfield(boost::shared_ptr<osg_Object> osgObject)
{
	osgObject->mHeightFieldPoints			= new osg::Vec3Array;
	osgObject->mHeightFieldGeometry_quad	= new osg::Geometry;
	osgObject->mHeightFieldGeometry_line	= new osg::Geometry;
	osgObject->mGroundQuadColor				= new osg::Vec4Array;
	osgObject->mGroundLineColor				= new osg::Vec4Array;

	for (int i=0; i<159; i++) 
	{
		for (int j=0; j<119; j++) 
		{
			osgObject->mHeightFieldPoints->push_back(osg::Vec3(i-159, j-119, 0));
			osgObject->mHeightFieldPoints->push_back(osg::Vec3(i-158, j-119, 0));
			osgObject->mHeightFieldPoints->push_back(osg::Vec3(i-158, j-118, 0));
			osgObject->mHeightFieldPoints->push_back(osg::Vec3(i-159, j-118, 0));
		}
	}

}

void osg_Init::SetHeightfield(boost::shared_ptr<osg_Object> osgObject)
{
		osgObject->mHeightFieldGeometry_quad->setVertexArray(osgObject->mHeightFieldPoints); 
		osgObject->mHeightFieldGeometry_quad->addPrimitiveSet(new osg::DrawArrays( GL_QUADS, 0, osgObject->mHeightFieldPoints->size()));
		osgObject->mHeightFieldGeometry_quad->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

		osgObject->mHeightFieldGeometry_line->setVertexArray(osgObject->mHeightFieldPoints); 
		osgObject->mHeightFieldGeometry_line->addPrimitiveSet(new osg::DrawArrays( GL_LINES, 0, osgObject->mHeightFieldPoints->size()));
		osgObject->mHeightFieldGeometry_line->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);		

		//Set the Heightfield to be alpha invisible
		osgObject->mHeightFieldGeometry_quad->setColorBinding(osg::Geometry::BIND_OVERALL); 
		osgObject->mHeightFieldGeometry_line->setColorBinding(osg::Geometry::BIND_OVERALL); 

		//osg::Vec4Array* col = new osg::Vec4Array(); 
		osgObject->mHeightFieldGeometry_quad->setColorArray(osgObject->mGroundQuadColor); 
		osgObject->mHeightFieldGeometry_line->setColorArray(osgObject->mGroundLineColor);
		osgObject->mGroundQuadColor->push_back(osg::Vec4(1,1,1,0.0));
		osgObject->mGroundLineColor->push_back(osg::Vec4(1,1,1,0.0));
}

void osg_Init::CreateCarUnit(boost::shared_ptr<osg_Object> osgObject)
{

#if CAR_SIMULATION==1

	osg::ref_ptr<osg::Node> car1 = osgDB::readNodeFile(CAR1_BODY_FILENAME);
	if( !car1 ){
		std::cerr << "Model cound not be loaded!" << std::endl;
	}	

	osg::PositionAttitudeTransform * car_trans1 = new osg::PositionAttitudeTransform();
	car_trans1->setAttitude(osg::Quat(
		osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)
		));
	car_trans1->setPosition(osg::Vec3d(0.0, 0.0, 3.0));//shift body higher 3 units
	car_trans1->addChild(car1.get());
	osg::ref_ptr<osg::Node> wheel1 = osgDB::readNodeFile(CAR1_WHEEL_FILENAME);
	std::vector<osg::PositionAttitudeTransform*> wheel_tmp_trans1;

	for(int i = 0 ; i < 4; i++)  
	{ 
		wheel_tmp_trans1.push_back(new osg::PositionAttitudeTransform); 
		wheel_tmp_trans1.at(i)->addChild(wheel1.get());
		if(i == 0 || i == 3) 
		{
			wheel_tmp_trans1.at(i)->setAttitude(osg::Quat(osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0), osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)));
		} else {
			wheel_tmp_trans1.at(i)->setAttitude(osg::Quat(osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0), osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)));
		}
		wheel_tmp_trans1.at(i)->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
		osgObject->wheel_transform[0].push_back(new osg::PositionAttitudeTransform); 
		osgObject->wheel_transform[0].at(i)->setNodeMask(castShadowMask);
		osgObject->wheel_transform[0].at(i)->addChild(wheel_tmp_trans1.at(i));
	}

	//begin car 2--->
	osg::Node *car2 = osgDB::readNodeFile(CAR2_BODY_FILENAME);
	osg::PositionAttitudeTransform * car_trans2 = new osg::PositionAttitudeTransform();
	car_trans2->setAttitude(osg::Quat(
		osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0),
		osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)
		));
	car_trans2->setPosition(osg::Vec3d(0.0, 0.0, 8.0));
	car_trans2->addChild(car2);

	osg::ref_ptr<osg::Node> wheel2 = osgDB::readNodeFile(CAR2_WHEEL_FILENAME);
	std::vector<osg::PositionAttitudeTransform*> wheel_tmp_trans2;

	for(int i = 0 ; i < 4; i++)  { 
		wheel_tmp_trans2.push_back(new osg::PositionAttitudeTransform); 
		wheel_tmp_trans2.at(i)->addChild(wheel2.get());
		if(i == 0 || i == 3) {
			wheel_tmp_trans2.at(i)->setAttitude(osg::Quat(osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0), osg::DegreesToRadians(180.f), osg::Vec3d(0.0, 0.0, 1.0)));
		} else {
			wheel_tmp_trans2.at(i)->setAttitude(osg::Quat(osg::DegreesToRadians(0.f), osg::Vec3d(1.0, 0.0, 0.0),
			osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 1.0, 0.0), osg::DegreesToRadians(0.f), osg::Vec3d(0.0, 0.0, 1.0)));
		}
		wheel_tmp_trans2.at(i)->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
		osgObject->wheel_transform[1].push_back(new osg::PositionAttitudeTransform); 
		osgObject->wheel_transform[1].at(i)->setNodeMask(castShadowMask );
		osgObject->wheel_transform[1].at(i)->addChild(wheel_tmp_trans2.at(i));
	}
	//<---end car 2

	osgObject->car_transform.push_back(new osg::PositionAttitudeTransform());
	osgObject->car_transform.at(0)->setAttitude(osg::Quat(0,0,0,1));
	osgObject->car_transform.at(0)->addChild(car_trans1);
	osgObject->car_transform.at(0)->setNodeMask(rcvShadowMask|castShadowMask);

	osgObject->car_transform.push_back(new osg::PositionAttitudeTransform());
	osgObject->car_transform.at(1)->setAttitude(osg::Quat(0,0,0,1));
	osgObject->car_transform.at(1)->addChild(car_trans2);
	osgObject->car_transform.at(1)->setNodeMask(rcvShadowMask|castShadowMask);

#endif

}

void osg_Init::SetCarRenderBin(boost::shared_ptr<osg_Object> osgObject)
{
#if CAR_SIMULATION == 1
		osgObject->car_transform.at(0)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
		osgObject->car_transform.at(1)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
		for(int w=0; w<4; w++){
			osgObject->wheel_transform[0].at(w)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
			osgObject->wheel_transform[1].at(w)->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
		}
#endif /* CAR_SIMULATION == 1 */
}
