/*
 * osg_Object.h
 *
 *  Created on: 2012/12/20
 *      Author: umakatsu
 */

#ifndef OSG_OBJECT_H_
#define OSG_OBJECT_H_

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

#include <vector>

#include "src\UserConstant.h"

class osg_Object
{
public:
	osg_Object();
	~osg_Object();

	void SetObjTransformArrayIndex( const int & idx, osg::Quat quat, osg::Vec3d vec);
	void PushObjNodeArray(osg::Node* n){ obj_node_array.push_back(n); }
	osg::ref_ptr<osg::Node> GetObjNodeIndex(const int & idx) const;
	std::vector<osg::Node*> getObjNodeArray() const {
		return obj_node_array;
	}

	int SizeObjNodeArray(void) const ;
	int SizeObjTransformArray(void) const;

	void setObjectIndex(int objectIndex) {
		this->objectIndex = objectIndex;
	}

	int getVirtualObjectsCount() const {
		return Virtual_Objects_Count;
	}

	int getObjectIndex() const {
		return objectIndex;
	}

	void setObjNodeArray(std::vector<osg::Node*> objNodeArray) {
		obj_node_array = objNodeArray;
	}

	std::vector<osg::PositionAttitudeTransform*> getObjTransformArray() const {
		return obj_transform_array;
	}

	void setObjTransformArray(
			std::vector<osg::PositionAttitudeTransform*> objTransformArray) {
		obj_transform_array = objTransformArray;
	}

	osg::ref_ptr<osg::PositionAttitudeTransform> getObjTexturePosAtt() const {
		return mObjTexturePosAtt;
	}

	void setObjTexturePosAtt(
			osg::ref_ptr<osg::PositionAttitudeTransform> objTexture) {
		mObjTexturePosAtt = objTexture;
	}

	osg::ref_ptr<osg::Node> getObjTexture() const {
		return objTexture;
	}

	void setObjTexture(osg::ref_ptr<osg::Node> objTexture) {
		this->objTexture = objTexture;
	}

	bool isSoftTexture() const {
		return softTexture;
	}

	void setSoftTexture(bool softTexture) {
		this->softTexture = softTexture;
	}

	void IncrementObjCount(void){ Virtual_Objects_Count++; }
	void DecrementObjCount(void){ Virtual_Objects_Count--; }
	void IncrementObjIndex(void){ objectIndex++; }
	void IncrementObjIndex(const int & val){ objectIndex+=val; }
	void DecrementObjIndex(void){ objectIndex--; }

	void osg_resetNodes();
public:

	//HeightField
	osg::ref_ptr<osg::Vec3Array>	mHeightFieldPoints;
	osg::ref_ptr<osg::Geometry>		mHeightFieldGeometry_quad;
	osg::ref_ptr<osg::Geometry>		mHeightFieldGeometry_line;
	osg::ref_ptr<osg::Vec4Array>	mGroundQuadColor;
	osg::ref_ptr<osg::Vec4Array>	mGroundLineColor;


#if CAR_SIMULATION==1
	//car parameter <CAUTION>not smart solution
	std::vector<osg::PositionAttitudeTransform*> car_transform;
	std::vector<osg::PositionAttitudeTransform*> wheel_transform[NUM_CARS];
#endif

private:
	//osg_root‚©‚çˆÚ“®-->//
	//for virtual objects
	std::vector<osg::ref_ptr<osg::Node>> obj_node_array;
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> obj_transform_array;

	std::vector<osg::ref_ptr<osg::Node>> hand_object_array;


	std::vector<osg::PositionAttitudeTransform*> hand_object_global_array;
	std::vector<osg::PositionAttitudeTransform*> hand_object_transform_array[MAX_NUM_HANDS];
	std::vector<osg::ShapeDrawable*> hand_object_shape_array;
	//<--osg_root‚©‚çˆÚ“®//

	//for virtual objects
	int Virtual_Objects_Count;

	int objectIndex;

	//for soft texture
	osg::ref_ptr<osg::Node> objTexture;
	osg::ref_ptr<osg::PositionAttitudeTransform> mObjTexturePosAtt;
	bool softTexture;
};

#endif