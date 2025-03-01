/*
 * KeyboardControls.cpp
 *
 *
 *  Created on: 2013/1/15
 *      Author: umakatsu
 */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "KeyboardControls.h"
#include "../Physics/bt_ARMM_world.h"
#include "../Rendering/osg_Root.h"

#include "src\constant.h"

#include <Windows.h>
#include <iostream>

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
using namespace std;

extern int collide_counter;
extern double prev_collide_clock;
extern interaction interact_state;
extern bool running;
extern CvSize markerSize;
extern CvPoint2D32f marker_origin;
extern float WORLD_SCALE;
extern float WORLD_ANGLE;
extern float MARKER_DEPTH;
extern float WORLD_ORIGIN_X;
extern float WORLD_ORIGIN_Y;
extern CvMat *kinectTransform;

double x = 0, y=0, z=0;
bool WIREFRAME_MODE = false;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
void KeyboardController::PressB()
{
	// Add box shape objects
	osgAddObjectNode(world->CreateSoftTexture("Data/tex.bmp"));
	cout << "create the soft body object" << endl;
}

void KeyboardController::PressN()
{
	
	int index = world->create_Sphere();
	osgAddObjectNode(osgNodeFromBtSphere(SPHERE_SIZE, world->get_Object_Transform(index)));
	Virtual_Objects_Count++;
}

int KeyboardController::TransmitInput(const int & input)
{
	const int transmitOffset = 200;
	return (input>0? (transmitOffset+input) : 0);
}


int KeyboardController::check_input(boost::shared_ptr<osg_Root> osgRoot)
{
#if CAR_SIMULATION == 1
	//Car Number 1
	if (getKey(VK_UP)) {
		world->accelerateEngine(0);
	} else if (getKey(VK_DOWN)) {
		world->decelerateEngine(0);
	} else {
		world->resetEngineForce(0);
	}
	if (getKey(VK_LEFT)) {
		world->turnEngineLeft(0);
	} else if (getKey(VK_RIGHT)) {
		world->turnEngineRight(0);
	} else {
			world->turnReset(0);
	}
	//Car Number 2
	if (getKey(87)) {//W
		world->accelerateEngine(1);
	} else if (getKey(83)) {//S
		world->decelerateEngine(1);
	} else {
		world->resetEngineForce(1);
	}
	if (getKey(65)) {//A
		world->turnEngineLeft(1);
	} else if (getKey(68)) {//D
		world->turnEngineRight(1);
	} else {
		world->turnReset(1);
	}
#endif /* CAR_SIMULATION == 1 */

	//A 65 S 83 D 68 W 87 F 70 V 86
	if (getKey(VK_ESCAPE)) running = false;
#ifdef SIM_MICROMACHINE
	if (getKey(82)) world->resetCarScene(0); //R
	if (getKey(84)) world->resetCarScene(1); //T
#endif /*SIM_MICROMACHINE*/

	if (getKey(86)) { //V
		WIREFRAME_MODE = !WIREFRAME_MODE;
		if(WIREFRAME_MODE) 
		{
			osgRoot->ShowGroundGeometry();
		} 
		else 
		{
			osgRoot->HideGroundGeometry();
		}
		printf("Wireframe Mode = %d \n",WIREFRAME_MODE);
		return 86;
	}

#ifdef SIM_MICROMACHINE
	if(Virtual_Objects_Count < MAX_NUM_VIR_OBJ) 
	{
		if (getKey(66)) { //B
			PressB();
			return 66;
		}
		if (getKey(77)) { //M
			//interact_state = PINCH;
			osgArInputButton = 203;
			return 203;
		}
		if (getKey(79)) { //o
			//string modelname(DATABASEDIR);
			//modelname+="cube/cube.3ds";
			////string modelname = "HatuneMiku.3ds";
			//int index = world->create_3dsmodel(modelname.c_str());
			//osgAddObjectNode(osgNodeFrom3dsModel(world->GetModelName(), world->get3dsScale(), world->get_Object_Transform(index)));
			//Virtual_Objects_Count++;
			//world->ChangeAttribute(25, -5, 5, index);
			//				
			return 79;
		}

		if (getKey(80)) { //p
			
			string modelname(ARMM::ConstParams::DATABASEDIR);
			modelname+="keyboard/keyboard.3ds";
			//string modelname = "HatuneMiku.3ds";
			int index = world->create_3dsmodel(modelname.c_str());
			osgAddObjectNode(osgNodeFrom3dsModel(world->GetModelName(), world->get3dsScale(), world->get_Object_Transform(index)));
			Virtual_Objects_Count++;
			world->ChangeAttribute(10, -5, 5, index);

			return 80;
		}
		if (getKey(81)) { //q
			string modelname(ARMM::ConstParams::DATABASEDIR);
			modelname+="cube/cube.3ds";
			//string modelname = "HatuneMiku.3ds";
			int index = world->create_3dsmodel(modelname.c_str());
			osgAddObjectNode(osgNodeFrom3dsModel(world->GetModelName(), world->get3dsScale(), world->get_Object_Transform(index)));
			Virtual_Objects_Count++;
			world->ChangeAttribute(25, -5, 5, index);
								
			return 81;
		}

		if (getKey(78) ) { //N
			PressN();
			return 78;
		}
	}
#endif /*SIM_MICROMACHINE*/
	if (getKey(VK_SPACE)) 
	{
		return VK_SPACE;
	}

	if (getKey(VK_RETURN))
	{
		if (kinectTransform)
		{
			CvFileStorage *fs = cvOpenFileStorage(KINECT_TRANSFORM_FILENAME, 0, CV_STORAGE_WRITE);
			cvStartWriteStruct(fs, "MarkerSize", CV_NODE_MAP); 
				cvWriteInt(fs, "width", markerSize.width);
				cvWriteInt(fs, "height", markerSize.height);
			cvEndWriteStruct(fs);

			cvStartWriteStruct(fs, "MarkerOrigin", CV_NODE_MAP); 
				cvWriteInt(fs, "x", marker_origin.x);
				cvWriteInt(fs, "y", marker_origin.y);
			cvEndWriteStruct(fs);

			cvWriteReal(fs, "WorldScale", WORLD_SCALE);
			cvWriteReal(fs, "WorldAngle", WORLD_ANGLE);
			cvWriteReal(fs, "MARKER_DEPTH", MARKER_DEPTH);

			cvWrite(fs, "KinectTransform", kinectTransform);
			cvReleaseFileStorage( &fs );
			printf("Saved Kinect Transform\n");
		}
	}
	return 0;
}

inline bool KeyboardController::getKey(int key)
{
	return GetAsyncKeyState(key)& 0x8000; 
}
