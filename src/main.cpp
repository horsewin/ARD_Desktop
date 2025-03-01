//TODO オブジェクトロスト後の処理がおかしいので修正の必要あり

#define UPDATE_TRIMESH 1
#define SIM_FREQUENCY 10
#define SHOWSEGMENTATION 1
#define SHOWKINECTIMG 0

#include "main.h"
#include <windows.h>
#pragma comment(lib, "winmm.lib")

#include <XnOS.h>
#include <XnCppWrapper.h> 

//OpenCV
#include "opencv\cv.h"
#include "opencv\highgui.h"

//OPIRA
#include "CaptureLibrary.h"
#include "OPIRALibrary.h"
#include "OPIRALibraryMT.h"
#include "RegistrationAlgorithms/OCVSurf.h"

//Bullet
#include "Physics/bt_ARMM_world.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
 
//Graphics calls
#include "Rendering\osg_Root.h"
#include "Rendering\osg_Object.h"

#include "leastsquaresquat.h"

//Transforms
#include "transforms.h"

//Controller Input
#include "Controls/KeyboardControls.h"
//#include "Controls/XBoxControls.h"

//Network using VRPN
#ifdef USE_ARMM_VRPN
#include "Network\Communicator.h"
#endif

#ifdef USE_SKIN_SEGMENTATION
#include "Skin_Col_Segment/HandRegion.h"
#include "Skin_Col_Segment/FlowCapture.h"
#endif

//STL
#include <deque>
#include <assert.h>

#include "UserConstant.h"
#include "constant.h"

const int MAX_FPS = 100;

extern int collide[2];
extern btVector3 pCollision;
extern int collisionInd;
extern interaction interact_state;

using namespace std; 
using namespace xn;

bt_ARMM_world *m_world;
boost::shared_ptr<osg_Root> pOsgRoot;

KeyboardController *kc;
//XboxController *xc;

// OpenNI Global
Context niContext;
DepthMetaData niDepthMD;
ImageMetaData niImageMD;

#ifdef USE_OPTICAL_FLOW
FlowCapture * flow_capture;
#endif

bool running = true;

bool loadKinectParams(char *filename, CvMat **params, CvMat **distortion);
void loadKinectTransform(char *filename);
osg::Node* osgNodeFromBtCollisionShape( const btConvexHullShape* hull, const btTransform& trans );
void RenderScene(IplImage *arImage, Capture *capture);
osg::Node* createVArrayFromHField(const btTransform& trans);
void GenerateTrimeshGroundFromDepth(IplImage* depthIm, float markerDepth);
osg::Node* createVArrayFromDepth();
void inpaintDepth(DepthMetaData *niDepthMD, bool halfSize);
float FindMarkerAffineRotation();
void setWorldOrigin();
void TransformImage(IplImage* depthIm, IplImage* ResDepth, float markerDepth, CvSize img_size, bool isDepth);
void UpdateHandRegions();
void removeNoise( IplImage* src, int size );
void FindHands(IplImage *depthIm, IplImage *colourIm);
void DetectFingertips(cv::Mat handMask);
void DetectFingertips(cv::Ptr<IplImage> handMask, vector< vector<cv::Point> > & fingerTips);
int CreateHand(int lower_left_corn_X, int lower_left_corn_Y) ;
void UpdateAllHands();
int Find_Num_Hand_Pixel(float depth);
void AssignPhysics2Osgmenu();
void ResetAddModelMode();
void ResetTextureTransferMode();
void ResetPanelCond();
void ExecuteAction(const int & val);
void CheckerArInput();
int	 CheckerArModelButtonType();

double cal_mean();
double cal_std(double mean);

Capture *capture;
CvMat *RegistrationParams;

//fo updating terrain
int counter = 5;
float *ground_grid, *voxel_grid;
float MaxHeight, MinHeight;

//image processing
IplImage *colourIm, *depthIm, *prev_gray, *curr_gray ,*transDepth160, *transDepth320, *transColor320;
IplImage *depthmask_for_mesh;

//for segmentation hand region
int cont_num; 
std::vector <CvRect>	cont_boundbox; 
std::vector <CvBox2D>	cont_boundbox2D; 
std::vector <CvPoint>	cont_center;

#if CAR_SIMULATION == 1
//Graphical objects
osg::Quat CarsOrientation[NUM_CARS];
osg::Quat WheelsOrientaion[NUM_CARS][NUM_WHEELS];
osg::Vec3d CarsPosition[NUM_CARS];
osg::Vec3d WheelsPosition[NUM_CARS][NUM_WHEELS];
#endif /* CAR_SIMULATION == 1 */

int input_key;
bool panelCollisionLock;	//for mutual exclusion of panel input
panelinput panelInput;
bool bTextureTransfer;

#ifdef USE_ARMM_VRPN
vrpn_Connection_IP* m_Connection;
ARMM_Communicator* ARMM_server;
vrpn_Imager_Server* ARMM_img_server;
vrpn_float32 ARMM_img_buffer[19200];
int	channel_id;

#ifdef USE_ARMM_VRPN_RECEIVER
	//Receiver part added by Atsushi 
	char *ARMM_CLIENT_IP = "ARMM_Client_Server@192.168.102.128";
	vrpn_Tracker_Remote	*ARMM_sever_receiver;
	int pass_key;
#endif

#endif

#ifdef USE_SKIN_SEGMENTATION
HandRegion _HandRegion;
CvPoint curr_hands_corners[MAX_NUM_HANDS], prev_hands_corners[MAX_NUM_HANDS];
float hand_depth_grids[MAX_NUM_HANDS][HAND_GRID_SIZE];
float curr_hands_ratio[MAX_NUM_HANDS];
float ratio;
#endif

//Optical Flow
#ifdef USE_OPTICAL_FLOW
int OPT_X_STEP = 20;
int OPT_Y_STEP = 20;
int LK_SIZE = 15;
bool RunOnce = true;
#endif

/////////////////////
// To calculate FPS//
/////////////////////
double previous_time = 0.0;
int    count_frame   = 0;
deque<double> fps;
double time_spent    = 0.0;
/////////////////////

namespace{
	double cal_mean() {
		double sum = 0;
		for(unsigned int i = 0; i < fps.size(); i++){
			sum += fps[i];
		}
		sum /= fps.size();
		return sum;
	}

	double cal_std(double mean) {
		double sum = 0;
		for(unsigned int i = 0; i < fps.size(); i++){
			sum += (fps[i] - mean) * (fps[i] - mean);
		}
		sum /= fps.size();
		return ( sqrt(sum) );
	}

	void TickCountAverageBegin()
	{
		previous_time = static_cast<double>(cv::getTickCount());
	}

	bool TickCountAverageEnd()
	{
		count_frame++;
		double current_time = static_cast<double>(cv::getTickCount());
		time_spent += ( current_time - previous_time ) / cv::getTickFrequency();		
		if( count_frame == 30){ // you can change the frame count if you want
			if( fps.size() < MAX_FPS){
				//fps.push_back(count_frame/time_spent);
				fps.push_back(1000*time_spent/count_frame);
			}else{
				fps.pop_front();
				fps.push_back(1000*time_spent/count_frame);
				//fps.push_back(count_frame/time_spent);
			}
			count_frame = 0;	
			time_spent = 0.0;		
			double mean = cal_mean();
			cout << "MEAN = " << mean << "ms  " << "SIGMA = " << cal_std(mean) << endl;
			cout << 1000/mean << endl;
			previous_time = current_time;
			return true;
		}
		return false;
	}
}

inline CvMat* scaleParams(CvMat *cParams, double scaleFactor) 
{
	CvMat *sParams = cvCloneMat(cParams);
	sParams->data.db[0]*= scaleFactor;	sParams->data.db[4]*= scaleFactor;
	sParams->data.db[2]*= scaleFactor;	sParams->data.db[5]*= scaleFactor;
	return sParams;
}

#ifdef USE_ARMM_VRPN_RECEIVER
void VRPN_CALLBACK handle_object (void * userData, const vrpn_TRACKERCB t)
{
	switch(t.sensor){
		case 78:
			pass_key = 78; //N
			break;

		case 2:
			break;

		default:
			pass_key = 0;
			break;
	}
}
#endif

//////////////////// Entry point //////////////////// 
int main(int argc, char* argv[]) 
{
	depthmask_for_mesh = cvCreateImage(MESH_SIZE, IPL_DEPTH_8U, 1);
	markerSize.width = -1; 
	markerSize.height = -1;

	//init OpenNI
	EnumerationErrors errors;
	switch (XnStatus rc = niContext.InitFromXmlFile(KINECT_CONFIG_FILENAME, &errors)) {
		case XN_STATUS_OK:
			break;
		case XN_STATUS_NO_NODE_PRESENT:
			XnChar strError[1024];	errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return rc; break;
		default:
			printf("Open failed: %s\n", xnGetStatusString(rc));
			return rc;
	}

	//set camera parameter
	capture = new Camera(1, CAPTURE_SIZE, CAMERA_PARAMS_FILENAME);
	RegistrationParams = scaleParams(capture->getParameters(), double(REGISTRATION_SIZE.width)/double(CAPTURE_SIZE.width));

	//init parameter for rendering
	osg_init(calcProjection(RegistrationParams, capture->getDistortion(), REGISTRATION_SIZE));

	//for Kinect view
	loadKinectParams(KINECT_PARAMS_FILENAME, &kinectParams, &kinectDistort);
	kinectDistort =0;
	kinectParams->data.db[2]=320.0; 
	kinectParams->data.db[5]=240.0;

	//setting kinect context
	niContext.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	niContext.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	g_depth.GetMirrorCap().SetMirror(false);
	g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);

	//registration
	kinectReg = new RegistrationOPIRA(new OCVSurf());
	kinectReg->addResizedMarker(MARKER_FILENAME, 400);

	//creating rendering object
	pOsgRoot = boost::shared_ptr<osg_Root>(new osg_Root());

	//physics
	m_world = new bt_ARMM_world();
	ground_grid = new float[ARMM::ConstParams::GRID_SIZE];
	for (int i =0;i < ARMM::ConstParams::GRID_SIZE; i++) 
	{
		ground_grid[i] = 0; 
	}
#ifdef SIM_PARTICLES
	voxel_grid = new float[1200];
	for (int i =0;i < 1200; i++) {
		voxel_grid[i] = 0;
	}
#endif

	//controls
	KeyboardController *kc = new KeyboardController(m_world);
	//XboxController *xc = new XboxController(m_world);

	loadKinectTransform(KINECT_TRANSFORM_FILENAME);

#ifdef USE_ARMM_VRPN
	//----->Server part
	m_Connection = new vrpn_Connection_IP();
	ARMM_server = new ARMM_Communicator(m_Connection);

	//Open the imager server and set up channel zero to send our data.
	//if ( (ARMM_img_server = new vrpn_Imager_Server("ARMM_Image", m_Connection, MESH_SIZE.width, MESH_SIZE.height)) == NULL) {
	//	fprintf(stderr, "Could not open imager server\n");
	//	return -1;
	//}
	//if ( (channel_id = ARMM_img_server->add_channel("Grid")) == -1) {
	//	fprintf(stderr, "Could not add channel\n");
	//	return -1;
	//}
	ARMM_server->SetObjectsData(&(m_world->m_objectsBody));
	ARMM_server->SetHandsData(&(m_world->HandObjectsArray));

  cout << "Created VRPN server." << endl;
	//<-----
#ifdef USE_ARMM_VRPN_RECEIVER 	//----->Receiver part
	ARMM_sever_receiver = new vrpn_Tracker_Remote (ARMM_CLIENT_IP);
	ARMM_sever_receiver->register_change_handler(NULL, handle_object);
#endif 	//<----- 

#endif

#ifdef USE_SKIN_SEGMENTATION	//Skin color look up
	_HandRegion.LoadSkinColorProbTable();
#endif

#ifdef USE_OPTICAL_FLOW
	prev_gray = cvCreateImage(cvSize(OPFLOW_SIZE.width, OPFLOW_SIZE.height), IPL_DEPTH_8U, 1);
	curr_gray = cvCreateImage(cvSize(OPFLOW_SIZE.width, OPFLOW_SIZE.height), IPL_DEPTH_8U, 1);
	flow_capture = new FlowCapture();
	flow_capture->Init();
#endif

#if USE_OSGMENU == 1
	AssignPhysics2Osgmenu();
	ResetAddModelMode();
	ResetPanelCond();
	ResetTextureTransferMode();
#endif

/////////////////////////////////////////////Main Loop////////////////////////////////////////////////
	while (running) 
	{
    //start kinect
		if (XnStatus rc = niContext.WaitAnyUpdateAll() != XN_STATUS_OK) 
		{
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return rc;
		}

		//get image and depth data from Kinect
		g_depth.GetMetaData(niDepthMD);
		g_image.GetMetaData(niImageMD);

		colourIm = cvCreateImage(cvSize(niImageMD.XRes(), niImageMD.YRes()), IPL_DEPTH_8U, 3);
		memcpy(colourIm->imageData, niImageMD.Data(), colourIm->imageSize); cvCvtColor(colourIm, colourIm, CV_RGB2BGR);
		cvFlip(colourIm, colourIm, 1);

		depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
		transDepth160 = cvCreateImage(cvSize(MESH_SIZE.width, MESH_SIZE.height), IPL_DEPTH_32F, 1);
		transDepth320 = cvCreateImage(cvSize(ARMM::ConstParams::SKIN_X, ARMM::ConstParams::SKIN_Y), IPL_DEPTH_32F, 1);
		transColor320 = cvCreateImage(cvSize(ARMM::ConstParams::SKIN_X, ARMM::ConstParams::SKIN_Y), IPL_DEPTH_8U, 3);
		memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);	
		//cvCircle(colourIm, cvPoint(marker_origin.x,marker_origin.y), 5, CV_BLUE, 3);
#ifdef SHOWKINECTIMG
		cvShowImage("Kinect View", colourIm);
#endif
		IplImage *arImage = capture->getFrame();
		cvWaitKey(1); 

		//check input device
		input_key = kc->check_input(pOsgRoot);
		ExecuteAction(input_key);

		//xc->check_input();
#ifdef USE_ARMM_VRPN_RECEIVER
		if( pass_key != 0){
			kc->check_input(pass_key);
			pass_key = 0;
		}
#endif

		// kinect transform as cvmat* for use
		if(kinectTransform) 
		{ 
			if( counter >= SIM_FREQUENCY) 
			{
#ifdef UPDATE_TRIMESH
				//TickCountAverageBegin();

				inpaintDepth(&niDepthMD, true); 
				memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);				
				TransformImage(depthIm, transDepth160, MARKER_DEPTH, MESH_SIZE, true);
				GenerateTrimeshGroundFromDepth(transDepth160, MARKER_DEPTH); /*Trimesh generation*/
				m_world->updateTrimeshRefitTree(ground_grid);//opencl?
				osg_UpdateHeightfieldTrimesh(ground_grid);//opencl?

				//TickCountAverageEnd();
#endif

#ifdef SIM_PARTICLES
/*World spheres simulation*/
//				GenerateVoxelFromDepth(depthIm, MARKER_DEPTH);
//				m_world->updateWorldSphereTransform(voxel_grid);
//				osgUpdateWorldSphereTransform(voxel_grid);
#endif
				counter = 0;
			} else {
#ifdef USE_SKIN_SEGMENTATION /*Skin color segmentation*/ // may be reduce resolution first as well as cut off depth make processing faster
				// (2)Sphere representation
				FindHands(depthIm, colourIm);
				//TickCountAverageBegin();
				UpdateAllHands();
				//TickCountAverageEnd();
#endif

#ifdef USE_PARTICLES
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#endif
				counter++;
			}
#if USE_OSGMENU == 1
			//(3) Checker for AR button input
			CheckerArInput();
			CheckerArModelButtonType();
#endif

			//do hand pose recognition
			//TickCountAverageBegin();
			m_world->Update();
			//TickCountAverageEnd();

			//(B)normal client only rendering
			RenderScene(arImage, capture);
		} /* if(kinectTransform) */
//		TickCountAverageEnd();

#ifdef USE_ARMM_VRPN
	//Send Car position+orientation			
	ARMM_server->mainloop();
#ifdef USE_ARMM_VRPN_RECEIVER
	ARMM_sever_receiver->mainloop();
#endif
	////Copy depth info
	//for (int i = 0; i < ARMM::ConstParams::GRID_SIZE;i++) {
	//	ARMM_img_buffer[i] = ground_grid[i];
	//}

	//Send depth grid info	
	//ARMM_img_server->send_begin_frame(0, MESH_SIZE.width-1, 0, MESH_SIZE.height-1);
 //   ARMM_img_server->mainloop();
 //   int nRowsPerRegion= ((int) vrpn_IMAGER_MAX_REGIONf32)/ MESH_SIZE.width;
 //   for(int y=0; y<MESH_SIZE.height; y+=nRowsPerRegion) {
 //     ARMM_img_server->send_region_using_base_pointer(channel_id,0,MESH_SIZE.width-1,y,min(MESH_SIZE.width,y+nRowsPerRegion)-1, ARMM_img_buffer, 1, MESH_SIZE.width, MESH_SIZE.height);
 //     ARMM_img_server->mainloop();
 //   }
 //   ARMM_img_server->send_end_frame(0, MESH_SIZE.width-1, 0, MESH_SIZE.height-1);
 //   ARMM_img_server->mainloop();


	//<--Exec data transmission
	//TickCountAverageBegin();
	m_Connection->mainloop();
	//TickCountAverageEnd();
#endif

#ifdef USE_OPTICAL_FLOW
		if(!RunOnce) RunOnce = true;
		cvCopyImage(curr_gray, prev_gray);
#endif

		cvReleaseImage(&arImage);
		cvReleaseImage(&depthIm); 
		cvReleaseImage(&colourIm);
		cvReleaseImage(&transDepth160);
#ifdef USE_SKIN_SEGMENTATION
		cvReleaseImage(&transDepth320);
		cvReleaseImage(&transColor320);
#endif
	}
#ifdef USE_OPTICAL_FLOW
	cvReleaseImage(&prev_gray); cvReleaseImage(&curr_gray);
#endif

	//memory release
	delete m_world;
	delete kinectReg;
	cvReleaseMat(&RegistrationParams);
	delete kc;
	//delete xc;

	return 0;
}

void RenderScene(IplImage *arImage, Capture *capture) 
{
	float scale = 10;
	osg::Vec3d worldVec;
#if CAR_SIMULATION == 1
	for(int i = 0; i < NUM_CARS; i++) {
		//get cars position and orientation
		btTransform trans = m_world->getCarPose(i);
		btQuaternion quat = trans.getRotation();
		CarsOrientation[i] = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW()); 
		CarsPosition[i]	   = osg::Vec3d(trans.getOrigin().getX()*scale, trans.getOrigin().getY()*scale,trans.getOrigin().getZ()*scale);
		//get wheels of each car position and orientation
		for(int j = 0; j < 4; j++) {
			btTransform trans_tmp = m_world->getWheelTransform(i,j);
			btQuaternion quattmp = trans_tmp.getRotation();
			WheelsOrientaion[i][j] = osg::Quat(quattmp.getX(), quattmp.getY(), quattmp.getZ(), quattmp.getW()); 	
			WheelsPosition[i][j] = osg::Vec3d(trans_tmp.getOrigin().getX()*scale, trans_tmp.getOrigin().getY()*scale,trans_tmp.getOrigin().getZ()*scale);
		}
	}
#endif /* CAR_SIMULATION == 1 */

	std::vector <osg::Quat> quat_obj_array;
	std::vector <osg::Vec3d> vect_obj_array;
	
	const int num_of_objects = pOsgRoot->mOsgObject->getVirtualObjectsCount();
	if(num_of_objects > 0) 
	{
		if(!objVectorDeletable.empty())
		{
			for(int i= objVectorDeletable.size()-1; i>=0; i--)
			{	
				DeleteVirtualObject(objVectorDeletable[i]);
			}
			objVectorDeletable.clear();
		}

		if( interact_state == TEXTUREGET)
		{
			interact_state = KEEP;
			cout << "Interaction state changed TEXTUREGET to KEEP" << endl;

			//TODO 別の場所で生成するように書き換え
			//add soft texture object into the environment
			osgAddObjectNode(m_world->CreateSoftTexture("Data/tex.bmp"));
			cout << "create the soft body object" << endl;
		}

		for(int i = 0; i < num_of_objects; i++) 
		{
			btTransform trans2 = m_world->get_Object_Transform(i);
			btQuaternion quat2 = trans2.getRotation();
			quat_obj_array.push_back(osg::Quat(quat2.getX(), quat2.getY(), quat2.getZ(), quat2.getW())); 
			vect_obj_array.push_back(osg::Vec3d(trans2.getOrigin().getX()*scale, trans2.getOrigin().getY()*scale,trans2.getOrigin().getZ()*scale));		

			////for debug for collision
			//if(collisionInd >= 444){
			//	osg::Node * node = obj_node_array.at(0);		
			//	osg::NodePathList paths = node->getParentalNodePaths();
			//	osg::Matrix world2local = osg::computeWorldToLocal(paths.at(paths.size()-1));
			//	osg::Vec3d localVec = worldVec * world2local;
			//	//printf("(%d)%f,%f,%f -- %f,%f,%f\n",obj_node_array.size(),localVec.x(), localVec.y(), localVec.z(), vect_obj_array[0].x(), vect_obj_array[0].y(), vect_obj_array[0].z());
			//}
		}
#if CAR_SIMULATION == 1
		osg_render(arImage, CarsOrientation, CarsPosition, WheelsOrientaion, WheelsPosition, RegistrationParams, capture->getDistortion(), quat_obj_array, vect_obj_array);
#else 
		osg_render(arImage, NULL, NULL, NULL, NULL, RegistrationParams, capture->getDistortion(), quat_obj_array, vect_obj_array);
#endif /* CAR_SIMULATION == 1 */
	} 

	else // Virtual_Objects_Count <= 0
	{
#if CAR_SIMULATION == 1
		osg_render(arImage, CarsOrientation, CarsPosition, WheelsOrientaion, WheelsPosition, RegistrationParams, capture->getDistortion(), quat_obj_array, vect_obj_array);
#else 
		osg_render(arImage, NULL, NULL, NULL, NULL, RegistrationParams, capture->getDistortion(), quat_obj_array, vect_obj_array);
#endif /* CAR_SIMULATION == 1 */
	}
}


bool loadKinectParams(char *filename, CvMat **params, CvMat **distortion) 
{
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==0) return false; 

	CvFileNode* fileparams;
	//Read the Camera Parameters
	fileparams = cvGetFileNodeByName( fs, NULL, "camera_matrix" );
	*params = (CvMat*)cvRead( fs, fileparams );

	//Read the Camera Distortion 
	fileparams = cvGetFileNodeByName( fs, NULL, "distortion_coefficients" );
	*distortion = (CvMat*)cvRead( fs, fileparams );
	cvReleaseFileStorage( &fs );

	return true;
}

void loadKinectTransform(char *filename) 
{
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );

	if (fs !=0 ) 
	{
		CvSeq *s = cvGetFileNodeByName(fs, 0, "MarkerSize")->data.seq;
		markerSize.width = cvReadInt((CvFileNode*)cvGetSeqElem(s, 0));
		markerSize.height = cvReadInt((CvFileNode*)cvGetSeqElem(s, 1));
		printf("Makerker Size = %d\n", markerSize.width);
		s = cvGetFileNodeByName(fs, 0, "MarkerOrigin")->data.seq;
		marker_origin.x = cvReadInt((CvFileNode*)cvGetSeqElem(s, 0));
		marker_origin.y = cvReadInt((CvFileNode*)cvGetSeqElem(s, 1));
		setWorldOrigin();
		WORLD_SCALE = cvReadRealByName(fs, 0, "WorldScale", 1);
		WORLD_ANGLE = cvReadRealByName(fs, 0, "WorldAngle", 0);
		MARKER_DEPTH = cvReadRealByName(fs, 0, "MARKER_DEPTH", 0);

		CvFileNode* fileparams = cvGetFileNodeByName( fs, NULL, "KinectTransform" );
		kinectTransform = (CvMat*)cvRead( fs, fileparams );
		cvReleaseFileStorage( &fs );

		if (niContext.WaitAnyUpdateAll() == XN_STATUS_OK) 
		{
			//Load in the marker for registration
			osg_inittracker(MARKER_FILENAME, 400, markerSize.width);

			m_world->setWorldDepth(MARKER_DEPTH);
			m_world->setWorldScale(WORLD_SCALE);
			setOSGTrimeshScale(WORLD_SCALE);

			g_depth.GetMetaData(niDepthMD);
			inpaintDepth(&niDepthMD, true);
			depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
			transDepth160 = cvCreateImage(cvSize(MESH_SIZE.width, MESH_SIZE.height), IPL_DEPTH_32F, 1);
			memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);	

			TransformImage(depthIm, transDepth160, MARKER_DEPTH, MESH_SIZE, true);
			GenerateTrimeshGroundFromDepth(transDepth160, MARKER_DEPTH);
			m_world->updateTrimesh(ground_grid);
			m_world->setMinHeight(MinHeight);
			m_world->setMaxHeight(MaxHeight);
			m_world->initPhysics();
#ifdef SIM_PARTICLES
			CreateOSGSphereProxy();//osg spheres representation
#endif
#ifdef SIM_MICROMACHINE
			m_world->resetCarScene(0);
			m_world->resetCarScene(1);
#endif /*SIM_MICROMACHINE*/
		}
	}
}

void TransformImage(IplImage* src_img, IplImage* dst_img, float markerDepth, CvSize img_size, bool isDepth) {

	float ground_depth = (float)markerDepth/10;
	int Depth, Channels;
	if(isDepth) {
		Depth = IPL_DEPTH_32F;
		Channels = 1;
	} else {
		Depth = IPL_DEPTH_8U;
		Channels = 3;
	}
	IplImage *tmp_img = cvCreateImage(cvGetSize(src_img), Depth, Channels);//problem using cvthres with 64F
	IplImage *lowerResImg1 = cvCreateImage(cvSize(img_size.width,img_size.height), Depth, Channels);
	IplImage *lowerResImg2 = cvCreateImage(cvSize(img_size.width,img_size.height), Depth, Channels);
	if(isDepth) 
		cvConvertScale(src_img, tmp_img, .1);//convert img from 16U to 32F and mm to cm
	else
		cvCopyImage(src_img, tmp_img);
	cvResize(tmp_img, lowerResImg1, CV_INTER_NN);//use nearest neighbor interpolation
	if(isDepth) {
		cvSubRS(lowerResImg1, cvScalar(ground_depth), lowerResImg1);
		for (int i = 0; i < img_size.height; i++) {
			for (int j = 0; j < img_size.width; j++) {
				if (i < 5 || j < 3)
					CV_IMAGE_ELEM(lowerResImg1, float, i, j) = 0;
			}
		}
	}
	int scale =  (int) img_size.width/MESH_SIZE.width;
	cvSetZero(lowerResImg2);
	CvMat* rot_mat = cvCreateMat(2,3,CV_32FC1);
	cv2DRotationMatrix( cvPoint2D32f(WORLD_ORIGIN_X*scale,WORLD_ORIGIN_Y*scale), WORLD_ANGLE, 1, rot_mat );//correct back
	cvWarpAffine( lowerResImg1, lowerResImg2, rot_mat );
	//if(isDepth && img_size.width == MESH_SIZE.width) cvShowImage("Depth Transformed", lowerResImg2); else cvShowImage("Color Transformed", lowerResImg2);
	cvCopyImage(lowerResImg2, dst_img);
	cvReleaseImage(&tmp_img);
	cvReleaseImage(&lowerResImg1);
	cvReleaseImage(&lowerResImg2);		

}

void GenerateTrimeshGroundFromDepth(IplImage* depthIm, float markerDepth) 
{
	float ground_depth = (float)markerDepth/10;
	IplImage * depthtmp = cvCreateImage(MESH_SIZE, IPL_DEPTH_8U, 1);
	//cvConvertScale(depthmask_for_mesh,depthtmp);
	MaxHeight = ground_depth-40;
	MinHeight = -MaxHeight;

	//reverse depth mask
	cvThreshold(depthmask_for_mesh, depthtmp, 0, 255, CV_THRESH_BINARY_INV);
	cvDilate(depthtmp,depthtmp, NULL, 5);
	
	//set cvScalar(0) in hand region in depth image
	cvSet(depthIm, cvScalar(0), depthtmp);

	float val = 0;
	for (int i = 0; i < MESH_SIZE.width; i++) {
		for (int j = 0; j < MESH_SIZE.height; j++) {
			int index = j*MESH_SIZE.width+i;
			//val = CV_IMAGE_ELEM(lowerResDepth2, float, 119-j, i);
			val = CV_IMAGE_ELEM(depthIm, float, (MESH_SIZE.height-1)-j, i);
			if(val > MaxHeight){
				val = MaxHeight;
			}
			else if(val < 0.001 && -0.001 < val){
				val = ground_grid[index];
			}
			else if(val < MinHeight){
				val = MinHeight;
			}
			ground_grid[index] =  val;
		}
	}
	//cvShowImage("depth",depthtmp);
	//cvShowImage("depth2",depthIm);
	cvReleaseImage(&depthtmp);
}

void inpaintDepth(DepthMetaData *niDepthMD, bool halfSize)
{
	IplImage *depthIm, *depthImFull;

	const float div = 4.0;

	if (halfSize) {
		depthImFull = cvCreateImage(cvSize(niDepthMD->XRes(), niDepthMD->YRes()), IPL_DEPTH_16U, 1);
		depthImFull->imageData = (char*)niDepthMD->WritableData();
		depthIm = cvCreateImage(cvSize(depthImFull->width/div, depthImFull->height/div), IPL_DEPTH_16U, 1);
		cvResize(depthImFull, depthIm, 0);
	} else {
		depthIm = cvCreateImage(cvSize(niDepthMD->XRes(), niDepthMD->YRes()), IPL_DEPTH_16U, 1);
		depthIm->imageData = (char*)niDepthMD->WritableData();
	}
	
	IplImage *depthImMask = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	for (int y=0; y<depthIm->height; y++) {
		for (int x=0; x<depthIm->width; x++) {
			CV_IMAGE_ELEM(depthImMask, char, y, x)=CV_IMAGE_ELEM(depthIm, unsigned short,y,x)==0?255:0;
		}
	}

	IplImage *depthImMaskInv = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	cvNot(depthImMask, depthImMaskInv);

	double min, max; cvMinMaxLoc(depthIm, &min, &max, 0, 0, depthImMaskInv);
	
	IplImage *depthIm8 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	float scale = 255.0/(max-min);
	cvConvertScale(depthIm, depthIm8, scale, -(min*scale));

	IplImage *depthPaint = cvCreateImage(cvGetSize(depthIm8), IPL_DEPTH_8U, 1);
	cvInpaint(depthIm8, depthImMask, depthPaint, 3, CV_INPAINT_NS);
	
	IplImage *depthIm16 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_16U, 1);
	cvConvertScale(depthPaint, depthIm16, 1/scale, min);

	if (halfSize) {
		IplImage *depthPaintedFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_16U, 1);
		cvResize(depthIm16, depthPaintedFull,0);
		IplImage *depthImMaskFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_8U, 1);
		for (int y=0; y<depthImFull->height; y++) for (int x=0; x<depthImFull->width; x++)
			CV_IMAGE_ELEM(depthImMaskFull, char, y, x)=CV_IMAGE_ELEM(depthImFull, unsigned short,y,x)==0?255:0;
		cvCopy(depthPaintedFull, depthImFull, depthImMaskFull);
		cvReleaseImage(&depthPaintedFull); cvReleaseImage(&depthImMaskFull);
		cvReleaseImage(&depthImFull);
	} else {
		cvCopy(depthIm16, depthIm, depthImMask);
	}

	cvReleaseImage(&depthIm8); cvReleaseImage(&depthIm16);
	cvReleaseImage(&depthPaint);
	cvReleaseImage(&depthImMask); cvReleaseImage(&depthImMaskInv);
	cvReleaseImage(&depthIm);
}

void setWorldOrigin() 
{
	const int div = 4;
	WORLD_ORIGIN_X = marker_origin.x/div; 
	WORLD_ORIGIN_Y = marker_origin.y/div; 
	center_trimesh = cvPoint2D32f(WORLD_ORIGIN_X, WORLD_ORIGIN_Y);
	m_world->set_center_trimesh(WORLD_ORIGIN_X,WORLD_ORIGIN_Y);
}

void registerMarker() 
{
	if (calcKinectOpenGLTransform(colourIm, depthIm, &kinectTransform)) 
	{
		//Load in the marker for registration
		osg_inittracker(MARKER_FILENAME, 400, markerSize.width);	
		printf("Makerker Size = %d\n", markerSize.width);
		//Set OSG Menu

		//Recreat world and controls
		delete kc;

		//delete xc;
		delete m_world;
		m_world = new bt_ARMM_world();
		kc = new KeyboardController(m_world);
		//xc = new XboxController(m_world);

		m_world->setWorldDepth(MARKER_DEPTH);
		m_world->setWorldScale(WORLD_SCALE);
		setOSGTrimeshScale(WORLD_SCALE);
		setWorldOrigin();
		transDepth160 = cvCreateImage(cvSize(MESH_SIZE.width, MESH_SIZE.height), IPL_DEPTH_32F, 1);
		TransformImage(depthIm, transDepth160, MARKER_DEPTH, MESH_SIZE, true);
		GenerateTrimeshGroundFromDepth(transDepth160, MARKER_DEPTH);
		m_world->updateTrimesh(ground_grid);
		m_world->setMinHeight(MinHeight);
		m_world->setMaxHeight(MaxHeight);
		m_world->initPhysics();
#ifdef SIM_PARTICLES
		CreateOSGSphereProxy();//osg spheres representation
#endif
#ifdef SIM_MICROMACHINE
		m_world->resetCarScene(0);
		m_world->resetCarScene(1);
#endif /*SIM_MICROMACHINE*/
	} else {
		printf("Couldn't find marker, please try again!\n");
	}
}

#ifdef USE_SKIN_SEGMENTATION
void FindHands(IplImage *depthIm, IplImage *colourIm) 
{
	//<----(Hand Segmentation) Count up
	//TickCountAverageBegin();

	//----->Transform both depth and color
	TransformImage(depthIm, transDepth320, MARKER_DEPTH, SKIN_SEGM_SIZE, true);
	TransformImage(colourIm, transColor320, MARKER_DEPTH, SKIN_SEGM_SIZE, false);

	//Create image storages
	IplImage* depthTmp		  = cvCreateImage(cvSize(SKIN_SEGM_SIZE.width, SKIN_SEGM_SIZE.height), IPL_DEPTH_8U, 1);
	IplImage* colourImResized = cvCreateImage(cvSize(SKIN_SEGM_SIZE.width, SKIN_SEGM_SIZE.height), IPL_DEPTH_8U, 3);
	IplImage* depthImResized  = cvCreateImage(cvSize(SKIN_SEGM_SIZE.width, SKIN_SEGM_SIZE.height), IPL_DEPTH_32F, 1);
	IplImage* colourIm640	  = cvCreateImage(CAPTURE_SIZE, IPL_DEPTH_8U, 3);
	cvResize(transDepth320, depthImResized);
	cvSmooth(depthImResized, depthImResized, CV_MEDIAN, 3);

	//----->Threshold too near region from Kinect
	for ( int i = 0 ; i < transDepth320->height ; i ++ )
	{
		for ( int j = 0 ; j < transDepth320->width ; j ++ )
		{
			float depth_ =  CV_IMAGE_ELEM(transDepth320, float, i, j);

			// dist from ground is larger than 30cm => removing the pixel
			if( depth_ > 30 || depth_ <= 1)
			{
				CV_IMAGE_ELEM(transDepth320, float, i, j) = 0;
			}
		}
	}

	//----->Threshold at marker's depth
	cvThreshold(transDepth320, depthTmp, 0, 255, CV_THRESH_BINARY_INV); //thres at 1cm above marker
	cvResize(transColor320, colourImResized);
	cvSet(colourImResized, cvScalar(0), depthTmp);
	cvResize(colourImResized, colourIm640, CV_INTER_NN);

  //----->Segment skin color
	cont_num = MAX_NUM_HANDS;//up to MAX_NUM_HANDS contours
	cvCopyImage( _HandRegion.GetHandRegion( colourImResized, &cont_num,cont_boundbox, cont_boundbox2D, cont_center), depthTmp);
	cvThreshold(depthTmp, depthTmp, 0, 255, CV_THRESH_BINARY_INV);	
	cvSet(colourImResized, cvScalar(0), depthTmp); //apply Hand mask image to colour image
	cvSet(depthImResized,  cvScalar(0), depthTmp); //apply Hand mask image to depth image
	cvResize(depthTmp, depthmask_for_mesh, CV_INTER_LINEAR);

  //----->Display image
	IplImage* colourIm160 = cvCreateImage(cvSize(SKIN_SEGM_SIZE.width, SKIN_SEGM_SIZE.height), IPL_DEPTH_8U, 3);
	cvCopyImage(colourImResized,colourIm160);
	#ifdef USE_OPTICAL_FLOW
	cvCvtColor(colourIm160, curr_gray, CV_BGR2GRAY);
	#endif

	//----->Draw center of contour
	float *center_depth; 
	int numb_hands = cont_center.size();//tmp
	center_depth = new float[numb_hands];//tmp
	int num_hand_in_scene = cont_center.size();

	// correct skin image size to 160x120
	float skin_ratio = SKIN_SEGM_SIZE.width / MESH_SIZE.width;
	assert( fabs( (SKIN_SEGM_SIZE.height / MESH_SIZE.height) - skin_ratio ) < 0.1); 

	CvPoint upperLeft = cvPoint(0,0);
	CvPoint bottomRight = cvPoint(0,0);
	REP(i,num_hand_in_scene)
	{
		center_depth[i] = CV_IMAGE_ELEM(transDepth320, float, cont_center.at(i).y, cont_center.at(i).x);
		const int box_size = skin_ratio * Find_Num_Hand_Pixel(center_depth[i]);
		const int offset = (box_size-1)/2;

		//  (x2,y2)-----(x1,y2)
		//	   |           | 
		//  (x2,y1)-----(x1,y1)
		int x1 = cont_center.at(i).x + offset;//lower right corner
		int y1 = cont_center.at(i).y + offset;
		int x2 = cont_center.at(i).x - offset;//upper left corner
		int y2 = cont_center.at(i).y - offset;					
		curr_hands_corners[i] = cvPoint(x2, SKIN_Y-y1);
		bottomRight = cvPoint(x1,y1); 
		upperLeft   = cvPoint(x2,y2);

		//----->Sort all the points using nearest neighbor
	
	//temporary test 1 hand
	//----->Copy height info to grid and display
	//				for(int i = 0; i < num_hand_in_scene; i++) {
		IplImage* depth11		= cvCreateImage(cvSize(MIN_HAND_PIX, MIN_HAND_PIX), IPL_DEPTH_32F, 1);
		IplImage* depth11_2 = cvCreateImage(cvSize(MIN_HAND_PIX, MIN_HAND_PIX), IPL_DEPTH_32F, 1);

		cvSetImageROI(depthImResized, cvRect(x2, y2 ,box_size,box_size));

		cvResize(depthImResized, depth11, CV_INTER_NN);
		//cvSmooth(depth11, depth11_2, CV_MEDIAN, 3);

		//int resize_ratio_x = SKIN_SEGM_SIZE.width / MIN_HAND_PIX;
		//int resize_ratio_y = SKIN_SEGM_SIZE.height/ MIN_HAND_PIX;
		int depth_offset = 0.0; //depth offset
		for(int j = 0; j < MIN_HAND_PIX; j++)
		{
			for(int k = 0; k < MIN_HAND_PIX; k++) 
			{
				int ind = j * MIN_HAND_PIX + k;
				hand_depth_grids[i][ind] = CV_IMAGE_ELEM(depth11, float, ((int)MIN_HAND_PIX-1)-k, j) - depth_offset ;
			}
		}

		//char win_name[50];
		//sprintf(win_name,"XXXX %d", i);
		//IplImage* depth_tmp_640 = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F, 1);
		//cvResize(depth11_2, depth_tmp_640, CV_INTER_NN);
		//cvShowImage(win_name,depth_tmp_640);
		//cvReleaseImage(&depth_tmp_640);

		cvReleaseImage(&depth11_2); 
		cvReleaseImage(&depth11); 

		//----->Display
		char center_depth_print[50];
		sprintf(center_depth_print,"%.2lf cm", center_depth[i]);

		//for displaying distance from ground
		//CvFont font;//tmp
		//cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.3 * skin_ratio , 0.3 * skin_ratio , 0, 1);//tmp
		//cvPutText (colourIm160, center_depth_print,  cont_center.at(i), &font, cvScalar(0,255,0));

		//cvCircle(colourIm160, cont_center.at(i),2,cvScalar(255,0,0));
		//cvRectangle(colourIm160, cvPoint(x1,y1) ,cvPoint(x2,y2),cvScalar(0,0,255));
	    cvRectangle(transColor320, cvPoint(x1,y1) ,cvPoint(x2,y2),cvScalar(0,0,255));

		//HACK TODO you should change this ratio depended on hand depth
		curr_hands_ratio[0] = (float)Find_Num_Hand_Pixel(-1)/MIN_HAND_PIX;

		curr_hands_corners[i].x /= skin_ratio;
		curr_hands_corners[i].y /= skin_ratio;
		if(m_world->getTotalNumberHand() < num_hand_in_scene) 
		{
			CreateHand(curr_hands_corners[i].x, curr_hands_corners[i].y);
		}

		cvResetImageROI(transDepth320);
	}

	cont_boundbox.clear();
	cont_boundbox2D.clear();
	cont_center.clear();

	#ifdef USE_OPTICAL_FLOW
	if(RunOnce){
		flow_capture->Run(prev_gray, curr_gray, colourIm160);
	}
	#endif

	IplImage* col_640 = cvCreateImage(cvSize(SKIN_X, SKIN_Y), IPL_DEPTH_8U, 3);
	cvResize(colourIm160, col_640, CV_INTER_LINEAR);
	//for(int i = 0; i < num_hand_in_scene; i++) {
	//	cvCircle(col_640,cvPoint(curr_hands_corners[i].x, 160 - curr_hands_corners[i].y),5,cvScalar(0,255,255));
	//}
	//<-- (HAND SEGMENTATION) end
	//TickCountAverageEnd();

	//<---Finger detection
	//TickCountAverageBegin();
	vector< vector<cv::Point> > fingerTips;
	cv::Ptr<IplImage> grey_640 = cvCreateImage(cvGetSize(col_640), 8, 1);	
	cvCvtColor(col_640, grey_640, CV_BGR2GRAY);	
	cvThreshold(grey_640, grey_640, 1, 255, cv::THRESH_BINARY);
	DetectFingertips(grey_640, fingerTips);

	// Draw fingertips
	fingerIndex.clear();

	const float diffX = abs(upperLeft.x - bottomRight.x)/MIN_HAND_PIX;
	const float diffY = abs(upperLeft.y - bottomRight.y)/MIN_HAND_PIX;

	REP(i,fingerTips.size())
	{
		REP(j,fingerTips[i].size())
		{
			//check pixels matching fingertips
			REP(dy,MIN_HAND_PIX) REP(dx,MIN_HAND_PIX)
			{
				CvPoint tmpFingertips;
				tmpFingertips.x = upperLeft.x+dx*diffX;
				tmpFingertips.y = upperLeft.y+dy*diffY;
				if(tmpFingertips.x < 0 
				|| tmpFingertips.x >= SKIN_X
				|| tmpFingertips.y < 0
				|| tmpFingertips.y >= SKIN_Y
				){
					continue;
				}

				if( abs(fingerTips[i][j].x-tmpFingertips.x) <= 2
				&&  abs(fingerTips[i][j].y-tmpFingertips.y) <= 2)
				{
					//OSGで関連付けているIndexの走査方向によってIndexは定まる
					fingerIndex.push_back( dx*MIN_HAND_PIX + (MIN_HAND_PIX-1)-dy);
				}
			}
#ifdef SHOWSEGMENTATION
			cvCircle(col_640, fingerTips[i][j] , 10, cv::Scalar(255,255,0), 4);
#endif
		}
	}
	//printf("Upper=(%d,%d) Bottom=(%d,%d)\n",upperLeft.x, upperLeft.y, bottomRight.x, bottomRight.y);
	//printf("DiffX=%f, DiffY=%f\n",diffX, diffY);
	//cout << "Finger=" << fingerIndex.size() << endl;
	//<--(FINGERTIPS DETECTION) end
	//TickCountAverageEnd();
#ifdef SHOWSEGMENTATION
	cvShowImage("Op_Flow_640",col_640);
	cvShowImage("transcolor",transColor320);
#endif

	//memory release
	delete center_depth;
	cvReleaseImage(&col_640);
	cvReleaseImage(&colourIm160);
	cvReleaseImage(&depthTmp);
	cvReleaseImage(&colourImResized);
	cvReleaseImage(&depthImResized);
	cvReleaseImage(&colourIm640);
}
#endif

void DetectFingertips(cv::Ptr<IplImage> handMask, 
	vector< vector<cv::Point> > & fingerTips)
{
	fingerTips.clear();
	vector< vector<cv::Point> > contours;

	//cv::findContours(src, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	CvMemStorage* storage   = cvCreateMemStorage( 0 );     
	CvSeq* cs = NULL;    
	CvScalar black          = CV_RGB( 0, 0, 0 ); 
	CvScalar white          = CV_RGB( 255, 255, 255 ); 
	int contours_count= cvFindContours( handMask, storage, &cs, sizeof( CvContour ), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );  

	int loop = 0;
	contours.resize(contours_count);
    for( ; cs!= 0; cs= cs->h_next, loop++)
    {
        REP(i,cs->total)
        {
			CvPoint tmpPoint = *CV_GET_SEQ_ELEM( CvPoint, cs, i );
			contours[loop].push_back(cv::Point(tmpPoint));
        }

		cv::Mat contourMat = cv::Mat(contours[loop]);
		double area = cv::contourArea(contourMat);

		if (area > 200)  { // possible hand
			vector<cv::Point> tmp_fingertips;
			tmp_fingertips.clear();

			cv::Scalar center = mean(contourMat);
			cv::Point centerPoint = cv::Point(static_cast<int>(center.val[0]), static_cast<int>(center.val[1]) );
			vector<cv::Point> approxCurve;
			CvSeq *approx = cvApproxPoly(cs, sizeof(CvContour), NULL, CV_POLY_APPROX_DP, 15);
			//cv::approxPolyDP(contourMat, approxCurve, 15, true); //error part

			REP(i,approx->total)
			{
				CvPoint tmpPoint = *CV_GET_SEQ_ELEM( CvPoint, approx, i );
				approxCurve.push_back(cv::Point(tmpPoint));
			}

			vector<int> hull;
			vector<cv::Point> hullShape;
			//cv::convexHull(cv::Mat(approxCurve), hull);
			CvSeq* h = cvConvexHull2(approx,0,CV_CLOCKWISE,0);
			//cout << "hogehoge4" << endl;


			REP(i,h->total)
			{
				//CvPoint tmpPoint = *CV_GET_SEQ_ELEM( CvPoint, h, i ); //this code occurs assertion
				CvPoint tmpPoint = **CV_GET_SEQ_ELEM( CvPoint*, h, i ); //this one is okay?
				hullShape.push_back(cv::Point(tmpPoint));
			}

			int approxPointer = 0;
			REP(i,hullShape.size())
			{
				for(;approxPointer<approxCurve.size();approxPointer++)
				{
					if(hullShape[i] == approxCurve[approxPointer]){
						hull.push_back(approxPointer);
						break;
					}
				}
			}
			//cout << "Hull size = " << hull.size() << endl;
			// find upper and lower bounds of the hand and define cutoff threshold (don't consider lower vertices as fingers)
			int upper = 600, lower = 0;
			for (unsigned int j=0; j<hull.size(); j++) {
				int idx = hull[j]; // corner index
				if (approxCurve[idx].y < upper) upper = approxCurve[idx].y;
				if (approxCurve[idx].y > lower) lower = approxCurve[idx].y;
			}

			float cutoff = lower - (lower - upper) * 0.1f;
			// find interior angles of hull corners
			for (unsigned int j=0; j<hull.size(); j++) {
				int idx = hull[j]; // corner index
				int pdx = idx == 0 ? approxCurve.size() - 1 : idx - 1; //  predecessor of idx
				int sdx = idx == approxCurve.size() - 1 ? 0 : idx + 1; // successor of idx
				cv::Point v1 = approxCurve[sdx] - approxCurve[idx];
				cv::Point v2 = approxCurve[pdx] - approxCurve[idx];
				double angle = acos( (v1.x*v2.x + v1.y*v2.y) / (cv::norm(v1) * cv::norm(v2)) );
				// low interior angle + within upper 90% of region -> we got a finger
				if ( angle < 1 && approxCurve[idx].y < cutoff) {
					tmp_fingertips.push_back(cv::Point( approxCurve[idx].x , approxCurve[idx].y) );
				}
			}
			fingerTips.push_back(tmp_fingertips);
		}
	}
}

int Find_Num_Hand_Pixel(float depth) 
{
	float box_pix = (ARMM::ConstParams::HAND_BOX_CM/WORLD_SCALE+depth*ARMM::ConstParams::KINECT_PIX_PER_DEPTH);
	int ans;
	if(((int) ceil(box_pix)%2) == 0)
		ans = floor(box_pix);
	else
		ans = ceil(box_pix);
	//printf("pixel width = %d \n", ans);
	return ans;
}

#ifdef USE_SKIN_SEGMENTATION

int CreateHand(int lower_left_corn_X, int lower_left_corn_Y) 
{
	int index = m_world->getTotalNumberHand();

	ratio = curr_hands_ratio[0];
	m_world->createHand( lower_left_corn_X, lower_left_corn_Y, MIN_HAND_PIX, ratio);	
	osg_createHand(index, lower_left_corn_X, lower_left_corn_Y, WORLD_SCALE, ratio);
	return index; //return the index of current hand
}

void UpdateAllHands() 
{
	for(int i = 0; i < m_world->getTotalNumberHand();i++) 
	{
		//In this case, spheres are displayed on the bottom-left corner of the marker
		m_world->updateHandDepth(i, curr_hands_corners[i].x, curr_hands_corners[i].y, ratio, hand_depth_grids[i]);
		osg_UpdateHand(i, m_world->debugHandX(i), m_world->debugHandY(i), m_world->debugHandZ(i));
	}

}
#endif

#ifdef SIM_PARTICLES
void CreateOSGSphereProxy() {
	for(int i = 0; i < 30; i++) {
		for(int j = 0; j < 40; j++) {
			int index = i*40 + j;
			osgAddWorldSphereProxyNode(osgNodeFromBtSphere(m_world->getWorldSphereTransform(index)));
		}
	}
	printf("Spheres proxy created! \n");
}
#endif

//TODO osgクラスのメソッドに変更
void DeleteVirtualObject(const int & index)
{
	//vector<osg::ref_ptr<osg::PositionAttitudeTransform>>::iterator it = obj_transform_array.begin() + index;
	//vector<osg::ref_ptr<osg::Node>>::iterator it2 = obj_node_array.begin() + index;

	//if( shadowedScene->getNumChildren() <= index ||
	//	shadowedScene->getNumChildren() == 0)
	//{
	//	cerr << "Error: Out of range in ShadowedScene children arrays(DeleteVirtualObject)" << endl;
	//	return;
	//}
	//if(obj_transform_array.empty()) 
	//{
	//	cerr << "No object " << endl;
	//	return;
	//}
	//cout << shadowedScene->getNumChildren() << " " << index << endl;
	//shadowedScene->removeChild(obj_transform_array.at(index));
	//obj_transform_array.erase(it);
	//obj_node_array.erase(it2);
	//Virtual_Objects_Count--;
	//cout << index << "'s virtual objects LOST : Remain " << Virtual_Objects_Count << endl;
}

#if USE_OSGMENU == 1
void AssignPhysics2Osgmenu()
{
	m_world->CreateMenu(osgMenu);
	m_world->CreateModelButton(osgMenu);
}

void ResetAddModelMode()
{
	pOsgRoot->SetAddArModel(false);
	pOsgRoot->
	ResetModelButtonPos();
	m_world->ResetARButtonInput();
}

void ResetTextureTransferMode()
{
	bTextureTransfer = false;
}

void ResetPanelCond()
{
	panelCollisionLock	= false;
	panelInput			= NOTHING;
	osgArInputButton	= -1;
}

void CheckerArInput()
{
	if(osgArInputButton > 0)
	{
		string buttonStr = osgMenu->getObjMenuNodeArray().at(osgArInputButton)->getName(); 

		//set transmitted key input to client nodes
		//TODO : implementation network part
		input_key = kc->TransmitInput(osgArInputButton);

		//change panelInput state
		if(buttonStr.find("model.3ds") != string::npos)	//MODE: Add model
		{
			panelInput = ADDARMODEL;
			ModelButtonInput();
		}
		else if(buttonStr.find("reset.3ds") != string::npos)	//MODE: Reset virtual models
		{
			pOsgRoot->mOsgObject
			osg_resetNodes();
			ResetPanelCond();
		}
		else if(buttonStr.find("Transfer") != string::npos)	//MODE: Texture Transfer
		{
			if(m_world->m_objectsBody.size() < 2)
			{
				cerr << "No enough model is found : need two at least" << endl;
				ResetPanelCond();

				//play some effect
				PlaySound(_T("jump02.wav"), NULL, SND_ASYNC);	

			}
			//having two models at least in AR env
			else
			{
				bTextureTransfer = true;
				pOsgRoot->ToggleMenuVisibility();	//menu should be disappeared
			}
		}

		cout << buttonStr.c_str() << endl;

		//reset
		osgArInputButton = -1;		
	}
}

int CheckerArModelButtonType()
{
	if(osgArAddModelIndex > 0)
	{
		string touchStr = osgMenu->getMenuModelObjectArray().at(osgArAddModelIndex)->getName();
		cout << touchStr.c_str() << endl;

		//pushing cancel button
		if( touchStr.find("cancel") != string::npos )
		{
			input_key = 301;
			MessageBeep(MB_OK);
		}
		else
		{
			osg::Vec3 pos(osgMenu->getMenuModelTransArray().at(osgArAddModelIndex)->getPosition());

			input_key = osgMenu->GetKeyAssignment(static_cast<unsigned int>(osgArAddModelIndex));

			//create model unit with osg::Node
			osg::ref_ptr<osg::Node> node = dynamic_cast<osg::Node*>
			(
				osgMenu->getMenuModelObjectArray().at(osgArAddModelIndex)->clone(osg::CopyOp::SHALLOW_COPY)
			);

			//add physics model
			int index = m_world->create_3dsmodel(touchStr.c_str());

			float scale = 10;
			osgAddObjectNode(node.get());
			obj_transform_array.at(index)->setScale(osg::Vec3d(scale,scale,scale));
			obj_transform_array.at(index)->setNodeMask(invisibleMask);

			Virtual_Objects_Count++;

			m_world->ChangeAttribute(pos.x()/10, pos.y()/10, pos.z()/10, index);

			//play some effect
			PlaySound(_T("pickup04.wav"), NULL, SND_ASYNC);	
		}

		//reset
		ToggleMenuVisibility();
		ToggleModelButtonVisibility();
		ToggleVirtualObjVisibility();
		ResetAddModelMode();
		ResetPanelCond();
		osgArAddModelIndex = -1;
	}
	return 0;
}

void ExecuteAction(const int & val)
{
	if(val == VK_SPACE)
	{
		registerMarker();
	}
}

#endif