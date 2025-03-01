﻿//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "bt_ARMM_world.h"
#include "bt_ARMM_hand.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#include <windows.h>
#include <tchar.h>

#ifdef USE_PARALLEL_DISPATCHER
#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"

#ifdef USE_LIBSPE2
#include "BulletMultiThreaded/SpuLibspe2Support.h"
#elif defined (_WIN32)
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#elif defined (USE_PTHREADS)
#include "BulletMultiThreaded/PosixThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#else
//other platforms run the parallel code sequentially (until pthread support or other parallel implementation is added)
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif //USE_LIBSPE2

#ifdef USE_PARALLEL_SOLVER
#include "BulletMultiThreaded/btParallelConstraintSolver.h"
#include "BulletMultiThreaded/SequentialThreadSupport.h"

btThreadSupportInterface* createSolverThreadSupport(int maxNumThreads) {
#ifdef _WIN32
	Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("solverThreads",SolverThreadFunc,SolverlsMemoryFunc,maxNumThreads);
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(threadConstructionInfo);
	threadSupport->startSPU();
#endif
	return threadSupport;
}
#endif //USE_PARALLEL_SOLVER

#endif//USE_PARALLEL_DISPATCHER

//OSGBullet added by Atsushi.U 2012/07/28
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>
#include <osg/PositionAttitudeTransform>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/LaunchHandler.h>
#include <osgbInteraction/SaveRestoreHandler.h>

#include <osgwTools/Shapes.h>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>

#include <osg/io_utils>

//C++ include
#include <iostream>

//OpenCV
#include "opencv/cv.h"
#include "opencv/highgui.h"

//for loading model
#include "../ViewingModel.h"


#include "src\constant.h"

using namespace std;

//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
//#define USE_PARALLEL_SOLVER 1 //experimental parallel solver
//#define USE_PARALLEL_DISPATCHER 1
#define CUBE_HALF_EXTENTS 1
#define SIM_MICROMACHINE 1
//#define SIM_PARTICLES 1

const float SPHERE_SIZE_ = 2;//FULL SIZE
const float CUBE_SIZE_ = 2;//half_size
const int REGULAROBJECTNUM = 21*21+3;

const int maxProxies = 32766;
const int maxOverlap = 65535;

//for bullet
const int SHAPE_BOX    = 0;
const int SHAPE_CONVEX = 4;
const int SHAPE_SPHERE = 8;

#define REP(i,n) for(int i=0;i<(int)n;++i)

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
int collide_counter = 0, panel_counter = 0;
double prev_collide_clock = 0.0, prev_panel_clock = .0;
extern int Virtual_Objects_Count;
extern vector<int> objVectorDeletable;
extern vector<int> fingersIdx;

int		collide[2];       //indices  of collided objects
btVector3	pCollision;  //coordinate with a collided object
btVector3 pColLocalOnObj;
int		selector = 0;
interaction interact_state = INIT;
bool	touch; //(true)touch  (false)not touch

btCollisionObject * actualCollisionObject = NULL; //for collision detection
int actualID = -1;
int selectID;

int collisionInd = -1;
bool changeCollisionObject = false; //手と物体の間の衝突判定で手のほうを受け取らないようにするためのもの

btVector3 btHandPos;
btScalar _timeStep;

osg::Vec3d softTexture_array[resX*resY];
osg::Geometry* softGeom;

extern bool panelCollisionLock;
extern panelinput panelInput;
extern bool bTextureTransfer;

bool bSelectModel;
extern int osgArInputButton;
extern int osgArAddModelIndex;

std::ostream& operator<<(std::ostream& out, const btVector3 & v) 
{
	return out << v.getX() << " " << v.getY() << " " << v.getZ() ;
}

/** Create Collision Callback  **/
struct ContactSensorCallback : public btDiscreteDynamicsWorld::ContactResultCallback
{	
	//! Constructor, pass whatever context you want to have available when processing contacts
	/*! You may also want to set m_collisionFilterGroup and m_collisionFilterMask
	 *  (supplied by the superclass) for needsCollision() */
	ContactSensorCallback()
		: btDiscreteDynamicsWorld::ContactResultCallback()
	{}
	
	//! Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
	//! indexはよくわからない値が入っていた
	//! partIdは両方とも0が入ってることがほとんどだった
	virtual btScalar addSingleResult(btManifoldPoint& cp,
		const btCollisionObject* colObj0,int partId0,int index0,
		const btCollisionObject* colObj1,int partId1,int index1)
	{
		double current_clock = static_cast<double>(cv::getTickCount());
		double time_spent = ( current_clock - prev_collide_clock ) / cv::getTickFrequency();

		touch = true;
		// collision frequency is set to at least 3.0 sec upward
		//if(true){
		if( time_spent > 0.1)
		{

			switch(interact_state)
			{
				case INIT:
					interact_state = STROKE1;
					break;

			}
			//rewrite previous collision time
			prev_collide_clock = current_clock;

			//get collision position on each object
			btVector3 tmp1 = cp.getPositionWorldOnA();
			btVector3 tmp2 = cp.getPositionWorldOnB();
			
			//cout << "COLLIDE : " << collide_counter << endl;

			if(!changeCollisionObject)
			{
				pColLocalOnObj = colObj0->getWorldTransform().getOrigin() - tmp1;
				pCollision = tmp1 * 10;
				//cout << "pColWorldOn " << colObj0->getWorldTransform().getOrigin() << endl;
				//cout << "pCollision " << tmp1 << endl;
				//TODO const_castを使わない方法
				// 現在衝突しているオブジェクトのポインタを保存
				actualCollisionObject = const_cast<btCollisionObject * >(colObj0);

				osg::Vec3 pos = osgbCollision::asOsgVec3( cp.getPositionWorldOnA() ); // position of the collision on object A
				//osg::notify( osg::ALWAYS ) << "\tPosition: " << pos << std::endl;

			}
			else
			{
				pColLocalOnObj = colObj1->getWorldTransform().getOrigin() - tmp2;
				pCollision = tmp2 * 10;
				//cout << "pColWorldOn " << colObj1->getWorldTransform().getOrigin() << endl;
				//cout << "pCollision " << tmp2 << endl;
				//TODO const_castを使わない方法
				// 現在衝突しているオブジェクトのポインタを保存
				actualCollisionObject = const_cast<btCollisionObject * >(colObj1);

				osg::Vec3 pos = osgbCollision::asOsgVec3( cp.getPositionWorldOnB() ); // position of the collision on object B
				//osg::notify( osg::ALWAYS ) << "\tPosition: " << pos << std::endl;
			}

			collide_counter++;
		}

		return 0; // not actually sure if return value is used for anything...?
	}
};
/** create Collision Callback **/

/** Create Collision Callback  **/
struct PinchContactSensorCallback : public btDiscreteDynamicsWorld::ContactResultCallback
{	
	//! Constructor, pass whatever context you want to have available when processing contacts
	/*! You may also want to set m_collisionFilterGroup and m_collisionFilterMask
	 *  (supplied by the superclass) for needsCollision() */
	PinchContactSensorCallback ()
		: btDiscreteDynamicsWorld::ContactResultCallback()
	{}
	
	//! Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
	//! indexはよくわからない値が入っていた
	//! partIdは両方とも0が入ってることがほとんどだった
	virtual btScalar addSingleResult(btManifoldPoint& cp,
		const btCollisionObject* colObj0,int partId0,int index0,
		const btCollisionObject* colObj1,int partId1,int index1)
	{	
		//cout << "Pinch interection" << endl;
		//colObj0->setWorldTransform(colObj1->getWorldTransform());
		return 0; // not actually sure if return value is used for anything...?
	}
};
/** create Collision Callback **/

/** Create Collision Callback abouttouching the panel**/
struct PanelContactSensorCallback : public btDiscreteDynamicsWorld::ContactResultCallback
{	
	//! Constructor, pass whatever context you want to have available when processing contacts
	/*! You may also want to set m_collisionFilterGroup and m_collisionFilterMask
	 *  (supplied by the superclass) for needsCollision() */
	PanelContactSensorCallback()
		: btDiscreteDynamicsWorld::ContactResultCallback()
	{}
	
	//! Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
	//! indexはよくわからない値が入っていた
	//! partIdは両方とも0が入ってることがほとんどだった
	virtual btScalar addSingleResult(btManifoldPoint& cp,
		const btCollisionObject* colObj0,int partId0,int index0,
		const btCollisionObject* colObj1,int partId1,int index1)
	{	
		double current_clock = static_cast<double>(cv::getTickCount());
		double time_spent = ( current_clock - prev_panel_clock ) / cv::getTickFrequency();
		prev_panel_clock = current_clock;

		panelCollisionLock = true;

		//なぜかstaticオブジェクトの判定が
		//colObj0 と colObj1で逆転していた
		if(colObj0->isStaticObject())
		{
			//temporary ID variable
			actualID = static_cast<int>(colObj1->getFriction());
		}
		else if(colObj1->isStaticObject())
		{
			//temporary ID variable
			actualID = static_cast<int>(colObj0->getFriction());
		}
		else
		{
			cout << "No static object with panel interaction" << endl;
		}

		return 0; // not actually sure if return value is used for anything...?
	}
};

/** Create Collision Callback abouttouching the panel**/
struct SelectContactSensorCallback : public btDiscreteDynamicsWorld::ContactResultCallback
{	
	//! Constructor, pass whatever context you want to have available when processing contacts
	/*! You may also want to set m_collisionFilterGroup and m_collisionFilterMask
	 *  (supplied by the superclass) for needsCollision() */
	SelectContactSensorCallback()
		: btDiscreteDynamicsWorld::ContactResultCallback()
	{}
	
	//! Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
	//! indexはよくわからない値が入っていた
	//! partIdは両方とも0が入ってることがほとんどだった
	virtual btScalar addSingleResult(btManifoldPoint& cp,
		const btCollisionObject* colObj0,int partId0,int index0,
		const btCollisionObject* colObj1,int partId1,int index1)
	{	
		//パネルに同時に触れることを防止するためにパネルに触れた判定にする
		double current_clock = static_cast<double>(cv::getTickCount());
		prev_panel_clock = current_clock;

		bSelectModel= true;

		//なぜかstaticオブジェクトの判定が
		//colObj0 と colObj1で逆転していた
		if(colObj0->isStaticObject())
		{
			//temporary ID variable
			selectID = static_cast<int>(colObj1->getFriction());
		}
		else if(colObj1->isStaticObject())
		{
			//temporary ID variable
			selectID = static_cast<int>(colObj0->getFriction());
		}
		else
		{
			cout << "No static object with panel interaction" << endl;
		}

		return 0; // not actually sure if return value is used for anything...?
	}
};

struct MeshUpdater : public osg::Drawable::UpdateCallback
{
    MeshUpdater(  btSoftBody* softBody, const unsigned int size )
      : _softBody( softBody ),
        _size( size )
    {}
    virtual ~MeshUpdater()
    {}

    virtual void update( osg::NodeVisitor*, osg::Drawable* draw )
    {
        osg::Geometry* geom( draw->asGeometry() );
        osg::Vec3Array* verts( dynamic_cast< osg::Vec3Array* >( geom->getVertexArray() ) );

        // Update the vertex array from the soft body node array.
        btSoftBody::tNodeArray& nodes = _softBody->m_nodes;
        osg::Vec3Array::iterator it( verts->begin() );
        unsigned int idx = 0;
		
		idx = _size/2;
		btVector3 delta = (btHandPos*10 - nodes[idx].m_x);		

		static const btScalar	maxdrag=10;
		if(delta.length2()>(maxdrag*maxdrag))
		{
			delta=delta.normalized()*maxdrag;
		}

		nodes[idx].m_v *= 0;
		nodes[idx].m_v += (delta/_timeStep)*50;
		//printf("nodes=(%f,%f,%f)\n",nodes[idx].m_x.getX(),nodes[idx].m_x.getY(),nodes[idx].m_x.getZ());

		//nodes[idx].m_x.setValue(btHandPos.x()*10, btHandPos.y()*10, btHandPos.z()*10);
		//goal.setValue(btHandPos.x()*10, btHandPos.y()*10, btHandPos.z()*10);

		for( idx=0; idx<_size; idx++)
        {
			softTexture_array[idx] = osgbCollision::asOsgVec3( nodes[ idx ].m_x );
            *it++ = softTexture_array[idx];
			//*it++ = osg::Vec3d(nodes[ idx ].m_x.getX(), nodes[ idx ].m_x.getY(), nodes[ idx ].m_x.getZ());
        }
		verts->dirty();
        draw->dirtyBound();

        // Generate new normals.
        osgUtil::SmoothingVisitor smooth;
        smooth.smooth( *geom );
        geom->getNormalArray()->dirty();
    }

    btSoftBody* _softBody;
    const unsigned int _size;
};
/** \endcond */



void pickingPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	//printf("%lf\n",timeStep);
	_timeStep = timeStep;
}

btVector3 worldMin(-1000,-1000,-1000);
btVector3 worldMax(1000,1000,1000);
ContactSensorCallback callback;
PinchContactSensorCallback pinchcallback;
PanelContactSensorCallback panelcallback;
SelectContactSensorCallback selectcallback;

namespace{
	double GetMaxNumber(const double & x, const double & y, const double & z){
		if(x>y){
			if(x>z){
				return x;
			}else{
				return z;
			}
		}else{
			if(y>z){
				return y;
			}else{
				return z;
			}
		}
	}
}



//---------------------------------------------------------------------------
// Code	
//---------------------------------------------------------------------------
bt_ARMM_world::bt_ARMM_world(void)
	:m_carChassis(0),m_indexVertexArrays(0),m_vertices(0), m_sphere_rep(true)
{

	hasInit = false;
	touch   = false;

	//initialize flags for detecting collision
	REP(i,2)
	{
		collide[i] = 0;
	}

	count = 0;
	rightIndex = 0; 
	upIndex = 2; 
	forwardIndex = 1;

#if CAR_SIMULATION == 1

	wheelDirectionCS0[0] = btVector3(0,0,-1);
	wheelAxleCS[0] = btVector3(1,0,0);
	suspensionRestLength[0] = btScalar(0.6);

	wheelDirectionCS0[1] = btVector3(0,0,-1);
	wheelAxleCS[1] = btVector3(1,0,0);
	suspensionRestLength[1] = btScalar(0.6);

	Car_Array[0].chassis_width = 1.f;
	Car_Array[0].chassis_length = 2.8f;
	Car_Array[0].chassis_height = 0.7f;
	Car_Array[0].car_mass = 250;
	Car_Array[0].gEngineForce = 0.f;
	Car_Array[0].gBreakingForce = 0.f;
	Car_Array[0].maxEngineForce = 2500.f;//this should be engine/velocity dependent //1000
	Car_Array[0].maxBreakingForce = 200.f;//100
	Car_Array[0].defaultBreakingForce = 50.f;//10
	Car_Array[0].gVehicleSteering = 0.f;
	Car_Array[0].steeringIncrement = 0.2f;//0.04f
	Car_Array[0].steeringClamp = 0.5f;//0.3f
	Car_Array[0].wheelRadius = 0.5f;
	Car_Array[0].wheelWidth = 0.4f;
	Car_Array[0].wheelFriction = 1000;//BT_LARGE_FLOAT;
	Car_Array[0].suspensionStiffness = 20.f;
	Car_Array[0].suspensionDamping = 2.3f;
	Car_Array[0].suspensionCompression = 4.4f;
	Car_Array[0].rollInfluence = 1.0f;//0.1f;
	Car_Array[0].chassisDistFromGround = 1.3f;
	Car_Array[0].connectionHeight = 1.3f;
	Car_Array[0].wheelLengthOffsetFront = 1.1f;
	Car_Array[0].wheelLengthOffsetBack = 1.2f;
	Car_Array[0].wheelWidthOffsetFront = 1.1f;
	Car_Array[0].wheelWidthOffsetBack = 1.1f;

	Car_Array[1].chassis_width = 1.3f;
	Car_Array[1].chassis_length = 2.8f;
	Car_Array[1].chassis_height = 0.5f;
	Car_Array[1].car_mass = 250;
	Car_Array[1].gEngineForce = 0.f;
	Car_Array[1].gBreakingForce = 0.f;
	Car_Array[1].maxEngineForce = 2500.f;//this should be engine/velocity dependent //1000
	Car_Array[1].maxBreakingForce = 200.f;//100
	Car_Array[1].defaultBreakingForce = 50.f;//10
	Car_Array[1].gVehicleSteering = 0.f;
	Car_Array[1].steeringIncrement = 0.2f;//0.04f
	Car_Array[1].steeringClamp = 0.5f;//0.3f
	Car_Array[1].wheelRadius = 0.5f;
	Car_Array[1].wheelWidth = 0.4f;
	Car_Array[1].wheelFriction = 1000;//BT_LARGE_FLOAT;
	Car_Array[1].suspensionStiffness = 20.f;
	Car_Array[1].suspensionDamping = 2.3f;
	Car_Array[1].suspensionCompression = 4.4f;
	Car_Array[1].rollInfluence = 1.0f;//0.1f;
	Car_Array[1].chassisDistFromGround = 1.3f;
	Car_Array[1].connectionHeight = 1.2f;
	Car_Array[1].wheelLengthOffsetFront = 1.1f;
	Car_Array[1].wheelLengthOffsetBack = 1.2f;
	Car_Array[1].wheelWidthOffsetFront = 1.3f;
	Car_Array[1].wheelWidthOffsetBack = 1.2f;

#endif /* CAR_SIMULATION == 1*/

	worldDepth = 0;
	m_rawHeightfieldData = new float[ARMM::ConstParams::GRID_SIZE];
	HF_Size = ARMM::ConstParams::GRID_SIZE;

	textureSoftBody = 0;

	ResetARButtonInput();
}

bt_ARMM_world::~bt_ARMM_world(void) { //cleanup in the reverse order of creation/initialization
	//remove the rigidbodies from the dynamics world and delete them
	int i;

	if (hasInit)
	{
		for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) 
		{
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState()) {
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject( obj );
			delete obj;
		}
#ifdef SIM_MICROMACHINE
		chassisMotionState.clear();
#endif /*SIM_MICROMACHINE*/
		//delete collision shapes
		for (int j=0;j<m_collisionShapes.size();j++) 
		{
			delete m_collisionShapes[j];
		}
		m_collisionShapes.clear();

		Convex_Shape.clear();
		m_objectsBody.clear();
		HandObjectsArray.clear();
		HandFingersArray.clear();

		delete m_indexVertexArrays;
		delete m_vertices;
		delete m_dynamicsWorld;
#if CAR_SIMULATION == 1
		chassisMotionState.clear();
		m_carChassis.clear();
		m_vehicleRayCaster.clear();
		m_vehicle.clear();
		m_wheelShape.clear();
#endif /*SIM_MICROMACHINE*/
		delete m_constraintSolver;
		delete m_overlappingPairCache; //delete broadphase
		delete m_dispatcher; //delete dispatcher
		delete m_collisionConfiguration;
	}
}

void bt_ARMM_world::initPhysics() 
{
#ifdef USE_PARALLEL_DISPATCHER
	m_threadSupportSolver = 0;
	m_threadSupportCollision = 0;
#endif

	//set ground model with physics
  groundShape = new btBoxShape(btVector3(150,1,150));

  //	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	btDefaultCollisionConstructionInfo cci;
	cci.m_defaultMaxPersistentManifoldPoolSize = maxProxies;
	m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);

	//for cloth rendering
	m_SoftCollisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

#ifdef USE_PARALLEL_DISPATCHER
	int maxNumOutstandingTasks = 3;

#ifdef USE_WIN32_THREADING

m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#endif
	m_dispatcher = new	SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxNumOutstandingTasks,m_collisionConfiguration);
	//m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#else
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax, 1000);
	
#ifdef USE_PARALLEL_SOLVER
	m_threadSupportSolver = createSolverThreadSupport(maxNumOutstandingTasks);
	m_constraintSolver = new btParallelConstraintSolver(m_threadSupportSolver);
	//this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
	m_dispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);
#else
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	//btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	//m_constraintSolver = solver;
#endif //USE_PARALLEL_SOLVER

	//m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
	//m_dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_SIMD | SOLVER_USE_WARMSTARTING | SOLVER_USE_2_FRICTION_DIRECTIONS;
	//m_dynamicsWorld->getSolverInfo().m_splitImpulse=true;

	//for cloth rendering by Atsushi.U 2012/07/28
	m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_SoftCollisionConfiguration);
#ifdef USE_PARALLEL_SOLVER
	m_dynamicsWorld->getSimulationIslandManager()->setSplitIslands(false);
	m_dynamicsWorld->getSolverInfo().m_numIterations = 4;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD+SOLVER_USE_WARMSTARTING;//+SOLVER_RANDMIZE_ORDER;
	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
#endif

	m_dynamicsWorld->setGravity(btVector3(0,0,-9.8*10));

    m_worldInfo.m_sparsesdf.Initialize();
	m_worldInfo.m_dispatcher	= m_dispatcher;
	m_worldInfo.m_broadphase	= m_overlappingPairCache;
	m_worldInfo.m_gravity		= btVector3(0,0,-9.8*10);
	m_worldInfo.air_density		= btScalar( 1.2 );
    m_worldInfo.water_density	= 0;
    m_worldInfo.water_offset	= 0;
    m_worldInfo.water_normal	= btVector3( 0, 0, 0 );
	
	m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback, this, true);

	btTransform tr;
	tr.setIdentity();

	/* Trimesh ground creation */
	const float TRIANGLE_SIZE = world_scale;
	int vertStride = sizeof(btVector3);
	int indexStride = 3*sizeof(int);
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	m_vertices = new btVector3[totalVerts];
	int* gIndices = new int[totalTriangles*3];

	for (int i=0;i<NUM_VERTS_X;i++) {
		for (int j=0;j<NUM_VERTS_Y;j++) {
			float x = (i-center_trimesh.getX())*TRIANGLE_SIZE;
			float y = (j-(120-center_trimesh.getY()))*TRIANGLE_SIZE;
			float height = m_rawHeightfieldData[j*NUM_VERTS_X+i];
			m_vertices[i+j*NUM_VERTS_X].setValue(x, y, height);
		}
	}
	int index=0;
	for (int i=0;i<NUM_VERTS_X-1;i++) {
		for (int j=0;j<NUM_VERTS_Y-1;j++) {
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
		}
	}
	
	m_indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles, gIndices,indexStride,totalVerts,(btScalar*) &m_vertices[0].x(),vertStride);
	//m_indexVertexArraysのm_aabbMin, m_aabbMaxのどっちかがおかしい？めっちゃ大きな値入ってる

	bool useQuantizedAabbCompression = true;

	trimeshShape = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression,worldMin,worldMax);
	groundShape = trimeshShape;
	//btVector3 localScaling(world_scale,world_scale,1);
	btVector3 localScaling(1,1,1);
	localScaling[upIndex]=1.f;
	groundShape->setLocalScaling(localScaling);

	tr.setOrigin(btVector3(0,0,0));
	m_collisionShapes.push_back(groundShape);
	m_groundMotionState = new btDefaultMotionState(tr);
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,m_groundMotionState,groundShape,btVector3(0,0,0));
    groundRigidBody = new btRigidBody(groundRigidBodyCI);
	//groundRigidBody->setFriction(btScalar(1000));//0.8

	const float GROUND_FRICTION = 1.0f;
	groundRigidBody->setFriction(btScalar(GROUND_FRICTION));//modified
	groundRigidBody->setRestitution(btScalar(RESTITUTION));//modified

	cout << "Set Ground Friction : " << GROUND_FRICTION << endl;
  
	m_dynamicsWorld->addRigidBody(groundRigidBody);
	/*End trimesh ground*/

#ifdef SIM_MICROMACHINE
	/*Car Creation - Chassis*/
	for (int i = 0; i < NUMBER_CAR; i++ ) {	
		btCollisionShape* chassisShape = new btBoxShape(btVector3(Car_Array[i].chassis_width,Car_Array[i].chassis_length, Car_Array[i].chassis_height));
		m_collisionShapes.push_back(chassisShape);
		btCompoundShape* compound = new btCompoundShape();
		m_collisionShapes.push_back(compound);

		btTransform localTrans;
		localTrans.setIdentity();
		localTrans.setOrigin(btVector3(0,0,Car_Array[i].chassisDistFromGround));//localTrans effectively shifts the center of mass with respect to the chassis

		compound->addChildShape(localTrans,chassisShape);
	
		chassisMotionState.push_back(boost::shared_ptr<btDefaultMotionState>(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)))));
		btVector3 localInertia(0, 0, 0);
		compound->calculateLocalInertia(Car_Array[i].car_mass,localInertia);
		btRigidBody::btRigidBodyConstructionInfo carRigidBody1CI( Car_Array[i].car_mass, chassisMotionState.at(i).get(), compound, localInertia);
		m_carChassis.push_back(boost::shared_ptr<btRigidBody>( new btRigidBody(carRigidBody1CI)));

		m_dynamicsWorld->addRigidBody(m_carChassis.at(i).get());

		m_wheelShape.push_back(boost::shared_ptr<btCylinderShapeX>(new btCylinderShapeX(btVector3(Car_Array[i].wheelWidth,Car_Array[i].wheelRadius,Car_Array[i].wheelRadius))));
		{
			m_vehicleRayCaster.push_back(boost::shared_ptr<btDefaultVehicleRaycaster>(new btDefaultVehicleRaycaster(m_dynamicsWorld)));
			m_vehicle.push_back(boost::shared_ptr<btRaycastVehicle>(new btRaycastVehicle(m_tuning[i], m_carChassis.at(i).get() ,m_vehicleRayCaster.at(i).get())));
			///never deactivate the vehicle
			m_carChassis.at(i)->setActivationState(DISABLE_DEACTIVATION);
			m_dynamicsWorld->addVehicle(m_vehicle.at(i).get());
			bool isFrontWheel=true;

			m_vehicle.at(i)->setCoordinateSystem(rightIndex,upIndex,forwardIndex);
			btVector3 connectionPointCS0(Car_Array[i].wheelWidthOffsetFront*CUBE_HALF_EXTENTS-(0.3*Car_Array[i].wheelWidth),Car_Array[i].wheelLengthOffsetFront*2*CUBE_HALF_EXTENTS-Car_Array[i].wheelRadius, Car_Array[i].connectionHeight);
			m_vehicle.at(i)->addWheel(connectionPointCS0,wheelDirectionCS0[i],wheelAxleCS[i],suspensionRestLength[i],Car_Array[i].wheelRadius,m_tuning[i],isFrontWheel);
			connectionPointCS0 = btVector3(Car_Array[i].wheelWidthOffsetFront*-CUBE_HALF_EXTENTS+(0.3*Car_Array[i].wheelWidth),Car_Array[i].wheelLengthOffsetFront*2*CUBE_HALF_EXTENTS-Car_Array[i].wheelRadius, Car_Array[i].connectionHeight);
			m_vehicle.at(i)->addWheel(connectionPointCS0,wheelDirectionCS0[i],wheelAxleCS[i],suspensionRestLength[i],Car_Array[i].wheelRadius,m_tuning[i],isFrontWheel);
			connectionPointCS0 = btVector3(Car_Array[i].wheelWidthOffsetBack*-CUBE_HALF_EXTENTS+(0.3*Car_Array[i].wheelWidth),Car_Array[i].wheelLengthOffsetBack*(-2)*CUBE_HALF_EXTENTS+Car_Array[i].wheelRadius, Car_Array[i].connectionHeight);
			isFrontWheel = false;
			m_vehicle.at(i)->addWheel(connectionPointCS0,wheelDirectionCS0[i],wheelAxleCS[i],suspensionRestLength[i],Car_Array[i].wheelRadius,m_tuning[i],isFrontWheel);
			connectionPointCS0 = btVector3(Car_Array[i].wheelWidthOffsetBack*CUBE_HALF_EXTENTS-(0.3*Car_Array[i].wheelWidth),Car_Array[i].wheelLengthOffsetBack*(-2)*CUBE_HALF_EXTENTS+Car_Array[i].wheelRadius, Car_Array[i].connectionHeight);
			m_vehicle.at(i)->addWheel(connectionPointCS0,wheelDirectionCS0[i],wheelAxleCS[i],suspensionRestLength[i],Car_Array[i].wheelRadius,m_tuning[i],isFrontWheel);
			for (int k=0;k<m_vehicle.at(i)->getNumWheels();k++) 
			{
				btWheelInfo& wheel = m_vehicle.at(i)->getWheelInfo(k);
				wheel.m_suspensionStiffness = Car_Array[i].suspensionStiffness;
				wheel.m_wheelsDampingRelaxation = Car_Array[i].suspensionDamping;
				wheel.m_wheelsDampingCompression = Car_Array[i].suspensionCompression;
				wheel.m_frictionSlip = Car_Array[i].wheelFriction;
				wheel.m_rollInfluence = Car_Array[i].rollInfluence;
			}
		}
	}

	resetCarScene(0);
	resetCarScene(1);

#endif /*SIM_MICROMACHINE*/

#ifdef SIM_PARTICLES
	createWorldSphereProxy();
#endif

	m_clock.reset();

	hasInit = true;
}


void bt_ARMM_world::Update() 
{
	SoftTextureUpdate();

	DecideCollisionCondition();
	DecideCollisionPanel();
	DecideCollisionModelButton();

#ifdef SIM_MICROMACHINE

	//for debug to emerge the error of collision coord
  	//m_carChassis.at(0)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(pCollision[0], pCollision[1], pCollision[2])));
	
	setCarMovement();
	for (int i = 0; i < NUMBER_CAR; i++) 
	{
		for (int j=0; j < m_vehicle.at(i)->getNumWheels(); j++) {
			m_vehicle.at(i)->updateWheelTransform(j,true); 
		}
	}
#endif /*SIM_MICROMACHINE*/

	//check the object to delete it
	objVectorDeletable.clear();

	REP(i,m_objectsBody.size()){
		btVector3 trans = m_objectsBody[i]->getCenterOfMassTransform().getOrigin();
		if( trans.getZ() < -10 ){
			objVectorDeletable.push_back(i);
		}
	}

	if( !objVectorDeletable.empty() )
	{
		for(int i= objVectorDeletable.size()-1; i>=0; i--){
			vector< boost::shared_ptr<btRigidBody> >::iterator it
				= m_objectsBody.begin() + objVectorDeletable[i];
			vector<btDefaultMotionState*>::iterator it2 = ObjectsMotionState.begin() + objVectorDeletable[i];

			//for out of arrays
			unsigned int ind = objVectorDeletable[i];
			if( ObjectsMotionState.size() < ind
			||	m_objectsBody.size() < ind) continue;

			//m_objectsBody.erase(it);
			//ObjectsMotionState.erase(it2);
		}
	}

	//start simulation
	m_dynamicsWorld->stepSimulation(1/20.f,5); //60s

	//m_dynamicsWorld->performDiscreteCollisionDetection();
//	m_worldInfo.m_sparsesdf.GarbageCollect();
}

void bt_ARMM_world::setCarMovement() 
{
	//Drive front wheels
	for (int i = 0; i < NUMBER_CAR; i++) {
		m_vehicle.at(i)->setSteeringValue(Car_Array[i].gVehicleSteering,0);
		m_vehicle.at(i)->setBrake(Car_Array[i].gBreakingForce,0);
		m_vehicle.at(i)->setSteeringValue(Car_Array[i].gVehicleSteering,1);
		m_vehicle.at(i)->setBrake(Car_Array[i].gBreakingForce,1);

		//Drive rear wheels
		m_vehicle.at(i)->applyEngineForce(Car_Array[i].gEngineForce,2);
		m_vehicle.at(i)->setBrake(Car_Array[i].gBreakingForce,2);
		m_vehicle.at(i)->applyEngineForce(Car_Array[i].gEngineForce,3);
		m_vehicle.at(i)->setBrake(Car_Array[i].gBreakingForce,3);
	}
	m_carChassis.at(0)->setRestitution(RESTITUTION);
	m_carChassis.at(1)->setRestitution(RESTITUTION);
}

btTransform bt_ARMM_world::getCarPose(int index) {
	btTransform trans;
	m_carChassis.at(index)->getMotionState()->getWorldTransform(trans);
	return trans;
}

btTransform bt_ARMM_world::getWheelTransform(int carIndex, int wheelInt) {
	btTransform trans = m_vehicle.at(carIndex)->getWheelTransformWS(wheelInt);
	//btQuaternion rot = m_vehicle.at(carIndex)->getWheelInfo(wheelInt).m_worldTransform.getRotation();
	//m_vehicle.at(carIndex)->getWheelInfo(wheelInt).m_worldTransform.getOpenGLMatrix(m);
	//m_carChassis.at(index)->getMotionState()->getWorldTransform(trans);
	return trans;
}


btScalar bt_ARMM_world::getDeltaTimeMicroseconds() {
	btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
	m_clock.reset();
	return dt;
}

void bt_ARMM_world::resetEngineForce(int index) {
	Car_Array[index].gEngineForce = 0.f;
	Car_Array[index].gBreakingForce = Car_Array[index].defaultBreakingForce; 
	//printf("reset engine force\n");
}

void bt_ARMM_world::accelerateEngine(int index) {
	Car_Array[index].gEngineForce = Car_Array[index].maxEngineForce;
	Car_Array[index].gBreakingForce = 0.f;
	//printf("accelerate \n");
}

void bt_ARMM_world::decelerateEngine(int index) {
	Car_Array[index].gEngineForce = -Car_Array[index].maxEngineForce;
	Car_Array[index].gBreakingForce = 0.f;
	//printf("decelerate \n");
}

void bt_ARMM_world::turnReset(int index) {
	Car_Array[index].gVehicleSteering = 0;
}

void bt_ARMM_world::turnEngineLeft(int index) {
	Car_Array[index].gVehicleSteering += Car_Array[index].steeringIncrement;
	if (Car_Array[index].gVehicleSteering > Car_Array[index].steeringClamp)	
		Car_Array[index].gVehicleSteering = Car_Array[index].steeringClamp;
	//printf("turn left \n");
}

void bt_ARMM_world::turnEngineRight(int index) {
	Car_Array[index].gVehicleSteering -= Car_Array[index].steeringIncrement;
	if (Car_Array[index].gVehicleSteering < -Car_Array[index].steeringClamp)
		Car_Array[index].gVehicleSteering = -Car_Array[index].steeringClamp;
	//printf("turn right \n");
}

void bt_ARMM_world::resetCarScene(int car_index)
{
	//for (int i =0; i < NUMBER_CAR; i++) {
		Car_Array[car_index].gVehicleSteering = 0.f;
		Car_Array[car_index].gBreakingForce = Car_Array[car_index].defaultBreakingForce;
		Car_Array[car_index].gEngineForce = 0.f;

		//m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
		m_carChassis.at(car_index)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(20*car_index+500,-5,5)));
		m_carChassis.at(car_index)->setLinearVelocity(btVector3(0,0,0));
		m_carChassis.at(car_index)->setAngularVelocity(btVector3(0,0,0));
		m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis.at(car_index)->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());
		if (m_vehicle.at(car_index))
		{
			m_vehicle.at(car_index)->resetSuspension();
			for (int j=0;j<m_vehicle.at(car_index)->getNumWheels();j++)
			{
				//synchronize the wheels with the (interpolated) chassis worldtransform
				m_vehicle.at(car_index)->updateWheelTransform(j,true);
			}
		}
}

void bt_ARMM_world::setWorldDepth(float w) {
	worldDepth = w;
}

float* bt_ARMM_world::getTrimeshGround() {
	return m_rawHeightfieldData;
}

btTransform bt_ARMM_world::getTrimeshGroundTransform() {
	btTransform trans;
	groundRigidBody->setRestitution(RESTITUTION);
	groundRigidBody->getMotionState()->getWorldTransform(trans);
	return trans;
}

void bt_ARMM_world::setMinHeight(float min) {
	m_minHeight = btScalar(min);
}

void bt_ARMM_world::setMaxHeight(float max) {
	m_maxHeight = btScalar(max);
}

void bt_ARMM_world::setWorldScale(float scale) {
	world_scale = scale;
}

int bt_ARMM_world::create_Sphere() 
{
	float sphereMass = 50;
	Sphere_Shape =  new btSphereShape(SPHERE_SIZE_);
	m_collisionShapes.push_back(Sphere_Shape);
	ObjectsMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0))));
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	Sphere_Shape->calculateLocalInertia(sphereMass, localInertia);

	int index = ObjectsMotionState.size()-1;
	btRigidBody::btRigidBodyConstructionInfo Sphere_Body_CI(sphereMass,ObjectsMotionState.at(index),Sphere_Shape,localInertia);
	boost::shared_ptr<btRigidBody> tmpRigid = boost::shared_ptr<btRigidBody>(new btRigidBody(Sphere_Body_CI));
	m_objectsBody.push_back(tmpRigid);

	m_dynamicsWorld->addRigidBody(m_objectsBody.at(index).get());

	m_objectsBody.at(index)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(20,-5,30)));
	m_objectsBody.at(index)->setFriction(FRICTION);
	m_objectsBody.at(index)->setRestitution(RESTITUTION);
	return index;
}

int bt_ARMM_world::create_Box()
{
	float boxMass = 50;
//	Convex_Shape.push_back(new btConvexHullShape());//place holder
//	int shape_index = Convex_Shape.size()-1;
//	float scale = 2;
	Box_Shape =  new btBoxShape(btVector3(CUBE_SIZE_, CUBE_SIZE_, CUBE_SIZE_));
	m_collisionShapes.push_back(Box_Shape);
	ObjectsMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0))));
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	Box_Shape->calculateLocalInertia(boxMass, localInertia);

	int index = ObjectsMotionState.size()-1;
	btRigidBody::btRigidBodyConstructionInfo Box_Body_CI(boxMass,ObjectsMotionState.at(index),Box_Shape,localInertia);

	boost::shared_ptr<btRigidBody> tmpRigid = boost::shared_ptr<btRigidBody>(new btRigidBody(Box_Body_CI));
	m_objectsBody.push_back(tmpRigid);

	m_dynamicsWorld->addRigidBody(m_objectsBody.at(index).get());

	m_objectsBody.at(index)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(30,-5,0)));
	m_objectsBody.at(index)->setFriction(FRICTION);
	m_objectsBody.at(index)->setRestitution(RESTITUTION);
	return index;
}

//オブジェクトの回転・並進をさせるためにはmotion stateの値を変更
int bt_ARMM_world::create_3dsmodel(string modelname, osg::ref_ptr<osg::Node> sample)
{
	//load 3ds model
	this->modelname = modelname;
	ViewingModel model(modelname);
	int mesh_num = model.GetMeshSize();

	//set object mass
	float boxMass = 50;

	if(!sample.valid())
	{
		sample = osgDB::readNodeFile(modelname.c_str());
	}

	//create bounding box
	btDefaultMotionState* state = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));  //ここをいじる
	boost::shared_ptr<btRigidBody> btObject = boost::shared_ptr<btRigidBody>(new btRigidBody(50, state, NULL));
	btObject->setCollisionShape( osgbCollision::btTriMeshCollisionShapeFromOSG(sample.get() ) );
	btObject->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );

	m_objectsBody.push_back(btObject);
	m_dynamicsWorld->addRigidBody(btObject.get());

	int index = m_objectsBody.size() - 1;
	m_objectsBody.at(index)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
	m_objectsBody.at(index)->setFriction(FRICTION);
	m_objectsBody.at(index)->setRestitution(RESTITUTION);

	return index;
}

void bt_ARMM_world::CreateMenu(ARMM::osg_Menu* osgMenu)
{
	std::vector<osg::ref_ptr<osg::Node> > pObjMenuNodeArray;
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pObjMenuTransformArray;

	//set object mass
	float boxMass = 5;

	//set the info of rendered objects
	pObjMenuNodeArray		= osgMenu->getObjMenuNodeArray();
	pObjMenuTransformArray	= osgMenu->getObjMenuTransformArray();

	//all menu nodes are registered in bullet dynamics world
	mMenuIndexOffset = m_dynamicsWorld->getCollisionObjectArray().size()-1;
	REP(i,pObjMenuNodeArray.size())
	{
		int index = mMenuBody.size();

		osg::ref_ptr<osg::Node> sample = pObjMenuNodeArray.at(i);

		//create bounding box
		const float scale = 10.0;
		osg::Vec3 pos = pObjMenuTransformArray.at(i)->getPosition();
		osg::Vec4 att = pObjMenuTransformArray.at(i)->getAttitude().asVec4();

		//printf("Pos=(%f,%f,%f)\n",pos.x(), pos.y(), pos.z());
		btDefaultMotionState* state = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
		boost::shared_ptr<btRigidBody> btObject		= boost::shared_ptr<btRigidBody>(new btRigidBody(boxMass, state, NULL));
		btObject->setCollisionShape( osgbCollision::btTriMeshCollisionShapeFromOSG(sample.get() ) );
		btObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT ); //because a menu button do not need to move

		//register it to dynamics world
		mMenuBody.push_back(btObject);
		m_dynamicsWorld->addRigidBody(btObject.get());

		//set attribute of this object
		btVector4 v4 = osgbCollision::asBtVector4(att);
		btQuaternion quat(v4.x(), v4.y(), v4.z(), v4.w());
		mMenuBody.at(index)->setCenterOfMassTransform(btTransform(quat, btVector3(pos.x()/scale, pos.y()/scale, pos.z()/scale)));

		//temporary ID variable
		mMenuBody.at(index)->setFriction(index);
	}
}

void bt_ARMM_world::CreateModelButton(ARMM::osg_Menu* osgMenu)
{
	std::vector<osg::ref_ptr<osg::Node> > pObjModelButtonNodeArray;
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > pModelButtonTransformArray;

	//set object mass
	float boxMass = 5;

	//set the info of rendered objects
	pObjModelButtonNodeArray	= osgMenu->getMenuModelObjectArray();
	pModelButtonTransformArray	= osgMenu->getMenuModelTransArray();

	//all menu nodes are registered in bullet dynamics world
	mModelButtonIndexOffset = m_dynamicsWorld->getCollisionObjectArray().size()-1;
	REP(i,pObjModelButtonNodeArray.size())
	{
		int index = mModelButton.size();

		osg::ref_ptr<osg::Node> sample = pObjModelButtonNodeArray.at(i);

		//create bounding box
		const float scale = 10.0;
		osg::Vec3 pos = pModelButtonTransformArray.at(i)->getPosition();
		osg::Vec4 att = pModelButtonTransformArray.at(i)->getAttitude().asVec4();

		//printf("Pos=(%f,%f,%f)\n",pos.x(), pos.y(), pos.z());
		btDefaultMotionState* state = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
		boost::shared_ptr<btRigidBody> btObject		= boost::shared_ptr<btRigidBody>(new btRigidBody(boxMass, state, NULL));
		btObject->setCollisionShape( osgbCollision::btTriMeshCollisionShapeFromOSG(sample.get() ) );
		btObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT ); //because a menu button do not need to move

		//register it to dynamics world
		mModelButton.push_back(btObject);
		m_dynamicsWorld->addRigidBody(btObject.get());

		//set attribute of this object
		btVector4 v4 = osgbCollision::asBtVector4(att);
		btQuaternion quat(v4.x(), v4.y(), v4.z(), v4.w());
		mModelButton.at(index)->setCenterOfMassTransform(btTransform(quat, btVector3(pos.x()/scale, pos.y()/scale, 5))); //TODO デプス値を固定値にしない

		printf("%d BulletPos(%lf,%lf,0)\n",i, pos.x()/scale, pos.y()/scale);
		//temporary ID variable
		mModelButton.at(index)->setFriction(index);
	}
}
osg::Node* bt_ARMM_world::CreateSoftTexture(string texturename)
{
	osg::Vec3d pos =  osgbCollision::asOsgVec3(btHandPos*10);
	pos.set(pos.x(), pos.y(), pos.z()/10);
    osg::ref_ptr< osg::Geode > geode( new osg::Geode );
	const osg::Vec3 llCorner(pos);
    const osg::Vec3 uVec( 100.0, 0.0, 0.0 );
    const osg::Vec3 vVec( 0.0, 100.0, 0.0 ); // Must be at a slight angle for wind to catch it.
    softGeom = osgwTools::makePlane( llCorner,
        uVec, vVec, osg::Vec2s( resX-1, resY-1 ) );
    geode->addDrawable( softGeom );

    // Set up for dynamic data buffer objects
    softGeom->setDataVariance( osg::Object::DYNAMIC );
    softGeom->setUseDisplayList( false );
    softGeom->setUseVertexBufferObjects( true );
    softGeom->getOrCreateVertexBufferObject()->setUsage( GL_DYNAMIC_DRAW );

    // Flag state: 2-sided lighting and a texture map.
    {
        osg::StateSet* stateSet( softGeom->getOrCreateStateSet() );

        osg::LightModel* lm( new osg::LightModel() );
        lm->setTwoSided( true );
        stateSet->setAttributeAndModes( lm );

        const std::string texName( "Data/tex.bmp" );
        osg::Texture2D* tex( new osg::Texture2D(
            osgDB::readImageFile( texName ) ) );
        if( ( tex == NULL ) || ( tex->getImage() == NULL ) )
            osg::notify( osg::WARN ) << "Unable to read texture: \"" << texName << "\"." << std::endl;
        else
        {
            tex->setResizeNonPowerOfTwoHint( false );
            stateSet->setTextureAttributeAndModes( 0, tex );
        }
    }


    // Create the soft body using a Bullet helper function. Note that
    // our update callback will update vertex data from the soft body
    // node data, so it's important that the corners and resolution
    // parameters produce node data that matches the vertex data.

	//printf("(%f,%f,%f)G\n",m_worldInfo.m_gravity.x(),m_worldInfo.m_gravity.y(),m_worldInfo.m_gravity.z());
	textureSoftBody = btSoftBodyHelpers::CreatePatch( m_worldInfo,
        osgbCollision::asBtVector3( llCorner ),
        osgbCollision::asBtVector3( llCorner + uVec ),
        osgbCollision::asBtVector3( llCorner + vVec ),
        osgbCollision::asBtVector3( llCorner + uVec + vVec ),
        resX, resY, 0, true );

    // Configure the soft body for interaction with the wind.
    textureSoftBody->getCollisionShape()->setMargin( 0.1 );
    textureSoftBody->m_materials[ 0 ]->m_kLST = 0.3;
    textureSoftBody->generateBendingConstraints( 2, textureSoftBody->m_materials[ 0 ] );
    textureSoftBody->m_cfg.kLF = 0.05;
    textureSoftBody->m_cfg.kDG = 0.01;
	textureSoftBody->m_cfg.kDF = 1.0;
	//textureSoftBody->generateClusters(64);
    textureSoftBody->m_cfg.piterations = 2;
	textureSoftBody->m_cfg.collisions |= btSoftBody::fCollision::SDF_RS; // for collision BUT no operation;;
	textureSoftBody->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
	textureSoftBody->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSided;
    textureSoftBody->setWindVelocity( btVector3( 0., 0., -9.8 ) );
    textureSoftBody->setTotalMass( 100 );
	//textureSoftBody->get
	////体積保存の係数
	//textureSoftBody->m_cfg.kVC = 0.5;

	////第1引数で体積保存のフラグをON
	//textureSoftBody->setPose(true,false);

	textureSoftBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

	this->getSoftDynamicsWorld()->addSoftBody(textureSoftBody);
	//softGeom->setUpdateCallback( new MeshUpdater( textureSoftBody, resX*resY));

    return( geode.release() );
}

double bt_ARMM_world::get3dsScale(void)
{
	return scale_3ds;	
}

std::string bt_ARMM_world::GetModelName(void) const{ 
	return modelname;
}

btConvexHullShape* bt_ARMM_world::get_Convex_Shape(int index) {
	return Convex_Shape.at(index);
}

btTransform bt_ARMM_world::get_Object_Transform(int index) 
{
	btTransform trans;
	m_objectsBody.at(index)->getMotionState()->getWorldTransform(trans);
	return trans;
}

int bt_ARMM_world::Num_Object() {
	return m_objectsBody.size();
}

void bt_ARMM_world::set_center_trimesh(float x, float y) {
	center_trimesh.setX(x);
	center_trimesh.setY(y);
}

void bt_ARMM_world::clearTrimesh() {
	for (int i = 0; i < HF_Size; i++) {
		m_rawHeightfieldData[i] = 0;
	}
}

void bt_ARMM_world::updateTrimesh(float* hf) {
	 memcpy(m_rawHeightfieldData, hf, sizeof(m_rawHeightfieldData));
/*	for (int i = 0; i < HF_Size; i++) {
		if(m_rawHeightfieldData[i] != hf[i])
			m_rawHeightfieldData[i] = hf[i];
	}
	*/
}

void bt_ARMM_world::updateTrimeshRefitTree(float* hf) {
	for (int i = 0; i < HF_Size; i++) {
		if(m_rawHeightfieldData[i] != hf[i])
			m_rawHeightfieldData[i] = hf[i];
	}

	//btVector3 aabbMin(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	//btVector3 aabbMax(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);
	for (int i=0;i<NUM_VERTS_X-1;i++) {
		for (int j=0;j<NUM_VERTS_Y-1;j++) {	
//			worldMax.setMax(m_vertices[i+j*NUM_VERTS_X]); worldMin.setMin(m_vertices[i+j*NUM_VERTS_X]);
			float height = m_rawHeightfieldData[j*NUM_VERTS_X+i];
			m_vertices[i+j*NUM_VERTS_X].setZ(height);					
//			worldMin.setMin(m_vertices[i+j*NUM_VERTS_X]); worldMax.setMax(m_vertices[i+j*NUM_VERTS_X]);
		}
	}
	//trimeshShape->partialRefitTree(aabbMin,aabbMax);
	//trimeshShape->partialRefitTree(worldMin,worldMax);
	trimeshShape->refitTree(worldMin,worldMax);
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(groundRigidBody->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());
}

void bt_ARMM_world::CalcGlobalValue(float * global_x, float * global_y, const int & hand_x, const int & hand_y)
{
	(*global_x) = (float) (hand_x - center_trimesh.x())*world_scale;
	(*global_y) = (float) (hand_y - (120-center_trimesh.y()))*world_scale;
}

void bt_ARMM_world::createHand(int hand_x, int hand_y, int sphere_resolution, float ratio) 
{
	float global_x, global_y;
	CalcGlobalValue(&global_x, &global_y, hand_x, hand_y);

	boost::shared_ptr<bt_ARMM_hand> tmpHand = boost::shared_ptr<bt_ARMM_hand>(new bt_ARMM_hand(m_dynamicsWorld, m_collisionShapes, world_scale, global_x, global_y, sphere_resolution, ratio, center_trimesh.x(), center_trimesh.y()));
	HandObjectsArray.push_back(tmpHand);
	HandFingersArray.push_back(false);
}

void bt_ARMM_world::updateHandDepth(int index, int hand_x, int hand_y, float curr_hands_ratio, float* depth_grid)
{
	float global_x, global_y;
	CalcGlobalValue(&global_x, &global_y, hand_x, hand_y);
	global_x = hand_x; global_y = hand_y;
	HandObjectsArray.at(index)->Update(global_x, global_y, curr_hands_ratio, depth_grid);

	REP(idx,fingersIdx.size())
	{
		HandFingersArray[fingersIdx[idx]]=true;
	}

	//Texture Transfer ON
	if(bTextureTransfer)
	{
		//check collision to kinematic box objects with hands
		REP(obj, m_objectsBody.size())
		{
			if( m_objectsBody.at(obj)->isKinematicObject() )
			{
				REP(i, MIN_HAND_PIX*MIN_HAND_PIX)
				{
					//衝突のチェック
					if(HandFingersArray[i])
					{
						//between a finger-tip and a virtual object
		   				m_dynamicsWorld->contactPairTest(m_objectsBody.at(obj).get(), HandObjectsArray.at(index)->handSphereRigidBody.at(i), callback);
					}
	   				//m_dynamicsWorld->contactPairTest(m_objectsBody.at(obj), HandObjectsArray.at(index)->handSphereRigidBody.at(i), callback);
					//collision check with a texture soft body
					if(textureSoftBody)
					{
						//衝突のチェック
						m_dynamicsWorld->contactPairTest(HandObjectsArray.at(index)->handSphereRigidBody.at(i), textureSoftBody, pinchcallback);
					}
				}
			}
		}
	}

	//check collision between finger-tips and menu buttons
	if(!panelCollisionLock)
	{
		double current_clock = static_cast<double>(cv::getTickCount());
		double time_spent = ( current_clock - prev_panel_clock ) / cv::getTickFrequency();

		//１秒経過したかどうか
		if(time_spent > 1.0)
		{
			for(int obj=1;obj<mMenuBody.size();obj++)
			{
				REP(i, MIN_HAND_PIX*MIN_HAND_PIX)
				{
					//衝突のチェック
					if(HandFingersArray[i])
					{
						//between a finger-tip and a menu button
						m_dynamicsWorld->contactPairTest(mMenuBody.at(obj).get(), HandObjectsArray.at(index).get()->handSphereRigidBody.at(i), panelcallback);

						if(panelCollisionLock) break;
					}
				}
				if(panelCollisionLock)
				{
					cout << "Press button" << endl;
					break;		
				}
			}
		}
	}

	if(panelCollisionLock && panelInput == ADDARMODEL && selectID == -1)
	{
		for(int obj=0;obj<mModelButton.size();obj++)
		{
			REP(i, MIN_HAND_PIX*MIN_HAND_PIX)
			{
				//衝突のチェック
				if(HandFingersArray[i])
				{
					//between a finger-tip and a menu button
					m_dynamicsWorld->contactPairTest(mModelButton.at(obj).get(), HandObjectsArray.at(index).get()->handSphereRigidBody.at(i), selectcallback);

					if(bSelectModel) break;
				}
			}
			if(bSelectModel)
			{
				cout << "Select a virtual model done by contactPairTest" << endl;
				break;		
			}
		}
	}

	REP(i,MIN_HAND_PIX*MIN_HAND_PIX)
	{
		btTransform trans;
		HandObjectsArray[index]->handSphereRigidBody.at(i)->getMotionState()->getWorldTransform(trans);
		if(trans.getOrigin().getZ() < 500 && trans.getOrigin().getZ() > 0)
		{
			btHandPos.setX(trans.getOrigin().getX());
			btHandPos.setY(trans.getOrigin().getY());
			btHandPos.setZ(trans.getOrigin().getZ());

			return;
		}
		HandFingersArray[i]=false;
	}
}

//Bad only need one transformation and grid to update all the height
btTransform bt_ARMM_world::getHandTransform(int index)
{
	btTransform trans = HandObjectsArray.at(index)->getHandSphereTransform();
	return trans;
}

int bt_ARMM_world::getTotalNumberHand() {
	return HandObjectsArray.size();
}

float* bt_ARMM_world::debugHandX(int index) {
	return HandObjectsArray.at(index)->debugHandX();
}
float* bt_ARMM_world::debugHandY(int index) {
	return HandObjectsArray.at(index)->debugHandY();
}
float* bt_ARMM_world::debugHandZ(int index) {
	return HandObjectsArray.at(index)->debugHandZ();
}

bool bt_ARMM_world::ToggleSphereRep( void ){
	m_sphere_rep = !m_sphere_rep;
	return m_sphere_rep;
}

bool bt_ARMM_world::GetSphereRep( void )
{
	return m_sphere_rep;
}

void bt_ARMM_world::ChangeAttribute(int x, int y, int z, int index)
{
	if( index >= m_objectsBody.size())
	{
		cerr << "Error: Out of range in m_objectsBody array" << endl;
		cerr << "Error: index = " << index << endl;
		return;
	}
	btTransform trans;
	m_objectsBody.at(index)->getMotionState()->getWorldTransform(trans);
	trans.getOrigin().setX(x);
	trans.getOrigin().setY(y);
	trans.getOrigin().setZ(z);
	m_objectsBody.at(index)->getMotionState()->setWorldTransform(trans);
	m_objectsBody.at(index)->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

	printf("Changed Object %d attributes to Kinematics\n", index);

	return;
}

void bt_ARMM_world::ChangeRotation(double x, double y, double z)
{
	btTransform trans;
	m_objectsBody.at(0)->getMotionState()->getWorldTransform(trans);
	trans.getRotation().setValue(x,y,z,1);
	m_objectsBody.at(0)->getMotionState()->setWorldTransform(trans);

}

/*********** Private Method ************/
/*
 *
 */
void bt_ARMM_world::DecideCollisionCondition()
{
  btCollisionObjectArray colObjs = m_dynamicsWorld->getCollisionObjectArray();
  int collisionSize= colObjs.size();
  
	//set touch value to false if no collision continues for 2 sec
	double current_clock = static_cast<double>(cv::getTickCount());
	double time_spent = ( current_clock - prev_collide_clock ) / cv::getTickFrequency();
	if(time_spent > 1.0)
	{
		touch = false;
	}

	for(int i=0; i<collisionSize; i++)
	{
		if(colObjs[i] == actualCollisionObject )
		{
			//except for a regular object, such hand objects or cars
			if( i < REGULAROBJECTNUM)
			{
				changeCollisionObject = true;
				//printf("ChangeCollisionObject is TRUE\n");
				return;
			}

		  //1回目の接触
			if( interact_state == STROKE1)
			{
				cout << " Collision to " << i << " in " 
					<< pCollision << ","
					<< endl;  

				//play some effect
					PlaySound(_T("Crrect_answer3.wav"), NULL, SND_ASYNC);	

			}
			else if(interact_state == PINCH)
			{
				//2回目の接触 <sourceオブジェクト決定>
				if( i == collisionInd)
				{
					cout << " Collision to " << collisionInd << " with parts" <<  " in " 
					<< pCollision << ","
					<< endl;
					cout << "Interaction state changed PINCH to TEXTUREGET " << endl;
					interact_state = TEXTUREGET;
				}
			}
			else if( interact_state == KEEP)
			{
				//3回目の接触 <destinationオブジェクト決定>
				if( i != collisionInd)
				{
					cout << " Collision to " << i << " and finished transfer from " << collisionInd <<  " in " 
					<< pCollision << ","
					<< endl;

					cout << "Interaction state changed KEEP to PASTE" << endl;
					interact_state = PASTE;
				}
			}

			collisionInd = i;
			actualCollisionObject = NULL;

			if( selector > 0 && i != collide[0])
			{
				collide[1] = i;
				///        selector = 0;
			}
			else
			{
				collide[0] = i;
				selector = 1;
			}
			return;
		}
	}


	if( collisionInd > 0 && interact_state==STROKE1 && !touch)
	{
		//[Collide some object]->[release it]
		// it make system unable to stroke
		cout << "Interaction state changed STROKE to PINCH" << endl;
		interact_state = PINCH;

		//play some effect
		PlaySound(_T("attack01.wav"), NULL, SND_ASYNC);	

	}

	if( interact_state==PASTE && !touch)
	{
		//[Collide some object]->[release it]
		// it make system unable to stroke
		cout << "Interaction state changed PASTE to INIT" << endl;
		interact_state = INIT;
	}	

	changeCollisionObject = false;
}

void bt_ARMM_world::DecideCollisionPanel()
{
	//set touch value to false if no collision continues for 2 sec
	double current_clock = static_cast<double>(cv::getTickCount());
	double time_spent = ( current_clock - prev_collide_clock ) / cv::getTickFrequency();

	int collisionSize = mMenuBody.size();
	for(int i=1; i<collisionSize; i++)
	{
		if( static_cast<int>(mMenuBody.at(i)->getFriction()) == actualID)
		{
			osgArInputButton = actualID;
			actualID = -1;
		}
	}
}

void bt_ARMM_world::DecideCollisionModelButton()
{
	//no selected model's button is found
	if(selectID == -1 ) return;

	//check each model's button
	int collisionSize = mModelButton.size();
	for(int i=1; i<collisionSize; i++)
	{
		//check which model's ID(instead of using Friction as ID) is matched to the selected
		if( static_cast<int>(mModelButton.at(i)->getFriction()) == selectID)
		{
			cout << "Select object done!! " << selectID << endl;
			osgArAddModelIndex = selectID;
		}
	}
}

void bt_ARMM_world::ResetARButtonInput( void )
{
	selectID		= -1;
	bSelectModel	= false;
}

void bt_ARMM_world::SoftTextureUpdate( void )
{
	if(textureSoftBody == 0 ) return;

	const int _size = resX*resY;

#ifdef USE_ARMM_SERVER_VIEW
	osg::Geometry* geom = softGeom;
	osg::Drawable* draw = dynamic_cast<osg::Drawable*>(geom);
    osg::Vec3Array* verts( dynamic_cast< osg::Vec3Array* >( geom->getVertexArray() ) );
    osg::Vec3Array::iterator it( verts->begin() );
#endif
    // Update the vertex array from the soft body node array.
    btSoftBody::tNodeArray& nodes = textureSoftBody->m_nodes;

	unsigned int idx = _size/2;		
	static const btScalar	maxdrag=10;
	btVector3 delta = (btHandPos*10 - nodes[idx].m_x);		
	if(delta.length2()>(maxdrag*maxdrag))
	{
		delta=delta.normalized()*maxdrag;
	}

	nodes[idx].m_v = (delta/_timeStep)*50; //50 is masic number. you can adjust this value with your opnion

	//printf("nodes=(%f,%f,%f)\n",nodes[idx].m_x.getX(),nodes[idx].m_x.getY(),nodes[idx].m_x.getZ());
	//nodes[idx].m_x.setValue(btHandPos.x()*10, btHandPos.y()*10, btHandPos.z()*10);
	//goal.setValue(btHandPos.x()*10, btHandPos.y()*10, btHandPos.z()*10);

	for( idx=0; idx<_size; idx++)
    {
		softTexture_array[idx] = osgbCollision::asOsgVec3( nodes[ idx ].m_x );
#ifdef USE_ARMM_SERVER_VIEW
        *it++ = softTexture_array[idx];
#endif
    }
#ifdef USE_ARMM_SERVER_VIEW
	verts->dirty();
    draw->dirtyBound();

    // Generate new normals.
    osgUtil::SmoothingVisitor smooth;
    smooth.smooth( *geom );
    geom->getNormalArray()->dirty();
#endif
}
