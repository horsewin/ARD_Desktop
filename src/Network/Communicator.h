#ifndef ARMM_COMMUNICATOR_H_
#define ARMM_COMMUNICATOR_H_

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <vector>

//VRPN
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include "vrpn_Imager.h"
//OSG
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osgShadow/ShadowedScene>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>

//Bullet
#include <btBulletDynamicsCommon.h>

//Boost Library
#include <boost\shared_ptr.hpp>

//User definition
#include "UserConstant.h"
#include "Physics\bt_ARMM_hand.h"

enum datatype{REGULAR, SOFTBODY};

//---------------------------------------------------------------------------
// Struct
//---------------------------------------------------------------------------
typedef	struct _vrpn_TRACKERHANDCB {
	struct timeval	msg_time;	// Time of the report
	vrpn_int32	sensor;		// Which sensor is reporting
	vrpn_float32 hand[UDP_LIMITATION][3];
} vrpn_TRACKERHANDCB;
typedef void (VRPN_CALLBACK *vrpn_TRACKERHANDCHANGEHANDLER)(void *userdata,
					     const vrpn_TRACKERHANDCB info);

typedef	struct _vrpn_TRACKERSOFTTEXTURECB {
	struct timeval	msg_time;	// Time of the report
	vrpn_int32	sensor;		// Which sensor is reporting
	vrpn_float32 softT[resX*resY][3];
} vrpn_TRACKERSOFTTEXTURECB;
typedef void (VRPN_CALLBACK *vrpn_TRACKERSOFTTEXTURECHANGEHANDLER)(void *userdata,
					     const vrpn_TRACKERSOFTTEXTURECB info);

//---------------------------------------------------------------------------
// Class definition
//---------------------------------------------------------------------------
class ARMM_Communicator : public vrpn_Tracker
{
public:
	ARMM_Communicator( vrpn_Connection_IP *c = 0);
	virtual ~ARMM_Communicator();
	virtual void mainloop();

	virtual void SetObjectsData(std::vector< boost::shared_ptr<btRigidBody> > * obj);
	virtual void SetHandsData(std::vector< boost::shared_ptr<bt_ARMM_hand>> * hand);

protected:
	virtual int register_types( void );
	virtual int encode_hand_to(char *buf, int division);
	virtual int encode_softtexture_to(char *buf, int division);

	inline void ObjectMessagePacking( void );
	inline void SoftTextureMessagePacking( void );
	inline void HandMessagePacking( void );

protected:
    struct timeval _timestamp;
		//Atsushi
		vrpn_int32 softtexture_m_id;	// ID of tracker soft texture message					
		vrpn_int32 hand_m_id;	// ID of tracker hand message					
		vrpn_float32 hand[UDP_LIMITATION][3];	
		vrpn_float32 softT[resX*resY][3];

		//for getting bullet coordinate
		std::vector< boost::shared_ptr<btRigidBody> > * m_objects_body;
		std::vector< boost::shared_ptr<bt_ARMM_hand> >* m_hands_body;

		//	float* HeightfieldData;

private:
	datatype mDataType;
};

#endif