#ifndef CONSTANT_H
#define CONSTANT_H

namespace ARMM
{
	class ConstParams
	{
	public:

		static const float	KINECT_PIX_PER_DEPTH;
		static const int	GRID_SIZE;

		static const float SPHERE_SIZE;
		static const float CUBE_SIZE;

		//for hand segmentation
		static const float	HAND_BOX_CM;
		static const int	SKIN_X;
		static const int	SKIN_Y;

		//for vrpn transmission
		static const int HAND_MAX_TRANSMIT_SIZE;

		//for file loading
		static const char *	DATABASEDIR;
		static const char *	MENUDATADIR;

		static const int NUM_OBJECTS;

		static const int CAR_PARAM;
		static const int COLLISION_PARAM;

		//window size
		static const int WINDOW_WIDTH;
		static const int WINDOW_HEIGHT;

		//for virtual objects
		static const int MAX_NUM_VIR_OBJ;
	};
}
#endif
