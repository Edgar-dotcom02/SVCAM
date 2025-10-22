#ifndef TRACKING_CAM_3D_ZMQ_H
#define TRACKING_CAM_3D_XMQ_H

#include <tlive/tlive_client.h>

class TrackingCam3d_Client : public tlive::Client
{
public:
	TrackingCam3d_Client() {
		// support("left");
		// support("left_rect");
		// support("left_gray_rect");
		// support("right");
		// support("right_rect");
		// support("right_gray_rect");
		// support("depth");
		// support("disparity");
		// support("test");
	};
};


#endif