#ifndef HELPER_H
#define HELPER_H
#define  USE_MATH_DEFINES
//#define 	DEBUG

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <math.h>
#include <numeric>
#define equals(x, y) (abs(x-y) < 0.001)
#define INVALIDCMD -2

#define MAX_LIN_VEL 0.26 //m/s
#define MIN_LIN_VEL 0.2 //m/s

#define MAX_ANG_VEL 1.5 //rads/s
#define MIN_ANG_VEL 1.69 //rads/s

#define NULL_POSE Pose(0,0,0)

/*
 * Needed explicit parameters:
 * Wheelchair dimensions, centre of wheelchair motion,
 * occupancy gridsize,
 * wheelchair kinematic parameters such as acc, max speed,
 *
 */
typedef enum {
	Joystick, Button
} InterfaceType; // We use this to inform on the probability distribution for the user's intention from input interface.
struct Speed {
	float v;
	float w;

	Speed(const Speed& speed) :
			v(speed.v), w(speed.w) {
	}
	Speed(float vt, float wt) :
			v(vt), w(wt) {
	}
	Speed() {
		w = v = 0;
	}
	Speed operator*(float s) // copy/move constructor is called to construct arg
				{
			Speed speed = Speed();
			speed.v = this->v * s;
			speed.w = this->w * s;
			return speed;
		}
	Speed operator+(Speed rhs) // copy/move constructor is called to construct arg
			{
		Speed speed = Speed();
		speed.v = this->v + rhs.v;
		speed.w = this->w + rhs.w;
		return speed;
	}

	Speed operator-(Speed rhs) // copy/move constructor is called to construct arg
				{
			Speed speed = Speed();
			speed.v = this->v - rhs.v;
			speed.w = this->w - rhs.w;
			return speed;
		}
	bool operator==(Speed s) // copy/move constructor is called to construct arg
			{
		if (equals(this->v,s.v) && equals(this->w, s.w)) {
			return true;
		} else {
			return false;
		}
	}
};

struct Pose {
	float x;
	float y;
	float th;
	Pose(float xt, float yt, float tht) :
			x(xt), y(yt), th(tht) {
	}
	bool operator==(Pose pose) // copy/move constructor is called to construct arg
			{
		if (equals(this->x,
				pose.x) && equals(this->x, pose.x) && equals(this->x, pose.x)) {
			return true;
		} else {
			return false;
		}
	}

};

struct DynamicWindow {
	Speed upperbound;
	Speed lowerbound;
};
#endif
