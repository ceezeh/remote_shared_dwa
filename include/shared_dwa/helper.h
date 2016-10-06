<<<<<<< HEAD
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

=======
#ifndef SHAREDDWA_HELPER_H
#define SHAREDDWA_HELPER_H
#define  USE_MATH_DEFINES
//#define 	DEBUG
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <numeric>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include <map/point.h>
#include "nav_msgs/Odometry.h"
#include "shared_dwa/arithmetic.h"
#include "shared_dwa/pose.h"
#include "shared_dwa/speed.h"


#define NULL_POSE Pose(0,0,0)

using namespace std;

float angDiff(float a1, float a2);
float wraparound(float th);
float vectorNorm(Pose p);
int getQuadrant (float upper);
float vectorNorm(Pose p);
float vectorNorm(Speed p);
float magSquared(Speed p);
float sqrt_approx(float z);
Speed getRealSpeed(Speed speed_old);
Speed normaliseSpeed(Speed speed_old);
void rotateFromBody(RealPoint &pose, Pose T);
int getQuadrant(float upper);
bool isAngleInRegion(float ang, float upper, float lower);
//***********************************************************************//
>>>>>>> 4fe40d94807dc0918ca6567664966a60b5ae0759
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
<<<<<<< HEAD
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
=======
//***********************************************************************//

//***********************************************************************//

//***********************************************************************//
struct DynamicWindow {
	Speed upperbound;
	Speed lowerbound;
};
//***********************************************************************//
class DeOscillator {
private:
	float upperbound, lowerbound;
	Pose start_pose; // The pose at the start of deoscillation.
	static float lin_dist_thres, ang_dist_thres_lower, ang_dist_thres_upper;
public:
	DeOscillator () {
		upperbound = M_PI+1;
		lowerbound = -M_PI-1;
		start_pose = Pose();
	}
	// This function examines if we have travelled far enough to ensure deoscillation.
	void updateOdom(const nav_msgs::Odometry& cmd) {
		float xt = cmd.pose.pose.position.x;
		float yt = cmd.pose.pose.position.y;
		float tht = atan2(cmd.twist.twist.angular.z,cmd.twist.twist.linear.x);


		float lin_dist = sqrt(pow(start_pose.x-xt,2) + pow(start_pose.y-yt,2));
		float ang_dist = abs(angDiff(start_pose.th, tht));
		if ((lin_dist > lin_dist_thres) ||
			((ang_dist > ang_dist_thres_lower) && (ang_dist < ang_dist_thres_upper))) {
			start_pose = Pose(xt, yt, tht);
//			cout << "NEW DIRECTION" <<endl;
		}

	}	
	
	void getAdmissibleDirection(float& upperbound, float& lowerbound) {
		upperbound = start_pose.th + M_PI * 150 / 180;
		upperbound = wraparound(upperbound);
		lowerbound = start_pose.th- M_PI * 150/ 180;
		lowerbound = wraparound(lowerbound);
	}
>>>>>>> 4fe40d94807dc0918ca6567664966a60b5ae0759
};
#endif
