/*
 * SHARED DWA runs in its own thread.
 */

#ifndef SHARED_DWA_H
#define SHARED_DWA_H

#define  USE_MATH_DEFINES
#define 	DEBUG

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

class SharedDWA {
public:
	SharedDWA(const char * topic, ros::NodeHandle &n_t);
	~SharedDWA();
	void run();
private:

	vector<Speed> inputDist;
	// -----------Occupancy Grid Variables-------------------
	/*
	 * The goalstep is a measure used for our distribution to
	 * determine the resolution of our speed distribution measured
	 * in degrees on an Argand chart of w againt v.
	 */
	float refresh_period; // rate at which we evaluate prediction as measures in counts representing time in seconds
	DWAMap* dwa_map;

	float gridsize; // The size in m of each occupancy grid.
	float mapsize;
	int noOfgrids; // number of grids in one side of the square occupancy.

//------------ Motor Variables ---------------
	Speed humanInput;
	Speed odom;

//-------------------ROS-----------------------
	ros::NodeHandle n;
	ros::Publisher command_pub;
	ros::Subscriber interface_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber occupancy_sub;

	float dt;

	void occupancyCallback(const nav_msgs::OccupancyGrid& og);
	void odomCallback(const nav_msgs::Odometry& cmd);
	void usercommandCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd);
	void updateInputCommand(float v, float w, InterfaceType In);

// ----------------------WC Kinematics------------------------
	float acc_lim_v, acc_lim_w;
	float decc_lim_v, decc_lim_w;
	float max_trans_vel, min_trans_vel;
	float max_rot_vel, min_rot_vel;

	float wc_length, wc_width;
//	 ----------------------Visualisation------------------------
#ifdef DEBUG
	visualization_msgs::MarkerArray clearance_ma;
	visualization_msgs::MarkerArray G_ma;
	visualization_msgs::MarkerArray coupling_ma;
	visualization_msgs::MarkerArray heading_ma;
	visualization_msgs::MarkerArray velocity_ma;
	ros::Publisher markerarray_pub;
	ros::Publisher G_pub;
	ros::Publisher heading_pub;
	ros::Publisher clearance_pub;
	ros::Publisher velocity_pub;
	ros::Publisher coupling_pub;

	float vstep, wstep;
	void visualiseCostFn();
	void addCostFn(Speed speed, float G, float heading, float clearance,
			float velocity, float coupling);
#endif
	/*
	 * These parameters are used for accessing the right back side of the wheelchair
	 * as the start position to fill or check occupancy.
	 */
	float length_offset, width_offset;

	// inverse of normalise speed.
	Speed getRealSpeed(Speed speed_old) {
		Speed speed = Speed(speed_old);
		if (speed.v >= 0) {
			speed.v *= ( MAX_LIN_VEL);
		} else {
			speed.v *= ( MIN_LIN_VEL);
		}
		if (speed.w >= 0) {
			speed.w *= ( MAX_ANG_VEL);
		} else {
			speed.w *= ( MIN_ANG_VEL);
		}
		return speed;
	}

	Speed normaliseSpeed(Speed speed_old) {
		Speed speed = Speed(speed_old);
		if (speed.v >= 0) {
			speed.v /= ( MAX_LIN_VEL);
		} else {
			speed.v /= ( MIN_LIN_VEL);
		}
		if (speed.w >= 0) {
			speed.w /= ( MAX_ANG_VEL);
		} else {
			speed.w /= ( MIN_ANG_VEL);
		}
		return speed;
	}
// -------------DWA----------
	/*
	 * Angles made by normalised linear and angular velocities on an
	 * argand chart are stored as trajectory parameters.
	 */
	vector<float> trajectories;
// Assuming const time horizon as goal.
	float horizon; // time steps in the future.
	float computeHeading(Speed usercommand, Speed candidateSpeed);
	float computeClearance(Speed candidateSpeed);
	float computeDistToNearestObstacle(Speed candidateSpeed);
	vector<Pose> getObstacles(Pose pose, vector<Pose> obstacles);
	float computeVelocity(Speed candidateSpeed);
	vector<Speed> getAdmissibleVelocities(vector<Speed> admissibles);
	DynamicWindow computeDynamicWindow(DynamicWindow dw);
	vector<Speed> getResultantVelocities(vector<Speed> resultantVelocities);
	Speed computeNextVelocity(Speed chosenSpeed);
// ---------------Trigonometry----------------------
	float angDiff(float a1, float a2) {
		float a = a1 - a2;
		a = fmod((a + M_PI), (2 * M_PI)) - M_PI;
		return a;
	}

	float wraparound(float th) { // [-pi, pi]
		if (th > M_PI)
			th -= 2 * M_PI;
		if (th < -M_PI)
			th += 2 * M_PI;
		return th;
	}
	float vectorNorm(Pose p) {
		std::vector<float> v { p.x, p.y };
		float res = inner_product(v.begin(), v.end(), v.begin(), 0.0f);
		return sqrt(res);
	}
	float vectorNorm(Speed p) {
		return sqrt(magSquared(p));
	}
	float magSquared(Speed p) {
		std::vector<float> v { p.v, p.w };
		float res = inner_product(v.begin(), v.end(), v.begin(), 0.0f);
		return res;
	}
	float sqrt_approx(float z);

}
;

#endif
