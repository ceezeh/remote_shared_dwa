/*
 * SHARED DWA runs in its own thread.
 */

#ifndef SHARED_DWA_H
#define SHARED_DWA_H

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

#include <math.h>
#include <numeric>
<<<<<<< HEAD
#define equals(x, y) (abs(x-y) < 0.001)
#define INVALIDCMD -2

#define MAX_LIN_VEL 0.1 //m/s
#define MIN_LIN_VEL 0.05 //m/s

#define MAX_ANG_VEL 1 //rads/s
#define MIN_ANG_VEL 1 //rads/s
=======
#include "shared_dwa/pose.h"
#include "shared_dwa/speed.h"
#include "shared_dwa/helper.h"
#include "shared_dwa/map.h"
>>>>>>> 4fe40d94807dc0918ca6567664966a60b5ae0759

#define INVALIDCMD -2
using namespace std;


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

//------------ Motor Variables ---------------
	Speed humanInput;
	Speed odom;

//-------------------ROS-----------------------
	ros::NodeHandle n;
	ros::Publisher command_pub;
	ros::Publisher usercommand_pub;

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
	float vstep, wstep;
<<<<<<< HEAD
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


	void visualiseCostFn();
	void addCostFn(Speed speed, float G, float heading, float clearance,
			float velocity, float coupling);
#endif
=======
>>>>>>> 4fe40d94807dc0918ca6567664966a60b5ae0759
	/*
	 * These parameters are used for accessing the right back side of the wheelchair
	 * as the start position to fill or check occupancy.
	 */
	float length_offset, width_offset;
<<<<<<< HEAD

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
	bool compareQuadrant(float ang, float upper, float lower);
	int getQuadrant (float upper);
=======
>>>>>>> 4fe40d94807dc0918ca6567664966a60b5ae0759
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
	vector<Speed> getAdmissibleVelocities(vector<Speed> admissibles, float upperbound, float lowerbound);
	DynamicWindow computeDynamicWindow(DynamicWindow dw);
	vector<Speed> getResultantVelocities(vector<Speed> resultantVelocities, float upperbound, float lowerbound );
	Speed computeNextVelocity(Speed chosenSpeed);
<<<<<<< HEAD
// ---------------Trigonometry----------------------
	float angDiff(float a1, float a2) {
		float a = a1-a2;
		a = fmod((a + M_PI), (2 * M_PI)) - M_PI;
		return wraparound(a);
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
=======
	DeOscillator deOscillator;
	void restrictVelocitySpace(float &upperbound, float &lowerbound,
			Speed inputcmd);
	void getData();
>>>>>>> 4fe40d94807dc0918ca6567664966a60b5ae0759

}
;

#endif
