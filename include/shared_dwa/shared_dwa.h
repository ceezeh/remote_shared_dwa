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
#include <dwa/dwa.h>

#define INVALIDCMD -2
using namespace std;


class SharedDWA: public DWA {
public:
	SharedDWA(const char * topic, ros::NodeHandle &n_t);
private:

	vector<Speed> inputDist;
	//------------ Motor Variables ---------------
	geometry_msgs::TwistStamped usercmd;
	ros::Publisher usercommand_pub;
	ros::Subscriber interface_sub;

	void usercommandCallback( geometry_msgs::TwistStamped cmd);
	void updateInputCommand(float v, float w, InterfaceType In);

// -------------DWA----------
	Speed computeNextVelocity(Speed chosenSpeed);
	void getData();


}
;

#endif
