/*
 * SHARED DWA runs in its own thread.
 */

#ifndef PSC_DWA_H
#define PSC_DWA_H

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

typedef struct {Speed speed;float pval;} Distribution;

#define INVALIDCMD -2
using namespace std;


class PSCDWA: public DWA {
public:
	PSCDWA(const char * topic, ros::NodeHandle &n_t);
protected:
	// -------------DWA----------
	virtual Speed computeNextVelocity(Speed chosenSpeed);
	void getInputCandidates(Speed input, vector<Distribution> &candidates);
	geometry_msgs::TwistStamped usercmd;

private:
	//------------ Motor Variables ---------------

	ros::Publisher usercommand_pub;
	ros::Publisher clearance_pub;
	ros::Subscriber interface_sub;

	void usercommandCallback( geometry_msgs::TwistStamped cmd);
	void updateInputCommand(float v, float w, InterfaceType In);

// -------------DWA----------
	void getData();
}
;

#endif
