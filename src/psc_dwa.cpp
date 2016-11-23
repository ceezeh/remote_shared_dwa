#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <numeric>
#include <map/helper.h>
#include "shared_dwa/psc_dwa.h"
#include <chrono>

//#define LINEAR

#define USER_CMD_BIT 2

using namespace std;
using namespace std::chrono;

PSCDWA::PSCDWA(const char * topic, ros::NodeHandle &n_t) :
		DWA(topic, n_t) {

	usercommand_pub = n.advertise < geometry_msgs::TwistStamped
			> ("user_command_logger", 100);
	interface_sub = n.subscribe("user_command", 1,
			&PSCDWA::usercommandCallback, this);
	this->DATA_COMPLETE = 3;
}

void PSCDWA::usercommandCallback(geometry_msgs::TwistStamped cmd) {

	//-------------------------------------------------------
	// Update dataflag.
	dataflag |= (1 << USER_CMD_BIT);
	this->usercmd = geometry_msgs::TwistStamped(cmd);

	Speed humanInput = Speed(this->usercmd.twist.linear.x,
			usercmd.twist.angular.z);
	//-------------------------------------------------------

	// Here, we estimate the goal pose from the user command.
	float dx, dy, th;
	th = atan2(humanInput.w, humanInput.v);
	float length = 1;	// As in 1 meter.
	dx = length * cos(th);
	dy = length * sin(th);
	// Here we are assuming the robot will use this speed for the next few time steps. // This could be because of latency

	cout << "Requested Direction: " << th << endl;
	Pose currentpose = Pose(odom_all.pose.pose.position.x,
			odom_all.pose.pose.position.y, odom_all.pose.pose.position.z);
	// Now dx, dy are in the body frame and will need to be rotated to the global frame.
	float xt = cos(currentpose.th) * dx - sin(currentpose.th) * dy;
	float yt = sin(currentpose.th) * dx + cos(currentpose.th) * dy;
	Pose newGoalPose = currentpose + Pose(xt, yt, th);
	this->updateGoalPose(newGoalPose, th);

}

void PSCDWA::getData() {
	while (dataflag != this->DATA_COMPLETE && ros::ok()) { // Data bits are arranged n order of testing priority.
		if (dataflag & 1 << USER_CMD_BIT) {
			break;
		}
		ros::spinOnce();
	}
	if (!(dataflag & 1 << USER_CMD_BIT)) {
		usercommandCallback (usercmd);
	}
	dataflag = 0;
	cout << " exiting data acquisition" << endl;

}
/*
 * This is the part that does the probabilistic conditioning based on the user's input.
 */

/*
 * Let G (v,w) = Fn( �� �� heading + ����clearance + ����velocity), where Fn is arbitrary.
 * C = G(v,w) ��� exp(-1/(2*s)*(f-h)(f-h)') ���  W(H), for each of our possible goal location.
 */
Speed PSCDWA::computeNextVelocity(Speed chosenSpeed) {

	Speed humanInput = Speed(this->usercmd.twist.linear.x,
			usercmd.twist.angular.z);
	if (humanInput == Speed(0, 0)) {
		cout << "Stopping!";
		return Speed(0, 0); // Stop!!
	}

	vector < Speed > resultantVelocities;
	resultantVelocities.clear();

	/*
	 * Create optimized search space.
	 */
	float upperbound = -M_PI - 1;
	float lowerbound = M_PI + 1;
	Speed input = humanInput;

	deOscillator.getAdmissibleDirection(upperbound, lowerbound);
	ROS_INFO("upperbound: %f, lowerbound: %f", upperbound, lowerbound);
	resultantVelocities = getResultantVelocities(resultantVelocities,
			upperbound, lowerbound);
	float maxCost = 0;
	// More means permit less agreement. default .1 , 10 is more assistance. 100 much more asistance
	// Put weightings here
	float a = 100; // agreement factor between user command and resultant velocity.
	string topic = this->topic + "/coupling";
	this->n.getParam(topic.c_str(), a);
	cout << "COUPLING=" << a << endl;
	float alpha = 0.1;	// For heading.
	float beta = 0.4;	// For clearance.
	float gamma = 1;	// For velocity.
	float final_clearance = 0;
	cout << "Number of resultant velocities" << resultantVelocities.size()
			<< endl;
	for (int i = 0; i < resultantVelocities.size(); i++) {

		Speed realspeed = resultantVelocities[i];
		const Pose goalpose = this->getGoalPose();
		float heading = computeHeading(realspeed, goalpose);
		float clearance = computeClearance(realspeed);

		float velocity = computeVelocity(realspeed);
		float G = alpha * heading + beta * clearance + gamma * velocity;

		// Compute preference for user's input
		// 1 here is the weighting for that particular input speed,
		// which will change in time!
		Speed temp = normaliseSpeed(realspeed) - humanInput;
		float x = magSquared(temp);
		x *= -0.5 / a;
		float coupling = expf(x);
		float cost = G * coupling;
		Speed robotSpeed = normaliseSpeed(realspeed);
		float ang = atan2(humanInput.w, humanInput.v);
		float candidate_ang = atan2(robotSpeed.w, robotSpeed.v);

		ROS_INFO("Printing out PSCDWA parameters for specific velocity ...");
		ROS_INFO(
				"RealVel[v = %f, w= %f], heading=%f, clearance=%f, "
						"velocity = %f, coupling = %f, G = %f, cost = %f, Goal Pose (x: %f, y: %f, th: %f), Current Pose (x: %f, y: %f, th: %f)",
				realspeed.v, realspeed.w, heading, clearance, velocity,
				coupling, G, cost, goalpose.x, goalpose.y, goalpose.th,
				odom_all.pose.pose.position.x, odom_all.pose.pose.position.y,
				odom_all.pose.pose.position.z);
		if (cost > maxCost) {
			maxCost = cost;

			chosenSpeed = realspeed;

			final_clearance = clearance;
		}
	}

	ROS_INFO("Chosen speed: [v=%f, w=%f]", chosenSpeed.v, chosenSpeed.w);
	return chosenSpeed;
}
