#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <numeric>
#include <costmap/helper.h>
#include "shared_dwa/psc_dwa.h"
#include <chrono>

//#define LINEAR

#define USER_CMD_BIT 2

using namespace std;
using namespace std::chrono;
MyTimer timer = MyTimer();

PSCDWA::PSCDWA(const char * topic, ros::NodeHandle &n_t) :
		DWA(topic, n_t) {

	usercommand_pub = n.advertise<geometry_msgs::TwistStamped>(
			"user_command_logger", 100);
	clearance_pub = n.advertise<geometry_msgs::Vector3Stamped>("clearance", 10);
	interface_sub = n.subscribe("user_command", 1, &PSCDWA::usercommandCallback,
			this);
	this->DATA_COMPLETE = 3;
	this->coupling = 100;
	string ctopic = this->topic + "/coupling";
	this->n.getParam(ctopic.c_str(), this->coupling);
}

void PSCDWA::getInputCandidates(Speed input, vector<Distribution> &candidates) {
	Distribution p;
	float std = 0;
	if (this->usercmd.header.frame_id == "JS") {
		p.speed = input;
//		std = M_PI / 5;
		p.pval = 1;
//
//		//If straight backward or forward, return only one value.
		candidates.emplace_back(p);
		return;
	} else if (this->usercmd.header.frame_id == "HA") {
		std = M_PI / 5;
	} else if (this->usercmd.header.frame_id == "SP") {
		std = M_PI / 4;
	} else {
		p.speed = input;
		p.pval = 1;
		candidates.emplace_back(p);
		return;
	}

	float var = std * std; //(pi/5)^2

	p.speed = input;
	p.pval = 1 / pow(2 * M_PI * var, 0.5);

	//If straight backward or forward, return only one value.
	candidates.emplace_back(p);

	if (!((equals(input.w, 0) && input.v > 0))) {
		// find magnitude and angle.
		float th_0 = atan2(input.w, input.v);
		float mag = vectorNorm(input);
		int step = 1;
		float d = std / step;
		for (float dth = d; dth <= std; dth += d) {
			if (equals(dth, 0))
				continue;
			float th = th_0 - dth;
			float v = cos(th) * mag;
			float w = sin(th) * mag;
			float pval = 1; //exp(-dt * dt / (2 * var)) / pow(2 * var * M_PI, 0.5);
			Distribution p;
			p.speed.v = v;
			p.speed.w = w; //= Speed(v, w);
			p.pval = pval;
			candidates.emplace_back(p);

			th = th_0 + dth;
			v = cos(th) * mag;
			w = sin(th) * mag;
			p.speed = Speed(v, w);
			candidates.emplace_back(p);

		}
	}
//	cout << "emplacing input vel....." << endl;

	//Print out inputs
	for (int i = 0; i < candidates.size(); i++) {
		cout << "input v:" << candidates[i].speed.v << ", w:"
				<< candidates[i].speed.w << endl;
	}
}
void PSCDWA::usercommandCallback(geometry_msgs::TwistStamped cmd) {

//	this->usercmd.twist.linear.x=1;
//	this->usercmd.twist.angular.z =0.5;
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
	float length = 2;	// As in 1 meter.
	dx = length * cos(th);
	dy = length * sin(th);
	// Here we are assuming the robot will use this speed for the next few time steps. // This could be because of latency

//	cout << "Requested Direction: " << th << endl;
	Pose currentPose;
	this->getCurrentPose(currentPose);
	// Now dx, dy are in the body frame and will need to be rotated to the global frame.
	float xt = cos(currentPose.th) * dx - sin(currentPose.th) * dy;
	float yt = sin(currentPose.th) * dx + cos(currentPose.th) * dy;
	Pose newGoalPose = currentPose;
	if (!(equals(humanInput.v,0) && equals(humanInput.w, 0)))
		newGoalPose = newGoalPose + Pose(xt, yt, th);
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
		usercommandCallback(usercmd);
	}
	usercommand_pub.publish(this->usercmd);
	dataflag = 0;
//	cout << " exiting data acquisition" << endl;

}
/*
 * This is the part that does the probabilistic conditioning based on the user's input.
 */

/*
 * Let G (v,w) = Fn( ������ ������ heading + ������������clearance + ������������velocity), where Fn is arbitrary.
 * C = G(v,w) ��������� exp(-1/(2*s)*(f-h)(f-h)') ���������  W(H), for each of our possible goal location.
 */
Speed PSCDWA::computeNextVelocity(Speed chosenSpeed) {
	timer.start();
	Speed humanInput = Speed(this->usercmd.twist.linear.x,
			usercmd.twist.angular.z);

	if (humanInput == Speed(0, 0)) {
//		cout << "Stopping!";
		return Speed(0, 0); // Stop!!
	}

	concurrent_vector<Speed> resultantVelocities;
	resultantVelocities.clear();

	/*
	 * Create optimized search space.
	 */
	float upperbound = -M_PI - 1;
	float lowerbound = M_PI + 1;

	deOscillator.getAdmissibleDirection(upperbound, lowerbound);
	ROS_INFO("upperbound: %f, lowerbound: %f", upperbound, lowerbound);
	resultantVelocities = getResultantVelocities(resultantVelocities,
			upperbound, lowerbound);
	float maxCost = 0;
	// More means permit less agreement. default .1 , 10 is more assistance. 100 much more asistance
	// Put weightings here
	float a = this->coupling; // agreement factor between user command and resultant velocity.
	if ((equals(humanInput.w, 0) && humanInput.v > 0) ||
			(equals(humanInput.v, 0) && !equals(humanInput.w, 0))) {
		a = .100;
	}
	//	cout << "COUPLING=" << a << endl;
	float inva = 1 / a;
	float alpha = .06;		// For heading.
	float beta = .5;		// For clearance.
	float gamma = 1;		// For velocity.
	float final_clearance = 0;
	cout << "Number of resultant velocities" << resultantVelocities.size()
			<< endl;
	timer.stop();
//	ROS_INFO("PSC Getting Resultant Velocities: Max Duration: %d", timer.getMaxDuration());
//	ROS_INFO("PSC Getting Resultant Velocities: Average Duration: %d ", timer.getAveDuration());
//	ROS_INFO("PSC Getting Resultant Velocities: Last Duration: %d ", timer.getLastDuration());

	vector<Distribution> inputDistribution;
	inputDistribution.clear();
	getInputCandidates(humanInput, inputDistribution);

	Pose currentPose;
	this->getCurrentPose(currentPose);
	std::mutex mylock;
//	cout << "Number of input candidates" << inputDistribution.size() << endl;
#pragma omp parallel for
	for (int j = 0; j < inputDistribution.size(); j++) {
		for (int i = 0; i < resultantVelocities.size(); i++) {
//			timer.start();
			Distribution d = inputDistribution[j];
			Speed input = d.speed;

			Speed realspeed = resultantVelocities[i];
			const Pose goalpose = this->getGoalPose();
			float heading = computeHeading(realspeed, goalpose);//This is an issue though.
			float clearance = computeClearance(realspeed);
			if (equals(clearance, 0))
				continue;
			float velocity = computeVelocity(realspeed);
			float G = alpha * heading + beta * clearance + gamma * velocity;

			// Compute preference for user's input
			// 1 here is the weighting for that particular input speed,
			// which will change in time!
			Speed temp = normaliseSpeed(realspeed) - input;
			float x = magSquared(temp);
			x *= -0.5 * inva;
			float coupling = expf(x);
			float cost = G * coupling*d.pval;

			ROS_INFO(
					"Printing out PSCDWA parameters for specific velocity ...");
			ROS_INFO(
					"Candidate Vel[v = %f, w= %f],Inpt Vel[v = %f, w= %f]  heading=%f, clearance=%f, "
							"velocity = %f, coupling = %f, G = %f,"
							"cost = %f, Goal Pose (x: %f, y: %f, th: %f), Current Pose (x: %f, y: %f, th: %f)",
					realspeed.v, realspeed.w, input.v, input.w, heading,
					clearance, velocity, coupling, G, cost, goalpose.x,
					goalpose.y, goalpose.th, currentPose.x, currentPose.y,
					currentPose.th);
			mylock.lock();
			if (cost > maxCost) {
				maxCost = cost;

				chosenSpeed = realspeed;

				final_clearance = clearance;
			}
			mylock.unlock();
//			timer.stop();
//			ROS_INFO("PSC Max Duration: %d", timer.getMaxDuration());
//			ROS_INFO("PSC Average Duration: %d ", timer.getAveDuration());
//			ROS_INFO("PSC Last Duration: %d ", timer.getLastDuration());
		}
	}

	ROS_INFO("Chosen speed: [v=%f, w=%f]", chosenSpeed.v, chosenSpeed.w);
	geometry_msgs::Vector3Stamped c;
	c.header.stamp = ros::Time::now();
	c.vector.z = final_clearance;
	c.vector.x = chosenSpeed.v;
	c.vector.y = chosenSpeed.w;
	clearance_pub.publish(c);
	return chosenSpeed;
}
