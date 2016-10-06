#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <numeric>
#include <map/helper.h>
#include "shared_dwa/shared_dwa.h"
#include <chrono>

#define LINEAR

#define USER_CMD_BIT 2
#define ODOM_BIT 1
#define MAP_BIT 0

using namespace std;
using namespace std::chrono;

int dataflag = 0;
SharedDWA::SharedDWA(const char * topic, ros::NodeHandle &n_t) {

	// Trajectories.
	for (float i = -M_PI; i < M_PI; i += 0.5235987756) { // Split into 12 angles  for each quadrant.
		trajectories.push_back(i);
	}
	dt = 0.2; // seconds.

	//	 TODO: Verify these parameters.
	horizon = 20 / dt; // 10 seconds
	refresh_period = 0; // 1 / dt;
	// WC kinematics
	acc_lim_v = 0.06 * 20; // 0.06 original but tooooo small. Tooooo.
	acc_lim_w = 3;
	decc_lim_v = -0.96;
	decc_lim_w = -3;
	max_trans_vel = MIN_LIN_VEL;
	min_trans_vel = -MIN_LIN_VEL;
	max_rot_vel = MAX_ANG_VEL;
	min_rot_vel = -MIN_ANG_VEL;
	// WC dimensions.
	wc_length = 1.300; //m
	wc_width = .800; //m

	vstep = (max_trans_vel - min_trans_vel) / 5;
	wstep = (max_rot_vel - min_rot_vel) / 5;

	/*
	 * These parameters are used for accessing the right back side of the wheelchair
	 * as the start position to fill or check occupancy.
	 */
	length_offset = .50;
	width_offset = wc_width / 2;

	humanInput = Speed(0, 0);
	odom = Speed(0, 0);
	float gridsize = 0.05;
	float mapsize = 4;
	dwa_map = new DWAMap(gridsize, mapsize);

	deOscillator = DeOscillator();
	// ROS
	n = n_t;
	command_pub = n.advertise < geometry_msgs::TwistStamped
			> ("motor_command", 10);
	usercommand_pub = n.advertise < geometry_msgs::TwistStamped
			> ("user_command_logger", 100);
	odom_sub = n.subscribe("odom", 1, &SharedDWA::odomCallback, this);
	interface_sub = n.subscribe("user_command", 1,
			&SharedDWA::usercommandCallback, this);
	occupancy_sub = n.subscribe("local_map", 1, &SharedDWA::occupancyCallback,
			this);

	dataflag = 0;
//	while (occupancy_sub.getNumPublishers() == 0)
//		;
//	sleep(5);
}

SharedDWA::~SharedDWA() {
	delete this->dwa_map;
}

void SharedDWA::odomCallback(const nav_msgs::Odometry& cmd) {
	// Update dataflag.
	dataflag |= (1 << ODOM_BIT);
	// update v
	this->odom.v = cmd.twist.twist.linear.x;
//#ifdef DEBUG
	ROS_INFO("I heard something, v= %f", this->odom.v);
//#endif

	this->odom.w = cmd.twist.twist.angular.z; // scaling factor that maps user's command to real world units.
//#ifdef DEBUG
			ROS_INFO("I heard something, w= %f", this->odom.w);
//#endif

	deOscillator.updateOdom(cmd);
}

void SharedDWA::occupancyCallback(const nav_msgs::OccupancyGrid& og) {
	// Update dataflag.
	dataflag |= (1 << MAP_BIT);
	this->dwa_map->updateMap(og);

}

void SharedDWA::usercommandCallback(
		const geometry_msgs::TwistStamped::ConstPtr& cmd) {
	// Update dataflag.
	dataflag |= (1 << USER_CMD_BIT);
	if (!equals(cmd->twist.linear.x, INVALIDCMD)) {
		// update v
//		this->humanInput.v = .1;
		this->humanInput.v = cmd->twist.linear.x;
#ifdef DEBUG
		ROS_INFO("I heard something, v= %f", this->humanInput.v);
#endif
	}

	if (!equals(cmd->twist.angular.z, INVALIDCMD)) {
//		this->humanInput.w = 0.5;
		this->humanInput.w = cmd->twist.angular.z;
#ifdef DEBUG
		ROS_INFO("I heard something, w= %f", this->humanInput.w);
#endif
	}
}

/*
 * Heading is defined in the first paper on DWA
 * as the bearing of the robot’s direction from the goal,
 * such that heading is maximum when the robot is facing the goal.
 * It is a measure to motivate the robot to progress towards the goal.
 * It is computed at the position the robot will be in after maximum deceleration from the next time step.
 *
 * We normalise this value to the -1,1 range.
 */
float SharedDWA::computeHeading(Speed usercommand, Speed candidateSpeed) {
	// Compute goal from user command.

	// All calculations is done is local frame.
	// Normalise user's speed first.
	Speed goal = normaliseSpeed(usercommand);
	/*
	 * normalise the test speed as well.
	 * Here we are asking what speed would closely match the user's
	 * intention given his joystick angle of deflection.
	 * We assume that the user is aiming for an instantenous speed
	 * rather than that speed over a long distance.
	 * Thus there is no need to predict intention as a position.
	 * Rather it is sufficient to consider prediction over intended
	 * instantaneous direction and magnitude of motion (i.e over velocity space)
	 */
	float goalth = atan2(goal.w, goal.v); // gets the starting motion direction from the user's input.
	Speed test = normaliseSpeed(candidateSpeed);
	float candididateth = atan2(test.w, test.v);

	// Compute heading here.
	float heading = M_PI - angDiff(candididateth, goalth);
	// Normalise heading to [0,1]
	heading = wraparound(heading);
	heading = fabs(heading) / M_PI;
	return heading;
}

/*
 * Clearance measures the distance to the closest obstacle on the trajectory.
 * TODO: Populate values on a lookup table to prevent re-evaluation.
 */

float SharedDWA::computeClearance(Speed candidateSpeed) {
	// clearance is normalised to [0,1] by d
	if (candidateSpeed == Speed(0, 0)) {
		return .0;
	}
	float x = computeDistToNearestObstacle(candidateSpeed);
#ifdef DEBUG
	ROS_INFO("Pre clearance: %f",x);
#endif
	x = (x < .1) ? 0 : x / 2.550;
//	x = (x > 1) ? 1 : x;
//	x /= 2.55;

//	float clearance = -1+ 2	/ (1+ exp(-2* sqrt_approx(x)));
	return x;

}

float SharedDWA::computeDistToNearestObstacle(Speed candidateSpeed) {
	// Compute rectangular dimension depicting wheelchair in  occupancy map.
	// Compute and normalize clearance.
	float x, y, th;
	x = y = th = 0;
	float clearance = 2.55; // 2.55m is maximum detectable distance by sonar
	int count = refresh_period; // we want 2 seconds = 2/dt counts
	for (int i = 0; i < horizon; i++) {
		x += candidateSpeed.v * cos(th) * dt;
		y += candidateSpeed.v * sin(th) * dt;
		th += candidateSpeed.w * dt;
		th = wraparound(th);
		// the below is expensive to compute so
		// compute only after every 2 seconds into the future.
		if (count-- <= 0) {
			count = refresh_period;

			Pose pose = Pose(x, y, th);
			vector < Pose > obstacles;
			obstacles.clear();
			obstacles = getObstacles(pose, obstacles);
			if (obstacles.empty()) {
				continue;
			} else {
//				for (int j = 0; j < obstacles.size(); j++) {
//					Pose pose = obstacles[j];
//					float score = vectorNorm(pose);
//					if (score < clearance) {
//						clearance = score;
//					}
//				}
				clearance = vectorNorm(pose);
				return clearance;
			}
		}
	}
	return clearance;
}

/*
 * Checks if there is any obstacle in the occupancy grid to obstruct the
 * wheelchair if its centre where at pose. Returns the location of the obstacles.
 */
vector<Pose> SharedDWA::getObstacles(Pose pose, vector<Pose> obstacles) {
	// Get corners of wheelchair rectangle.
	RealPoint topLeft = RealPoint(-length_offset, width_offset);
	RealPoint topRight = RealPoint(wc_length - length_offset, width_offset);
	RealPoint bottomRight = RealPoint(wc_length - length_offset, -width_offset);
	RealPoint bottomLeft = RealPoint(-length_offset, -width_offset);
	// Transform corners into pose coordinate.
	rotateFromBody(topLeft, pose);
	rotateFromBody(topRight, pose);
	rotateFromBody(bottomLeft, pose);
	rotateFromBody(bottomRight, pose);

	// Now get the map equivalent of points.
	IntPoint topLeftInt = IntPoint(dwa_map->realToMap(topLeft.x),
			dwa_map->realToMap(topLeft.y));
	IntPoint topRightInt = IntPoint(dwa_map->realToMap(topRight.x),
			dwa_map->realToMap(topRight.y));
	IntPoint bottomLeftInt = IntPoint(dwa_map->realToMap(bottomLeft.x),
			dwa_map->realToMap(bottomLeft.y));
	IntPoint bottomRightInt = IntPoint(dwa_map->realToMap(bottomRight.x),
			dwa_map->realToMap(bottomRight.y));

	// Compute the outline of the rectangle.

	vector < IntPoint > outline;
	bresenham(topLeftInt.x, topLeftInt.y, topRightInt.x, topRightInt.y,
			outline);

	bresenham(topRightInt.x, topRightInt.y, bottomRightInt.x, bottomRightInt.y,
			outline);
	bresenham(bottomRightInt.x, bottomRightInt.y, bottomLeftInt.x,
			bottomLeftInt.y, outline);
	bresenham(bottomLeftInt.x, bottomLeftInt.y, topLeftInt.x, topLeftInt.y,
			outline);

	for (int i = 0; i < outline.size(); i++) {
		IntPoint point = outline[i];
		if (this->dwa_map->at(point.x, point.y) > 0) {
			RealPoint realpoint;
			dwa_map->mapToReal(point.x, point.y, realpoint);
			obstacles.emplace_back(realpoint.x, realpoint.y, 0);
		}
	}
	return obstacles;
}

/*
 *  This function normalises s the forward velocity of the robot and supports fast motion.
 *  We want to include angular velocity as well so as to obey the user's commands whilst supporting faster motion.
 *
 */
float SharedDWA::computeVelocity(Speed candidateSpeed) {
	// Normalise velocity to [0, 1] range.
	Speed speed = normaliseSpeed(candidateSpeed);
	return vectorNorm(speed);
}
/*
 * Function gets all possible admissible velocities according to the paper.
 * The returned velocities are indexed to align with the trajectory vector variable.
 *
 * This function really establishes an upperbound on velocity since all very slow speeds are theoretically reachable.
 * We optimize this function by only considering trajectories a certain degree of user input.
 */
vector<Speed> SharedDWA::getAdmissibleVelocities(vector<Speed> admissibles,
		float upperbound = M_PI + 1, float lowerbound = -M_PI - 1) {
	admissibles.clear();
	// We search for velocities for by keeping a list of radii of curvature.
	for (int i = 0; i < trajectories.size(); i++) {
		// Construct a speed object from angle and find the clearance.

//		if ((angDiff(trajectories[i], upperbound) > 0)
//				|| (angDiff(trajectories[i], lowerbound) < 0)) {
		if (!isAngleInRegion(trajectories[i], upperbound, lowerbound)) {
			admissibles.emplace_back(0, 0); // This is because the list of admissibles must match with the list of trajectories.
			continue;
		}
		Speed trajectory;
		if ((trajectories[i] < M_PI / 2) && ((trajectories[i] >= -M_PI / 2))) { // +ve v space
			trajectory.v = 0.1 * max_trans_vel;
		} else {
			trajectory.v = 0.1 * min_trans_vel;
		}

		// need to convert velocities from normalised to real values.
		trajectory.w = trajectory.v * tan(trajectories[i]);

		float dist = computeDistToNearestObstacle(trajectory);
		float va = copysign(sqrt(fabs(2 * dist * decc_lim_v)), trajectory.v);
		// Here we are simply getting the velocity restriction for each trajectory.
		// In this case, wa is bound to va so a slight deviation from the paper's wa calculation.
		float wa = va * tan(trajectories[i]);

		admissibles.emplace_back(va, wa);
	}
	return admissibles;
}

DynamicWindow SharedDWA::computeDynamicWindow(DynamicWindow dw) {
	dw.upperbound.v = odom.v + acc_lim_v * dt;
	dw.upperbound.w = odom.w + acc_lim_w * dt;
	dw.lowerbound.v = odom.v - acc_lim_v * dt;
	dw.lowerbound.w = odom.w - acc_lim_w * dt;
	return dw;
}

vector<Speed> SharedDWA::getResultantVelocities(
		vector<Speed> resultantVelocities, float upperbound = M_PI + 1,
		float lowerbound = -M_PI - 1) {

	DynamicWindow dw;
	dw = computeDynamicWindow(dw);
	vector < Speed > admissibles;
	admissibles.clear();
	admissibles = getAdmissibleVelocities(admissibles, upperbound, lowerbound);

	bool zeroVisited = false; // ensures that we add v=w= 0 only once.
	for (int i = 0; i < trajectories.size(); i++) {
		cout << "Trajectory Heading : " << trajectories[i] << endl;
//		if ((angDiff(trajectories[i], upperbound) > 0)
//				|| (angDiff(trajectories[i], lowerbound) < 0)) {
//			continue;
//		}
		if (!isAngleInRegion(trajectories[i], upperbound, lowerbound))
			continue;
		cout << "Passed trajectory Heading : " << trajectories[i] << endl;
		// for very large tan, the data becomes skewed so just use the dw as boundary
		//Here trajectory is either pi/2 or -PI/2
		if (equals(abs(trajectories[i]), M_PI / 2)) {
			float vel = 0;
			float upperbound_w, lowerbound_w;
			if (equals(trajectories[i], M_PI / 2)) {
				upperbound_w = dw.upperbound.w;
				lowerbound_w = (dw.lowerbound.w < 0) ? 0 : dw.lowerbound.w;
			} else {
				upperbound_w = (dw.upperbound.w > 0) ? 0 : dw.upperbound.w;
				lowerbound_w = dw.lowerbound.w;
			}
			upperbound_w = min(upperbound_w, max_rot_vel);
			lowerbound_w = max(lowerbound_w, min_rot_vel);
			float stepw = (upperbound_w - lowerbound_w) / 6;
//			cout << "upperbound: " << upperbound << "lowerbound" << lowerbound
//					<< "upperbound_w: " << upperbound_w << "lowerbound_w"
//					<< lowerbound_w << endl;
			for (float w = lowerbound_w; w < upperbound_w; w += stepw) {
				if ((w >= lowerbound_w) && ((w <= upperbound_w))) {
//					cout
//							<< "isAngleInRegion(atan2(w, vel), upperbound, lowerbound)"
//							<< isAngleInRegion(atan2(w, vel), upperbound,
//									lowerbound) << endl;
					if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
						continue;
					} else if (equals(vel, 0) && (equals(w, 0))) {
						zeroVisited = true;
					}
					if (isAngleInRegion(atan2(w, vel), upperbound,
							lowerbound)) {
						resultantVelocities.emplace_back(vel, w);

					}
				}
			}
			continue;
		}

		// For small tan near zero, w = 0

		if (abs(tan(trajectories[i])) < 0.001) {
			float w = 0;
			float lowerbound_v, upperbound_v;
			if ((trajectories[i] < M_PI / 2)
					&& ((trajectories[i] >= -M_PI / 2))) { // +ve v space
				// So compute lowerbound on trajectories.

				upperbound_v = min(dw.upperbound.v, admissibles[i].v);
				lowerbound_v = dw.lowerbound.v;
			} else {
				upperbound_v = dw.upperbound.v;
				lowerbound_v = max(dw.lowerbound.v, admissibles[i].v);
			}
			upperbound_v = min(upperbound_v, max_trans_vel);
			lowerbound_v = max(lowerbound_v, min_trans_vel);
			/*
			 * We are assuming focus within a narrow angle so that 0 and PI can not both be within focus.
			 * Thus if zero is present, pi is not.
			 */
			if (isAngleInRegion(0, upperbound, lowerbound)) { // If 0 is present.
				lowerbound_v = (lowerbound_v < 0) ? 0 : lowerbound_v;
			} else {
				upperbound_v = (upperbound_v > 0.0) ? 0 : upperbound_v;
			}
			float step = (upperbound_v - lowerbound_v) / 3;
			for (float vel = lowerbound_v; vel <= upperbound_v; vel += step) {
				if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
					continue;
				} else if (equals(vel, 0) && (equals(w, 0))) {
					zeroVisited = true;
				}
				if (isAngleInRegion(atan2(w, vel), upperbound, lowerbound))
					resultantVelocities.emplace_back(vel, w);
			}
			continue;
		}

		float lowerbound_v, upperbound_v;
		if ((trajectories[i] < M_PI / 2) && ((trajectories[i] >= -M_PI / 2))) { // +ve v space
			// So compute lowerbound on trajectories.

			upperbound_v = min(dw.upperbound.v, admissibles[i].v);
			lowerbound_v = dw.lowerbound.v;
		} else {
			upperbound_v = dw.upperbound.v;
			lowerbound_v = max(dw.lowerbound.v, admissibles[i].v);
		}
		upperbound_v = min(upperbound_v, max_trans_vel);
		lowerbound_v = max(lowerbound_v, min_trans_vel);

		float step = (upperbound_v - lowerbound_v) / 3;
		for (float vel = lowerbound_v; vel <= upperbound_v; vel += step) {

			// For small tan near zero, w = 0
			float w = vel * tan(trajectories[i]);

			float upperbound_w, lowerbound_w;
			if (tan(trajectories[i] < 0)) {
				upperbound_w = dw.upperbound.w;
				lowerbound_w = max(dw.lowerbound.w, admissibles[i].w);
			} else {
				upperbound_w = min(dw.upperbound.w, admissibles[i].w);
				lowerbound_w = dw.lowerbound.w;
			}
			upperbound_w = min(upperbound_w, max_rot_vel);
			lowerbound_w = max(lowerbound_w, min_rot_vel);
			if ((w >= lowerbound_w) && ((w <= upperbound_w))) {
				if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
					continue;
				} else if (equals(vel, 0) && (equals(w, 0))) {
					zeroVisited = true;
				}
				if (isAngleInRegion(atan2(w, vel), upperbound, lowerbound))
					resultantVelocities.emplace_back(vel, w);
			}

		}

	}

//	ROS_INFO("Printing resultant velocities ...\n");
//	for (int i = 0; i < resultantVelocities.size(); i++) {
//		ROS_INFO("Vel[%d]; [v = %f, w= %f]", i, resultantVelocities[i].v,
//				resultantVelocities[i].w);
//	}
//	ROS_INFO("Printing resultant velocities ended\n");
	return resultantVelocities;
}

void SharedDWA::restrictVelocitySpace(float &upperbound, float &lowerbound,
		Speed input) {
	float upperboundt, lowerboundt;
	deOscillator.getAdmissibleDirection(upperbound, lowerbound);
	float ang = atan2(input.w, input.v);

	upperboundt = ang + M_PI * 60 / 180;
	upperboundt = wraparound(upperboundt);
	lowerboundt = ang - M_PI * 60 / 180;
	lowerboundt = wraparound(lowerboundt);

	float upper, lower;
	if (isAngleInRegion(upperboundt, upperbound, lowerbound)) {
		upper = upperboundt;
	} else {
		upper = upperbound;
	}
	if (isAngleInRegion(lowerboundt, upperbound, lowerbound)) {
		lower = lowerboundt;
	} else {
		lower = lowerbound;
	}
	upperbound = upper;
	lowerbound = lower;
}

/*
 * This is the part that does the probabilistic conditioning based on the user's input.
 */

/*
 * Let G (v,w) = Fn( α · heading + β·clearance + Ɣ·velocity), where Fn is arbitrary.
 * C = G(v,w) ⨉ exp(-1/(2*s)*(f-h)(f-h)') ⨉  W(H), for each of our possible goal location.
 */
Speed SharedDWA::computeNextVelocity(Speed chosenSpeed) {

	if (humanInput == Speed(0, 0))
		return Speed(0, 0); // Stop!!

	vector < Speed > resultantVelocities;
	resultantVelocities.clear();

	/*
	 * Create optimized search space.
	 */
	float upperbound = -M_PI - 1;
	float lowerbound = M_PI + 1;
	Speed input = humanInput;

	restrictVelocitySpace(upperbound, lowerbound, input);
//	deOscillator.getAdmissibleDirection(upperbound, lowerbound);
	ROS_INFO("upperbound: %f, lowerbound: %f", upperbound, lowerbound);
	resultantVelocities = getResultantVelocities(resultantVelocities,
			upperbound, lowerbound);
	float maxCost = 0;
	float a = .1; // agreement factor between user command and resultant velocity. More means less agreement.
	// Put weightings here.
	float alpha = 0.1;	// For heading.
	float beta = 0.4;	// For clearance.
	float gamma = 0.2;	// For velocity.
	float final_clearance = 0;
	cout << "Number of resultant velocities" << resultantVelocities.size()
			<< endl;
	for (int i = 0; i < resultantVelocities.size(); i++) {
#ifdef LINEAR
		Speed speed = resultantVelocities[i];
		const Speed input = (const Speed) (getRealSpeed(humanInput));
		float heading = computeHeading(input, speed);
		float clearance = computeClearance(speed);

		float velocity = computeVelocity(speed);
		float G = alpha * heading + beta * clearance + gamma * velocity;

		float cost = G;
#else
		Speed speed = resultantVelocities[i];
		const Speed input = (const Speed) (getRealSpeed(humanInput));
//			float heading = computeHeading(input, speed);
		float clearance = computeClearance(speed);

//			float velocity = computeVelocity(speed);
//			float G = alpha * heading + beta * clearance + gamma * velocity;
		// Compute preference for user's input
		// 1 here is the weighting for that particular input speed,
		// which will change in time!
		Speed temp = speed - input;
		temp = normaliseSpeed(temp);
		float x = magSquared(temp);
		x *= -0.5 / a;
		float coupling = expf(x);
		float inputWeight = 1;
		float cost = clearance * coupling * inputWeight;
		Speed robotSpeed = normaliseSpeed(speed);
		float ang = atan2(humanInput.w, humanInput.v);
		float candidate_ang = atan2(robotSpeed.w, robotSpeed.v);
//		ROS_INFO(
//				"Input[v = %f, w= %f], Vel[v = %f, w= %f], clearance=%f, "
//						" coupling = %f, cost = %f, input heading = %f, robot heading =%f",
//				input.v, input.w, speed.v, speed.w, clearance, coupling, cost,
//				ang, candidate_ang);
#endif
#ifdef DEBUG
		Speed robotSpeed = normaliseSpeed(speed);
		float candidate_ang = atan2(robotSpeed.w, robotSpeed.v);

		ROS_INFO("Printing out DWA parameters for specific velocity ...");
//			ROS_INFO(
//					"Vel[v = %f, w= %f], Input[v = %f, w= %f], heading=%f, clearance=%f, "
//							"velocity = %f, G = %f, coupling = %f, cost = %f",
//					speed.v, speed.w, input.v, input.w, heading, clearance,
//					velocity, G, coupling, cost);
		ROS_INFO(
				"Input[v = %f, w= %f], Vel[v = %f, w= %f], clearance=%f, "
				" coupling = %f, cost = %f, input heading = %f, robot heading =%f",
				input.v, input.w, speed.v, speed.w, clearance, coupling, cost,
				ang, candidate_ang);
#endif

		if (cost > maxCost) {
			maxCost = cost;
#ifdef LINEAR
			chosenSpeed = (speed + input) * 0.5;
#else
			chosenSpeed = speed;
#endif
			final_clearance = clearance;
		}
	}
#ifndef LINEAR
	chosenSpeed = (equals(final_clearance, 0)) ? Speed(0, 0) : chosenSpeed;
#endif
	ROS_INFO("Chosen speed: [v=%f, w=%f]", chosenSpeed.v, chosenSpeed.w);
	return chosenSpeed;
}

void SharedDWA::getData() {
	while (dataflag < 7 && ros::ok()) { // Data bits are arranged n order of testing priority.
		ros::spinOnce();

	}
	dataflag = 0;

}
void SharedDWA::run() {
	ros::Rate loop_rate(1 / dt);

	Speed chosenSpeed;
	int maxduration;
	int aveduration;
	MyTimer timer = MyTimer();

	while (ros::ok()) {
		timer.start();
		getData();
		chosenSpeed = computeNextVelocity(chosenSpeed);
		timer.stop();

		geometry_msgs::TwistStamped motorcmd;
		motorcmd.header.stamp = ros::Time::now();
		motorcmd.twist.linear.x = chosenSpeed.v;
		motorcmd.twist.angular.z = chosenSpeed.w;
		command_pub.publish(motorcmd);

		// Publish input corresponding to dwa resultant velocity
		geometry_msgs::TwistStamped usercmd;
		usercmd.header.stamp = motorcmd.header.stamp;
		Speed userspeed = getRealSpeed(humanInput);
		usercmd.twist.linear.x = userspeed.v;
		usercmd.twist.angular.z = userspeed.w;
		usercommand_pub.publish(usercmd);
		loop_rate.sleep();

		ROS_INFO("DWA Max Duration: %d", timer.getMaxDuration());
		ROS_INFO("DWA Average Duration: %d ",timer.getAveDuration());
	}

}
