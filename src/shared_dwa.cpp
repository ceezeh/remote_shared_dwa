#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <numeric>
#include "shared_dwa/map.h"
#include "shared_dwa/shared_dwa.h"

using namespace std;

bool debug = true;

SharedDWA::SharedDWA(const char * topic, ros::NodeHandle &n_t) {

	// Trajectories.
	for (float i = -M_PI; i < M_PI; i += 0.3926990817) { // Split into 16 angles four for each quadrant.
		trajectories.push_back(i);
	}
	dt = 0.1; // seconds.

	//	 TODO: Verify these parameters.
	horizon = 5 / dt; // 10 seconds
	refresh_period = 1 / dt;
	// WC kinematics
	acc_lim_v = 0.06 * 15; // 0.06 original but tooooo small. Tooooo.
	acc_lim_w = 1.8;
	decc_lim_v = -0.96;
	decc_lim_w = -4.8;
	max_trans_vel = MAX_LIN_VEL;
	min_trans_vel = -MIN_LIN_VEL;
	max_rot_vel = MAX_ANG_VEL;
	min_rot_vel = -MIN_ANG_VEL;
	// WC dimensions.
	wc_length = 1.400; //m
	wc_width = .700; //m

	vstep = (max_trans_vel - min_trans_vel) / 16;
	wstep = (max_rot_vel - min_rot_vel) / 16;

	/*
	 * These parameters are used for accessing the right back side of the wheelchair
	 * as the start position to fill or check occupancy.
	 */
	length_offset = .550;
	width_offset = wc_width / 2;

	humanInput = Speed(0, 0);
	odom = Speed(0, 0);
	gridsize = .100;
	mapsize = 4.000; //mm
	noOfgrids = mapsize / gridsize;
	dwa_map = new DWAMap(gridsize);
	n = n_t;
	command_pub = n.advertise<geometry_msgs::TwistStamped>("motor_command",
			100);
	odom_sub = n.subscribe("odom", 1, &SharedDWA::odomCallback, this);
	interface_sub = n.subscribe("user_command", 3,
			&SharedDWA::usercommandCallback, this);
	occupancy_sub = n.subscribe("local_map", 1, &SharedDWA::occupancyCallback,
			this);

	G_pub = n.advertise<visualization_msgs::MarkerArray>("G_viz", 10);
	heading_pub = n.advertise<visualization_msgs::MarkerArray>("heading_viz",
			10);
	clearance_pub = n.advertise<visualization_msgs::MarkerArray>(
			"clearance_viz", 10);
	velocity_pub = n.advertise<visualization_msgs::MarkerArray>("velocity_viz",
			10);
	coupling_pub = n.advertise<visualization_msgs::MarkerArray>("coupling_viz",
			10);
}

SharedDWA::~SharedDWA() {
	delete this->dwa_map;
}

void SharedDWA::odomCallback(const nav_msgs::Odometry& cmd) {
	if (!equals(cmd.twist.twist.linear.x, INVALIDCMD)) {
		// update v
		this->odom.v = cmd.twist.twist.linear.x;
		ROS_INFO("I heard something, v= %f", this->odom.v);
	}

	if (!equals(cmd.twist.twist.angular.z, INVALIDCMD)) {
		this->odom.w = cmd.twist.twist.angular.z;// scaling factor that maps user's command to real world units.
		ROS_INFO("I heard something, w= %f", this->odom.w);
	}

}

void SharedDWA::occupancyCallback(const nav_msgs::OccupancyGrid& og) {
	this->dwa_map->updateMap(og);

}

void SharedDWA::usercommandCallback(
		const geometry_msgs::TwistStamped::ConstPtr& cmd) {

	if (!equals(cmd->twist.linear.x, INVALIDCMD)) {
		// update v
//		this->humanInput.v = -.3;
		this->humanInput.v = cmd->twist.linear.x;
		ROS_INFO("I heard something, v= %f", this->humanInput.v);
	}

	if (!equals(cmd->twist.angular.z, INVALIDCMD)) {
//		this->humanInput.w = 0;
		this->humanInput.w = cmd->twist.angular.z;
		ROS_INFO("I heard something, w= %f", this->humanInput.w);
	}
	this->updateInputCommand(this->humanInput.v, this->humanInput.w, Joystick);
}

void SharedDWA::updateInputCommand(float v, float w, InterfaceType In) { // input command here is assumed to be normalised to [-1,1]
	inputDist.clear();

	// Create probability distribution for different interfaces.
	if (In == Joystick) { // uses a dirac function as distribution.
		inputDist.emplace_back(v, w);
	} else { // Button uses a uniform distribution
		// Assume uniform distribution of +-22 degrees around specified command.

		for (int i = -23; i < 23; i += 5) {
			float v_t = cos(i * M_PI / 180) * v - sin(i * M_PI / 180) * w;
			float w_t = sin(i * M_PI / 180) * v + cos(i * M_PI / 180) * w;
			inputDist.emplace_back(v_t, w_t);
		}
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

//	float xgoal = 0;
//	float ygoal = 0;
////	float goalth = 0; // specifies the predicted end pose from user's input.
//	for (int step = 0; step < horizon; step++) {
//
//		xgoal += usercommand.v * dt * cos(goalth);
//		ygoal += usercommand.v * dt * sin(goalth);
////		goalth += dt * usercommand.w;
////		goalth = this->wraparound(goalth);
//	}
//
//	// Compute position at next time step if candidate command is taken towards goal.
//	// Assume theta is constant for each time step
//	float x, y, th;
//	x = y = th = 0;
//	x += candidateSpeed.v * dt * cos(th);
//	y += candidateSpeed.v * dt * sin(th);
//	th += dt * candidateSpeed.w;
//	th = this->wraparound(th);
//
//	// Compute the position after maximum deceleration at next time step.
//	float finalv = candidateSpeed.v; // start from the candidate velocity.
//	float finalw = candidateSpeed.w;
//
//	//	float deltaT = 0.05;
//	float tol_v = fabs(dt * decc_lim_v);
//	float tol_w = fabs(dt * decc_lim_w);
//	while (fabs(finalv) >= tol_v || fabs(finalw) >= tol_w) {
//		if (fabs(finalv) >= tol_v) {
//
//			float dist = finalv * dt + 0.5 * pow(dt, 2) * decc_lim_v;
//			finalv += dt * copysign(decc_lim_v, -finalv);
//			x += dist * cos(th);
//			y += dist * sin(th);
//		}
//		if (fabs(finalw) >= tol_w) {
//			th += finalw * dt + 0.5 * pow(dt, 2) * decc_lim_w;
//			finalw += dt * copysign(decc_lim_w, -finalw);
//			th = this->wraparound(th);
//		}
//	}
	// Compute heading here.
	float heading = M_PI - angDiff(candididateth, goalth);
	// Normalise heading to [0,1]
	heading = this->wraparound(heading);
	heading = fabs(heading) / M_PI;
	return heading;
}

/*
 * Clearance measures the distance to the closest obstacle on the trajectory.
 * TODO: Populate values on a lookup table to prevent re-evaluation.
 */

float SharedDWA::sqrt_approx(float z) {
	int val_int = *(int*) &z; /* Same bits, but as an int */
	/*
	 * To justify the following code, prove that
	 *
	 * ((((val_int / 2^m) - b) / 2) + b) * 2^m = ((val_int - 2^m) / 2) + ((b + 1) / 2) * 2^m)
	 *
	 * where
	 *
	 * b = exponent bias
	 * m = number of mantissa bits
	 *
	 * .
	 */

	val_int -= 1 << 23; /* Subtract 2^m. */
	val_int >>= 1; /* Divide by 2. */
	val_int += 1 << 29; /* Add ((b + 1) / 2) * 2^m. */

	return *(float*) &val_int; /* Interpret again as float */
}

float SharedDWA::computeClearance(Speed candidateSpeed) {
	// clearance is normalised to [0,1] by d
	if (candidateSpeed == Speed(0, 0)) {
		return 1;
	}
	float x = computeDistToNearestObstacle(candidateSpeed);
	x = (x < .3) ? 0 : x / 2.550;
	x = (x > 1) ? 1 : x;
//	float clearance = -1+ 2	/ (1+ exp(-2* sqrt_approx(x)));
	return x;

}

float SharedDWA::computeDistToNearestObstacle(Speed candidateSpeed) {
	// Compute rectangular dimension depicting wheelchair in  occupancy map.
	// Compute and normalize clearance.
	float x, y, th;
	x = y = th = 0;
	float clearance = 2.55; // 2.55m is maximum detectable distance by sonar
	int count = 10; // we want 2 seconds = 2/dt counts
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
			vector<Pose> obstacles;
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
	vector<int> visitedIndices;
	visitedIndices.clear();
	for (float i = 0; i < wc_length; i += .96 * gridsize) {
		for (float j = 0; j < wc_width; j += .96 * gridsize) {
			float x = (i - length_offset);
			float y = (j - width_offset);
			float xt = cos(pose.th) * x - sin(pose.th) * y;
			float yt = sin(pose.th) * x + cos(pose.th) * y;

			xt += pose.x;
			yt += pose.y;

			int xindx = (int) ((mapsize / 2 + xt) / gridsize);
			int yindx = (int) ((mapsize / 2 + yt) / gridsize);
			int index = xindx + noOfgrids * yindx;
			bool exists = std::find(std::begin(visitedIndices),
					std::end(visitedIndices), index)
					!= std::end(visitedIndices);
			if (exists) {
				continue;
			} else {
				visitedIndices.push_back(index);
			}
			try {

				if (this->dwa_map->at(index) > 0) {

					obstacles.emplace_back(xt, yt, 0);
				}
			} catch (const std::exception& e) {
				cout << "Error accessing occupancy at x: !" << xt << " ,y: "
						<< yt << endl;
			}
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
 *
 */
vector<Speed> SharedDWA::getAdmissibleVelocities(vector<Speed> admissibles) {
	admissibles.clear();
	// We search for velocities for by keeping a list of radii of curvature.
	for (int i = 0; i < trajectories.size(); i++) {
		// Construct a speed object from angle and find the clearance.
		Speed trajectory;
		if ((trajectories[i] < M_PI / 2) && ((trajectories[i] >= -M_PI / 2))) { // +ve v space
			trajectory.v = 0.5 * max_trans_vel;
		} else {
			trajectory.v = 0.5 * min_trans_vel;
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
		vector<Speed> resultantVelocities) {

	DynamicWindow dw;
	dw = computeDynamicWindow(dw);
	vector<Speed> admissibles;
	admissibles.clear();
	admissibles = getAdmissibleVelocities(admissibles);

	bool zeroVisited = false; // ensures that we add v=w= 0 only once.
	for (int i = 0; i < trajectories.size(); i++) {
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
		float step = (upperbound_v - lowerbound_v) / 6;
		for (float vel = lowerbound_v; vel <= upperbound_v; vel += step) {

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
			// for very large tan, the data becomes skewed so just use the dw as boundary

			if (abs(tan(trajectories[i])) > 9e3) {
				float stepw = (upperbound_w - lowerbound_w) / 6;
				for (float w = lowerbound_w; w < upperbound_w; w += stepw) {
					if ((w >= lowerbound_w) && ((w <= upperbound_w))) {
						resultantVelocities.emplace_back(vel, w);
					}
				}
			} else if ((w >= lowerbound_w) && ((w <= upperbound_w))) {
				if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
					continue;
				} else if (equals(vel, 0) && (equals(w, 0))) {
					zeroVisited = true;
				}
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
/*
 * This is the part that does the probabilistic conditioning based on the user's input.
 */

/*
 * Let G (v,w) = Fn( α · heading + β·clearance + Ɣ·velocity), where Fn is arbitrary.
 * C = G(v,w) ⨉ exp(-1/(2*s)*(f-h)(f-h)') ⨉  W(H), for each of our possible goal location.
 */
Speed SharedDWA::computeNextVelocity(Speed chosenSpeed) {
	vector<Speed> resultantVelocities;
	resultantVelocities.clear();
	resultantVelocities = getResultantVelocities(resultantVelocities);

	float maxCost = 0;
	float a = 01; // agreement factor between user command and resultant velocity. More means less agreement.
	// Put weightings here.
	float alpha = 0.1; // For heading.
	float beta = 0.4; // For clearance.
	float gamma = 0.2; // For velocity.
	float final_clearance = 0;
#ifdef DEBUG
	visualiseCostFn();
#endif
	for (int j = 0; j < inputDist.size(); j++) {
		if (inputDist[j] == Speed(0, 0))
			return Speed(0, 0); // Stop!!
		for (int i = 0; i < resultantVelocities.size(); i++) {
			if (j != 0 && i == 0)
				continue;
			Speed speed = resultantVelocities[i];
			const Speed input = (const Speed) (getRealSpeed(inputDist[j]));
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

#ifdef DEBUG
			ROS_INFO("Printing out DWA parameters for specific velocity ...");
//			ROS_INFO(
//					"Vel[v = %f, w= %f], Input[v = %f, w= %f], heading=%f, clearance=%f, "
//							"velocity = %f, G = %f, coupling = %f, cost = %f",
//					speed.v, speed.w, input.v, input.w, heading, clearance,
//					velocity, G, coupling, cost);
			ROS_INFO("Vel[v = %f, w= %f], Input[v = %f, w= %f], clearance=%f, "
					" coupling = %f, cost = %f", speed.v, speed.w, input.v,
					input.w, clearance, coupling, cost);

			addCostFn(speed, cost, clearance, coupling, 000, 000);
#endif
			if (cost > maxCost) {
#ifdef DEBUG
				ROS_INFO("[NEW] MAX COST");
#endif
				maxCost = cost;
				chosenSpeed = speed;
				final_clearance = clearance;
			}
		}
	}

#ifdef DEBUG
	G_pub.publish(G_ma);
	heading_pub.publish(heading_ma);
	clearance_pub.publish(clearance_ma);
	velocity_pub.publish(velocity_ma);
	coupling_pub.publish(coupling_ma);
#endif
	chosenSpeed = (equals(final_clearance, 0)) ? Speed(0, 0) : chosenSpeed;
	ROS_INFO("Chosen speed: [v=%f, w=%f]", chosenSpeed.v, chosenSpeed.w);
	return chosenSpeed;
}
#ifdef DEBUG
/*
 * Sends out visualisation messages of the total cost function, heading, clearance, velocity and coupling.
 */
void SharedDWA::visualiseCostFn() {

	G_ma.markers.clear();
	heading_ma.markers.clear();
	clearance_ma.markers.clear();
	velocity_ma.markers.clear();
	coupling_ma.markers.clear();

	for (float v = min_trans_vel; v <= max_trans_vel; v += vstep) {
		for (float w = min_rot_vel; w <= max_rot_vel; w += wstep) {
			Speed speed(v, w);
			addCostFn(speed, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001);
		}
	}
	G_pub.publish(G_ma);
	heading_pub.publish(heading_ma);
	clearance_pub.publish(clearance_ma);
	velocity_pub.publish(velocity_ma);
	coupling_pub.publish(coupling_ma);

	G_ma.markers.clear();
	heading_ma.markers.clear();
	clearance_ma.markers.clear();
	velocity_ma.markers.clear();
	coupling_ma.markers.clear();
}

void SharedDWA::addCostFn(Speed speed, float G, float heading, float clearance,
		float velocity, float coupling) {

	float v = speed.v;
	float w = speed.w;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one

	marker.ns = "dwa_viz";
	int vindx = (v - min_trans_vel) / vstep;
	int windx = (w - min_rot_vel) / wstep;

	marker.id = vindx + 16 * windx;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = v;
	marker.pose.position.y = w;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = vstep;
	marker.scale.y = wstep;
	marker.scale.z = 0.2;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration();
	marker.scale.z = G;
	marker.pose.position.z = G / 2;
	marker.ns = "/G";
	G_ma.markers.push_back(marker);

	marker.pose.position.x = v + 2 * (max_trans_vel - min_trans_vel);
	marker.pose.position.y = w;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.scale.z = heading;
	marker.pose.position.z = heading / 2;
	marker.ns = "/heading";
	heading_ma.markers.push_back(marker);

	marker.pose.position.x = v + 4 * (max_trans_vel - min_trans_vel);
	marker.pose.position.y = w;
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.scale.z = clearance;
	marker.pose.position.z = clearance / 2;
	marker.ns = "/clearance";
	clearance_ma.markers.push_back(marker);

	marker.pose.position.x = v + 6 * (max_trans_vel - min_trans_vel);
	marker.pose.position.y = w;
	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.scale.z = velocity;
	marker.pose.position.z = velocity / 2;
	marker.ns = "/velocity";
	velocity_ma.markers.push_back(marker);

	marker.pose.position.x = v + 8 * (max_trans_vel - min_trans_vel);
	marker.pose.position.y = w;
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 1.0f;
	marker.scale.z = coupling;
	marker.pose.position.z = coupling / 2;
	marker.ns = "/coupling";
	coupling_ma.markers.push_back(marker);
}
#endif

void SharedDWA::run() {
	ros::Rate loop_rate(1 / dt);
	int count = 10;
	while (count-- > 0) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	Speed chosenSpeed;
	while (ros::ok()) {
		chosenSpeed = computeNextVelocity(chosenSpeed);
		geometry_msgs::TwistStamped motorcmd;
		motorcmd.header.stamp = ros::Time::now();
		motorcmd.twist.linear.x = chosenSpeed.v;
		motorcmd.twist.angular.z = chosenSpeed.w;
		command_pub.publish(motorcmd);
		ros::spinOnce();
		loop_rate.sleep();
	}

}
