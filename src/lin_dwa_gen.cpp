#include "shared_dwa/lin_dwa_gen.h"


#define USER_CMD_BIT 5

using namespace std;
using namespace std::chrono;
MyTimer timer = MyTimer();


/*
 * This is the part that does the probabilistic conditioning based on the user's input.
 */

/*
 * Let G (v,w) = Fn( ������ ������ heading + ������������clearance + ������������velocity), where Fn is arbitrary.
 * C = G(v,w) ��������� exp(-1/(2*s)*(f-h)(f-h)') ���������  W(H), for each of our possible goal location.
 */
Speed LinDWAGen::computeNextVelocity(Speed chosenSpeed) {
	Speed humanInput = Speed(this->usercmd.twist.linear.x,
			usercmd.twist.angular.z);

//	if (humanInput == Speed(0, 0))
//		return Speed(0, 0); // Stop!!

	concurrent_vector<Speed> resultantVelocities;
	resultantVelocities.clear();

	/*
	 * Create optimized search space.
	 */
	float upperbound = -M_PI - 1;
	float lowerbound = M_PI + 1;
	Speed input = getRealSpeed(humanInput);

	deOscillator.getAdmissibleDirection(upperbound, lowerbound);
	ROS_INFO("upperbound: %f, lowerbound: %f", upperbound, lowerbound);
	resultantVelocities = getResultantVelocities(resultantVelocities,
			upperbound, lowerbound);
	float maxCost = 0;
	// Put weightings here.
	float alpha = .02;		// For heading.
	float beta = .03;		// For clearance.
	float gamma = 1;
	float final_clearance = 0;
	cout << "Number of resultant velocities" << resultantVelocities.size()
			<< endl;
	for (int i = 0; i < resultantVelocities.size(); i++) {

		Speed realspeed = resultantVelocities[i];
		const Pose goalpose = this->getGoalPose();
		float heading = computeHeading(realspeed, goalpose);
		float clearance = computeClearance(realspeed);
		if (equals(clearance, 0))
			continue;

		float velocity = computeVelocity(realspeed);
		float G = alpha * heading + beta * clearance + gamma * velocity;
		float cost = G;

		ROS_INFO("Printing out SharedDWA parameters for specific velocity ...");
		ROS_INFO("RealVel[v = %f, w= %f], heading=%f, clearance=%f, "
				"velocity = %f, cost = %f, Goal Pose (x: %f, y: %f, th: %f)",
				realspeed.v, realspeed.w, heading, clearance, velocity, cost,
				goalpose.x, goalpose.y, goalpose.th);
		if (cost > maxCost) {
			maxCost = cost;
			//chosenSpeed= Speed(input.v, realspeed.w);
//			chosenSpeed = (input+realspeed)*0.5;
			final_clearance = clearance;
//			float a = exp(-0.2 * clearance);
			chosenSpeed = (realspeed  + input)*.5;
		}

	}

	ROS_INFO("Chosen speed: [v=%f, w=%f]. Maxcost=%f", chosenSpeed.v,
			chosenSpeed.w, maxCost);
	return chosenSpeed;
}
