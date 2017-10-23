#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <dwa/helper.h>
#include <dwa/speed.h>
ros::Publisher pub;
bool isSim = false;

float max_lin_vel = 1;
float min_lin_vel = 1;
float max_ang_vel = 1.5;
float min_ang_vel = 1.5;

Speed getRealSpeed(Speed speed_old) {
	Speed speed = Speed(speed_old);
	if (speed.v >= 0) {
		speed.v *= max_lin_vel;
	} else {
		speed.v *= min_lin_vel;
	}
	if (speed.w >= 0) {
		speed.w *= max_ang_vel;
	} else {
		speed.w *= min_ang_vel;
	}
	return speed;
}

void inputCallback(const geometry_msgs::TwistStamped& cmd) {
	geometry_msgs::Twist in(cmd.twist);
	if (isSim) {
		in.angular.z *= -1;
	}
	Speed speed = getRealSpeed(Speed(in.linear.x, in.angular.z));
	in.linear.x = speed.v;
	in.angular.z = speed.w / 2.4;
	pub.publish(in);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "controller_forwarder");
	ros::NodeHandle n;

	string isSimStr = "noassist/issim";
	n.getParam(isSimStr.c_str(), isSim);
	cout << "Is Sim?: :" << isSim << endl;

	ros::Subscriber occupancy_sub = n.subscribe("user_command", 1000,
			inputCallback);

	string cmd_topic = "noassist/cmd_topic";
	n.getParam(cmd_topic.c_str(), cmd_topic);
	cout << "cmd_topic: " << cmd_topic << endl;

	pub = n.advertise<geometry_msgs::Twist>(cmd_topic.c_str(), 10);

	ros::spin();
}
