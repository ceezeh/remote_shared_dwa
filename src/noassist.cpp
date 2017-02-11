#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <dwa/helper.h>
#include <dwa/speed.h>
ros::Publisher pub;
bool isSim = false;
void inputCallback(const geometry_msgs::TwistStamped& cmd) {
	geometry_msgs::Twist in(cmd.twist);
	if (isSim) {
		in.angular.z *= -1;
	}
	Speed speed = getRealSpeed(Speed(in.linear.x, in.angular.z));
	in.linear.x = speed.v;
	in.angular.z = speed.w;
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

	pub = n.advertise < geometry_msgs::Twist > (cmd_topic.c_str(), 10);

	ros::spin();
}
