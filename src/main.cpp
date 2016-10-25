#include "shared_dwa/map.h"
#include "shared_dwa/shared_dwa.h"



int main(int argc, char **argv) {
	string topic = "shared_dwa";
	ros::init(argc, argv, topic.c_str());
	ros::NodeHandle n;

	SharedDWA shared_dwa = SharedDWA(topic.c_str(), n);
	shared_dwa.run();
}
