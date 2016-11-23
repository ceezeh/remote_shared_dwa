#include "shared_dwa/psc_dwa.h"



int main(int argc, char **argv) {
	string topic = "psc_dwa";
	ros::init(argc, argv, topic.c_str());
	ros::NodeHandle n;

	PSCDWA psc_dwa = PSCDWA(topic.c_str(), n);
	psc_dwa.run();
}
