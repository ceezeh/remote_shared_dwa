#include "shared_dwa/lin_dwa_gen.h"



int main(int argc, char **argv) {
	string topic = "lin_dwa_gen";
	ros::init(argc, argv, topic.c_str());
	ros::NodeHandle n;

	LinDWAGen lin_dwagen = LinDWAGen(topic.c_str(), n);
	lin_dwagen.run();
}
