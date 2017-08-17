#include "shared_dwa/psc_dwa_gen.h"



int main(int argc, char **argv) {
	string topic = "psc_dwa_gen";
	ros::init(argc, argv, topic.c_str());
	ros::NodeHandle n;

	PSCDWAGen psc_dwagen = PSCDWAGen(topic.c_str(), n);
	psc_dwagen.run();
}
