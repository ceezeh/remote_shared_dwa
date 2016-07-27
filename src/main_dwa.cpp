#include "shared_dwa/map.h"
#include "shared_dwa/dwa.h"



int main(int argc, char **argv) {
	ros::init(argc, argv, "dwa");
	ros::NodeHandle n;

	DWA dwa = DWA(n);
	dwa.run();
}
