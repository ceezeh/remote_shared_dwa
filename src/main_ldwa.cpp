#include "shared_dwa/map.h"
#include "shared_dwa/linear_dwa.h"



int main(int argc, char **argv) {
	ros::init(argc, argv, "linear_dwa");
	ros::NodeHandle n;

	LinearDWA linear_dwa = LinearDWA(n);
	linear_dwa.run();
}
