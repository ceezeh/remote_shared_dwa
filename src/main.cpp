#include "shared_dwa/map.h"
#include "shared_dwa/shared_dwa.h"



int main(int argc, char **argv) {
	ros::init(argc, argv, "shared_dwa");
	ros::NodeHandle n;

	SharedDWA shared_dwa = SharedDWA("Shared_DWA", n);
	shared_dwa.run();
}
