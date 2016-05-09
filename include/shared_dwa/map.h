
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <pthread.h>
#include <sstream>
#include <stdlib.h>
#include <cstdint>
#include <boost/circular_buffer.hpp>
#include <memory>

using namespace std;
class DWAMap {
public:
	DWAMap(int gridsize_t):gridsize(gridsize_t) {}

	void updateMap(nav_msgs::OccupancyGrid newMap) {
		grid.header.stamp = newMap.header.stamp;
		grid.info = newMap.info;
		grid.data.clear();
		for (int i = 0; i < newMap.data.size() ; i++) {
		grid.data.push_back(newMap.data[i] );
		}
	}
	int at (int indx) {
		return grid.data[indx];
	}
private:
	int gridsize;
	nav_msgs::OccupancyGrid grid;

};
typedef std::unique_ptr<DWAMap> DWAMapPtr;
