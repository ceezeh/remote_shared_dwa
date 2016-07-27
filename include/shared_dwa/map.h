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
#include <map/helper.h>
using namespace std;
class DWAMap {
public:
	DWAMap(float resolution_t, float sideWidth) :
			resolution(resolution_t) {
		noOfGrids = int(0.5 + sideWidth / resolution);
		localMap.info.origin.position.x = int(noOfGrids / 2 + 0.5); // ceiling of approximation.
		localMap.info.origin.position.y = int(noOfGrids / 2 + 0.5); // ceiling of approximation.
	}

	void updateMap(nav_msgs::OccupancyGrid newMap) {
		localMap.header.stamp = newMap.header.stamp;
		localMap.info = newMap.info;
		localMap.data.clear();
		for (int i = 0; i < newMap.data.size(); i++) {
			localMap.data.push_back(newMap.data[i]);
		}
	}
	int at(int xindx, int yindx) {
		int indx = getIndex(xindx, yindx);
		if ((indx < 0) || (indx >= noOfGrids * noOfGrids)) {
			ROS_ERROR(
					"Error accessing index. Invalid index. Index out of range.");
			return 0;
		}
		return localMap.data[indx];
	}
	int at(int indx) {
		return localMap.data[indx];
	}

	int getIndex(int xindx, int yindx) {
		int indx = xindx + noOfGrids * yindx;
		return indx;
	}


// Takes in point in the boday frame.
	void mapToReal(int x, int y, RealPoint &point) {
			point.x = (x-localMap.info.origin.position.x)*resolution;
			point.y = (y-localMap.info.origin.position.y)*resolution;
		}
	int realToMap(float real) {
		int indx;
		indx = localMap.info.origin.position.y + (real / resolution);
		if (indx < 0) {
			ROS_ERROR(
					"Error accessing index. Invalid index. Index out of Map range.");
			indx = 0;
		} else if (indx >= noOfGrids) {
			ROS_ERROR(
					"Error accessing index. Invalid index. Index out of Map range.");
			indx = noOfGrids - 1;
		}
		return indx;
	}

private:
	float resolution;// The size in m of each occupancy grid.
	int noOfGrids; // number of grids in one side of the square occupancy.
	nav_msgs::OccupancyGrid localMap;


};
typedef std::unique_ptr<DWAMap> DWAMapPtr;
