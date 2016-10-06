#ifndef POSE_H
#define POSE_H
#include <map/helper.h>
using namespace std;
struct Pose {
	float x;
	float y;
	float th;
	Pose(float xt, float yt, float tht) :
			x(xt), y(yt), th(tht) {
	}
	Pose() {
		x=y=th =0;
	}
	bool operator==(Pose pose) // copy/move constructor is called to construct arg
			{
		if (equals(this->x,
				pose.x) && equals(this->x, pose.x) && equals(this->x, pose.x)) {
			return true;
		} else {
			return false;
		}
	}

};
#endif
