/*
 * SHARED DWA runs in its own thread.
 */

#ifndef LINEAR_DWA_H
#define LINEAR_DWA_H

#include "shared_dwa/psc_dwa.h"
class LinearDWA: public PSCDWA {
public:
	LinearDWA(const char * topic, ros::NodeHandle &n_t):PSCDWA(topic, n_t){}
private:
	Speed computeNextVelocity(Speed chosenSpeed);
};

#endif
