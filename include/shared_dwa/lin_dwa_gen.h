/*
 * SHARED DWA runs in its own thread.
 */

#ifndef LIN_DWA_GEN_H
#define LIN_DWA_GEN_H

#define  USE_MATH_DEFINES
#include <shared_dwa/psc_dwa_gen.h>

#define INVALIDCMD -2
using namespace std;

class LinDWAGen: public PSCDWAGen {
public:
	LinDWAGen(const char * topic, ros::NodeHandle &n_t) :
			PSCDWAGen(topic, n_t) {
	}
protected:
	// -------------DWA----------
	Speed computeNextVelocity(Speed chosenSpeed);

};

#endif
