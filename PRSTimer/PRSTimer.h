// Copyright ETHZ 2017
// author: mathias, mathias.rothermel@geod.baug.ethz.ch
#pragma once

#include <sys/time.h>

class PRSTimer{

    public:
	// Constructor
	PRSTimer();
	// Destructor
	~PRSTimer();
	// Start or restart timer
	void start();
	// Pause or stop timer
	void stop();
	// Reset timer
	void reset();
	// Get current runtime
	double getCurrentRuntime ();
	// Get the time in millisecs
	double getTimeMsec();
	// Get the time in secs
	double getTimeSec();
	// Get the time in minutes
	double getTimeMin();
	// Get the time in hours
	double getTimeHours();

    private:
	// Compute time difference
	double timevaldiff(struct timeval *starttime, struct timeval *endtime);

	bool _isrunning;
	struct timeval _starttime;
	struct timeval _endtime;
	double _uptime;
};
