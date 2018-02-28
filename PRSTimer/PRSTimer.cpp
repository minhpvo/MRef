// Copyright ETHZ 2017
#include <time.h>

#include <iostream>
#include <stdlib.h>

#include "PRSTimer.h"


PRSTimer::PRSTimer()
{

    	reset();
}

PRSTimer::~PRSTimer()
{

}

double PRSTimer::timevaldiff(struct timeval *starttime, struct timeval *endtime)
{

      	double milisec;
       	milisec=(endtime->tv_sec-starttime->tv_sec)*1000;
	milisec+=(endtime->tv_usec-starttime->tv_usec)/1000;
	return milisec;

}

void PRSTimer::reset()
{
    	_isrunning=false;
	_uptime=0.0;
}

void PRSTimer::start()
{
	// Get current time
 	if(!_isrunning)
	{
		_isrunning=true;
		gettimeofday(&_starttime, NULL);
	}
	else
	{
	    std::cout<< "\nStart Timer Error: timer was already started "; exit(0);
	}
}

void PRSTimer::stop()
{
	if(_isrunning){
		// Get current time
		gettimeofday(&_endtime, NULL);
		_isrunning=false;
		// Get ellapsed time in millisecs
		_uptime+=timevaldiff(&_starttime, &_endtime);
	}
	else{
		std::cout<< "\nTimer Error: timer was not started "; exit(0);
	}
}


double PRSTimer::getCurrentRuntime ()
{
    	if ( this->_isrunning == true )
    	{
 		gettimeofday(&_endtime, NULL);
		return ( _uptime+timevaldiff(&_starttime, &_endtime) / 1000.0);
	}
	else{ return ( this->getTimeSec() ); }
};

double PRSTimer::getTimeSec()
{
    	double start = (double)(_starttime.tv_sec) +(double)(_starttime.tv_usec) / 1000000.0 ;
	double end = (double)(_endtime.tv_sec) + (double)(_endtime.tv_usec) / 1000000.0 ;
	return end-start;
}

double PRSTimer::getTimeMin()
{
    	double start = ((double)(_starttime.tv_sec) +(double)(_starttime.tv_usec) / 1000000.0)/60.0 ;
	double end = ((double)(_endtime.tv_sec) + (double)(_endtime.tv_usec) / 1000000.0)/60.0 ;
	return end-start;
}

double PRSTimer::getTimeHours()
{

    	double start = ((double)(_starttime.tv_sec) +(double)(_starttime.tv_usec) / 1000000.0)/3600.0 ;
	double end = ((double)(_endtime.tv_sec) + (double)(_endtime.tv_usec) / 1000000.0)/3600.0 ;
	return end-start;
}
