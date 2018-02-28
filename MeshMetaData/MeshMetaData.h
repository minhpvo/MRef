// Copyright ETHZ 2017
// author: mathias.rothermel@geod.baug.ethz.ch
#pragma once

#include<string>

class MeshMetaData
{
    public:

	MeshMetaData();
	MeshMetaData( const std::string& file );

	~MeshMetaData();

	void readMetaData( const std::string& file );
	void writeMetaData( const std::string& file);

	double _offsetx;
	double _offsety;
	double _offsetz;

    	double _minx;
    	double _miny;
    	double _minz;
    	double _maxx;
    	double _maxy;
    	double _maxz;

	// The actual gsd if derived from DSM
	double _gsd;

    private:

	void init();
	int _numelements;

};
