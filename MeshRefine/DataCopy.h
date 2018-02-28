// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rotheremel@geod.baug.ethz.ch
#pragma once
#include "../IOList/IOList.h"
#include <string>

class DataCopy
{

    public:

	DataCopy();
	~DataCopy();

	// Set Tempdir environment for cluster
	void setTempDir();

	// Actual data copy
	void copyData(IOList& list, const std::vector<bool>& cpvec);

    private:

    	std::string _tempdir;
};
