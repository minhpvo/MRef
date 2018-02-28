// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#pragma once
#include <string>
#include <vector>
#include <fstream>

class ControlWriter
{
    public:
	ControlWriter(const std::string& filename);
	~ControlWriter();

 	template <typename T> void writeVar(std::string varname, const T& var);

	template <typename T> void writeVec(std::string vecname, const std::vector<T>& vec);

    private:
	
	std::fstream _file;


};
