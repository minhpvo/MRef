// Copyright ETHZ 2017
//author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "ControlWriter.h"
#include <iostream>

ControlWriter::ControlWriter(const std::string& filename)
{
	_file.open(filename.c_str(), std::fstream::out);
	if(!_file)
	{
	    std::cout<<"\n"<<"ControlWriter: problems openeing file: "<<filename;
	    exit(1);
	}
	_file<<"\n";
}

ControlWriter::~ControlWriter()
{
	_file.close();
}

template<>
void ControlWriter::writeVar(std::string varname, const bool& var)
{
    _file<<varname<<" = ";
    if(var) { _file<<"1"; }
    else{ _file<<"0"; }
    _file<<"\n\n";
}

template<>
void ControlWriter::writeVar(std::string varname, const int& var)
{
	_file<<varname<<" = "<< var<<"\n\n";
}

template<>
void ControlWriter::writeVar(std::string varname, const float& var)
{
	_file<<varname<<" = "<< var<<"\n\n";
}

template<>
void ControlWriter::writeVar(std::string varname, const double& var)
{
	_file<<varname<<" = "<< var<<"\n\n";
}

template<>
void ControlWriter::writeVec(std::string vecname, const std::vector<float>& vec)
{

	_file<<vecname<<" = [ ";
	for(int i=0;i<vec.size();i++)
	{
	    _file<<vec[i]<<" ";
	}
	_file<<"]\n\n";

}
template<>
void ControlWriter::writeVec(std::string vecname, const std::vector<bool>& vec)
{

	_file<<vecname<<" = [ ";
	for(int i=0;i<vec.size();i++)
	{
		if(vec[i]){ _file<<"1 "; }
		else{ _file<<"0 "; }
	}
	_file<<"]\n\n";
}

template<>
void ControlWriter::writeVec(std::string vecname, const std::vector<int>& vec)
{

	_file<<vecname<<" = [ ";
	for(int i=0;i<vec.size();i++)
	{
	    _file<<vec[i]<<" ";
	}
	_file<<"]\n\n";
}

template<>
void ControlWriter::writeVec(std::string vecname, const std::vector<double>& vec)
{

	_file<<vecname<<" = [ ";
	for(int i=0;i<vec.size();i++)
	{
	    _file<<vec[i]<<" ";
	}
	_file<<"]\n\n";
}
