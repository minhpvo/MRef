// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "ControlReader.h"
#include <fstream>
#include <sstream>
#include <iostream>

ControlReader::ControlReader()
{
}

ControlReader::~ControlReader()
{
}

void ControlReader::readFile(const std::string& name)
{

	std::fstream fid;
	fid.open(name.c_str(),std::ifstream::in);
	if(!fid)
	{
	    std::cout<<"\n ControlReader: Problems openeing file: "<<name;
	    exit(1);
	}

	std::string buffer;
	std::stringstream ss;
	int i,helper,j;

	// Start reading file
	while(fid>>buffer)
	{
		// Parse Vars
		for(i=0;i<varboolnamevec.size();i++)
		{
			if(buffer==varboolnamevec[i])
			{
			    fid>>buffer;
			    fid>>buffer;
			    ss<<buffer;
			    ss>>helper;
			    if(helper>0) { *(varboolvec.at(i))=true; }
			    else {*(varboolvec[i])=false;}
			    ss.clear();
			}
		}
		for(i=0;i<varintnamevec.size();i++)
		{
			if(buffer==varintnamevec[i])
			{
			    fid>>buffer;
			    fid>>buffer;
			    ss<<buffer;
			    ss>>*(varintvec[i]);
			    ss.clear();
			}
		}
	    	for(i=0;i<varfloatnamevec.size();i++)
		{
			if(buffer==varfloatnamevec[i])
			{
			    fid>>buffer;
			    fid>>buffer;
			    ss<<buffer;
			    ss>>*(varfloatvec[i]);
			    ss.clear();
			}
		}
	    	for(i=0;i<vardoublenamevec.size();i++)
		{
			if(buffer==vardoublenamevec[i])
			{
			    fid>>buffer;
			    fid>>buffer;
			    ss<<buffer;
			    ss>>*(vardoublevec[i]);
			    ss.clear();
			}
		}

		// Parse Vecs
	    	for(i=0;i<vecboolnamevec.size();i++)
		{
			if(buffer==vecboolnamevec[i])
			{
			    fid>>buffer; // =
			    fid>>buffer; // [
			    for(j=0;j<vecboolvec[i]->size();j++)
			    {
				fid>>buffer;
				ss<<buffer;
				ss>>helper;
				   if(helper>0) { vecboolvec[i]->at(j)=true; }
			    	   else {vecboolvec[i]->at(j)=false;}
				   ss.clear();
			    }
			    fid>>buffer; // ]
			}
		}
	    	for(i=0;i<vecintnamevec.size();i++)
		{
			if(buffer==vecintnamevec[i])
			{
				fid>>buffer; // =
			 	fid>>buffer; // [
				for(j=0;j<vecintvec[i]->size();j++)
				{
					fid>>buffer;
					ss<<buffer;
					ss>>vecintvec[i]->at(j);
					ss.clear();
				}
			    fid>>buffer; // ]
			}
		}
	    	for(i=0;i<vecfloatnamevec.size();i++)
		{
			if(buffer==vecfloatnamevec[i])
			{
				fid>>buffer; // =
			 	fid>>buffer; // [
				for(j=0;j<vecfloatvec[i]->size();j++)
				{
					fid>>buffer;
					ss<<buffer;
					ss>>vecfloatvec[i]->at(j);
					ss.clear();
				}
			    fid>>buffer; // ]
			}
		}
	    	for(i=0;i<vecdoublenamevec.size();i++)
		{
			if(buffer==vecdoublenamevec[i])
			{
				fid>>buffer; // =
			 	fid>>buffer; // [
				for(j=0;j<vecdoublevec[i]->size();j++)
				{
					fid>>buffer;
					ss<<buffer;
					ss>>vecdoublevec[i]->at(j);
					ss.clear();
				}
			    fid>>buffer; // ]
			}
		}
	}
	fid.close();
}

// Variables
template <>
void ControlReader::setVar(std::string name, bool* var)
{
	varboolnamevec.push_back(name);
	varboolvec.push_back(var);
}

template <>
void ControlReader::setVar(std::string name, int* var)
{
	varintnamevec.push_back(name);
	varintvec.push_back(var);
}

template <>
void ControlReader::setVar(std::string name, float* var)
{
	varfloatnamevec.push_back(name);
	varfloatvec.push_back(var);
}

template <>
void ControlReader::setVar(std::string name, double* var)
{
	vardoublenamevec.push_back(name);
	vardoublevec.push_back(var);
}

// Vectors
template <>
void ControlReader::setVec(std::string name, std::vector<bool>* vec)
{
    vecboolnamevec.push_back(name);
    vecboolvec.push_back(vec);
}

template <>
void ControlReader::setVec(std::string name, std::vector<int>* vec)
{
    vecintnamevec.push_back(name);
    vecintvec.push_back(vec);
}

template <>
void ControlReader::setVec(std::string name, std::vector<float>* vec)
{
    vecfloatnamevec.push_back(name);
    vecfloatvec.push_back(vec);
}

template <>
void ControlReader::setVec(std::string name, std::vector<double>* vec)
{
    vecdoublenamevec.push_back(name);
    vecdoublevec.push_back(vec);
}

