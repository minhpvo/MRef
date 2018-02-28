// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#pragma once
#include <string>
#include <vector>

class ControlReader
{
    public: 

	ControlReader();
	~ControlReader();
   
	template <typename T>
	void setVar(std::string name, T* var);

	template <typename T>
	void setVec(std::string name, std::vector<T>* vec);

	void readFile(const std::string& name);

    private:

	// Variables
	std::vector<std::string> varboolnamevec;
	std::vector<bool*> varboolvec;

	std::vector<std::string> varintnamevec;
	std::vector<int*> varintvec;

	std::vector<std::string> varfloatnamevec;
	std::vector<float*> varfloatvec;

	std::vector<std::string> vardoublenamevec;
	std::vector<double*> vardoublevec;

	// Vectors
	std::vector<std::string> vecboolnamevec;
	std::vector<std::vector<bool>*> vecboolvec;

	std::vector<std::string> vecintnamevec;
	std::vector<std::vector<int>*> vecintvec;

	std::vector<std::string> vecfloatnamevec;
	std::vector<std::vector<float>*> vecfloatvec;

	std::vector<std::string> vecdoublenamevec;
	std::vector<std::vector<double>*> vecdoublevec;

};
