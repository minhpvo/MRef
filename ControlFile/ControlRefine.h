// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#pragma once
#include <vector>
#include <string>

class ControlRefine
{
    public:

	ControlRefine();
	~ControlRefine();

	void writeFile(const std::string& name);
	void readFile(const std::string& name);
	
	int _startlevel;
	int _endlevel;

	// Number iterations per level
	std::vector<int> _numitervec;

	// Weight phototerm per level
	std::vector<float> _photoweightvec;

	// Weight thinplate per level
	std::vector<float> _smoothweightvec;

	bool _usesemanticdata;
	// Weight semantically informed thinplate
	std::vector<float> _semweightvec;

	bool _usesemanticsmooth;
	std::vector<float> _smoothweightvecunknown;
	std::vector<float> _smoothweightvecmobile;
	std::vector<float> _smoothweightvectrees;
	std::vector<float> _smoothweightvecground;
	std::vector<float> _smoothweightvecpavement;
	std::vector<float> _smoothweightvecbuilding;
	std::vector<float> _smoothweightvecwater;

	bool _usestraightedgegrad;
	std::vector<float> _straightedgeweightvec;

	bool _usemrflabelsmoothing;
	int _skipiterations;
	std::vector<int> _nummrfitervec;
	std::vector<float> _mrfsmoothvec;

	bool _preparedata;

	double _tnear;
	double _tfar;

	int _verboselevel;

    private:

	void init();

};
