// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "ControlRefine.h"
#include "ControlReader.h"
#include "ControlWriter.h"
#include "../FileSystemUtil/FDUtil.h"
#include <iostream>

ControlRefine::ControlRefine()
{
    init();
}

ControlRefine::~ControlRefine()
{
}

void ControlRefine::init()
{
	_startlevel=0;

	_endlevel=0;

	_usesemanticsmooth=false;

	_tnear=0.1;
	_tfar=2000.0;

	_verboselevel=0;

	// Number iterations per level
	_numitervec.resize(8);
	for(int i=0; i<8; i++) { _numitervec[i]=22; }

	// Weight phototerm per level
	_photoweightvec.resize(8);
	_photoweightvec[0]=0.2;
	for(int i=1; i<8; i++) { _photoweightvec[i]=240.0; }

	// Weight thinplate per level
	_smoothweightvec.resize(8);
	for(int i=0; i<8; i++) { _smoothweightvec[i]=0.6; }

	// Weight semantic data
	_usesemanticdata=false;
	_semweightvec.resize(8);
	for(int i=0; i<8; i++) { _semweightvec[i]=0.2; }
	_usesemanticsmooth=false;
	_smoothwaterweightvec.resize(8);
	for(int i=0; i<8; i++) { _smoothwaterweightvec[i]=0.8; }
	_smoothfacadeweightvec.resize(8);
	for(int i=0; i<8; i++) { _smoothfacadeweightvec[i]=0.2; }
	_smoothgroundweightvec.resize(8);
	for(int i=0; i<8; i++) { _smoothgroundweightvec[i]=0.6; }
	_smoothroofweightvec.resize(8);
	for(int i=0; i<8; i++) { _smoothroofweightvec[i]=0.1; }
	_smoothvegeweightvec.resize(8);
	for(int i=0; i<8; i++) { _smoothvegeweightvec[i]=0.0; }
	
	_usestraightedgegrad=true;
	_straightedgeweightvec.resize(8);
	for(int i=0; i<8; i++) { _straightedgeweightvec[i]=0.3; }
	
	_usemrflabelsmoothing=true;
	_skipiterations=5;
	_nummrfitervec.resize(8);
	for(int i=0; i<8; i++) { _nummrfitervec[i]=40; }
	_mrfsmoothvec.resize(8);
	for(int i=0; i<8; i++) { _mrfsmoothvec[i]=0.2; }

	_preparedata=false;

}

void ControlRefine::writeFile(const std::string& name)
{
	ControlWriter writer(name);

	writer.writeVar("#VerboseLevel", _verboselevel);
	writer.writeVar("#StartPyr", _startlevel);
	writer.writeVar("#EndPyr", _endlevel);
	writer.writeVec("#RefineIterations",_numitervec);
	writer.writeVec("#PhotoWeights",_photoweightvec);
	writer.writeVec("#SmoothWeights",_smoothweightvec);
	writer.writeVar("#UseSemanticData",_usesemanticdata);
	writer.writeVec("#SemDataWeights",_semweightvec);
	writer.writeVar("#UseSemanticSmooth", _usesemanticsmooth);
	writer.writeVec("#SemSmoothWeightsWater",_smoothwaterweightvec);
	writer.writeVec("#SemSmoothWeightsFacade",_smoothfacadeweightvec);
	writer.writeVec("#SemSmoothWeightsGround",_smoothgroundweightvec);
	writer.writeVec("#SemSmoothWeightsRoof",_smoothroofweightvec);
	writer.writeVec("#SemSmoothWeightsVege",_smoothvegeweightvec);
	writer.writeVar("#UseStraightEdgeGrad",_usestraightedgegrad);
	writer.writeVec("#StraightEdgeWeights",_straightedgeweightvec);
	writer.writeVar("#UseMrfLabeling", _usemrflabelsmoothing);
	writer.writeVar("#SkipIterations", _skipiterations);
	writer.writeVec("#MrfIterations",_nummrfitervec);
	writer.writeVec("#MrfSmoothWeights",_mrfsmoothvec);
	writer.writeVar("#PrepareData",_preparedata);
	writer.writeVar("#Tnear",_tnear);
	writer.writeVar("#Tfar",_tfar);


}

void ControlRefine::readFile(const std::string& name)
{
	ControlReader reader;

	// Set the stuff to read
	reader.setVar("#Verboselevel", &_verboselevel);
	reader.setVar("#StartPyr", &_startlevel);
	reader.setVar("#EndPyr", &_endlevel);
	reader.setVec("#RefineIterations",&_numitervec);
	reader.setVec("#PhotoWeights",&_photoweightvec);
	reader.setVec("#SmoothWeights",&_smoothweightvec);
	reader.setVar("#UseSemanticData", &_usesemanticdata);
	reader.setVec("#SemDataWeights", &_semweightvec);
	reader.setVar("#UseSemanticSmooth", &_usesemanticsmooth);
	reader.setVec("#SemSmoothWeightsWater",&_smoothwaterweightvec);
	reader.setVec("#SemSmoothWeightsFacade",&_smoothfacadeweightvec);
	reader.setVec("#SemSmoothWeightsGround",&_smoothgroundweightvec);
	reader.setVec("#SemSmoothWeightsRoof",&_smoothroofweightvec);
	reader.setVec("#SemSmoothWeightsVege",&_smoothvegeweightvec);
	reader.setVar("#UseStraightEdgeGrad",&_usestraightedgegrad);
	reader.setVec("#StraightEdgeWeights",&_straightedgeweightvec);
	reader.setVar("#UseMrfLabeling",&_usemrflabelsmoothing);
	reader.setVar("#SkipIterations",&_skipiterations);
	reader.setVec("#MrfIterations",&_nummrfitervec);
	reader.setVec("#MrfSmoothWeights",&_mrfsmoothvec);
	reader.setVar("#PrepareData",&_preparedata);
	reader.setVar("#Tnear",&_tnear);
	reader.setVar("#Tfar",&_tfar);

	// Read
	reader.readFile(name);

}
