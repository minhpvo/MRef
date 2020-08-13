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
	_verboselevel=0;
	_nrows = 0;
	_ncols = 0;
	_nlabels = 7;
	_startlevel=0;
	_endlevel=0;
	_usesemanticsmooth=false;
	_tnear=0.1;
	_tfar=2000.0;
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

	_smoothweightvecunknown.resize(8);
	for(int i=0; i<8; i++) { _smoothweightvecunknown[i]=0.0; }
	_smoothweightvecmobile.resize(8);
	for(int i=0; i<8; i++) { _smoothweightvecmobile[i]=0.0; }
	_smoothweightvectrees.resize(8);
	for(int i=0; i<8; i++) { _smoothweightvectrees[i]=0.1; }
	_smoothweightvecground.resize(8);
	for(int i=0; i<8; i++) { _smoothweightvecground[i]=0.2; }
	_smoothweightvecpavement.resize(8);
	for(int i=0; i<8; i++) { _smoothweightvecpavement[i]=0.6; }
	_smoothweightvecbuilding.resize(8);
	for(int i=0; i<8; i++) { _smoothweightvecbuilding[i]=0.2; }
	_smoothweightvecwater.resize(8);
	for(int i=0; i<8; i++) { _smoothweightvecwater[i]=0.8; }


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
	writer.writeVar("#nPixelsImgHeight",_nrows);
	writer.writeVar("#nPixelsImgWidth",_ncols);
	writer.writeVar("#nClasses",_nlabels);
	writer.writeVar("#StartPyr", _startlevel);
	writer.writeVar("#EndPyr", _endlevel);
	writer.writeVec("#RefineIterations",_numitervec);
	writer.writeVec("#PhotoWeights",_photoweightvec);
	writer.writeVec("#SmoothWeights",_smoothweightvec);
	writer.writeVar("#UseSemanticData",_usesemanticdata);
	writer.writeVec("#SemDataWeights",_semweightvec);
	writer.writeVar("#UseSemanticSmooth", _usesemanticsmooth);
	writer.writeVec("#SemSmoothWeightsUnknown",_smoothweightvecunknown);
	writer.writeVec("#SemSmoothWeightsMobile",_smoothweightvecmobile);
	writer.writeVec("#SemSmoothWeightsTrees",_smoothweightvectrees);
	writer.writeVec("#SemSmoothWeightsGround",_smoothweightvecground);
	writer.writeVec("#SemSmoothWeightsPavement",_smoothweightvecpavement);
	writer.writeVec("#SemSmoothWeightsBuilding",_smoothweightvecbuilding);
	writer.writeVec("#SemSmoothWeightsWater",_smoothweightvecwater);
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
	reader.setVar("#VerboseLevel", &_verboselevel);
	reader.setVar("#nPixelsImgHeight", &_nrows);
	reader.setVar("#nPixelsImgWidth", &_ncols);
	reader.setVar("#nClasses", &_nlabels);
	reader.setVar("#StartPyr", &_startlevel);
	reader.setVar("#EndPyr", &_endlevel);
	reader.setVec("#RefineIterations",&_numitervec);
	reader.setVec("#PhotoWeights",&_photoweightvec);
	reader.setVec("#SmoothWeights",&_smoothweightvec);
	reader.setVar("#UseSemanticData", &_usesemanticdata);
	reader.setVec("#SemDataWeights", &_semweightvec);
	reader.setVar("#UseSemanticSmooth", &_usesemanticsmooth);
	reader.setVec("#SemSmoothWeightsUnknown",&_smoothweightvecunknown);
	reader.setVec("#SemSmoothWeightsMobile",&_smoothweightvecmobile);
	reader.setVec("#SemSmoothWeightsTrees",&_smoothweightvectrees);
	reader.setVec("#SemSmoothWeightsGround",&_smoothweightvecground);
	reader.setVec("#SemSmoothWeightsPavement",&_smoothweightvecpavement);
	reader.setVec("#SemSmoothWeightsBuilding",&_smoothweightvecbuilding);
	reader.setVec("#SemSmoothWeightsWater",&_smoothweightvecwater);
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
