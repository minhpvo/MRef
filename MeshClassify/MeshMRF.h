// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermal@geod.baug.ethz.ch
#pragma once
#include <Eigen/Dense>
#include "../MeshUtil/MyMesh.h"
#include "../MeshUtil/MeshConv.h"
#include "../MeshMetaData/MeshMetaData.h"
#include "../RayTracer/RayTracer.h"
//#include "../GraphMRF/graph.h"
#include "../GraphMRF/lbp_graph.h"
#include <vector>
#include <string>

class MeshMRF
{
    public:

	// Types of data and smoothness cost
	enum SMOOTHNESSCOST{ POTTS };
	enum DATACOST{ DISTANCE };

	MeshMRF(MyMesh* mesh, MeshMetaData* mmd, const int numlabels);
    	~MeshMRF();
	
	// Main processing function
	void process(	const std::vector< std::string> &imglist,
			const std::vector<std::string> &orilist,
			const int maxiterations);
   
    private:

	float _avfacesize;

	// Smoothness Costs 
	static float Potts(int, int, int l1, int l2);
	static float PottsNormals(int s1, int s2 , int l1, int l2);

	// Helpers for smoothness Costs...  
	float pairwiseCost(MyMesh::FaceIter &f_it, MyMesh::FaceFaceIter &ff_it);
	
	// Helpers for data dosts
	static float DataOneDiff(float likelihood);
	static float DataLog(float likelihood);
	static float dataPrior(int label, MyMesh::Normal normal);

	void getBBCoords(MyMesh* mesh);

	MeshMetaData*  _mmd;
	const float TFAR =1000.0;
	const float TNEAR=0;

	// Fill precalculated costs to graph structure
	void fillGraph();

	// Cost computation
	void calcDataCosts( 	const std::vector<std::string> &imglist,
				const std::vector<std::string> &orilist);
	void calcDataCostPrior();
	
	// Select final costs 
	void assignMinLabels();
	
	std::vector <std::vector<float> > _facelabelcosts;
	std::vector <std::vector<float> > _facepriorcosts;
	std::vector <int> _facehitcount;
	std::vector <float> _facesizevec;
	void init(MyMesh* mesh);
 	MyMesh* _mesh;
    	int _numfaces;

   	std::vector< std::string > _imgnames;
   	std::vector <std::string > _orinames;

	bool hitsModel(Orientation &ori);
	// Bounding coordinates of mesh
	Eigen::Vector4d _bb1;
	Eigen::Vector4d _bb2;
	Eigen::Vector4d _bb3;
	Eigen::Vector4d _bb4;
	Eigen::Vector4d _bb5;
	Eigen::Vector4d _bb6;
	Eigen::Vector4d _bb7;
	Eigen::Vector4d _bb8;

	float _bbmaxx,_bbmaxy,_bbmaxz;

    	int _numlabels;
    	RayTracer* _rtracer;
	mrf::LBPGraph* _lbpgraph;
};

