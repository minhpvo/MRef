// Copyright ETHZ 2017
//author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#pragma once
#include "../MeshUtil/MyMesh.h"
#include "../MeshMetaData/MeshMetaData.h"
#include "../ControlFile/ControlRefine.h"
#include "../IOList/IOList.h"
#include "../Ori/Orientation.h"

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <vector>

class MeshRefine
{
    public:

	MeshRefine(
		MyMesh* tile,
		MeshMetaData* mmd,
		ControlRefine* ctr,
		Eigen::MatrixXf* adjacency,
		IOList* imglistphoto,
		IOList* imglistsem,
		IOList* orilist
		);

	~MeshRefine();

	// Processing of a single tile
	void process();

    private:

	void updateMesh(MyMesh& mesh, Eigen::MatrixXf &grad);

	double avEdgeLength(MyMesh& mesh);

	void downscale(cv::Mat& img, int level);

	void computeAdaptedOrientation(std::vector<Orientation>& adaptedori);

	int cleanGrad(const float avedgelength, Eigen::MatrixXf &grad, Eigen::VectorXi& counter, MyMesh& mesh);

	void scaleGradByCounter( Eigen::MatrixXf &grad, Eigen::VectorXi& counter);

	MyMesh* _mesh;
	MeshMetaData* _mmd;
	ControlRefine* _ctr;
	Eigen::MatrixXf* _adjacency;
	IOList* _imglistphoto;
	IOList* _imglistsem;
	IOList* _orilist;
	int _verboselevel;

};
