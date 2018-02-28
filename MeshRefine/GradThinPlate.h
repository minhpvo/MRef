// Copyright ETHZ 2017
// author: mathias, 2016, mathias.rothermel@geod.baug.ethz.ch
#pragma once

#include "../MeshUtil/MyMesh.h"
#include <vector>
#include <Eigen/Dense>

class GradThinPlate
{
public:

	enum LaplacianMode{ KOBBELT, MAYER};
    	GradThinPlate(MyMesh* mesh);
    	~GradThinPlate();

	// This sets thin plate gradient in the mesh  
	void assignGrad(const LaplacianMode mode);

	// This multiplies penalties to the gradienets currently assigned to the mesh 
	void weightClassSpecificPenalties(std::vector<float>& classpenalties);

	// Getter
	Eigen::MatrixXf& getGrad();

private:
	Eigen::MatrixXf _grad;
	MyMesh* _mesh;

	void assignGradKobbelt();
	template <typename T> bool UmbrellaOperator(T& v_it, MyMesh::Point& U);


};
