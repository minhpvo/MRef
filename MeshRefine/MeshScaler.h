// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#pragma once

#include "../MeshUtil/MyMesh.h"
#include "../Ori/Orientation.h"

class MeshScaler
{
    public:

	MeshScaler();
	~MeshScaler();

	void getScaledMesh(std::vector<Orientation> &orivec, MyMesh& mesh, const double avfacesize);

    private:

	void getAvFaceSize(std::vector<Orientation>& orivec, MyMesh &meshi, double& avfacesize_img, double& avfacesize_obj);

};
