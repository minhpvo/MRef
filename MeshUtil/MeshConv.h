// Copyright ETHZ 2017
#pragma once

#include <vector>
#include "MyMesh.h"

namespace MeshConv 
{
	void vectorsToOpenMesh(const std::vector<std::vector<float> > &verts, const std::vector<std::vector<int> > &faces, const std::vector<short> &lables, MyMesh &ommesh );	

	void openMeshToVectors( const MyMesh &ommesh, std::vector<std::vector<float> > &verts, std::vector<std::vector<int> > &faces, std::vector<short> &lables);	

	void vectorsToOpenMesh(float** &verts, int** faces, short** &lables, MyMesh &ommesh );

	void openMeshToVectors( const MyMesh &ommesh, float** &verts, int** &faces,short** &lables);

	void faceColorToFaceLabel(MyMesh &mesh);

	void faceLabelToFaceColor(MyMesh &mesh);

	void faceLabelToFaceColorRandom(MyMesh &mesh);

	void faceLabelToFaceColorICCV(MyMesh &mesh);

	void faceColorICCVToFacelabel(MyMesh &mesh);

}
