// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch

#pragma once
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

struct MyTraits : public OpenMesh::DefaultTraits
{
	VertexAttributes(OpenMesh::Attributes::Status);
	FaceAttributes(OpenMesh::Attributes::Status);
	EdgeAttributes(OpenMesh::Attributes::Status);

	 FaceTraits
	 {
	    	 private:
			 short _labelid;
			 int _buildingid;

		 public:
			FaceT() : _labelid(0), _buildingid(0) { }
			const short& labelid() const {return _labelid; }
		 	void setlabelid(const short& lid) { _labelid = lid; }

			const int& buildingid() const {return _buildingid; }
		 	void setbuildingid(const int& bid) { _buildingid = bid; }
	 };


};
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> MyMesh;

