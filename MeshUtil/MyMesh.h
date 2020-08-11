// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch

#pragma once
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

struct MyTraits : public OpenMesh::DefaultTraits
{

  // TODO (MAC): This is how you use double valued coordinates
  // typedef OpenMesh::Vec3d Point;
  typedef OpenMesh::Vec4uc Color;

  VertexAttributes(OpenMesh::Attributes::Status |
  OpenMesh::Attributes::Color |
  OpenMesh::Attributes::Normal);
  FaceAttributes(OpenMesh::Attributes::Status |
  OpenMesh::Attributes::Color);
  EdgeAttributes(OpenMesh::Attributes::Status);


  VertexTraits
  {
    private:
    int _classification;

    public:
    VertexT() : _classification(0) { }

    const int& classification() const {return _classification; }
    void setclassification(const int& vid) { _classification = vid; }
  };

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

