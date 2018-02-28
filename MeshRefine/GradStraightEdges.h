// Copyright ETHZ 2017
// author: maros, 2016, maros.blaha@geod.baug.ethz.ch
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch

#pragma once
//#include "io_tools.h"
#include <Eigen/Dense>
#include <math.h>
#include "../MeshUtil/MyMesh.h"
//#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Core/IO/writer/PLYWriter.hh>
//#include <OpenMesh/Core/IO/writer/OBJWriter.hh>
#include "../MeshUtil/MyMesh.h"
//#include "MeshConvertion.h"
//#include "LabelVertFaceConvertion.h"
#include "../PRSTimer/PRSTimer.h"

#define PI 3.14159265

class GradStraightEdges {

private:

    // mesh (vertices and faces)
    std::vector<std::vector<float>> _vertices, _verticesmov;
    std::vector<std::vector<int>> _faces;
    std::vector<int> _labels;
    Eigen::MatrixXf _vertmovsum;
    
    MyMesh _mesh;
    Eigen::MatrixXf _grad;

public:

	GradStraightEdges(MyMesh mesh);
   	 ~GradStraightEdges();

   	 void oneRingStraightEdge2();
	
	 Eigen::MatrixXf getGrad() const;

    GradStraightEdges(std::vector<std::vector<float>> vertices, std::vector<std::vector<int>> faces, std::vector<int> labels);
    // assign vertex labels to face labels
    void faceLabels();

    // straight edge for 1 ring
    void oneRingStraightEdge();

    // get vector with vertex movements
    Eigen::MatrixXf straightEdgeMov();

}; // end GradStraightEdges
