// Copyright ETHZ 2017
// author: mathias, 2017, mathias,rothermel@geod.baug.ethz.ch
#include "RayTracer.h"

#include "../MeshUtil/MyMesh.h"
#include "../MeshUtil/MeshIO.h" 
#include "../MeshUtil/MeshConv.h"
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

#include <iostream>

int check( const Eigen::Matrix3f &R, const Eigen::Matrix3f &K, const Eigen::Vector3f &C, int rows, int cols, MyMesh &mesh)
{

	// Build Pmat 
	Eigen::MatrixXf P(3,4);
	Eigen::Matrix3f KR(3,3);
	KR=K*R;

	std::cout<<"\nK "<<K;
	std::cout<<"\nR "<<R;
	std::cout<<"\nKR "<<KR;

	Eigen::Vector3f t(3,1);
	Eigen::Vector4f X(4,1);
	Eigen::Vector3f x(3,1);
	t=(-1.0)*K*R*C;

	std::cout<<"\nt "<<t;
	P.row(0)<<KR(0,0),KR(0,1),KR(0,2),t(0);
	P.row(1)<<KR(1,0),KR(1,1),KR(1,2),t(1);
	P.row(2)<<KR(2,0),KR(2,1),KR(2,2),t(2);
	std::cout<<"\nP "<<P;
	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
	{

		X << mesh.point(*v_it)[0] , mesh.point(*v_it)[1], mesh.point(*v_it)[2], 1.0;
		x=P*X;
//		std::cout << "\nx=" << x(0)/x(2) << "  y=" << x(1)/x(2); 
		if( x(0)/x(2)<cols && x(0)/x(2)>0 && x(1)/x(2)<rows && x(1)/x(2)>0 ) std::cout<< "\n Cool yo";
	}

}

int main()
{
	// Define camera matrix 
  	int rows=6732;
	int cols=9000;
	Eigen::Matrix3d R(3,3);
	R << -0.355033401337, -0.934853584435, -0.000244140683, -0.765176770496, 0.290744516791, -0.574432011512, 0.537080807541, -0.203755740105, -0.818552261340;
	Eigen::Matrix3d K(3,3);
	K<< 8833.33333333, 0.00000000, 4500.00000000, 0.00000000, 8833.33333333, 3366.00000000, 0.00000000, 0.00000000, 1.00000000;
	Eigen::Vector3d C(3,1);
	C<< 463783.613700, 5248853.239200, 991.759300;

	// Offset 
	Eigen::Vector3d O(3,1);
	O << 463505, 5248220, 0.0;
	O << 464225, 5248220, 0.0;
	O << 464225, 5248940, 0.0;
	O << 463505, 5248940, 0.0;
	C=C-O;

	Eigen::Vector3f Cf=C.cast<float>();
	
	// Load the mesh
	std::string loadname("/home/mathias/workspace/urbanup/data/facades.obj");
	MyMesh mesh;
        MeshIO::readMesh( mesh, loadname, false, true);
	MeshConv::faceColorToFaceLabel(mesh);

    	std::vector<std::vector<float> > verts;
    	std::vector<std::vector<int> > faces;
    	std::vector<short> labels;
    	MeshConv::openMeshToVectors( mesh, verts, faces, labels);

	// Render 
	RayTracer mytracer( verts, faces);
	mytracer.setView(1000.0,100.0,rows,cols,R,K,C);
	mytracer.traceRaysColumnWise();

	// Get the images
	cv::Mat idimg=mytracer.getIdImage();
	cv::Mat idvisimg(idimg.rows, idimg.cols, CV_8UC1);
	idvisimg.setTo(0);
	for(int y=0;y<idimg.rows;y++){
		for(int x=0;x<idimg.cols;x++){
			if(idimg.at<int>(y,x)!=0)
			{
				 idvisimg.at<unsigned char>(y,x)=labels[idimg.at<int>(y,x)]%255;
			}
		}
	}
	cv::Mat xyzimg=mytracer.getXYZImage();

	std::string savename("../data/labelimg.png");
	imwrite(savename,idvisimg);
	// Save the images

    	return 1;

}
