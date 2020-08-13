// Copyright ETHZ 2017
// author: mathias, 2017, mathias,rothermel@geod.baug.ethz.ch
#include "RayTracer.h"

#include "../MeshUtil/MyMesh.h"
#include "../MeshUtil/MeshIO.h" 
#include "../MeshUtil/MeshConv.h"
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

#include <iostream>

int check( const Eigen::Matrix3d &R, const Eigen::Matrix3d &K, const Eigen::Vector3d &C, int rows, int cols, MyMesh &mesh)
{

	int badcount = 0;
	int goodcount = 0;
	// Build Pmat 
	Eigen::MatrixXd P(3,4);
	Eigen::Matrix3d KR(3,3);
	KR=K*R;

	std::cout<<"K: "<<K<< std::endl;
	std::cout<<"R: "<<R<< std::endl;
	std::cout<<"KR: "<<KR<< std::endl;

	Eigen::Vector3d t(3,1);
	Eigen::Vector4d X(4,1);
	Eigen::Vector3d x(3,1);
	t=(-1.0)*K*R*C;

	std::cout<<"t: "<<t<< std::endl;
	P.row(0)<<KR(0,0),KR(0,1),KR(0,2),t(0);
	P.row(1)<<KR(1,0),KR(1,1),KR(1,2),t(1);
	P.row(2)<<KR(2,0),KR(2,1),KR(2,2),t(2);
	std::cout<<"P: "<<P<< std::endl;
	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
	{

		X << mesh.point(*v_it)[0] , mesh.point(*v_it)[1], mesh.point(*v_it)[2], 1.0;
		x=P*X;
//		std::cout << "\nx=" << x(0)/x(2) << "  y=" << x(1)/x(2); 
		if( x(0)/x(2)<cols && x(0)/x(2)>0 && x(1)/x(2)<rows && x(1)/x(2)>0 ) {
			goodcount++;
			// std::cout<< "\n Cool yo";
		} else{
			badcount++;
		}
	}
	std::cout << "goodcount: " << goodcount << "\tbadcount: " << badcount << std::endl;
return badcount;
}

int main()
{
	// Define camera matrix 
//  int rows=6732;
//	int cols=9000;
//	Eigen::Matrix3d R(3,3);
//	R << -0.355033401337, -0.934853584435, -0.000244140683, -0.765176770496, 0.290744516791, -0.574432011512, 0.537080807541, -0.203755740105, -0.818552261340;
//	Eigen::Matrix3d K(3,3);
//	K<< 8833.33333333, 0.00000000, 4500.00000000, 0.00000000, 8833.33333333, 3366.00000000, 0.00000000, 0.00000000, 1.00000000;
//	Eigen::Vector3d C(3,1);
//	C<< 463783.613700, 5248853.239200, 991.759300;
//
//	// Offset
//	Eigen::Vector3d O(3,1);
//	O << 463505, 5248220, 0.0;
//	O << 464225, 5248220, 0.0;
//	O << 464225, 5248940, 0.0;
//	O << 463505, 5248940, 0.0;
//	C=C-O;
	int rows=1080;
	int cols=1920;
	Eigen::Matrix3d R(3,3);
//	$R 0.82338698, 0.56550890, -0.04726055, 0.53586453, -0.80222270, -0.26322603, -0.18677016, 0.19141163, -0.96357589
//	$K 1920.00000000, 0.00000000, 876.12031494, 0.00000000, 1080.00000000, 520.69784667, 0.00000000, 0.00000000, 1.00000000
//	$C 91.0421370673, -104.4187912000, 119.4204260372

R << 0.8143938503575424, 0.5716183617162828, -0.10007549674425453,0.518120437983988, -0.7938925520921748, -0.3182543440015441,-0.2613692182394414, 0.20733322039983296, -0.9427083682006522;
Eigen::Matrix3d K(3,3);
	K<< 1205.478739731, 0.00000000, 876.12031494, 0.00000000, 1205.478739731, 520.69784667, 0.00000000, 0.00000000, 1.00000000;
	Eigen::Vector3d C(3,1);
	C<< 89.695040476741269, -103.84232057545159, 95.145437410101295;
//	 Offset
	Eigen::Vector3d O(3,1);
	O << 0, 0, 0.0;
	O << 0, 0, 0.0;
	O << 0, 0, 0.0;
	O << 0, 0, 0.0;
	C=C-O;

	Eigen::Vector3f Cf=C.cast<float>();
	
	// Load the mesh
	std::string loadname("/home/centos/MRef/data2/mesh/segmentation_mesh_n_asc_f.ply");
	MyMesh mesh;
  MeshIO::readMesh( mesh, loadname, true, false);
//	mesh.request_face_colors();
	mesh.request_vertex_colors();
	// takes the 4th channel of colors and turns it into the classification value
//	MeshConv::vertexAlphaToVertexLabel(mesh);
	MeshConv::vertexRGBToVertexGreyLabel(mesh);
	// sets face classification to the first vertex class
	MeshConv::vertexLabelToFaceLabel(mesh);
//	MeshConv::faceColorToFaceLabel(mesh);

    	std::vector<std::vector<float> > verts;
    	std::vector<std::vector<int> > faces;
    	std::vector<short> labels;
  MeshConv::openMeshToVectors( mesh, verts, faces, labels);

	// Render 
	 RayTracer mytracer( verts, faces);
	 mytracer.setView(1920.0,1.0,rows,cols,R,K,C);
	 mytracer.traceRaysColumnWise();

	 int intcheck = check(R,K,C,rows,cols,mesh);
	// Get the images
	cv::Mat idimg=mytracer.getIdImage();
	cv::Mat idvisimg(idimg.rows, idimg.cols, CV_8UC1);
	idvisimg.setTo(0);
	int nonzero = 0;
	int zeroes = 0;
	for(int y=0;y<idimg.rows;y++){
		for(int x=0;x<idimg.cols;x++){
			if(idimg.at<int>(y,x)!=0)
			{
				idvisimg.at<unsigned char>(y,x)=labels[idimg.at<int>(y,x)]*(255/(7-1))%255;
				if (nonzero < 5){
					std::cout << "pixel: " << nonzero << " / " << zeroes;
					std::cout << " \tidimg.at<int>(y,x)        :" << idimg.at<int>(y,x);
					std::cout << " \tlabels[tid]:  " << labels[idimg.at<int>(y,x)];
					std::cout << " \tlabel_scaled: " << labels[idimg.at<int>(y,x)]*(255/(7-1))%255<< std::endl;
				}
				nonzero++;
			}
			zeroes++;
		}
	}
	std::cout << "nonzero: " << nonzero << " /total: " << zeroes << std::endl;
	cv::Mat xyzimg=mytracer.getXYZImage();
	std::string savename("./labelimg.png");
	imwrite(savename,idvisimg);
	// Save the images

    	return 1;

}
