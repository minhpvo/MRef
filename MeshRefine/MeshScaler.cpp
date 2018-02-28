// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include <iostream>
#include "MeshScaler.h"
#include "../MeshUtil/MeshDivide.h"
#include "../MeshUtil/MeshSimplify.h"
#include "../MeshUtil/MeshGeom.h"
#include "../PRSTimer/PRSTimer.h"

MeshScaler::MeshScaler()
{
}

MeshScaler::~MeshScaler()
{
}

void MeshScaler::getAvFaceSize(std::vector<Orientation>& orivec, MyMesh &mesh, double& avfacesize_img, double& avfacesize_obj)
{
    	avfacesize_img=0.0;
    	avfacesize_obj=0.0;

    	int count=0;
	int jumper=20;
	for(std::vector<Orientation>::iterator it=orivec.begin(); it!=orivec.end(); ++it)
	{
	    	Eigen::MatrixXd P(3,4);
		P=it->getP();
		float m[3][2];
		float M[3][3];
		Eigen::Vector4d V;
		Eigen::Vector3d v;
		int i;

		int f=0;
		while( f*jumper<mesh.n_faces() )
		{
		    	MyMesh::FaceIter f_it=mesh.faces_begin()+jumper*f;
			i=0;

			// Compute face size in imgspace
			for(MyMesh::FaceVertexIter fv_it=mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			{

				V << mesh.point(*fv_it)[0], mesh.point(*fv_it)[1], mesh.point(*fv_it)[2], 1.0;
			    	v=P*V;
			    	m[i][0]=v(0)/v(2);
			    	m[i][1]=v(1)/v(2);
				M[i][0]=V(0);
				M[i][1]=V(1);
				M[i][2]=V(2);

				i++;
			}
			avfacesize_img+=MeshGeom::faceArea2D(m[0],m[1],m[2]);

			// Compute face size in object space
			avfacesize_obj+=MeshGeom::faceArea(M[0],M[1],M[2]);
			count++;
			f++;
		}
	}
	avfacesize_img/=(double)count;
	avfacesize_obj/=(double)count;

}

void MeshScaler::getScaledMesh(std::vector<Orientation> &orivec, MyMesh& mesh, const double avfacesize)
{

    	bool resok=false;

	while(!resok)
	{
		// Get avarge pixel range of all reprojeced faces
		double avfacesize_img;
		double avfacesize_obj;
		double avedgelegth_img;
		double avedgelegth_obj;
		getAvFaceSize(orivec, mesh, avfacesize_img, avfacesize_obj);

		// Desired Facesize in pix... approximation
		float edgelength=sqrt(2.0*avfacesize);
		float edgelength_img=sqrt(2.0*avfacesize_img);
		float edgelength_obj=sqrt(2.0*avfacesize_obj);

		std::cout<<"\navfacesize_img is: "<<avfacesize_img;
		std::cout<<"\navfacesize_obj is: "<<avfacesize_obj;

		// Decide if we should upscale or downscale
		if(avfacesize_img>avfacesize*3.2)
		{
	   		// Sub divide
		   	PRSTimer timer;
			timer.start();
	   		std::cout<<"\nStart Densify!"; 
	    		MeshDivide::subDivLoop(mesh,1);
			timer.stop();
			std::cout<<"\ndone! ["<<timer.getTimeSec()<<" sec]";
	   	 //	MeshDivide::subDivEdgeLength(mesh,10,edgelength/30);
	    	//	MeshDivide::subDivCatClark(mesh,1);
		}
		else if(avfacesize_img<avfacesize*0.32)
		{
			// Merge
		   	PRSTimer timer;
			timer.start();
	   		std::cout<<"\nStart Sparsify!"; 
	    		MeshSimplify::simplifyEdge(mesh,edgelength_obj);
			timer.stop();
			std::cout<<"\ndone! ["<<timer.getTimeSec()<<" sec]";
	    		//MeshSimplify::simplifyQuadric(mesh,.0000000001);
		}
		else
		{
	 	   	// Mesh resolution is okish
	 		resok=true;
		}
	}
}


