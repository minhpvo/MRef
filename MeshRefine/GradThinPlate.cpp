// Copyright ETHZ 2017
// author: mathias, 2016, mathias.rothermel@geod.baug.ethz.ch

#include "GradThinPlate.h"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <math.h>

GradThinPlate::GradThinPlate(MyMesh* mesh):_mesh(mesh)
{
    _grad.resize(_mesh->n_vertices(),3);
    _grad.setZero();
};

GradThinPlate::~GradThinPlate(){};

template<typename T>
bool GradThinPlate::UmbrellaOperator(T& v_it, MyMesh::Point& U)
{
	float x_c,y_c,z_c=0.0;
	int neighborcount=0;
	MyMesh::Point* pt;
	pt=&(_mesh->point(*v_it));
	x_c=(*pt)[0];
	y_c=(*pt)[1];
	z_c=(*pt)[2];

	U[0]=U[1]=U[2]=0.0; 

	// Iterate over all one ring vertices
	for (MyMesh::VertexVertexIter vv_it=_mesh->vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
	{
		// Check if boundary
		if(_mesh->is_boundary(*vv_it)) {return 0;}
		pt=&(_mesh->point(*vv_it));
		U[0]+=(*pt)[0]-x_c;
		U[1]+=(*pt)[1]-y_c;
		U[2]+=(*pt)[2]-z_c;
		neighborcount++;
	}
	U[0]/=(float)neighborcount;
	U[1]/=(float)neighborcount;
	U[2]/=(float)neighborcount;
	return 1;

}

Eigen::MatrixXf& GradThinPlate::getGrad()
{
    return _grad;
}

void GradThinPlate::assignGrad(const LaplacianMode mode)
{
	if(mode==KOBBELT) assignGradKobbelt(); 
}


// this is not to nice right now... 
void GradThinPlate::weightClassSpecificPenalties(std::vector<float>& classpenalties)
{

    	float penalty;
	unsigned short label=65535;
	unsigned short curlabel;
	MyMesh::Point grad;
	int noneidx=classpenalties.size();
	// this is _numclasses-length, with the smoothweightvec[class] penalties
	// so noneidx is 1 more than any of the possible real label

	// Figure label for vert
	int idx=0;
	for (MyMesh::VertexIter v_it=_mesh->vertices_begin(); v_it!=_mesh->vertices_end(); ++v_it)
	{
		// This is just a large number to signal that we're
		//   looking at the first face attached to the vertex, before we iterate through the 
		//   rest of the faces touching that vertex
		label=65535;
	    	for(MyMesh::VertexFaceIter vf_it=_mesh->vf_iter(*v_it); vf_it.is_valid(); ++vf_it)
		{
		    	if(label==65535) // first face
		    	{
		    		label=_mesh->data(*vf_it).labelid();
				continue;
		    	}
		    	else // second face to end 
		    	{
				curlabel=_mesh->data(*vf_it).labelid();
				if(curlabel!=label) 
				{
					// this means the face label differs from the very first face that was checked
					label=noneidx; //set face label to the impossible label
					break; //don't need to look at the rest of the faces sharing this vertex, then
					       // since we already know at least 1 is different
				}
		    	}
		}

	    	// scale
		//done looking at adjacent faces -- they're either all the same (and label is that shared label), 
		//  or there's at least 1 that's different, and label == noneidx
	    	if(label!=noneidx)
		{
			// all the face classes around this vertex have the same label. 
			// Gradient is scaled proportionally to that label's smoothness weight
			// Note: all penalties in [0,1) right now so this decreases the size of the gradient
			// and it's important if penalty is <1 or >1 !
			penalty=classpenalties[label];
			_grad(idx,0)*=penalty; 
			_grad(idx,1)*=penalty;
			_grad(idx,2)*=penalty;
		}
		else
		{
			// the classes around this vertex point are NOT the same; 
			//   0-out gradient at this vertex
			_grad(idx,0)=0;
			_grad(idx,1)=0;
			_grad(idx,2)=0;
		}
		idx++;
	}
}

void GradThinPlate::assignGradKobbelt()
{	
    	float x_c,y_c,z_c;
	MyMesh::Point U_c,U_i,U;
	U[0]=U[1]=U[2]=0.0;
	int count;
	bool foundboundary;
	MyMesh::Point Uvec[1000];
	
	// Skim all verts
	int vertcount=0;
	int idx=0;
	for (MyMesh::VertexIter v_it=_mesh->vertices_begin(); v_it!=_mesh->vertices_end(); ++v_it)
	{
	    	foundboundary=false;
	    	// Compute umbrella for current vert 
	    	if(!UmbrellaOperator(v_it,U_c))
		{
			_grad(idx,0)=0.0;
			_grad(idx,1)=0.0;
			_grad(idx,2)=0.0;
			idx++;
		    	continue;
		}

		count=0;
		// Compute umbrealla for neighboring verts
		for (MyMesh::VertexVertexIter vv_it=_mesh->vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
		{
		  	if( !UmbrellaOperator(vv_it, U_i))
			{ 
			    	foundboundary=true;
			    	break;
			}

			U[0]+=U_i[0]-U_c[0];
			U[1]+=U_i[1]-U_c[1];
			U[2]+=U_i[2]-U_c[2];
			Uvec[count][0]=U_i[0]-U_c[0];
			Uvec[count][1]=U_i[1]-U_c[1];
			Uvec[count][2]=U_i[2]-U_c[2];
			count++;
		}
		if(foundboundary)
		{
			_grad(idx,0)=0.0;
			_grad(idx,1)=0.0;
			_grad(idx,2)=0.0;
		}
		else
		{
			U[0]/=(float)count;
			U[1]/=(float)count;
			U[2]/=(float)count;

			_grad(idx,0)=U[0];
			_grad(idx,1)=U[1];
			_grad(idx,2)=U[2];

		}
		idx++;
	}
}

