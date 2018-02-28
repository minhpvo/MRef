// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "MeshRefine.h"
#include "GradCalcStereo.h"
#include "GradThinPlate.h"
#include "GradStraightEdges.h"
#include "../MeshClassify/MeshMRF.h"
#include "../LikelihoodImage/LikelihoodImage.h"
#include "../PRSTimer/PRSTimer.h"
#include "../MeshUtil/MeshDivide.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// This should be the class controling the tile-wise processing
MeshRefine::MeshRefine
	(
	MyMesh* tile,
	MeshMetaData* mmd,
	ControlRefine* ctr,
	Eigen::MatrixXf* adjacency,
	IOList* imglistphoto,
	IOList* imglistsem,
	IOList* orilist
	):
	_mesh(tile),
	_mmd(mmd),
	_ctr(ctr),
	_adjacency(adjacency),
	_imglistphoto(imglistphoto),
	_imglistsem(imglistsem),
	_orilist(orilist),
	_verboselevel(ctr->_verboselevel)
{
}

MeshRefine::~MeshRefine()
{
}


void MeshRefine::computeAdaptedOrientation(std::vector<Orientation>& adaptedori)
{
	// Get the connectivity matrix
	Eigen::Vector3d O;
	std::vector<Orientation> orivec;
	for(int i=0; i<_orilist->size();i++)
	{
	    	Orientation ori(_orilist->getElement(i));
	    	// Handle offset
	    	 O<<_mmd->_offsetx,_mmd->_offsety,_mmd->_offsetz;
		Eigen::Vector3d C=ori.getC(); C-=O; ori.setC(C);
		adaptedori.push_back(ori);
	}
}

void MeshRefine::downscale(cv::Mat& img, int level)
{
	for(unsigned int i=0; i<level; i++)
	{
	    cv::pyrDown(img,img);
	}
}

int MeshRefine::cleanGrad(const float avedgelength, Eigen::MatrixXf &grad, Eigen::VectorXi& counter, MyMesh& mesh)
{
	double gl;
    	double _MULTIPLICATOR=.5;
    	double avel=avedgelength*_MULTIPLICATOR;

	// Some counters
    	int clean=0;
	int count=0;
	int nancount=0;
	int sizecount=0;

	for(int y=0; y<grad.rows(); y++)
	{
		// Gradient length
		gl=sqrt( pow(grad(y,0),2.0) + pow(grad(y,1),2.0) + pow(grad(y,2),2.0) );

		// Rubber out nan
		if ( std::isnan(grad(y,0)) || std::isnan(grad(y,1)) || std::isnan(grad(y,2)) )
		{
                	grad(y,0)=0.0f;
			grad(y,1)=0.0f;
			grad(y,2)=0.0f;
			nancount++;
		}
		// Rubber out to long
		else if(gl>avel)
		{
                	grad(y,0)/=gl/avel;
			grad(y,1)/=gl/avel;
			grad(y,2)/=gl/avel;
			sizecount++;
			counter(y)++;
		}
		else
		{
		    clean++;
		    counter(y)++;
		}
		count++;
	}
	if(_verboselevel>1){ std::cout<<"\nInvalidated NAN -> "<<(float)nancount/(float)count*100.0<<"%"; }
	if(_verboselevel>1){ std::cout<<"\nInvalidated SIZE ->"<<(float)sizecount/(float)count*100.0 <<"%"; }
	return clean;
}

double MeshRefine::avEdgeLength(MyMesh& mesh)
{
   	double avel=0.0;
	int count=0;
    	for (MyMesh::EdgeIter e_it=mesh.edges_begin(); e_it!=mesh.edges_end(); ++e_it)
    	{
	    	avel+=mesh.calc_edge_length(*e_it);
		count++;
    	}
	return avel/(double)count;
}

void MeshRefine::updateMesh(MyMesh& mesh, Eigen::MatrixXf &grad)
{
	int idx=0;
    	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    	{
	    	//	std::cout<<grad(idx,0);
        	mesh.point(*v_it)[0]+=(grad(idx,0));
        	mesh.point(*v_it)[1]+=(grad(idx,1));
        	mesh.point(*v_it)[2]+=(grad(idx,2));
		idx++;
    	}
}

void MeshRefine::scaleGradByCounter( Eigen::MatrixXf &grad, Eigen::VectorXi& counter)
{
	int idx=0;
	for (int idx=0; idx<grad.rows();idx++)
    	{
	   	 if(counter(idx)>0)
	   	 {
        		grad(idx,0)/=(float)counter(idx);
        		grad(idx,1)/=(float)counter(idx);
        		grad(idx,2)/=(float)counter(idx);
	    	}
    	}
}

// Main procesing function
void MeshRefine::process()
{
	int numverts=_mesh->n_vertices();
	double avedgelength;
	double oldenergy=0;
	double energy=0;
	double pixsize;
	int nm;

	std::vector<Orientation> orivec;
	computeAdaptedOrientation(orivec);

	// Hirarchical loop
    	for(int pyr=_ctr->_startlevel; pyr>=_ctr->_endlevel; pyr--)
	{
		avedgelength=avEdgeLength(*_mesh);

		Eigen::MatrixXf grad(numverts,3);
		Eigen::VectorXi counter(numverts);
		Eigen::MatrixXf tempgrad(numverts,3);

		// Mesh refinement
    		for (int iter=0;iter<_ctr->_numitervec[pyr]; iter++)
		{
			// Reset gradvec
			grad.setZero();
			tempgrad.setZero();
			counter.setZero();

			PRSTimer itertimer; itertimer.start();
			energy=0;
			pixsize=0.0;
			nm=0;
			//  *** Do the Relabeling ***
			if(	( _ctr->_usemrflabelsmoothing && iter%_ctr->_skipiterations==0 ) ||
				( _ctr->_usesemanticsmooth && iter==0 ) ||
 				( _ctr->_usestraightedgegrad && iter==0) )
			{
				if(_verboselevel>=0)
				{
					std::cout<<"\n||> Semantic Optimization <||";
				}
				Eigen::Vector3d offset=Eigen::Vector3d::Zero(3);
        			MeshMRF meshmrf(_mesh,_mmd,5);
        			meshmrf.process(_imglistsem->getList(), _orilist->getList(),_ctr->_nummrfitervec[pyr]);
			}
			if(_verboselevel>=0)
			{
				std::cout<<"\n||> Geometric Optimization <||";
			}
			//  *** Photometric and Semantic Consistency ***
        		// Iterate over all images
        		if(_verboselevel>=0){ std::cout << "\nLevel: "<<pyr<<"\tIteration: " << (iter+1) << " / " << _ctr->_numitervec[pyr] ; }
        		for (int i=0; i<_adjacency->rows(); i++)
			{
            			// Check if image is a master
            			if ((*_adjacency)(i,i)>0.0 )
				{

			    		if(_verboselevel>=1) { std::cout <<"\nProcessing baseimage "<<i; }
					// Load base image stuff
			    		Orientation ori0=orivec[i];
					ori0.downscalePyr(pyr);

					PRSTimer timer; timer.start();
					cv::Mat img0=cv::imread(_imglistphoto->getElement(i), cv::IMREAD_GRAYSCALE);
					downscale(img0,pyr);
					if(img0.cols==0 || img0.rows==0){std::cout<<"\nProlems readingimg"; exit(1);}
					if(img0.type()!=CV_8UC1){img0.convertTo(img0,CV_8UC1); }
					timer.stop();

					LikelihoodImage limg0;
					if(_ctr->_usesemanticdata)
					{
						limg0.loadImage(_imglistsem->getElement(i));
						limg0.downscale(pyr);
					}

					GradCalcStereo gradcomp( img0, &limg0, &ori0, _mesh,
								 _ctr->_usesemanticdata ? GradCalcStereo::SEMANTICMODE::ON : GradCalcStereo::SEMANTICMODE::OFF, _ctr->_tnear, _ctr->_tfar, _ctr->_verboselevel);
                			// Get vector with all slave images
               				for (int j=0; j<_adjacency->cols(); j++)
					{
                    				// avoid reference image
                    				if ((*_adjacency)(i,j)>0.0 && j != i)
						{

			    				if(_verboselevel>=1) { std::cout <<"\nProcessing match image "<<j; }
							// Load slave img stuff
							PRSTimer timer2; timer2.start();
			    				Orientation ori1=orivec[j];
							ori1.downscalePyr(pyr);
							cv::Mat img1=cv::imread(_imglistphoto->getElement(j), cv::IMREAD_GRAYSCALE);
							downscale(img1,pyr);
							if(img1.type()!=CV_8UC1){img1.convertTo(img1,CV_8UC1,255); }
							LikelihoodImage limg1;
							if(_ctr->_usesemanticdata)
							{
								limg1.loadImage(_imglistsem->getElement(j));
								limg1.downscale(pyr);
							}
							timer2.stop();

							// Set the second view...
							gradcomp.setSecondView(img1,&limg1,&ori1);

							// This should compute Potometric
							// and if available Semantic data terms
		    					energy+=gradcomp.process();
							pixsize+=gradcomp.computeTriangleSizeInPix();

			    				// Actually here base should be scaled

							// Get the gradient, rubber nan and a
                    			 		tempgrad=gradcomp.getPhotoGrad()*_ctr->_photoweightvec[pyr]*pow(gradcomp.getMeanDist(),2.0)/pow((ori1.getK())(0,0),2.0);
					//	std::cout<<tempgrad;
							cleanGrad(avedgelength,tempgrad,counter,*_mesh);
							grad+=tempgrad;

							// Get the gradient, rubber nan and a
                    			 		tempgrad=gradcomp.getSemanticGrad()*_ctr->_semweightvec[pyr];
							cleanGrad(avedgelength,tempgrad,counter,*_mesh); 
							grad+=tempgrad;
							nm++;
                    				}
					}
				}
			}
			if(_verboselevel>=0){ std::cout<<"\nEnergy="<<energy<<" || Delta energy="<<energy-oldenergy<< " || Av. face size="<<floor( pixsize/(float)nm*10.0)*0.1<<"[pix]\n"; }
			oldenergy=energy;

			scaleGradByCounter(grad, counter);

        		// ********* Thin plate gradient ********
        		PRSTimer tptime; tptime.start();
			GradThinPlate smoothgen(_mesh);
        		smoothgen.assignGrad(GradThinPlate::LaplacianMode::KOBBELT);
			if(_ctr->_usesemanticsmooth)
			{
				// roof -> 2 | facade -> 0 | ground -> 1 | veg -> 3
			    	std::vector<float> penalties(4);
			    	// Set the penalties for level
			    	penalties[0]=_ctr->_smoothfacadeweightvec[pyr];
			    	penalties[1]=_ctr->_smoothgroundweightvec[pyr];
			    	penalties[2]=_ctr->_smoothvegeweightvec[pyr];
			    	penalties[3]=_ctr->_smoothroofweightvec[pyr];

				smoothgen.weightClassSpecificPenalties(penalties);
              			tempgrad=smoothgen.getGrad()*(-1.0);
			}
			else
			{
              			tempgrad=smoothgen.getGrad()*(-1.0)*_ctr->_smoothweightvec[pyr];
			}
			int clean=cleanGrad(avedgelength,tempgrad,counter,*_mesh);
			grad+=tempgrad;

			tptime.stop();
			if(_verboselevel>=1) { std::cout<<" Thinplate took: "<<tptime.getTimeSec()<<" sec. Valid:" << (float)clean/(float)numverts;}

			//********* Edge Gradient *********
			if(_ctr->_usestraightedgegrad)
			{
        			GradStraightEdges edgegen(*_mesh);
        			edgegen.oneRingStraightEdge2();
              			tempgrad=edgegen.getGrad();
				cleanGrad(avedgelength,tempgrad,counter,*_mesh);
				grad+=tempgrad*_ctr->_straightedgeweightvec[pyr];
			}

			// Here do the overall scaling of the gradient
			updateMesh( *_mesh, grad);
			itertimer.stop();
			if(_verboselevel>=1){ std::cout<<"\n Iteration took: "<<itertimer.getTimeMin()<<" min"; }

    		} // end iterations

		// Here upscaling has to be done, densify mesh
		if(pyr!=_ctr->_endlevel) 
		{
		    	MeshDivide::subDivideEqual(*_mesh);
			numverts=_mesh->n_vertices();
		}
	} //end level
}


