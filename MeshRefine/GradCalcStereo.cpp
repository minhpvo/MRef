// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
// author: maros, 2016, maros.blaha@geod.baug.ethz.ch

#include "GradCalcStereo.h"
#include "DMDI2.h"
#include "../PRSTimer/PRSTimer.h"
#include "../Ori/OriGeom.h"
//#include "../ImgUtil/ImgIO.h"
#include "../MeshUtil/MeshGeom.h"

#include <embree2/rtcore.h>
#include <embree2/rtcore_ray.h>
#include <xmmintrin.h>
#include <pmmintrin.h>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>

GradCalcStereo::GradCalcStereo
	(
	cv::Mat &img0,
	LikelihoodImage* limage0,
	Orientation* ori0,
	MyMesh* mesh,
	SEMANTICMODE smode,
	const double tnear,
	const double tfar,
	const int verboselevel
	)
    	:
	_img0(img0),
	_limage0(limage0),
	_ori0(ori0),
	_mesh(mesh),
	_smode(smode),
	_meandist(0.0),
	_tnear(tnear),
	_tfar(tfar),
	_verboselevel(verboselevel)
{

    	// Check that input images 8 bit!
    	if(img0.type()!=CV_8U) { std::cout<<"GradCalc: Input Grayscale must be 8bit."; exit(1);}

	// Build the canny images
	buildCannyImages(0);

	// Get number of labels
	_numlabels=limage0->labels();
	_numverts=_mesh->n_vertices();
	_numfaces=_mesh->n_faces();
	_cols0=img0.cols;
	_rows0=img0.rows;

	// Weighting image
	_weightimg.create(_rows0,_cols0,CV_32F);

	// Gradients
	_gradsemantic.resize(_numverts,3);
	_gradphoto.resize(_numverts,3);

	// Allocate the stuff for gradient computation
	 _simimg.create(_rows0,_cols0,CV_32F);
	 if(_smode==ON)_simlabels=new LikelihoodImage(_rows0,_cols0,_numlabels);

	// Reprojected
	 _img10.create(_rows0,_cols0, CV_8UC1);
	if(_smode==ON)_limage10=new LikelihoodImage(_rows0,_cols0,_numlabels);

	// Reprojected Jacobian 10
	_jacimg10x.create(_rows0,_cols0,CV_32F);
	_jacimg10y.create(_rows0,_cols0,CV_32F);
	if(_smode==ON)_jaclabels10x=new LikelihoodImage(_rows0,_cols0,_numlabels);
	if(_smode==ON)_jaclabels10y=new LikelihoodImage(_rows0,_cols0,_numlabels);

	_dix.resize(_rows0,_cols0);
	_diy.resize(_rows0,_cols0);
	_diz.resize(_rows0,_cols0);

	_Nx.resize(_rows0,_cols0);
	_Ny.resize(_rows0,_cols0);
	_Nz.resize(_rows0,_cols0);

	// Struct for valid verts
	 _vertsvalid=new bool[_numverts];
	 _facesvalid=new bool[_numfaces];

	// Struct for jacp
	_pjac=new float**[_rows0];
	for(int y=0;y<_rows0;y++)
	{
		_pjac[y]=new float*[_cols0];
		for(int x=0;x<_cols0;x++)
		{
		    _pjac[y][x]=new float[6];
		}
	}

	_rt0=new RayTracer(*_mesh);
    	_rt0->setView(_tfar,_tnear, *_ori0,false);
    	_rt0->traceRaysColumnWise();
}

void GradCalcStereo::setSecondView(	cv::Mat &img1,
					LikelihoodImage* limage1,
					Orientation* ori1 )
{
	// Set the members
	_img1=img1;
	_limage1=limage1;
	_ori1=ori1;

    	// Check that input images 8 bit!
    	if(img1.type()!=CV_8U) { std::cout<<"GradCalc: Input Grayscale must be 8bit."; exit(1);}

	// Build the canny images
	buildCannyImages(1);
	
	_cols1=img1.cols;
	_rows1=img1.rows;

	// Jacobians of image 1
	_jacimg1x.create(_rows1,_cols1,CV_32F);
	_jacimg1y.create(_rows1,_cols1,CV_32F);

	if(_smode==ON)_jaclabels1x=new LikelihoodImage(_rows1,_cols1,_numlabels);
	if(_smode==ON)_jaclabels1y=new LikelihoodImage(_rows1,_cols1,_numlabels);

	// Set up the Raytracer
	_rt1=new RayTracer(*_mesh);
    	_rt1->setView(_tfar,_tnear, *_ori1,false);
    	_rt1->traceRaysColumnWise();
}

GradCalcStereo::~GradCalcStereo()
{

    	if(_smode==ON)
    	{
    		delete _simlabels;
		delete _limage10;
		delete _jaclabels1x;
		delete _jaclabels1y;
		delete _jaclabels10x;
		delete _jaclabels10y;
	}
	delete[] _vertsvalid;
	delete[] _facesvalid;

	for(int y=0;y<_rows0;y++)
	{
		for(int x=0;x<_cols0;x++)
		{
		    delete[] _pjac[y][x];
		}
		delete[] _pjac[y];
	}
	delete[] _pjac;

	delete _rt0;
	delete _rt1;

}

// GET OPTIMIZED RESULTS
Eigen::MatrixXf& GradCalcStereo::getSemanticGrad() {
    return _gradsemantic;
}


Eigen::MatrixXf& GradCalcStereo::getPhotoGrad() {
    return _gradphoto;
}

double GradCalcStereo::getMeanDist(void) const {
 	return _meandist;
}
// Integrate over triangle backprojected rings, this should be expensive ... parallize crap out of this!!!
void GradCalcStereo::updateGrad(cv::Mat& bary0, cv::Mat& tid0, Orientation &ori0)
{

        Eigen::Vector4d X;
        Eigen::Vector3d x;
	Eigen::MatrixXd P0=_ori0->getP();

	int faceidx,runidx, ptidx, vertorder, facecount;
	MyMesh::VertexIter v_it;
	MyMesh::VertexFaceIter vf_it;
	MyMesh::FaceVertexIter fv_it;

    	// iter over all vertices
	int vertidx=0;

	#pragma omp parallel for \
	    	private (X,x,faceidx,runidx,ptidx,vertorder,facecount,v_it,vf_it,fv_it) \
		firstprivate(P0)
	for( vertidx=0; vertidx<_mesh->n_vertices(); vertidx++)  
	{
		// Reset grads
		_gradphoto(vertidx,0)=0.0;
		_gradphoto(vertidx,1)=0.0;
		_gradphoto(vertidx,2)=0.0;

		_gradsemantic(vertidx,0)=0.0;
		_gradsemantic(vertidx,1)=0.0;
		_gradsemantic(vertidx,2)=0.0;

        	// check if vertex is valid
        	if (_vertsvalid[vertidx])
		{
			float tx[3];
			float ty[3];
	    		v_it=_mesh->vertices_begin()+vertidx;

                	// iter over 1-ring faces
			facecount=0;
                	for (vf_it=_mesh->vf_iter(*v_it); vf_it.is_valid(); ++vf_it) {

                    		// get face ID
                    		faceidx=vf_it->idx();

				ptidx=0;
				for (fv_it=_mesh->fv_iter(*vf_it); fv_it.is_valid(); ++fv_it)
				{
				    	runidx=(*fv_it).idx();
				   	if( ! _vertsvalid[runidx]  )
				    	{
						continue;
				    	}
				    	else
				    	{
            					// get vertex coordinates
					    	MyMesh::Point &pt=_mesh->point(*fv_it);
						X <<pt[0],pt[1],pt[2],1.0;

						// Project to img
						x=P0*X;

						// Get image coordinates of all three points?
						tx[ptidx]=x(0)/x(2);
						ty[ptidx]=x(1)/x(2);

						// identify which id vert in face range
						if(vertidx==runidx){ vertorder=ptidx; }
				    	}
				    	ptidx++;
				}
                        	facecount=facecount+1;

				// Integrate over triangle
				integrateTriangle(_smode, faceidx, vertidx, vertorder, tx, ty, tid0, bary0);
				//std::cout<<"\nIntegration took "<<timer.getTimeSec()<<" "<<vertidx<<" "<<_numverts;
			}
		}
	}

	// Transform Gradient into world COS
	Eigen::Matrix3f R0=ori0.getR().cast<float>();
	Eigen::Matrix3f R0t=R0.transpose();
	Eigen::Vector3f g;

	//#pragma omp parallel for \
	    	private (g) \
		firstprivate(R0,R0t)
	for(int v=0; v<_numverts;v++)
	{
		g<<_gradphoto(v,0), _gradphoto(v,1), _gradphoto(v,2);
		g=R0t*g;
		_gradphoto(v,0)=g(0); _gradphoto(v,1)=g(1); _gradphoto(v,2)=g(2);
		
		g<<_gradsemantic(v,0), _gradsemantic(v,1), _gradsemantic(v,2);
		g=R0t*g;
		_gradsemantic(v,0)=g(0); _gradsemantic(v,1)=g(1); _gradsemantic(v,2)=g(2);
	}
}


// This can be parallized
void GradCalcStereo::updateTIDImagesOcclusions
(
	cv::Mat &tid0, cv::Mat &tid1,
	cv::Mat &XYZ0, cv::Mat XYZ1,
	const Orientation &ori0, const Orientation &ori1,
	cv::Mat& map0x, cv::Mat& map0y
)
{
	_meandist=0;
	int counter=0;

    	float _SMALLVALL_TRACECONSISTENT=0.001;
    	float distpt, dist0;
	int x1int, y1int;

    	int cols0=ori0.cols();
    	int rows0=ori0.rows();
    	int cols1=ori1.cols();
    	int rows1=ori1.rows();

	Eigen::MatrixXd eP1=ori1.getP();
	cv::Mat P1(3,4,CV_32F);
	for(int r=0; r<3;r++) for(int c=0; c<4;c++) P1.at<float>(r,c)=eP1(r,c);

	Eigen::Vector3d C0=ori0.getC();	

	float Xbuffer[4]; Xbuffer[3]=1.0;
	cv::Mat X(4,1,CV_32F, Xbuffer);
	float xbuffer[3];
	cv::Mat x(3,1,CV_32F, xbuffer);

	// Generate a validity mask of valid triangles in base image
	for(int y0=0;y0<rows0;y0++)
	{
		for(int x0=0;x0<cols0;x0++)
		{
			if(tid0.at<int>(y0,x0)>=0)
			{
			    	Xbuffer[0]=XYZ0.at<cv::Vec3f>(y0,x0)[0];
			    	Xbuffer[1]=XYZ0.at<cv::Vec3f>(y0,x0)[1];
				Xbuffer[2]=XYZ0.at<cv::Vec3f>(y0,x0)[2];
				Xbuffer[3]=1.0;

				// Reproject to slave view
				x=P1*X;
				xbuffer[0]/=xbuffer[2];
				xbuffer[1]/=xbuffer[2];

				x1int=(int)(xbuffer[0]+0.5);
				y1int=(int)(xbuffer[1]+0.5);

				if( !(x1int<cols1 && x1int>=0 && y1int>=0 && y1int<rows1) )
				{ // Is pt seen in second img?

			    	    tid0.at<int>(y0,x0)=-1;
				    continue;
				}
				else
				{
					// Bilinear interpolation is missing
				    	distpt=	pow(Xbuffer[0]-XYZ1.at<cv::Vec3f>(y1int,x1int)[0],2.0)+
					    	pow(Xbuffer[1]-XYZ1.at<cv::Vec3f>(y1int,x1int)[1],2.0)+
						pow(Xbuffer[2]-XYZ1.at<cv::Vec3f>(y1int,x1int)[2],2.0);

				    	dist0=	pow(Xbuffer[0]-C0(0),2.0)+
					    	pow(Xbuffer[1]-C0(1),2.0)+
						pow(Xbuffer[2]-C0(2),2.0);

					// Check distance
					if(fabs(distpt)>dist0*_SMALLVALL_TRACECONSISTENT)
					{
					    	tid0.at<int>(y0,x0)=-1;
						tid1.at<int>(y1int,x1int)=-1;
					}
					else
					{
					    	// Store values for remap functions
					    	map0x.at<float>(y0,x0)=xbuffer[0];
					    	map0y.at<float>(y0,x0)=xbuffer[1];

						_meandist+=sqrt(dist0);
						counter++;
					}
				}
			}
		}
	}
	_meandist/=(double)counter;
}


// Remap function, gpu remap available, Cpu can be done
void GradCalcStereo::remapAll(const SEMANTICMODE mode, const cv::Mat &mapx, const cv::Mat &mapy)
{

#pragma omp parallel sections
{
	#pragma omp section
	{
	// What is this -> grayscale! Cange 4 chan to 1 chan!
    	cv::remap(_img1, _img10, mapx, mapy, cv::INTER_LINEAR);
	}
	#pragma omp section
	{
	// Reproject Jacobian intensity
    	cv::remap(_jacimg1x, _jacimg10x, mapx, mapy, cv::INTER_LINEAR);

	}
	#pragma omp section
	{
	cv::remap(_jacimg1y, _jacimg10y, mapx, mapy, cv::INTER_LINEAR);
	}
	#pragma omp section
	{
	cv::remap(_canny1,_canny10, mapx, mapy, cv::INTER_LINEAR);
	}
}
	//cv::imwrite("/home/mathias/AlmostTrash/weightimg.tif",_canny10);
	//exit(1);
	if(_smode==ON)
	{

		for(int l=0; l<_numlabels; l++)
		{
			// Semantics
#pragma omp parallel sections
{
			#pragma omp section
			{
			cv::Mat l1(_rows1,_cols1,CV_32F);
			cv::Mat l10(_rows0,_cols0,CV_32F);
			_limage1->getLayer(l,l1);
    			cv::remap(l1, l10, mapx, mapy, cv::INTER_LINEAR);
			_limage10->setLayer(l,l10);
			}
			#pragma omp section
			{
			// Jacobian semantics
			cv::Mat l1(_rows1,_cols1,CV_32F);
			cv::Mat l10(_rows0,_cols0,CV_32F);
			_jaclabels1x->getLayer(l, l1);
    			cv::remap(l1, l10, mapx, mapy, cv::INTER_LINEAR);
			_jaclabels10x->setLayer(l,l10);
			}
			#pragma omp section
			{
			cv::Mat l1(_rows1,_cols1,CV_32F);
			cv::Mat l10(_rows0,_cols0,CV_32F);
			_jaclabels1y->getLayer(l, l1);
    			cv::remap(l1, l10, mapx, mapy, cv::INTER_LINEAR);
			_jaclabels10y->setLayer(l,l10);
			}
}
		}

}

}


double GradCalcStereo::calcSimilarity(const SEMANTICMODE mode, cv::Mat& tid0)
{
	double energy,dummy;
	cv::Mat varimg0, varimg10;
	// Generate valimg
	cv::Mat valimg0(_rows0,_cols0, CV_8U);
	for(int y=0; y<_rows0; y++)
	{
		for(int x=0; x<_cols0; x++)
		{
			if( tid0.at<int>(y,x)==-1 ){valimg0.at<unsigned char>(y,x)=0; }
			else { valimg0.at<unsigned char>(y,x)=255; }
		}
	}
	// Compute the drivation matching cost
    	DMDI2::compute(_img0,_img10, valimg0, 5, DMDI2::MATCHINGCOST::NCC, _simimg, varimg0, varimg10, energy );

	// Compute the weighting function
	buildWeightImages(varimg0,varimg10,valimg0);

	if(_smode==ON)
	{
		cv::Mat cvin0(_rows0, _cols0, CV_32F);
		cv::Mat cvin10(_rows0, _cols0, CV_32F);
		cv::Mat cvout(_rows0, _cols0, CV_32F);
		cv::Mat cvvar0(_rows0, _cols0, CV_32F);
		cv::Mat cvvar10(_rows0, _cols0, CV_32F);

		    for(int l=0; l<_numlabels; l++)
		    {
		    	_limage0->getLayer(l, cvin0);
		    	_limage10->getLayer(l, cvin10);

			DMDI2::compute(cvin0, cvin10, valimg0, 5, DMDI2::MATCHINGCOST::SSD, cvout, cvvar0, cvvar10,dummy);

			_simlabels->setLayer(l, cvout);
		}
	}
	return energy;
}

// JACOBIAN PROJECTION MATRIX P INT
void GradCalcStereo::jacobianP(const float X, const float Y, const float Z, const int x,const  int y, const Eigen::MatrixXd &P10)
{

    double nom0=P10(0,0)*X+P10(0,1)*Y+P10(0,2)*Z+P10(0,3);
    double nom1=P10(1,0)*X+P10(1,1)*Y+P10(1,2)*Z+P10(1,3);
    double denom=P10(2,0)*X+P10(2,1)*Y+P10(2,2)*Z+P10(2,3);
    double denom2=denom*denom;

    // Compute partial derivatives
    float* ptr=_pjac[y][x];
    ptr[0]=( P10(0,0)*denom - P10(2,0)*nom0 ) / denom2;
    ptr[1]=( P10(0,1)*denom - P10(2,1)*nom0 ) / denom2;
    ptr[2]=( P10(0,2)*denom - P10(2,2)*nom0 ) / denom2;
    ptr[3]=( P10(1,0)*denom - P10(2,0)*nom1 ) / denom2;
    ptr[4]=( P10(1,1)*denom - P10(2,1)*nom1 ) / denom2;
    ptr[5]=( P10(1,2)*denom - P10(2,2)*nom1 ) / denom2;

}

// Ok here comes the thing... parallize the crap out
// This could be simplified (without COS trafo)
void GradCalcStereo::calcHelpers(Orientation &ori0, Orientation &ori1, cv::Mat &xyzimg0, cv::Mat& tid0 )
{
    	MyMesh::Normal n;
	Eigen::Vector3f di;

	// Compute relative P
	Orientation ori10;
	OriGeom::computeRelative( ori0, ori1, ori10 );
	Eigen::MatrixXd P10=ori10.getP();
	Eigen::MatrixXd P1=ori1.getP();

	// Geometry stuff
	Eigen::Vector3d C0=ori0.getC();
	Eigen::Matrix3d R0=ori0.getR();
	Eigen::Vector3d t0=(-1.0)*R0*C0;
	Eigen::Vector3d C1=ori1.getC();
	Eigen::Matrix3d R1=ori1.getR();
	Eigen::Vector3d t1=(-1.0)*R1*C1;

	// Reset vertvalidity
	for(int v=0; v<_numverts;v++){_vertsvalid[v]=false;}
	for(int f=0; f<_numfaces;f++){_facesvalid[f]=false;}

    	for(int y=0; y<ori0.rows(); y++ )
	{
    		for(int x=0; x<ori0.cols(); x++ )
		{
			if(tid0.at<int>(y,x)>=0) // valid ray
			{
				// Compute di (vector in C->surface pt in cam frame)
                        	di(0)=xyzimg0.at<cv::Vec3f>(y,x)[0]-C0(0);
                        	di(1)=xyzimg0.at<cv::Vec3f>(y,x)[1]-C0(1);
                        	di(2)=xyzimg0.at<cv::Vec3f>(y,x)[2]-C0(2);
				di=R0.cast<float>()*di;
				_dix(y,x)=di(0);
				_diy(y,x)=di(1);
				_diz(y,x)=di(2);

                        	// Compute N (normal at surface point in cam frame)
				n=_mesh->normal(*(_mesh->faces_begin()+tid0.at<int>(y,x)));
				_Nx(y,x)=R0(0,0)*n[0]+R0(0,1)*n[1]+R0(0,2)*n[2];
				_Ny(y,x)=R0(1,0)*n[0]+R0(1,1)*n[1]+R0(1,2)*n[2];
				_Nz(y,x)=R0(2,0)*n[0]+R0(2,1)*n[1]+R0(2,2)*n[2];

				// Compute jacP (Jacobian of relative orientation)
				jacobianP( di(0), di(1), di(2), x, y, P10 ); // own function to get jacobian of P1
				_facesvalid[tid0.at<int>(y,x)]=true;

				for (MyMesh::FaceVertexIter fv_it=_mesh->fv_iter(*(_mesh->faces_begin()+tid0.at<int>(y,x))); fv_it.is_valid(); ++fv_it)
				{
				    _vertsvalid[fv_it->idx()]=true;
				}
			}
		}
	}
}

void GradCalcStereo::jacobianImage(cv::Mat &img1, LikelihoodImage* limage1)
{

    	int scale = 1;
	int delta = 0;

	// Grayscale
	cv::Sobel(img1, _jacimg1x, CV_32F, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
	cv::Sobel(img1, _jacimg1y, CV_32F, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);

	double minx, maxx, miny, maxy;
	cv::minMaxLoc(_jacimg1x,&minx,&maxx);
	cv::minMaxLoc(_jacimg1y,&miny,&maxy);
	minx=std::min(minx,miny);
	maxx=std::max(maxx,maxy);

	_jacimg1x/=maxx;
	_jacimg1y/=maxx;

	// Semantics
	if(_smode==ON){

		cv::Mat cvin(limage1->rows(),limage1->cols(), CV_32F);
		cv::Mat cvout(limage1->rows(),limage1->cols(), CV_32F);

		for(int l=0; l<_numlabels; l++)
		{
		    	limage1->getLayer(l, cvin);
			cv::Sobel(cvin, cvout, CV_32F, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
			_jaclabels1x->setLayer(l,cvout);

			cv::Sobel(cvin, cvout, CV_32F, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
			_jaclabels1y->setLayer(l,cvout);

		}
	}
}

// Build a Canny image of the base image for weighting contributions
void GradCalcStereo::buildCannyImages(const int i)
{
 	if(i==0)cv::Canny(_img0,_canny0,40.0,41.0,3,false);
	if(i==1)cv::Canny(_img1,_canny1,40.0,41.0,3,false);

	//cv::dilate(_canny0,_canny0,cv::Mat());
	//cv::dilate(_canny1,_canny1,cv::Mat());
	// Smooth the stuff?
	
//	cv::imwrite("/home/mathias/AlmostTrash/canny1.png", _canny0);
//	cv::imwrite("/home/mathias/AlmostTrash/canny2.png", _canny1);
//	exit(1);
}

void GradCalcStereo::buildWeightVariance(cv::Mat& varimg0,cv::Mat& varimg10, cv::Mat& valimg0)
{
	float minvar;
	float EPS=0.001;
	_weightimg.setTo(0.0);
	float* var0ptr=varimg0.ptr<float>(0);
	float* var1ptr=varimg10.ptr<float>(0);
	float* weightptr=_weightimg.ptr<float>(0);
	char* valptr=valimg0.ptr<char>(0);

	for(int y=0; y<_weightimg.rows; y++)
	{
		for(int x=0; x<_weightimg.cols; x++)
		{
			if(*valptr)
			{
				minvar=std::min(*var1ptr,*var1ptr);
				*weightptr=minvar/(minvar+EPS);
			}
			valptr++;
			var1ptr++;
			var0ptr++;
			weightptr++;
		}
	}
}


// Build the weigting images
void GradCalcStereo::buildWeightCanny(void)
{
    	unsigned char* canny0ptr=_canny0.ptr<unsigned char>(0);
    	unsigned char* canny10ptr=_canny10.ptr<unsigned char>(0);
    	float* weightptr=_weightimg.ptr<float>(0);

    	for(int y=0; y<_canny0.rows; y++)
	{
    		for(int x=0; x<_canny0.cols; x++)
		{
	    		*weightptr=std::max(*canny0ptr,*canny10ptr);
			weightptr++;
			canny0ptr++;
			canny10ptr++;
		}
	}

	// Scale to one
	float* ptr=_weightimg.ptr<float>(0);
	for(int y=0; y<_rows0; y++)
	{
		for(int x=0; x<_cols0; x++)
		{
			if(*ptr>0){ *ptr=1.0; }
			else{ *ptr=0.2; }
			ptr++;
		}
	}

	//ImgIO::saveVisFloat(_weightimg, "/home/mathias/AlmostTrash/weightimg.tif");
	//exit(1);
}

void GradCalcStereo::buildWeightImages(cv::Mat& varimg0,cv::Mat& varimg10, cv::Mat& valimg0)
{
	bool usecanny=true;
	if(usecanny)
	{
		buildWeightCanny();
	}
	else
	{
		buildWeightVariance(varimg0,varimg10,valimg0);
	}
}


double GradCalcStereo::computeTriangleSizeInPix()
{
	MyMesh::FaceIter f_it;
	MyMesh::FaceVertexIter fv_it;
	Eigen::MatrixXd P=_ori0->getP();
	Eigen::Vector4d X;
	Eigen::Vector3d x;
	float pt2d[3][2];
	MyMesh::Point pt;

	// Get the triangles
	int f=0,i;
	int count=0;
	double area=0.0;
	for(f_it=_mesh->faces_begin(); f_it!=_mesh->faces_end(); f_it++)
	{
		if(_facesvalid[f])
		{
			// Get the face
			i=0;
       			for (fv_it=_mesh->fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			{
				pt=_mesh->point(*fv_it);
			 	X<<pt[0],pt[1],pt[2],1.0;
				x=P*X;
				x/=x(2);
				pt2d[i][0]=x(0);
				pt2d[i++][1]=x(1);
			}
			area+=MeshGeom::faceArea2D(pt2d[0],pt2d[1],pt2d[2]);
			count++;
		}
		f++;
	}
	return area/(double)count;
}

// Parallel on higher level
int GradCalcStereo::integrateTriangle
	(
	const SEMANTICMODE smode,
	const int faceidx,
	const int vertindex,
	const int idvertinface,
	const float* tx,
	const float* ty,
	cv::Mat& tid0,
	cv::Mat& bary
	)
{

	float u,v;
	float baryweight, sim;
	Eigen::MatrixXf jacimg1(1,2);
	Eigen::MatrixXf jacp1(2,3);
	Eigen::Vector3f n;
	Eigen::Vector3f di;
	float nom, denom, weight;

	// Derive Boundingbox
	float minxf=std::min(tx[0],std::min(tx[1],tx[2]));
	float maxxf=std::max(tx[0],std::max(tx[1],tx[2]));
	float minyf=std::min(ty[0],std::min(ty[1],ty[2]));
	float maxyf=std::max(ty[0],std::max(ty[1],ty[2]));

	// make sure that in image
	int minx=std::max(0,(int)floor(minxf));
	int maxx=std::min(_cols0-1,(int)ceil(maxxf));
	int miny=std::max(0,(int)floor(minyf));
	int maxy=std::min(_rows0-1,(int)ceil(maxyf));

	int passedpixels=0; 
	// Skim snippet
	for(int y=miny; y<=maxy; y++ )
	{
		for(int x=minx; x<=maxx; x++)
		{
			if(tid0.at<int>(y,x)==faceidx)
			{
				u=bary.at<cv::Vec2f>(y,x)[0];
				v=bary.at<cv::Vec2f>(y,x)[1];

				// barycentric coordinate dependent on idvertinfaceid
				if (idvertinface==0) { baryweight=1.0-u-v; }
				else if (idvertinface==1) { baryweight=u; }
				else if (idvertinface==2) { baryweight=v; }
				sim=_simimg.at<float>(y,x);

                                // jacobian img1 (1x2)
				jacimg1(0)=_jacimg10x.at<float>(y,x);
				jacimg1(1)=_jacimg10y.at<float>(y,x);

				// jacobian P_j (2x3)
				float *ptr=_pjac[y][x];
                                jacp1<<ptr[0],ptr[1],ptr[2],ptr[3],ptr[4],ptr[5];

                                // di (3x1)
                                di << _dix(y,x),_diy(y,x),_diz(y,x);

				// normal (3x1)
				n << _Nx(y,x),_Ny(y,x),_Nz(y,x);

                                nom=baryweight*sim*(jacimg1*jacp1*di)(0,0);
				denom=n.dot(di);
				weight=nom/denom*_weightimg.at<float>(y,x);

				if( weight!=weight ) {weight=0.0;}// std::cout<<"\nhere";}
				// Update photo gradient
				_gradphoto(vertindex,0)+=weight*n(0);
				_gradphoto(vertindex,1)+=weight*n(1);
				_gradphoto(vertindex,2)+=weight*n(2);

				if(_smode==ON)
				{
					    for(int l=0; l<_numlabels; l++)
					    {
						jacimg1(0)=_jaclabels10x->getData(y,x,l);
						jacimg1(1)=_jaclabels10y->getData(y,x,l);
                                		nom=baryweight*sim*(jacimg1*jacp1*di)(0,0);
                                		denom=n.dot(di);
						weight=nom/denom;
						if( weight!=weight) {weight=0.0;}

						_gradsemantic(vertindex,0)+=weight*n(0);
						_gradsemantic(vertindex,1)+=weight*n(1);
						_gradsemantic(vertindex,2)+=weight*n(2);
					}
				}
			}

		}
	}
	return passedpixels;
}

// IMAGE REPROJECTION
// is this pairwise? meaning is the master image traced x times for each of the x slave images? -> no changed this
double GradCalcStereo::process()
{

	int x,y;
	double energy;
	int cols0=_ori0->cols();
	int rows0=_ori0->rows();
	SEMANTICMODE smode=OFF;

   	 // Measure time
   	 PRSTimer timer; timer.start();

    	// Update normals
	_mesh->request_face_normals();
	_mesh->update_normals();

    	// Check this: computes gradient images maybe use Scharr?
    	jacobianImage(_img1, _limage1);

	cv::Mat tid0,tid1,xyz0,xyz1,uvimg0,uvimg1; 
	timer.stop(); timer.reset(); timer.start();
    	// Begin FIRST raytracer
#pragma omp parallel
{
#pragma omp sections
{
	#pragma omp section
	{
    	// Deep copy output
   	tid0=_rt0->getIdImage().clone();
   	xyz0=_rt0->getXYZImage().clone();
	uvimg0=_rt0->getUVImage().clone();
	//std::string name("/home/mathias/AlmostTrash/testuv.tif");
	//ImgIO::saveVis32U(uvimg0,name);
	}
	#pragma omp section
	{
    	// Deep copy output
    	tid1=_rt1->getIdImage().clone();
    	xyz1=_rt1->getXYZImage().clone();
	uvimg1=_rt1->getUVImage().clone();
	}
}
}
	timer.stop();
	if(_verboselevel>=2){ std::cout<<"\n Raytracing stuff ["<<timer.getTimeSec()<<" sec]"; }
	// Update the masks and get undistortion maps ... (ocv GPU implemetation available)
	cv::Mat map0x(rows0,cols0,CV_32F);
	cv::Mat map0y(rows0,cols0,CV_32F);
	updateTIDImagesOcclusions(tid0, tid1, xyz0, xyz1, *_ori0, *_ori1, map0x, map0y);

	// Now REMAP the images all input images
	remapAll(smode, map0x, map0y);

	timer.reset(); timer.start();
	// Compute more global helpers
	calcHelpers(*_ori0, *_ori1, xyz0, tid0);
	timer.stop();
	if(_verboselevel>=2){ std::cout<<"\n calcHelpers done ["<<timer.getTimeSec()<<" sec]"; }

	//ImgIO::saveVisLikeli(*_limage1,2,"/home/mathias/AlmostTrash/test.png"); exit(1);
	// Compute the image similarity and the constrast sensitive weights
	timer.reset(); timer.start();
	energy=calcSimilarity(smode, tid0);
	timer.stop();
	if(_verboselevel>=2){ std::cout<<"\n calcSim done ["<<timer.getTimeSec()<<" sec]"; }
	
	//cv::imwrite("/home/mathias/AlmostTrash/test.png", _jacimg10x);

	// Now we have to update the gradients
	timer.reset(); timer.start();
	updateGrad(uvimg0,tid0,*_ori0);
	timer.stop();

	if(_verboselevel>=2){ std::cout<<"\n update grad done ["<<timer.getTimeSec()<<" sec]"; }
	return energy;
}

