// Copyright ETHZ 2017
#include "Convolution.h"
#include "../PRSTimer/PRSTimer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <omp.h>
#include <iostream>

#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <string>
Convolution::Convolution()
{
}

Convolution::~Convolution()
{
}

void Convolution::makeProcessImage(cv::Mat &valimg, const int ksize)
{
    	cv::Mat dst;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*ksize + 1, 2*ksize+1 ), cv::Point( ksize, ksize ) );

    	cv::erode( valimg, dst, element );
	unsigned char* valptr=valimg.ptr(0);
	unsigned char* dstptr=dst.ptr(0);

	for(unsigned int i=0; i<dst.cols*dst.rows;i++)
	{
	    if(*valptr && !*dstptr){ *valptr=127; }
	    valptr++;
	    dstptr++;
	}
}

template<typename T>
void Convolution::completeAll(cv::Mat &inimg, cv::Mat &valimg, const int ksize)
{

	int i,y,x,k;
	int cols=inimg.cols;
	int rows=inimg.rows;
	T lastval_l,lastval_r,lastval_u;


	T* imgptr_l; T* imgptr_c; T* imgptr_r; T* imgptr_u;
	char* valptr_c; char* valptr_l; char* valptr_r; char* valptr_u;

	int kmc=cols*ksize;

	// Complete Horizontal
	#pragma omp parallel for num_threads(6) private (i,y,x,k,lastval_l,lastval_r,lastval_u, imgptr_c,imgptr_l,imgptr_r,imgptr_u, valptr_l,valptr_c,valptr_r,valptr_u) firstprivate(cols, rows, ksize,kmc)
	for(y=0;y<rows;y++)
	{
		valptr_c=(char*) valimg.ptr(0)+y*cols+ksize;
		imgptr_c=(T*) inimg.ptr(0)+y*cols+ksize;


		for(x=ksize;x<cols-ksize;x++)
		{
			if(*valptr_c!=1) {imgptr_c++; valptr_c++; continue;} // Wont be computed
			else{
				imgptr_r=imgptr_c+1;
				imgptr_l=imgptr_c-1;
				valptr_l=valptr_c-1;
				valptr_r=valptr_c+1;
				lastval_r=*imgptr_c;
				lastval_l=*imgptr_c;

				for(k=0;k<ksize;k++)
				{
					if(!*valptr_l) { *imgptr_l=lastval_l; }
					else { lastval_l=*imgptr_l; }
					if(!*valptr_r) { *imgptr_r=lastval_r; }
					else { lastval_r=*imgptr_r; }
					valptr_r++; valptr_l--;
					imgptr_r++; imgptr_l--;
				}
			
				imgptr_c++;
				valptr_c++;
			}
		}
	}

	// Complete Vertical
	#pragma omp parallel for num_threads(6) private (i,y,x,k,lastval_l,lastval_r,lastval_u, imgptr_c,imgptr_l,imgptr_r,imgptr_u, valptr_l,valptr_c,valptr_r,valptr_u) firstprivate(cols, rows, ksize,kmc)
	for(x=0;x<cols;x++)
	{

		valptr_c=(char*) valimg.ptr(0)+x+kmc;
		imgptr_c=(T*) inimg.ptr(0)+x+kmc;

		for(y=ksize;y<rows-ksize;y++)
		{
			if(*valptr_c!=1) {imgptr_c+=cols; valptr_c+=cols; continue;} // Wont be computed
			else{
				imgptr_l=imgptr_c+cols;
				imgptr_u=imgptr_c-cols;
				valptr_l=valptr_c+cols;
				valptr_u=valptr_c-cols;
				lastval_l=*imgptr_c;
				lastval_u=*imgptr_c;
			
				for(k=0;k<ksize;k++)
				{
					if(!*valptr_l) { *imgptr_l=lastval_l;}
					else{ lastval_l=*imgptr_l; }
					if (!*valptr_u) { *imgptr_u=lastval_u;}
					else{ lastval_u=*imgptr_u; }
					valptr_l+=cols;valptr_u-=cols;
					imgptr_l+=cols;imgptr_u-=cols;
				}
				imgptr_c+=cols;
				valptr_c+=cols;
			}
		}
	}
}

void Convolution::completeImg(cv::Mat &inimg, cv::Mat &valimg, const int ksize)
{


	if(inimg.channels()==1 && inimg.depth()==0)
	{
		completeAll<char>(inimg,valimg,ksize);
	}
	else if(inimg.channels()==1 && inimg.depth()==1)
	{
		completeAll<short>(inimg,valimg,ksize);
	}
	else if(inimg.channels()==1 && inimg.depth()==CV_32F)
	{
	    	completeAll<float>(inimg,valimg,ksize);
	}
	else if(inimg.channels()==1 && inimg.depth()==CV_64F)
	{
	    	completeAll<double>(inimg,valimg,ksize);
	}
	else
	{
		std::cout << "\n Convolotion: only 8/16/32/64 bit single cannel implemented. Detected " << inimg.channels()<<" channels /" << inimg.depth()<<" depth.";
	}
}

template<typename T>
void Convolution::invalidate(cv::Mat &inimg, cv::Mat &valimg)
{
	unsigned char* valptr=(unsigned char*)valimg.ptr(0);
	T* imgptr=(T*)inimg.ptr(0);
	
	for(unsigned int i=0;i<inimg.cols*inimg.rows;i++)
	{
		if(*valptr==0){*imgptr=0;}
		if(*valptr==127){ *valptr=255; }
		//if(*valptr==255){ *imgptr=255; }
		valptr++;
		imgptr++;
	}
}

void Convolution::operator()(cv::Mat &imgin, cv::Mat &valimg, cv::Mat &imgout, const int kernelsize)
{

	imgout=imgin.clone();
	PRSTimer timer;

	timer.start();
	makeProcessImage(valimg,kernelsize);

	completeImg(imgout,valimg,kernelsize/2);
	timer.stop();
	//std::cout<<"\nPreparation took" << timer.getTimeSec()<<" sec";

	timer.reset();
	timer.start();
	cv::GaussianBlur( imgout, imgout, cv::Size( kernelsize, kernelsize ), 0, 0 );
	timer.stop();
	//std::cout<<"\nConvolution took" << timer.getTimeSec()<<" sec";

	if(imgin.channels()==1 && imgin.depth()==0) { invalidate<char>(imgout, valimg); }
	else if(imgin.channels()==1 && imgin.depth()==1) { invalidate<short>(imgout, valimg); }
	else if(imgin.channels()==1 && imgin.depth()==CV_32F) { invalidate<float>(imgout, valimg); }
	else if(imgin.channels()==1 && imgin.depth()==CV_64F) { invalidate<double>(imgout, valimg); }
	else
	{
	    std::cout<<"Convolution: Only 8/16/32/64 bit single cahnnels implemented."; exit(1);
	}

}



