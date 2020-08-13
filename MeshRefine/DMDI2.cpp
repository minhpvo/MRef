// Copyright ETHZ 2017
#include "DMDI2.h"
#include "../Convolution/Convolution.h"
#include "../PRSTimer/PRSTimer.h"
//#include "../ImgUtil/ImgIO.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <iostream>


Convolution* DMDI2::_convolution;
int DMDI2::_kernelsize;
cv::Mat DMDI2::_dummyimg;
float DMDI2::_BETA;

DMDI2::DMDI2() 
{
	// Init the Convolution module
	_convolution=new Convolution();
}
DMDI2::~DMDI2()
{
   	 delete _convolution;
}

void DMDI2::calcMeanImg(cv::Mat &inimg, cv::Mat &valimage, cv::Mat &meanimg)
{
	(*_convolution)( inimg, valimage, meanimg, _kernelsize);
}

void DMDI2::calcVarianceImg(cv::Mat &inimg, cv::Mat &valimage, cv::Mat &meanimg, cv::Mat &varimg)
{
   	cv::multiply(inimg,inimg,_dummyimg);
	(*_convolution)( _dummyimg, valimage, varimg, _kernelsize);
	cv::multiply(meanimg,meanimg,_dummyimg);
	varimg-=_dummyimg;
	varimg+=cv::Scalar(_BETA*_BETA);
}

void DMDI2::calcCoVarianceImg(cv::Mat &inimg1, cv::Mat &inimg2, cv::Mat &valimage, cv::Mat &meanimg1, cv::Mat &meanimg2, cv::Mat &covarimg)
{
   	cv::multiply(inimg1,inimg2,_dummyimg);
    	(*_convolution)( _dummyimg, valimage, covarimg, _kernelsize);
	cv::multiply(meanimg1,meanimg2,_dummyimg);
	covarimg-=_dummyimg;
	//covarimg+=cv::Scalar(_BETA*_BETA);
}

void DMDI2::calcCrossCorImg(cv::Mat &varimg1, cv::Mat &varimg2, cv::Mat &valimage, cv::Mat &covarimg, cv::Mat &crosscorimg, cv::Mat &sqrtv1v2img)
{
   	cv::multiply(varimg1,varimg2,_dummyimg);
	cv::sqrt(_dummyimg, sqrtv1v2img);
	cv::divide(covarimg,sqrtv1v2img,crosscorimg);
}

void DMDI2::calcAlpha(cv::Mat &sqrtv1v2img, cv::Mat imgval, cv::Mat &alphaimg)
{
    _dummyimg=-1.0/sqrtv1v2img;
    (*_convolution)( _dummyimg, imgval, alphaimg, _kernelsize);
}


void DMDI2::calcBeta(cv::Mat &crosscorimg, cv::Mat &varimg2, cv::Mat &imgval, cv::Mat &betaimg)
{
    cv::divide(crosscorimg,varimg2,_dummyimg);
    (*_convolution)( _dummyimg, imgval, betaimg, _kernelsize);
}

void DMDI2::calcGamma(cv::Mat &meanimg1, cv::Mat &meanimg2, cv::Mat &varimg2, cv::Mat &crosscorimg, cv::Mat &sqrtv1v2img, cv::Mat &imgval, cv::Mat &gammaimg)
{
    cv::divide(meanimg1,sqrtv1v2img,_dummyimg);
    cv::multiply(meanimg2,crosscorimg,gammaimg);
    cv::divide(gammaimg,varimg2,gammaimg);
    cv::subtract(_dummyimg, gammaimg,_dummyimg);
    (*_convolution)( _dummyimg, imgval, gammaimg, _kernelsize);
}


int DMDI2::getByteDepth(cv::Mat& img1, cv::Mat& img2)
{
 	int bytedepth=-1;
	if( (img1.type()==CV_8UC1 || img1.type()==CV_8SC1) && (img2.type()==CV_8UC1 || img2.type()==CV_8SC1) ){ bytedepth=1;}
	else if( (img1.type()==CV_16UC1 || img1.type()==CV_16SC1) && (img2.type()==CV_16UC1 || img2.type()==CV_16SC1) ){ bytedepth=2;}
	else if( img1.type()==CV_32FC1 && img2.type()==CV_32FC1) { bytedepth=4;}
	else {std::cout<<"\nDMDI2: Inbut image is not u8,u16, f32 type is:"<< img1.type()<< "/ " << img2.type(); exit(1); }
    //std::cout<<"\n detected bytes "<<bytedepth;

	if(bytedepth==4){
		double min, max, range;
		cv::minMaxLoc(img1, &min, &max);
		if(min<-0.00001 || max>1.0){ std::cout<<"\nDMDI2: Detected floating point input images outside [0 1]->" << min << " "<< max; exit(1); }
		cv::minMaxLoc(img2, &min, &max);
	//	ImgIO::saveVisFloat(img2,"/home/mathias/AlmostTrash/test.png");
		if(min<-0.00001 || max>1.0){ std::cout<<"\nDMDI2: Detected floating point input images outside [0 1]->" <<min<<" "<<max; exit(1); }
	}
	return bytedepth;
}

void DMDI2::scaleImgsToOneFixedRange(cv::Mat& fimg1, cv::Mat& fimg2, int bytedepth)
{
    	double min, max, range;
 	float* runptr;
	
	if(bytedepth==1)
	{
		min=0; max=255; range=255;
	}
	else if( bytedepth==2)
	{
		min=0; max=65535; range=65535;
	}
	else if( bytedepth=4 ) // This is case floating point and assumed to be in 0 1
	{
		min=0.0; max=1.0; range=1.0;
	} 

	// rescale
	runptr=fimg1.ptr<float>(0);
	for(unsigned int i=0; i<fimg1.cols*fimg1.rows; i++)
	{
	    	*runptr=(*runptr-min)/range;
		runptr++;

	}
	runptr=fimg2.ptr<float>(0);
	for(unsigned int i=0; i<fimg2.cols*fimg2.rows; i++)
	{
	    	*runptr=(*runptr-min)/range;
		runptr++;
	}
}

void DMDI2::scaleImgsToOne(cv::Mat& fimg1, cv::Mat& fimg2)
{
    	double min, max, range;
	float* runptr=NULL;

	cv::minMaxLoc(fimg1, &min, &max);
	range=max-min;
	runptr=fimg1.ptr<float>(0);
	for(unsigned int i=0; i<fimg1.cols*fimg1.rows; i++)
	{
	    	*runptr=(*runptr-min)/range;
		runptr++;
	}

	cv::minMaxLoc(fimg2, &min, &max);
	range=max-min;
	runptr=fimg2.ptr<float>(0);
	for(unsigned int i=0; i<fimg2.cols*fimg2.rows; i++)
	{
	    	*runptr=(*runptr-min)/range;
		runptr++;
	}
}

double DMDI2::calcEnergy(cv::Mat& valimg, cv::Mat& ncimg)
{
    	double energy=0.0;
	int count=0;
	for(int y=0; y<valimg.rows; y++)
	{
		for(int x=0; x<valimg.cols; x++)
		{
			if(valimg.at<unsigned char>(y,x)>0)
			{
				double nc_val = std::abs(ncimg.at<float>(y, x));
				if (std::isnan(nc_val)){
					nc_val = 0;
				}
				energy -= nc_val;
				count++;
			}
		}
	}
	return count>0 ? energy/(double)count : 0.0;
}

void DMDI2::compute(	cv::Mat &img1, cv::Mat &img2, cv::Mat &valimg, int kernelsize, MATCHINGCOST matchingcost,
			cv::Mat &dmdi2img, cv::Mat& varimg1, cv::Mat& varimg2, double& energy )
{
	PRSTimer timer;
	PRSTimer timerdmdi2;
	_kernelsize=kernelsize;
	_BETA=0.0;
	timerdmdi2.start();

	// Get depth
	int depth=getByteDepth(img1, img2);

        // Reference image-wise allocation could be avoi2ded by membering the images...
        cv::Mat _imgf1,_imgf2,meanimg1,meanimg2,covarimg,crosscorimg,sqrtv1v2img,alphaimg,betaimg,gammaimg;

    	// convert input images to floats ... subsequent calculations all floating point
    	img1.convertTo(_imgf1,CV_32FC1);
    	img2.convertTo(_imgf2,CV_32FC1);

	// Scale the images to one
	//scaleImgsToOneFixedRange(_imgf1, _imgf2, depth);

	// MatchingCost 
	if(matchingcost==SSD)
	{
		dmdi2img=img2-img1;
	}
	else if(matchingcost==NCC)
	{
   		_dummyimg.create(img1.rows, img2.cols, CV_32FC1);
    		// Calc mean img1 img2
		timer.start();
		calcMeanImg(_imgf1,valimg,meanimg1);
		calcMeanImg(_imgf2,valimg,meanimg2);
        	timer.stop();
        	//std::cout<<"\nMean images done [" << timer.getTimeSec() <<" sec]";
        	timer.reset();

    		// Calc variance img1 img2
		timer.start();
    		calcVarianceImg(_imgf1, valimg, meanimg1, varimg1);
   		calcVarianceImg(_imgf2, valimg, meanimg2, varimg2);
        	timer.stop();
        	//std::cout<<"\nVariance images done [" << timer.getTimeSec() <<" sec]";
        	timer.reset();

		// Calc co-variance
		timer. start();
		calcCoVarianceImg(_imgf1, _imgf2, valimg, meanimg1, meanimg2, covarimg);
        	timer.stop();
        	//std::cout<<"\nCovariance image done [" << timer.getTimeSec() <<" sec]";
        	timer.reset();

    		// Calc crosscorrelation
		timer.start();
		calcCrossCorImg(varimg1, varimg2, valimg, covarimg, crosscorimg, sqrtv1v2img);
        	timer.stop();
        	//std::cout<<"\nCross correlation image done [" << timer.getTimeSec() <<" sec]";
        	timer.reset();

		// Calc alpah
		timer.start();
		calcAlpha(sqrtv1v2img, valimg, alphaimg);
        	timer.stop();
        	//std::cout<<"\nAlpha image done [" << timer.getTimeSec() <<" sec]";
        	timer.reset();

		// Calc beta
		timer.start();
		calcBeta(crosscorimg, varimg2, valimg, betaimg);
        	timer.stop();
        	//std::cout<<"\nBeta image done [" << timer.getTimeSec() <<" sec]";
        	timer.reset();

		// Calc gamma
		timer.start();
		calcGamma(meanimg1,meanimg2, varimg2,crosscorimg, sqrtv1v2img, valimg, gammaimg);
        	timer.stop();
        	//std::cout<<"\nGamma image done [" << timer.getTimeSec() <<" sec]";
        	timer.reset();

		// Finally calc DMDI2
		cv::multiply(alphaimg,_imgf1,dmdi2img);
		cv::multiply(betaimg,_imgf2,_dummyimg);
		cv::add(dmdi2img,_dummyimg,dmdi2img);
		cv::add(gammaimg,dmdi2img,dmdi2img);

		_dummyimg.release(); 
		//dmdi2img=varimg1.clone();
		energy=calcEnergy(valimg,crosscorimg);
	}
	else
	{
		std::cout<<"\nDMDI2: Could not find matching cost...";
	}
    timerdmdi2.stop();
    //std::cout<<"\nDMDI2 image done [" << timerdmdi2.getTimeSec() <<" sec]";

}

