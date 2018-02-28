// Copyright ETHZ 2017
// (c) mathias rothermel, 2016, mathias.rothermel@geod.baug.ethz.ch
 
#pragma once

#include "../Convolution/Convolution.h" 
#include <opencv2/core/core.hpp>

class DMDI2{

public:

	enum MATCHINGCOST{ NCC, SSD };
   	DMDI2();
   	~DMDI2();

	// This is main function to compute DM/DI2
	static void compute(cv::Mat &img1, cv::Mat &img2, cv::Mat &valimg, int kernelsize, MATCHINGCOST matchingcost, cv::Mat &dmdi2img, cv::Mat& varimg1, cv::Mat& varimg2i, double& energy);
	// @param in: img 1
	static Convolution* _convolution;
	static int _kernelsize;
	static cv::Mat _dummyimg;
	static float _BETA;

private:

	static double calcEnergy(cv::Mat& valimg, cv::Mat& ncimg);
	static void scaleImgsToOne(cv::Mat& fimg1, cv::Mat& fimg2);
	static void calcMeanImg(cv::Mat &inimg, cv::Mat &valimage, cv::Mat &meanimg);
	static void calcVarianceImg(cv::Mat &inimg, cv::Mat &valimage, cv::Mat &meanimg, cv::Mat &varimg);
	static void calcCoVarianceImg(cv::Mat &inimg1, cv::Mat &inimg2, cv::Mat &valimage, cv::Mat &meanimg1, cv::Mat &meanimg2, cv::Mat &covarimg);
	static void calcCrossCorImg(cv::Mat &varimg1, cv::Mat &varimg2, cv::Mat &valimage, cv::Mat &covarimg1, cv::Mat &crosscorimg, cv::Mat &sqrtv1v2img);
	static void calcAlpha(cv::Mat &sqrtv1v2img, cv::Mat imgval, cv::Mat &alphaimg);
	static void calcBeta(cv::Mat &crosscorimg, cv::Mat &varimg2, cv::Mat &imgval, cv::Mat &betaimg);
	static void calcGamma(cv::Mat &meanimg1, cv::Mat &meanimg2, cv::Mat &varimg2, cv::Mat &crosscorimg, cv::Mat &sqrtv1v2img, cv::Mat &imgval, cv::Mat &gammaimg);
	static int getByteDepth(cv::Mat& img1, cv::Mat& img2);
	static void scaleImgsToOneFixedRange(cv::Mat& fimg1, cv::Mat& fimg2, int bytedepth);

};
