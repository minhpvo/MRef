// Copyright ETHZ 2017
#pragma once
#include <opencv2/imgcodecs/imgcodecs.hpp>

class Convolution
{

private:

 	void completeImg(cv::Mat &inimg, cv::Mat &valimg, const int ksize);
 	template <typename T> void completeAll(cv::Mat &inimg, cv::Mat &valimg,const int kernelsize);
	template <typename T> void invalidate(cv::Mat &imgin, cv::Mat &valimg);
	void makeProcessImage(cv::Mat &valimg, const int ksize);
	int _ksize;

public:

	Convolution();
	~Convolution();
	void operator()(cv::Mat &imgin, cv::Mat &imgval, cv::Mat &imgout, const int kernelsize);
};




