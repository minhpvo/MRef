// Copyright ETHZ 2017
// author: maros, 2016, maros.blaha@geod.baug.ethz.ch
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch

#pragma once
#include "opencv2/core/core.hpp"
#include <string>
#include <Eigen/Dense>
#include "../MeshUtil/MyMesh.h"
#include "../Ori/Orientation.h"
#include "../LikelihoodImage/LikelihoodImage.h"
#include "../RayTracer/RayTracer.h"

class GradCalcStereo
{

public:
	enum SEMANTICMODE{ON,OFF};

	GradCalcStereo(	cv::Mat &img0,
			LikelihoodImage* limage0,
			Orientation* ori0,
			MyMesh* mesh,
			SEMANTICMODE smode,
			const double tnear,
			const double tfar,
			const int verboselevel
			);

	~GradCalcStereo();

	// Setter for first view
	void setSecondView(	cv::Mat &img1,
				LikelihoodImage* limage1,
				Orientation* ori1 );

	// Main processsing function
	double process();

	Eigen::MatrixXf& getSemanticGrad();
	Eigen::MatrixXf& getPhotoGrad();
	double getMeanDist(void) const;

	double computeTriangleSizeInPix();

private:

	void buildWeightCanny(void);

	void buildWeightVariance(cv::Mat& varimg0,cv::Mat& varimg10, cv::Mat& valimg0);
	
	void buildWeightImages(cv::Mat& varimg0,cv::Mat& varimg10, cv::Mat& valimg0);

	void buildCannyImages(const int i);

	void updateGrad(cv::Mat& bary, cv::Mat& tid0, Orientation& ori0);

	void updateTIDImagesOcclusions(	cv::Mat &tid0, cv::Mat &tid1,
					cv::Mat &XYZ0, cv::Mat XYZ1,
					const Orientation &ori0, const Orientation &ori1,
					cv::Mat& map1, cv::Mat& map2 );

	void remapAll(const SEMANTICMODE mode, const cv::Mat &mapx, const cv::Mat &mapy);

	double calcSimilarity(const SEMANTICMODE mode, cv::Mat& tid0 );

	void jacobianP(	const float X, const float Y, const float Z,
			const int x,const  int y, const Eigen::MatrixXd &P10);

	void calcHelpers(Orientation &ori0, Orientation &ori1, cv::Mat& xyzimg0, cv::Mat& tid0);

	void jacobianImage(cv::Mat& img1, LikelihoodImage* limage1);

	int integrateTriangle(	const SEMANTICMODE smode,
				const int faceidx,
				const int vertindex,
				const int idvertinface,
				const float* tx,
				const float* ty,
				cv::Mat& tid0,
				cv::Mat& bary
				);

	int _cols0;
	int _rows0;
    	int _cols1;
	int _rows1;
	int _numverts;
	int _numfaces;
	int _numlabels;

	double _meandist;

	RayTracer* _rt0;
	RayTracer* _rt1;

	double _tnear;
	double _tfar;

	MyMesh* _mesh;

	// Switch on/off semantics
	SEMANTICMODE _smode;

	// Gradient of intensity image
    	Eigen::MatrixXf _gradphoto;

	// Gradient of semantic segmentations
	Eigen::MatrixXf _gradsemantic;

	// Grayscale images
	cv::Mat _img0, _img1;
	cv::Mat _img10;

	// Canny images
	cv::Mat _canny0,_canny1;
	cv::Mat _canny10;

	// Weight image for balancing data / smoothness
	cv::Mat _weightimg;

	// Label images
	LikelihoodImage* _limage0;
	LikelihoodImage* _limage1;
	LikelihoodImage* _limage10;

	// Orientations
	Orientation* _ori0;
	Orientation* _ori1;

	bool* _vertsvalid;
	bool* _facesvalid;

	// Simiarity images (float)
	cv::Mat _simimg;
	LikelihoodImage* _simlabels;

	// Jacobians of image 1
	cv::Mat _jacimg1x;
	cv::Mat _jacimg1y;
	LikelihoodImage* _jaclabels1x;
	LikelihoodImage* _jaclabels1y;

	// Jacobians 10 reprojected
	cv::Mat _jacimg10x;
	cv::Mat _jacimg10y;

	LikelihoodImage* _jaclabels10x;
	LikelihoodImage* _jaclabels10y;

	// Single elements of gradient
	Eigen::MatrixXf _dix;
	Eigen::MatrixXf _diy;
	Eigen::MatrixXf _diz;

	Eigen::MatrixXf _Nx;
	Eigen::MatrixXf _Ny;
	Eigen::MatrixXf _Nz;

	float*** _pjac;

	double _lastenergy;

	int _verboselevel;
};

