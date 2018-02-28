// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#pragma once
#include <vector>

#include <embree2/rtcore.h>
#include <embree2/rtcore_ray.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "../Ori/Orientation.h"
#include "../MeshUtil/MyMesh.h"

class RayTracer
{
    public:

	 RayTracer(MyMesh &mesh);
	 RayTracer(std::vector< std::vector<float> > &verts, std::vector< std::vector<int> > &faces);
   	 ~RayTracer();

	 cv::Mat& getIdImage();
	 cv::Mat& getXYZImage();
	 cv::Mat& getUVImage();

	 void traceRaysColumnWise();

	 void setView(  const float tfar, const float tnear,
		 	const int rows, const int cols,
		 	const Eigen::Matrix3d &R,
		 	const Eigen::Matrix3d &K,
			const Eigen::Vector3d &C,
			const bool neg=false);
	 void setView(  const float tfar, const float tnear, const Orientation &ori, const bool neg=false);

	 float getPercentage();

	 int check( std::vector< std::vector<float> > &verts );

    private:

	 void init();
	 void setScene( std::vector< std::vector<float> > &verts, std::vector< std::vector<int> > &faces);
	 void setScene( MyMesh& mesh);

	 struct vertex {float x, y, z, a;}; // the data is aligned to 16 bytes (for parallelization)
   	 struct face {int v0, v1, v2;};

	 Eigen::Vector3d _pt;
	 RTCDevice _device;
	 RTCScene _scene;
	 unsigned int _geomID;

	 bool _neg;
	 int _cols;
	 int _rows;
	 Eigen::Matrix3d _R;
	 Eigen::Matrix3d _Rt;
	 Eigen::Matrix3d _K;
	 Eigen::Matrix3d _Kinv;
	 Eigen::Vector3d _C;

	float _tfar;
	float _tnear;

	 cv::Mat _idimg;
	 cv::Mat _xyzimg;
	 cv::Mat _uvimg;
};

