// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
// author: maros, 2016, maros.blaha@geod.baug.ethz.ch


#include "RayTracer.h"
#include "../MeshUtil/MeshConv.h"
#include "../PRSTimer/PRSTimer.h"
#include <iostream>
#include <xmmintrin.h>
#include <pmmintrin.h>
#include <tbb/tbb.h>

RayTracer::RayTracer( MyMesh &mesh ) :
    _rows(-1),
    _cols(-1),
    _tfar(-1.0),
    _tnear(-1.0),
    _neg(false)
{
	//// Init the number of threads 
	tbb::task_scheduler_init tbbinit(16);
    	init();
    	setScene(mesh);
}

RayTracer::RayTracer( std::vector< std::vector<float> > &verts, std::vector< std::vector<int> > &faces ) :
    _rows(-1),
    _cols(-1),
    _tfar(-1.0),
    _tnear(-1.0),
    _neg(false)
{
    init();
    setScene(verts, faces);
}

cv::Mat& RayTracer::getIdImage()
{
    return _idimg;
}

cv::Mat& RayTracer::getXYZImage()
{
    return _xyzimg;
}

cv::Mat& RayTracer::getUVImage()
{
    return _uvimg;
}

void RayTracer::init()
{
    // start ray tracing withembree
    _device = rtcNewDevice(NULL);

    // recommended settings
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    // Set scene 
     _scene = rtcDeviceNewScene(_device, RTC_SCENE_STATIC, RTC_INTERSECT_STREAM);
}


RayTracer::~RayTracer()
{
    _idimg.release();
    _xyzimg.release();
    _uvimg.release();
    
    rtcDeleteGeometry; // delete geometry (triangle mesh)
    rtcDeleteScene(_scene); // delete scene (set of geometries)
    rtcDeleteDevice(_device); // delete embree device
}

void RayTracer::setView(  const float tfar, const float tnear, const Orientation &ori, const bool neg)
{
	_neg=neg;
	_rows=ori.rows();
	_cols=ori.cols();

	_idimg.create(_rows,_cols, CV_32S);
	_xyzimg.create(_rows,_cols,CV_32FC3);
	_uvimg.create(_rows,_cols,CV_32FC2);
	_idimg.setTo(0);
	_xyzimg.setTo(0.0);
	_uvimg.setTo(0.0);

	_R=ori.getR();
	_K=ori.getK();
	_C=ori.getC();
		
	_Kinv=_K.inverse();
	_Rt=_R.transpose();

	_tfar=tfar;
	_tnear=tnear;

}

void RayTracer::setView( const float tfar, const float tnear, const int rows, const int cols, const Eigen::Matrix3d &R, const Eigen::Matrix3d &K, const Eigen::Vector3d &C, const bool neg)
{
    	// Check if z-axis is pointing towards surface
    	Eigen::Vector3d z; z<<R(2,0), R(2,1), R(2,2);
	Eigen::Vector3d d; d<<_pt(0)-_C(0),_pt(1)-C(1), _pt(2)-C(2);
    	_neg= fabs(acos(d.dot(z)/d.norm()/z.norm()))>M_PI*0.5;
	_rows=rows;
	_cols=cols;

	_idimg.create(_rows,_cols, CV_32S);
	_xyzimg.create(_rows,_cols,CV_32FC3);
	_uvimg.create(_rows,_cols,CV_32FC2);
	_idimg.setTo(0);
	_xyzimg.setTo(0.0);
	_uvimg.setTo(0.0);

	_Kinv=K.inverse();
	_Rt=R.transpose();
	_R=R;
	_K=K;
	_C=C;

	_tfar=tfar;
	_tnear=tnear;
}

void RayTracer::setScene( MyMesh& mesh)
{
    std::vector<std::vector<float> > verts;
    std::vector<std::vector<int> > faces;
    std::vector<short> labels;

//    for (MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it){
// 	std::cout<<"\n"<<(unsigned short)mesh.data(*f_it).labelid();
//	}
	// Conversion
        MeshConv::openMeshToVectors( mesh, verts, faces, labels);

	// Store a surface point
	if(verts.size()>0){ _pt<< verts[0][0], verts[0][1], verts[0][2]; }
	//for(int i=0;i<labels.size();in++) {std::cout<<"\n"<<labels[i]; }
    	setScene(verts, faces);
}

int RayTracer::check(  std::vector< std::vector<float> > &verts )
{

	// Build Pmat 
	Eigen::MatrixXd P(3,4);
	Eigen::Matrix3d KR(3,3); KR=_K*_R;
	Eigen::Vector3d t(3,1);
	Eigen::Vector4d X(4,1);
	Eigen::Vector3d x(3,1);
	t=(-1.0)*_K*_R*_C;

	P << KR(0,0),KR(0,1),KR(0,2),t(0),KR(1,0),KR(1,1),KR(1,2),t(1),KR(2,0),KR(2,1),KR(2,2),t(2);

	for (int v=0; v<verts.size(); v++)
	{
		X << verts[v][0] , verts[v][1], verts[v][2], 1.0;
		x=P*X;
		if( x(0)/x(2)<_cols && x(0)/x(2)>0 && x(1)/x(2)<_rows && x(1)/x(2)>0 ) {
            std::cout<< "\n Vert Reprojected ";
        }
	}
}

void RayTracer::setScene( std::vector< std::vector<float> > &verts, std::vector< std::vector<int> > &faces)
{

     _geomID = rtcNewTriangleMesh(_scene, RTC_GEOMETRY_STATIC, faces.size(), verts.size(), 1);

    // Dump vertices
    vertex* vertsemb = (vertex*) rtcMapBuffer(_scene, _geomID, RTC_VERTEX_BUFFER);
    for (int i=0; i<verts.size(); i++) {
        vertsemb[i].x = verts[i][0];
        vertsemb[i].y = verts[i][1];
        vertsemb[i].z = verts[i][2];
        vertsemb[i].a = 0.0f;
    }
    rtcUnmapBuffer(_scene, _geomID, RTC_VERTEX_BUFFER);


    // Dump faces
    face* facesemb = (face*) rtcMapBuffer(_scene, _geomID, RTC_INDEX_BUFFER);
    for (int j=0; j<faces.size(); j++) {
	facesemb[j].v0 = faces[j][0];
        facesemb[j].v1 = faces[j][1];
        facesemb[j].v2 = faces[j][2];
    }
    rtcUnmapBuffer(_scene, _geomID, RTC_INDEX_BUFFER);

    // Attach
    rtcCommit (_scene);

}

float RayTracer::getPercentage()
{
    int valids=0;
    for(int y=0; y<_rows;y++)
    {
    	for(int x=0; x<_cols;x++)
	{
		if(_idimg.at<int>(y,x)!=-1) valids++;
	}
    }
    return (float)valids/(float)(_cols*_rows);
}

void RayTracer::traceRaysColumnWise()
{

    int x,y, offset;

    // measure time
    PRSTimer timer; timer.start();
    
    Eigen::Vector3d rayim(3,1);
    Eigen::Vector3d rayw(3,1);
    Eigen::Vector3d xyz(3,1);

    // Init embree rays
    RTCRay* rays = new RTCRay[_cols];

    // Trace rays for image columns
    for ( y=0; y<_rows; y++)
    {
        // Reset the raystruct for tracing next line...
        for ( x=0; x<_cols; x++)
	{
	    // Direction ray world coords
	    rayim << (double)x,(double)y,1.0f;
	    rayw=_Rt*_Kinv*rayim;
	    if(_neg) rayw*=-1.0;

	    // Setting the embree struct ...
	    RTCRay& r=rays[x];
            r.org[0]=_C(0); r.org[1]=_C(1); r.org[2]=_C(2); // origin
            r.dir[0]=rayw(0); r.dir[1]=rayw(1); r.dir[2]=rayw(2); // direction
            r.tnear = _tnear;
            r.tfar = _tfar;
            r.instID = RTC_INVALID_GEOMETRY_ID;
            r.geomID = RTC_INVALID_GEOMETRY_ID;
            r.primID = RTC_INVALID_GEOMETRY_ID;
            r.mask = 0xFFFFFFFF;
            r.time = 0.0f;

	    //std::cout<<"\norig="<<r.org[0]<<" "<<r.org[1]<<" "<<r.org[2]; 
	    //std::cout<<"\ndir="<<r.dir[0]<<" "<<r.dir[1]<<" "<<r.dir[2]; 
	    	
        }
        RTCIntersectContext context;
        context.flags = RTC_INTERSECT_INCOHERENT;
        context.userRayExt = nullptr;
        rtcIntersect1M(_scene, &context, rays, _cols, sizeof(RTCRay));

        // Harvest
        for (x=0; x<_cols; x++)
	{
            // Check if rays hits geometry
            if (rays[x].primID != RTC_INVALID_GEOMETRY_ID)
	    {

		// XYZ
	    	RTCRay& r=rays[x];
		rayw<< r.dir[0], r.dir[1], r.dir[2];
		xyz=_C+rayw*r.tfar;
		_xyzimg.at<cv::Vec3f>(y,x)[0]=xyz(0);
		_xyzimg.at<cv::Vec3f>(y,x)[1]=xyz(1);
		_xyzimg.at<cv::Vec3f>(y,x)[2]=xyz(2);


		// Triangle id 
		_idimg.at<int>(y,x)=r.primID; // get ID of hit primitive

		// Barycentric coords
		_uvimg.at<cv::Vec2f>(y,x)[0]=r.u; 
		_uvimg.at<cv::Vec2f>(y,x)[1]=r.v; 

	    }
	    else
	    {
		_idimg.at<int>(y,x)=-1;
	    }
	}
    }
    delete[] rays;

    timer.stop();
   // std::cout<<"\nTracing took "<< timer.getTimeSec()<< " seconds";
}

