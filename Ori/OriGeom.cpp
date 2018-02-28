// Copyright ETHZ 2017
// autor: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch  
#include "OriGeom.h"
#include "Orientation.h"
#include <math.h>
#include <Eigen/Dense>

namespace OriGeom
{

	void computeRelative( const Orientation &identity, const Orientation &other, Orientation &out )
	{
	    	// Copy
	    	out=other;

		// First Cam (after transform this is identity)
		Eigen::Vector3d C0=identity.getC();
		Eigen::Matrix3d R0=identity.getR();
		Eigen::Vector3d t0=(-1.0)*R0*C0;
		
		// Second cam
		Eigen::Vector3d C1=other.getC();
		Eigen::Matrix3d R1=other.getR();
		Eigen::Matrix3d K1=other.getK();
		Eigen::Vector3d t1=(-1.0)*R1*C1;

		// New cam
		Eigen::Matrix3d R10=R1*R0.transpose();
		Eigen::Vector3d C10=R1*C0+t1;
	
		out.setK(K1); 
		out.setR(R10); 
		out.setC(C10); 
	}
}
