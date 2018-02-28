// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "TfwIO.h"
#include <iostream>
#include <sstream>
#include <math.h>

namespace TfwIO
{

void readTfw(	const std::string &name,
		const int rows, const int cols,
		double& minx, double& maxx,
		double& miny, double& maxy,
		double& gsd )
{

    	// Helpers
    	double m00, m10, m01, m11;
	double x,y, x1, y1;

	std::fstream fid;
	fid.open(name.c_str(), std::ifstream::in);
	if (!fid)
	{
	    std::cout<<"\n TfwIO: Problems openeing file "<<name;
	    exit(1);
	}

	std::string s;
	std::stringstream ss;

	fid>>s; ss<<s; ss>>m00; ss.str(""); ss.clear(); // First matrix element
	fid>>s; ss<<s; ss>>m01; ss.str(""); ss.clear(); // Second matrix element
	fid>>s; ss<<s; ss>>m10; ss.str(""); ss.clear(); // Third matrix element
	fid>>s; ss<<s; ss>>m11; ss.str(""); ss.clear(); // Fourth matrix element

	fid>>s; ss<<s; ss>>x; ss.str(""); ss.clear(); // x
	fid>>s; ss<<s; ss>>y; ss.str(""); ss.clear(); // y

	x1=x+m00*(cols-1);
	y1=y+m11*(rows-1);

	minx=std::min(x1,x);
	maxx=std::max(x1,x);
	miny=std::min(y1,y);
	maxy=std::max(y1,y);

	if(fabs(m00)-fabs(m11)<10e-5)
	{
		gsd=fabs(m00);
	}
	else
	{
	    	std::cout<<"\nTfwIO: Detected different gsd sizes.";
		exit(1);
	}

	fid.close();
}

void writeTfw(	const std::string &name,
		const double ulx, const double uly,
		const double gsd)
{
	std::ofstream tfwfile;
	tfwfile.open (name.c_str(), std::ofstream::out);
	if (!tfwfile.is_open())
	{
	    std::cout<<"\n TfwIO: Problems openeing file "<<name;
	    exit(1);
	}

    	tfwfile<<gsd<<"\n";
	tfwfile<<0<<"\n";
	tfwfile<<0<<"\n";
    	tfwfile<<-gsd<<"\n";
    	tfwfile<<ulx<<"\n";
    	tfwfile<<uly<<"\n";

	tfwfile.close();

}

}
