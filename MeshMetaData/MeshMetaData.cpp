// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "MeshMetaData.h"
#include <fstream>
#include <sstream>
#include <iostream>

MeshMetaData::MeshMetaData() : _numelements(10)
{
    init();
}

MeshMetaData::MeshMetaData( const std::string& file ) : _numelements(10)
{
    init();
    readMetaData(file);
}

MeshMetaData::~MeshMetaData()
{
}

void MeshMetaData::init()
{
    _offsetx=0.0;
    _offsety=0.0;
    _offsetz=0.0;

    _minx=0.0;
    _miny=0.0;
    _minz=0.0;
    _maxx=0.0;
    _maxy=0.0;
    _maxz=0.0;

    _gsd=0.0;
}

void MeshMetaData::readMetaData( const std::string& file)
{
    	std::fstream fid;
	fid.open(file.c_str(), std::ifstream::in);
	if(!fid)
	{
	    std::cout<<"\nMeshMetaData: Problems openeing file: "<<file;
	    exit(1);
	}

	std::string s;
	std::stringstream ss;
	int c=0;

	while( fid>>s)
	{
	    	if(s=="$OffsetX")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_offsetx;
			ss.str("");
			ss.clear();
			c++;
	    	}
	    	if(s=="$OffsetY")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_offsety;
			ss.str("");
			ss.clear();
			c++;
	    	}
	    	if(s=="$OffsetZ")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_offsetz;
			ss.str("");
			ss.clear();
			c++;
	    	}
	    	if(s=="$MinX")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_minx;
			ss.str("");
			ss.clear();
			c++;
	    	}
	    	if(s=="$MaxX")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_maxx;
			ss.str("");
			ss.clear();
			c++;
	    	}
	    	if(s=="$MinY")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_miny;
			ss.str("");
			ss.clear();
			c++;
	    	}
	    	if(s=="$MaxY")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_maxy;
			ss.str("");
			ss.clear();
			c++;
	    	}
	    	if(s=="$MinZ")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_minz;
			ss.str("");
			ss.clear();
			c++;
	    	}
	    	if(s=="$MaxZ")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_maxz;
			ss.str("");
			ss.clear();
			c++;
	    	}
	    	if(s=="$Gsd")
	    	{
			fid>>s; // =
			fid>>s;
			ss<<s;
			ss>>_gsd;
			ss.str("");
			ss.clear();
			c++;
	    	}
	}
	fid.close();

	if(c!=_numelements)
	{
	    std::cout<<"\nMeshMetaData: Couldnt parse all elements";
	    std::cout<< _minx << " " <<_maxx <<" "<< _miny <<" " << _maxy<< " " << _minz<<" "<< _maxz;
	}
}

void MeshMetaData::writeMetaData( const std::string& file)
{

    	std::fstream fid;
	fid.precision(16);
	fid.open(file.c_str(), std::fstream::out );

	if(!fid.is_open())
	{
	    std::cout<<"\nMeshMetaData: Problems opening file: "<<file;
	    exit(1);
	}
	else
	{
	    fid<<"$OffsetX = "<< _offsetx;
	    fid<<"\n\n$OffsetY = "<< _offsety;
	    fid<<"\n\n$OffsetZ = "<< _offsetz;
	    fid<<"\n\n$MinX = "<< _minx;
	    fid<<"\n\n$MaxX = "<< _maxx;
	    fid<<"\n\n$MinY = "<< _miny;
	    fid<<"\n\n$MaxY = "<< _maxy;
	    fid<<"\n\n$MinZ = "<< _minz;
	    fid<<"\n\n$MaxZ = "<< _maxz;
	    fid<<"\n\n$Gsd = "<< _gsd;
	}
	fid.close();
}
