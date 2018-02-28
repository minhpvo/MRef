// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "ViewSelector.h"
#include <iostream>
#include <opencv2/core/core.hpp>

ViewSelector::ViewSelector(const IOList& orilist, const MeshMetaData& mmd) : _mmd(mmd)
{
	 _points.resize(8,3);

	 // Scale orivec to mesh COS
	 _orivec.resize(orilist.size());
	 for(int i=0;i<orilist.size();i++)
	 {
	 	Orientation ori(orilist.getElement(i));
		Eigen::Vector3d C=ori.getC();
		C(0)-=mmd._offsetx; C(1)-=mmd._offsety; C(2)-=mmd._offsetz;
		ori.setC(C);
		_orivec[i]=ori;
	 }

	 // Boostrap viewing diections
	 setViewingDirs(_orivec);
}

ViewSelector::~ViewSelector(){}

void ViewSelector::printClusterVec(const IOList& namelist)
{
       for(int i=0; i<_clustervec.size(); i++)
       {
	   std::cout<<"\nCluster "<<i;
	   for(int j=0; j<_clustervec[i].size();j++)
	   {
	       std::cout<<"\n"<<namelist.getElement(_clustervec[i][j]);
	   }
       }
}

void ViewSelector::validate(int id, Eigen::MatrixXf& adjacency)
{
	adjacency(id,id) = 1.0;
}

void ViewSelector::invalidateAllExcept(int id, Eigen::MatrixXf& adjacency)
{

    	for(int y=0; y<adjacency.rows(); y++)
	{
		if( y != id)
		{
		    adjacency(y,y) = 0.0;
		}
	}
}

void ViewSelector::printSummary(Eigen::MatrixXf& adjacency)
{
    	int countbase=0;
	int countmatch=0;
    	int maxmatch=0;
    	int minmatch=1000;
    	int meanmatch=0;

	std::vector<int> baseimgids;

    	for(int y=0; y<adjacency.rows(); y++)
	{
		if( adjacency(y,y) > 0.0)
		{
		    	std::cout<<"\n base "<<y<<":";
		    	baseimgids.push_back(y);
		    	countbase++;
			countmatch=0;
	    		for(int x=0; x<adjacency.cols(); x++)
    			{
				if( adjacency(y,x) > 0.0)
				{
				    countmatch++;
				    std::cout<<x<<" ";
				}
			}
			if(countmatch>maxmatch){maxmatch=countmatch;}
			if(countmatch<minmatch){minmatch=countmatch;}
			meanmatch+=countmatch;
		}
	}

	// Print Statistics
	std::cout<<"\n\n";
	std::cout<<"\nBaseimages to match: "<<countbase;
	std::cout<<"\nAv match images: "<<(float)meanmatch/(float)countbase;
	std::cout<<"\nMin match images: "<<minmatch;
	std::cout<<"\nMax match images: "<<maxmatch;

	std::cout<<"\nIds of base images: ";
	for(int i=0; i<baseimgids.size(); i++)
	{
	    std::cout<<" "<<baseimgids[i];
	}
}

// input is the activation by cluster
// c1 img1 img2 img7
// c2 img4 img6 im12
int ViewSelector::getNadirCluster(std::vector<std::vector<int>>& baseimages)
{
    	Eigen::Vector3d nadirdir; nadirdir<<0.0,0.0,-1.0;
	double angle;
	double clusterangle=0.0;
	int counter=0;

	int nadirid=-1;
    	// Figure out nadir cluster
    	for(int c=0; c<baseimages.size(); c++)
	{
		counter=0;
		clusterangle=0.0;
		angle=0;
		for(int i=0;i<baseimages[c].size(); i++)
		{
			angle=std::min(nadirdir.dot(_viewdirvec[baseimages[c][i]]),1.0);
			angle=std::max(-1.0,angle);
			angle=fabs(acos(angle))*180.0/M_PI;
			clusterangle+=angle;
			counter++;
		}
		clusterangle=fabs(clusterangle/(double)counter);
		if( clusterangle<5.0 || clusterangle>175.0 )
		{
		    	nadirid=c;
		    	break;
		}
	}
        return nadirid;
}

void ViewSelector::setClosestNeighborsDistance(	std::vector<int>& nadircluster,
						Eigen::MatrixXf& adjacency,
						int maxnummatchimages)
{
	for(int i=0;i<nadircluster.size();i++)
	{
	    	// Search for views closest distance
    		cv::Mat distancevec(1,_orivec.size(), CV_64F);
    		cv::Mat indexvec(1,_orivec.size(), CV_32S);
	    	for(int m=0; m<nadircluster.size();m++)
	    	{
		    	distancevec.at<double>(0,m)=(_orivec[nadircluster[i]].getC()-_orivec[nadircluster[m]].getC()).norm();
	    	}
		cv::sortIdx(distancevec,indexvec,CV_SORT_ASCENDING);

		std::vector<int> matchimages(std::min((int)nadircluster.size()-1,maxnummatchimages));
	    	for(int m=1; m<matchimages.size()+1;m++)
		{
		    	matchimages[m-1]=indexvec.at<int>(0,m);
		}

	        // Rubber in the matrix
		for(int m=0;m<matchimages.size();m++)
		{
			adjacency(nadircluster[i],matchimages[m])=1.0;
			adjacency(nadircluster[i],nadircluster[i])=1.0;
		}
	}
}

void ViewSelector::getAdjacencyTileNadir(Eigen::MatrixXf& adjacency)
{
    	int MAXNUMBASEPERCLUSTER=10;
    	int MAXNUMMATCH=4;
	bool INSTRIP=false;
	bool CROSSSTRIP=true;

	// Rubber out adjacency
	adjacency.setZero();

	// Assume cluster was build
    	//std::vector<std::vector<int>> baseimages=nearestImagesFromCluster(mmd, MAXNUMBASEPERCLUSTER);
    	std::vector<std::vector<int>> baseimages=nearestMidPointImagesFromCluster(MAXNUMBASEPERCLUSTER);

	// For each of the baseimages get neighbors
	for(int c=0;c<baseimages.size();c++)
	{
		for(int i=0;i<baseimages[c].size();i++)
		{
		    	// Search for views closest distance
	    		cv::Mat distancevec(1,_orivec.size(), CV_64F);
	    		cv::Mat indexvec(1,_orivec.size(), CV_32S);
		    	for(int m=0; m<_orivec.size();m++)
		    	{
			    	distancevec.at<double>(0,m)=(_orivec[baseimages[c][i]].getC()-_orivec[m].getC()).norm();
		    	}
			cv::sortIdx(distancevec,indexvec,CV_SORT_ASCENDING);

			std::vector<int> matchimages(MAXNUMMATCH);
		    	for(int m=1; m<std::min((int)_orivec.size(),MAXNUMMATCH+1);m++)
			{
			    	matchimages[m-1]=indexvec.at<int>(0,m);
			}

		        // Rubber in the matrix
			for(int m=0;m<matchimages.size();m++)
			{
				adjacency(baseimages[c][i],matchimages[m])=1.0;
				adjacency(baseimages[c][i],baseimages[c][i])=1.0;
			}
		}
	}
}

// Main processing function obliques
void ViewSelector::getAdjacencyTileOblique(Eigen::MatrixXf& adjacency)
{
    	int MAXNUMBASEPERCLUSTER=6;
    	int MAXNUMMATCH=2;
	bool INSTRIP=false;
	bool CROSSSTRIP=true;

	// Rubber out adjacency
	adjacency.setZero();

	// Assume cluster was build, get the base images clusterwise which are overlapping the tile
    	std::vector<std::vector<int>> baseimages=nearestMidPointImagesFromCluster(MAXNUMBASEPERCLUSTER);

	// Get the idx of the nadir cluster, if no nadir available
	int nadircluster=getNadirCluster(baseimages);
	if(nadircluster==-1){ std::cout<<"\nNo Nadir Cluster found..."; }
	else {std::cout<<"\nnadir cluster is "<<nadircluster;}

	// For each of the baseimages get neighbors
	for(int c=0;c<baseimages.size();c++)
	{
	    	if(c!=nadircluster)
	    	{
			for(int i=0;i<baseimages[c].size();i++)
			{
				std::vector<int> matchimages=nearestNeighborsInCluster(baseimages[c][i], MAXNUMMATCH, INSTRIP, CROSSSTRIP);
				// Rubber in the matrix
				for(int m=0;m<matchimages.size();m++)
				{
					adjacency(baseimages[c][i],matchimages[m])=1.0;
					adjacency(baseimages[c][i],baseimages[c][i])=1.0;
				}
			}
		}
		else  // Nadir cluster 
		{
		    	setClosestNeighborsDistance( baseimages[c], adjacency, MAXNUMMATCH );
		}
	}
}

void ViewSelector::buildClusterVec()
{

    	double anglethresh=15.0;

	// Mean shift algo
    	double angle,nxold,nyold,nzold;
	int ncount;
	int clustersize=0;
	int* neighbors=new int[_viewdirvec.size()];
	bool converged;

    	// Pic a vec
	Eigen::Vector3d curdir;
	Eigen::Vector3d olddir;
	std::vector<Eigen::Vector3d> clustern(_viewdirvec.size());
	std::vector<int> correspondence(_viewdirvec.size());

    	for(int i=0; i<_viewdirvec.size(); i++)
    	{
		curdir=_viewdirvec[i];
		olddir<<2.0,2.0,2.0;

		bool converged=false;

		int iter=0;
		while(!converged)
		{
		    iter++;
		    // Pick nearest
			ncount=0;
			for(int j=0; j<_viewdirvec.size(); j++)
			{
		    		if(i!=j)
				{
					angle=fabs(acos(curdir.dot(_viewdirvec[j])))*180.0/M_PI;
					if(angle<anglethresh)
					{
						neighbors[ncount++]=j;
					}
				}
			}

			// Calc update
			curdir<<0.0,0.0,0.0;
			for(int j=0; j<ncount; j++)
			{
				curdir+=_viewdirvec[neighbors[j]];
			}
			curdir/=(double)ncount;
			curdir.normalize();

			// On convergence
			angle=std::min(fabs(acos(curdir.dot(olddir)))*180.0/M_PI,1.0);
			angle=std::max(-1.0,angle);
			if(angle<0.1)
			{
				converged=true;
		    		bool found=false;
				for(int c=0;c<clustersize;c++)
				{
					angle=std::min(fabs(acos(curdir.dot(clustern[c])))*180.0/M_PI,1.0);
					angle=std::max(-1.0,angle);
					if( angle<0.1)
					{
						found=true;
						correspondence[i]=c;
						break;
					}
				}
				if(!found)
				{
			    		// Make new cluster setup correspondence
			 		clustern[clustersize++]=curdir;
					correspondence[i]=clustersize-1;
				}
			}
			else
			{
			   	// Update
			    	olddir=curdir;
			}
		}
	}
	delete[] neighbors;
	std::cout<<"\nFound "<<clustersize<<" clusters";
	// Boostrap the new clusters
	_clustervec.resize(clustersize);
	for(int i=0; i<correspondence.size(); i++)
	{
	    	//std::cout<<"\n"<<correspondence[i];
		_clustervec[correspondence[i]].push_back(i);
	}
}
std::vector<std::vector<int>> ViewSelector::nearestMidPointImagesFromCluster(const int& maxnumviews)
{

    	std::vector<std::vector<int>> outvec(_clustervec.size());

	// Compute middlepoint
	Eigen::Vector4d mpt;
	mpt<< 	(_mmd._maxx+_mmd._minx)/2.0-_mmd._offsetx,
	    	(_mmd._maxy+_mmd._miny)/2.0-_mmd._offsety,
	    	(_mmd._maxz+_mmd._minz)/2.0-_mmd._offsetz, 1.0;

	// Get n best views from cluster
	for(int i=0; i<_clustervec.size(); i++ )
	{
	    	cv::Mat distancevec(1,_clustervec[i].size(), CV_64F);
	    	cv::Mat indexvec(1,_clustervec[i].size(), CV_32S);
	    	for(int j=0; j<_clustervec[i].size(); j++ )
	    	{
		    	Eigen::MatrixXd P=_orivec[_clustervec[i][j]].getP();
			float colsm=(float)_orivec[_clustervec[i][j]].cols()/2.0;
			float rowsm=(float)_orivec[_clustervec[i][j]].rows()/2.0;
		    	Eigen::Vector3d ipt=P*mpt;
			ipt/=ipt(2);

		    	distancevec.at<double>(0,j)=sqrt(pow(ipt(0)-colsm,2)+pow(ipt(1)-rowsm,2));

	    	}
		cv::sortIdx(distancevec,indexvec,CV_SORT_ASCENDING);

		for(int h=0;h<std::min(maxnumviews,(int)_clustervec[i].size());h++)
		{
			outvec[i].push_back(_clustervec[i][indexvec.at<int>(0,h)]);
		}
    	}
	return outvec;
}

std::vector<std::vector<int>> ViewSelector::nearestImagesFromCluster( const int& maxnumviews)
{

    	std::vector<std::vector<int>> outvec(_clustervec.size());

	// Compute middlepoint
	Eigen::Vector3d mpt;
	mpt<<(_mmd._maxx+_mmd._minx)/2.0-_mmd._offsetx, 
	    (_mmd._maxy+_mmd._miny)/2.0-_mmd._offsety,
	    (_mmd._maxz+_mmd._minz)/2.0-_mmd._offsetz,1.0;

	// Get n best views from cluster
	for(int i=0; i<_clustervec.size(); i++ )
	{
	    	cv::Mat distancevec(1,_clustervec[i].size(), CV_64F);
	    	cv::Mat indexvec(1,_clustervec[i].size(), CV_32S);
	    	for(int j=0; j<_clustervec[i].size(); j++ )
	    	{
		    	distancevec.at<double>(0,j)=(_orivec[_clustervec[i][j]].getC()-mpt).norm();
		    //std::cout<<"\n"<<distancevec.at<double>(0,j);
	    	}
		cv::sortIdx(distancevec,indexvec,CV_SORT_ASCENDING);

		for(int h=0;h<std::min(maxnumviews,(int)_clustervec[i].size());h++)
		{
			outvec[i].push_back(_clustervec[i][indexvec.at<int>(0,h)]);
		}
    	}
	return outvec;
}

std::vector<int> ViewSelector::nearestNeighborsInCluster(const int imgidx, const int maxnumviews,
							 bool inflight, bool crossflight)
{

    	int clusterid;
    	std::vector<int> outvec;
	Eigen::Vector3d C=_orivec[imgidx].getC();
	Eigen::Vector3d baseline;
	Eigen::Vector3d dir;
	double angle;

	// Search the correct cluster
	bool found=false;
    	for(int i=0; i<_clustervec.size();i++)
	{
    		for(int j=0; j<_clustervec[i].size();j++)
		{
			if(_clustervec[i][j]==imgidx)
			{
				clusterid=i;
				found=true;
				break;
		    	}
		}
		if(found){ break; }
	}

	// Set the reference cluster
	std::vector<int>& cluster=_clustervec[clusterid];
	int clustersize=cluster.size();

	// Now search cluster for the nearest pairs
	if(inflight && crossflight)
	{
    		cv::Mat distancevec(1,clustersize, CV_64F);
    		cv::Mat indexvec(1,clustersize, CV_32S);
    		for(int j=0; j<clustersize; j++)
    		{
		    	distancevec.at<double>(0,j)=(_orivec[cluster[j]].getC()-C).norm();
    		}
		cv::sortIdx(distancevec,indexvec,CV_SORT_ASCENDING);

		for(int h=0;h<std::min(maxnumviews,(int)clustersize);h++)
		{
			outvec.push_back(cluster[indexvec.at<int>(0,h)]);
		}
	}
	else if(inflight^crossflight)
	{
		dir<<_orivec[imgidx].getR()(2,0),_orivec[imgidx].getR()(2,1),0.0; // Dump z-component
		dir.normalize();
		std::vector<int> subset;

		for(int i=0;i<clustersize;i++)
		{
		    	baseline=(C-_orivec[cluster[i]].getC());
		    	baseline(2)=0.0;
			baseline.normalize();
			angle=std::min(baseline.dot(dir),1.0);
			angle=std::max(-1.0,angle);
			angle=fabs(acos(angle))*180.0/M_PI;
			if(inflight)
			{
				// Check if inflight
				if(angle<10 || angle >170){ subset.push_back(cluster[i]); }
			}
		  	else if(crossflight)
			{
				if(angle>80.0 && angle<100.0){ subset.push_back(cluster[i]); }
			}
		}
		// Now we have the subset
    		cv::Mat distancevec(1,subset.size(),CV_64F);
    		cv::Mat indexvec(1,subset.size(),CV_32S);
    		for(int j=0; j<subset.size(); j++)
    		{
	    		distancevec.at<double>(0,j)=(_orivec[subset[j]].getC()-C).norm();
    		}
		cv::sortIdx(distancevec,indexvec,CV_SORT_ASCENDING);

		for(int h=0;h<std::min(maxnumviews,(int)subset.size());h++)
		{
			outvec.push_back(subset[indexvec.at<int>(0,h)]);
		}
    	}
	return outvec;
}


void ViewSelector::setPoints()
{
	_points(0,0)=_mmd._minx-_mmd._offsetx; _points(0,1)=_mmd._miny-_mmd._offsety; _points(0,2)=_mmd._minz-_mmd._offsetz;
	_points(1,0)=_mmd._minx-_mmd._offsetx; _points(1,1)=_mmd._maxy-_mmd._offsety; _points(1,2)=_mmd._minz-_mmd._offsetz;
	_points(2,0)=_mmd._maxx-_mmd._offsetx; _points(2,1)=_mmd._miny-_mmd._offsety; _points(2,2)=_mmd._minz-_mmd._offsetz;
	_points(3,0)=_mmd._maxx-_mmd._offsetx; _points(3,1)=_mmd._maxy-_mmd._offsety; _points(3,2)=_mmd._minz-_mmd._offsetz;

	_points(4,0)=_mmd._minx-_mmd._offsetx; _points(4,1)=_mmd._miny-_mmd._offsety; _points(4,2)=_mmd._maxz-_mmd._offsetz;
	_points(5,0)=_mmd._minx-_mmd._offsetx; _points(5,1)=_mmd._maxy-_mmd._offsety; _points(5,2)=_mmd._maxz-_mmd._offsetz;
	_points(6,0)=_mmd._maxx-_mmd._offsetx; _points(6,1)=_mmd._miny-_mmd._offsety; _points(6,2)=_mmd._maxz-_mmd._offsetz;
	_points(7,0)=_mmd._maxx-_mmd._offsetx; _points(7,1)=_mmd._maxy-_mmd._offsety; _points(7,2)=_mmd._maxz-_mmd._offsetz;
}

void ViewSelector::setViewingDirs(const std::vector<Orientation>& orivec)
{
    	_viewdirvec.resize(orivec.size());
	for(int i=0; i<orivec.size(); i++)
	{
	    	_viewdirvec[i](0)=orivec[i].getR()(2,0);
	    	_viewdirvec[i](1)=orivec[i].getR()(2,1);
	    	_viewdirvec[i](2)=orivec[i].getR()(2,2);
	}
}
/*
void ViewSelecor::setAreaImg(const std::vector<Orientation>& orivec)
{
    	Eigen::Vector4d curpt;
    	Eigen::Vector3d imgpt;
    	Eigen::MatrixXd P(3,4);
	for(int o=0; o<orivec.size(); o++)
	{
    		P=orivec[o].getP();

		bool inimg=false;
		for(int p=0; p<8; p++)
		{
			curpt<< _points(p,0),_points(p,1), _points(p,2),1.0;
			imgpt=P*curpt;
			imgpt/=imgpt(2);

			if(imgpt(0)>=1 && imgpt(0)<orivec[o].cols()-2 &&
			   imgpt(1)>=1 && imgpt(1)<orivec[o].rows()-2)
			{
			    	indices.push_back(o);
				break;
			}
		}
	}
}


void ViewSelector::getBestPairs(	const MeshMetaData& mmd,
					const Orientation& baseori,
					const std::vector<Orientation>& orivec,
					const double maxangle,
					const double numnearest,
					std::vector<int>& indexvec)
{

    	indexvec.clear();
    	std::vector<int> indices;

    	// Make points at tile corners see if atleat one is overlapping

    	Eigen::Vector4d curpt;
    	Eigen::Vector3d imgpt;
    	Eigen::MatrixXd P(3,4);
	for(int o=0; o<orivec.size(); o++)
	{
    		P=orivec[o].getP();

		bool inimg=false;
		for(int p=0; p<8; p++)
		{
			curpt<< _points(p,0), _points(p,1), _points(p,2),1.0;
			imgpt=P*curpt;
			imgpt/=imgpt(2);

			if(imgpt(0)>=1 && imgpt(0)<orivec[o].cols()-2 &&
			   imgpt(1)>=1 && imgpt(1)<orivec[o].rows()-2)
			{
			    	indices.push_back(o);
				break;
			}
		}
	}
*/

// This is most simple checker for single ori
void ViewSelector::computeOriCors(	const Orientation& baseori,
					const std::vector<Orientation>& orivec,
					const double maxangle,
					const double numnearest,
					std::vector<int>& indexvec)
{

    	indexvec.clear();
    	std::vector<int> indices;

	Eigen::MatrixXd points(8,3);
    	// Make points at tile corners see if atleat one is overlapping
	_points(0,0)=_mmd._minx-_mmd._offsetx; points(0,1)=_mmd._miny-_mmd._offsety; points(0,2)=_mmd._minz-_mmd._offsetz;
	_points(1,0)=_mmd._minx-_mmd._offsetx; points(1,1)=_mmd._maxy-_mmd._offsety; points(1,2)=_mmd._minz-_mmd._offsetz;
	_points(2,0)=_mmd._maxx-_mmd._offsetx; points(2,1)=_mmd._miny-_mmd._offsety; points(2,2)=_mmd._minz-_mmd._offsetz;
	_points(3,0)=_mmd._maxx-_mmd._offsetx; points(3,1)=_mmd._maxy-_mmd._offsety; points(3,2)=_mmd._minz-_mmd._offsetz;

	_points(4,0)=_mmd._minx-_mmd._offsetx; points(4,1)=_mmd._miny-_mmd._offsety; points(4,2)=_mmd._maxz-_mmd._offsetz;
	_points(5,0)=_mmd._minx-_mmd._offsetx; points(5,1)=_mmd._maxy-_mmd._offsety; points(5,2)=_mmd._maxz-_mmd._offsetz;
	_points(6,0)=_mmd._maxx-_mmd._offsetx; points(6,1)=_mmd._miny-_mmd._offsety; points(6,2)=_mmd._maxz-_mmd._offsetz;
	_points(7,0)=_mmd._maxx-_mmd._offsetx; points(7,1)=_mmd._maxy-_mmd._offsety; points(7,2)=_mmd._maxz-_mmd._offsetz;

    	Eigen::Vector4d curpt;
    	Eigen::Vector3d imgpt;
    	Eigen::MatrixXd P(3,4);
	for(int o=0; o<orivec.size(); o++)
	{
    		P=orivec[o].getP();

		bool inimg=false;
		for(int p=0; p<8; p++)
		{
			curpt<< _points(p,0), points(p,1), points(p,2),1.0;
			imgpt=P*curpt;
			imgpt/=imgpt(2);

			if(imgpt(0)>=1 && imgpt(0)<orivec[o].cols()-2 &&
			   imgpt(1)>=1 && imgpt(1)<orivec[o].rows()-2)
			{
			    	indices.push_back(o);
				break;
			}
		}
	}

	// Up to here we have cams covering the area ... check rotation
	Eigen::Matrix3d R=baseori.getR();
	Eigen::Vector3d basedir; basedir<<R(2,0),R(2,1),R(2,2);
	Eigen::Vector3d compdir;

	for(int o=0; o<indices.size(); o++)
	{
		// Angle between
		R=orivec[indices[o]].getR();
		double angle=fabs(acos(basedir(0)*R(2,0)+basedir(1)*R(2,1)+basedir(2)*R(2,2)))*180.0/M_PI;
		if(angle<maxangle && angle==angle)
		{
			indexvec.push_back(indices[o]);
		}
	}

	// Make a distance mat
	cv::Mat distances(1,indexvec.size(),CV_64F);
	Eigen::Vector3d C=baseori.getC();
	for(int o=0; o<indexvec.size(); o++)
	{
	    distances.at<double>(0,o)=(C-orivec[indexvec[o]].getC()).norm();

	}
	// Sort the thing
	cv::Mat order;
	cv:sortIdx(distances,order,CV_SORT_DESCENDING);

	// Check translation
	indices=indexvec;
	indexvec.clear();
	for(int o=0; o<numnearest+1; o++)
	{
		// Angle between
		indexvec.push_back(indices[order.at<int>(0,o)]);
	}
	//std::cout<<"\n"<<indexvec.size();

}

// This is most simple checker for all oris
void ViewSelector::computeConnectivity(	const std::vector<Orientation>& orivec,
					const double maxangle,
					const double numnearest,
					Eigen::MatrixXf& connectivity)
{
    	connectivity=Eigen::MatrixXf::Zero(orivec.size(), orivec.size());
    	std::vector<int> correspondences;

	// for each baseori get the correspondenes
    	for(int i=0; i< orivec.size();i++)
    	{
	    	correspondences.clear();
		computeOriCors( orivec[i], orivec, maxangle, numnearest, correspondences);
		for(int j=0;j<correspondences.size();j++)
		{
			connectivity(i,correspondences[j])=1.0;
		}
    	}
}
