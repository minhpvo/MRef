// Copyright ETHZ 2017
// author: maros blaha, eth zuerich, 2016, maros.blaha@geod.baug.ethz.ch
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch

#include "GradCalcStereo.h"
#include "GradThinPlate.h"
#include "ViewSelector.h"
#include "DataCopy.h"
#include "../IOList/IOList.h"
#include "../MeshClassify/MeshMRF.h"
#include "../MeshUtil/MyMesh.h"
#include "../MeshUtil/MeshIO.h"
#include "../MeshUtil/MeshDivide.h"
#include "../PRSTimer/PRSTimer.h"
#include "../FileSystemUtil/FDUtil.h"
#include "../MeshMetaData/MeshMetaData.h"
#include "../ControlFile/ControlRefine.h"
#include "MeshRefine.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

static void showUsage(std::string name )
{
	std::cerr << "Usage: " <<  name << " <option(s)> SOURCES"
		  << "Options:\n"
		  << "\t-h,--help\t\tShow this help message\n"
		  << "\t-b,--basepath \t\tSpecify the basepath \n"
		  << "\t-st,--starttile \t\tSpecify the starttile id \n"
		  << "\t-et,--endtile \t\tSpecify the endtile id \n";
}

static bool  parseArguments(	int argc, char* argv[],
				std::string& basepath,
				int& starttile,
				int& endtile)
{
	int nt=-1;
	basepath="";
	starttile=-1;
	endtile=-1;

	for(int i=1; i<argc; ++i)
	{
	   	std::string arg = argv[i];
		if ((arg == "-h") || (arg == "--help"))
		{
			showUsage(argv[0]);
			return 0;
		}
		else if ((arg == "-b") || (arg == "--basepath"))
		{
			if (i + 1 < argc){ basepath=argv[++i]; }
			else
			{
				std::cerr << "--basepath option requires argument.\n";
			        return 1;
			}
		}
		else if ((arg == "-st") || (arg == "--starttile"))
		{
			if (i + 1 < argc){ starttile=atoi(argv[++i]); }
			else
			{
				std::cerr << "--starttile option requires argument.\n";
			        return 1;
			}
		}
		else if ((arg == "-et") || (arg == "--endtile"))
		{
			if (i + 1 < argc){ endtile=atoi(argv[++i]); }
			else
			{
				std::cerr << "--starttile option requires argument.\n";
			        return 1;
			}
		}
	}

	//Verify
	if(basepath=="") { std::cout<<"\nBasepath required."; showUsage(argv[0]); exit(1); }

	return true;
}

void verifyInput(const IOList& imglist, const IOList& orilist, const IOList& likelilist, const IOList& meshlist, const ControlRefine& controlrefine)
{

    	// Check proper sizes
	if(imglist.size()<=1){std::cout<<"\nImglist: size=1/0."; exit(1);}
	if(orilist.size()<=1){std::cout<<"\nOrilist: size=1/0."; exit(1);}
	if(likelilist.size()<=1){std::cout<<"\nLiklilist: size=1/0.";exit(1);}
	if(meshlist.size()<1){std::cout<<"\nMeshlist: size=1/0."; exit(1);}

	// Check sizes
	int imgnum=imglist.size();
	if(orilist.size()!=imgnum) {std::cout<<"Number of oris different from number of imgs"; exit(1);}
	if(controlrefine._usesemanticdata || controlrefine._usesemanticsmooth)
	{
	    if(likelilist.size()!=imgnum) {std::cout<<"Number of likeliimgs different from number of imgs"; exit(1); }
	}
}

void prepareOutput(const std::string basepath, const IOList& tilelist, const std::vector<int>& tilenum, std::vector<std::string>& outfilenames)
{
	boost::filesystem::path fsbasepath(basepath);
	if(!boost::filesystem::exists(fsbasepath))
	{
	    	std::cout<<"Basepath does not exist.";
		exit(1);
	}

	// Make the output folder
	std::string outdir(basepath);
	outdir.append("/out");
	boost::filesystem::path fsoutpath(outdir);
	if(!boost::filesystem::exists(fsoutpath))
	{
		boost::filesystem::create_directory(fsoutpath);
	}

	// Make the new output files
	for(int i=0; i<tilenum.size(); i++)
	{
	    	std::string filename=basepath;
		filename.append("/out/");
		filename.append(tilelist.getNameWithoutEnding(tilenum[i]));
		filename.append(".obj");
		outfilenames.push_back(filename);
	}
}

// This is for cluster... precopy data
void prepareData(const Eigen::MatrixXf adjacency, IOList& imglist, IOList& orilist, IOList likelilist, const ControlRefine& ctr)
{
	std::vector<bool> cpvec(orilist.size(),false);
	for(int r=0; r<orilist.size(); r++)
	{
		for(int c=0; c<orilist.size(); c++)
		if(adjacency(r,c)>0)
		{
			 cpvec[c]=true;
		}
	}
	DataCopy cpmod;
	cpmod.setTempDir();
	cpmod.copyData(imglist,cpvec);
	cpmod.copyData(orilist,cpvec);
	if(ctr._usesemanticdata || ctr._usesemanticsmooth){ cpmod.copyData(likelilist,cpvec); }
}

void assembleTileList(std::vector<int>& tilestoprocess, const int starttile, const int endtile, const int numalltiles)
{
	// Update meshlist
	if( starttile==-1 || endtile==-1 )
	{
		tilestoprocess.resize(numalltiles);
		for(int i=0;i<numalltiles;i++)
		{
			tilestoprocess[i]=i;
		}
	}
	else // Verify if all in range
	{
	    	if(starttile>endtile){ std::cout<<"\nError: startile>endtile"; exit(1); }
		else if (starttile<0){ std::cout<<"\nError: startile<0"; exit(1); }
		else if (endtile>=numalltiles){ std::cout<<"\nError: endtile>numalltiles"; exit(1); }
		else
		{
			int count=0;	
		    	tilestoprocess.resize(endtile-starttile+1);
			for(int i=starttile;i<=endtile;i++)
			{
			    tilestoprocess[count++]=i;
			}
		}
	}
}

void printSummary(const std::vector<int>& tilestoprocess, const std::vector<std::string>& outfilenames, const IOList& tilelist)
{
    	for(int t=0;t<tilestoprocess.size();t++)
	{
	    std::cout<<"\n\n Processing tile: "<<tilestoprocess[t]<< " tile" << tilelist.getNameWithoutEnding(tilestoprocess[t]);
	    std::cout<<"\n Savename is: "<< outfilenames[t] <<"\n\n";
	}
}

int main(int argc, char* argv[])
{
	std::string basepath;
	int starttile,endtile;

    	// Parse input
	if(argc<2)
	{
	    showUsage("MeshRefine");
	    exit(1);
	}
	parseArguments(argc, argv, basepath, starttile, endtile);

	//std::string basepath;

	// Zuerich Test
	//std::string basepath="/home/mathias/AlmostTrash/Zuerich_RefineTest/";
	//std::string basepath="/cluster/home/mathiaro/projects/Zuerich/";
//	std::string basepath="/media/data/mathias/Muc24/Muenchen_DMC_Extended/project/";
       // std::string basepath="/media/data/mathias/Zuerich_ObliquePenta/NewLists/";

	std::string imglistname=basepath; imglistname.append("/imglist.txt");
    	std::string likelilistname=basepath; likelilistname.append("/likelilist.txt");
    	std::string orilistname=basepath; orilistname.append("/orilist.txt");
    	std::string meshlistname=basepath; meshlistname.append("/meshlist.txt");
	std::string controlfilename=basepath; controlfilename.append("/ControlRefine.txt");

	IOList imglist(imglistname);
	IOList likelilist(likelilistname);
	IOList orilist(orilistname);
	IOList meshlist(meshlistname);

        // Read Controlfile
	ControlRefine ctr;
	if(!boost::filesystem::exists(controlfilename.c_str()))
	{
	    ctr.writeFile(controlfilename);
	    std::cout<<"\n\nNo controlfile found, new one generated at:"<< controlfilename <<". Modify and rerun.\n\n";
	    return 1;
	}
	else
	{
		ctr.readFile(controlfilename);
		ctr.writeFile(controlfilename); // To cope with modified control file structure
	}

	// Should be enhanced at some point
	verifyInput(imglist,orilist,likelilist,meshlist,ctr);

	// Verify the list of tiles to be processed
	std::vector<int> tilestoprocess;
	assembleTileList(tilestoprocess, starttile,endtile,meshlist.size());

	// Preparing output folder and output tile names
	std::vector<std::string> outfilenames;
	prepareOutput(basepath, meshlist, tilestoprocess, outfilenames);

	printSummary(tilestoprocess,outfilenames,meshlist);
	//exit(1);

	// Iterate over all tiles
	for(int t=0;t<tilestoprocess.size();t++)
	{
	    	int i=tilestoprocess[t];

	    	std::string meshname(meshlist.getElement(i));
	    	std::cout<<"\n\n----------------------------------------------------";
		std::cout<<"\n Mesh Tile: "<< meshname;
	    	std::cout<<"\n------------------------------------------------------";

		MyMesh mesh; // mesh with original vertices
     		MeshIO::readMesh( mesh, meshname, false, false);
		int numverts=mesh.n_vertices();

		// Read mesh meta data
		FDUtil::exchangeExtension(meshname,"mmd");
		MeshMetaData mmd(meshname);
		
		// Boostrap full adjancy matrix
		Eigen::MatrixXf adjacency(orilist.size(),orilist.size());
	
		
		adjacency<< 	1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
				1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,
				0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,
				0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,
				0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
				0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,
				0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,
				0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,
				0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,
				0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,
				0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,
				0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,
				0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
				0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,
				0,0,0,0,0,0,0,0,0,0,0,0,0,1,1;

		// Prepare data (copy to local space near the node edge)
		if(ctr._preparedata)
		{
		    	prepareData(adjacency,imglist,orilist,likelilist,ctr);
		}
		PRSTimer tiletimer; tiletimer.start();
		MeshRefine refmod(&mesh, &mmd, &ctr, &adjacency, &imglist, &likelilist, &orilist );
		refmod.process();
		tiletimer.stop();
		std::cout<<"\nProcessed tile in ["<<tiletimer.getTimeMin()<<"] min.";

		// Save the stuff
		std::cout<<"\nSaving..."<<outfilenames[t];
		mesh.request_face_colors();
		MeshConv::faceLabelToFaceColorICCV(mesh);
		MeshIO::writeMesh(mesh,outfilenames[t],false,true);
		std::cout<<"..Done";

	}
	return 1;
}

double avEdgeLength(MyMesh& mesh)
{
   	double avel=0.0;
	int count=0;
    	for (MyMesh::EdgeIter e_it=mesh.edges_begin(); e_it!=mesh.edges_end(); ++e_it)
    	{
	    	avel+=mesh.calc_edge_length(*e_it);
		count++;
    	}
	return avel/(double)count;
}


int cleanGrad(Eigen::MatrixXf &grad, Eigen::VectorXi& counter, MyMesh& mesh)
{
	double gl;
    	double _MULTIPLICATOR=1.0; //0.5
    	double avel=avEdgeLength(mesh)*_MULTIPLICATOR;
    	int clean=0;
	int count=0;
	int nancount=0;
	int sizecount=0;
	for(int y=0; y<grad.rows(); y++)
	{
		
		if ( std::isnan(grad(y,0)) || std::isnan(grad(y,1)) ||  std::isnan(grad(y,2))  )
		{
                	grad(y,0)=0.0f;
			grad(y,1)=0.0f;
			grad(y,2)=0.0f;
			nancount++;
		}
		else if(fabs(grad(y,0))>avel || fabs(grad(y,1))>avel || fabs(grad(y,2))>avel)
		{
		    	gl=sqrt( pow(grad(y,0),2.0) + pow(grad(y,1),2.0) + pow(grad(y,2),2.0) );
                	grad(y,0)/=gl/avel;
			grad(y,1)/=gl/avel;
			grad(y,2)/=gl/avel;
			sizecount++;
			counter(y)++;
		}
		else
		{
		    clean++;
		    counter(y)++;
		}
		count++;
	}
	std::cout<<"\nInvalidated -> nan ["<<(float)nancount/(float)count*100.0<<"%] size [" <<(float)sizecount/(float)count*100.0 <<"%]";

	return clean;
}


void scaleGradByCounter( Eigen::MatrixXf &grad, Eigen::VectorXi& counter)
{
	int idx=0;
	for (int idx=0; idx<grad.rows();idx++)
    	{
	   	 if(counter(idx)>0)
	   	 {
        		grad(idx,0)/=(float)counter(idx);
        		grad(idx,1)/=(float)counter(idx);
        		grad(idx,2)/=(float)counter(idx);
	    	}
    	}
	
}

void updateMesh(MyMesh& mesh, Eigen::MatrixXf &grad)
{
	int idx=0;
    	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    	{
	    	//	std::cout<<grad(idx,0);
        	mesh.point(*v_it)[0]+=(grad(idx,0));
        	mesh.point(*v_it)[1]+=(grad(idx,1));
        	mesh.point(*v_it)[2]+=(grad(idx,2));
		idx++;
    	}
}

void downscale(cv::Mat& img, int level)
{
	for(unsigned int i=0; i<level; i++)
	{
	    cv::pyrDown(img,img);
	}
}

void showImage(cv::Mat &img)
{
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE ); // Create a window for display.
    cv::imshow( "Display window", img );                      // Show our image inside it.
    cv::waitKey(0);
}

void printCors(const int baseidx, const Eigen::MatrixXf& adjacency, IOList& imglist)
{
	std::cout<<"\n Baseimage="<<imglist.getElement(baseidx);
	for(int i=0; i<imglist.size(); i++)
	{
	    	if(adjacency(baseidx,i)>0.0)
		{
			std::cout<<"\n"<<i<<": "<<imglist.getElement(i);
		}
	}
}

// Should add factor 3?
void densifyMesh(MyMesh& mesh)
{
   	double edgel=avEdgeLength( mesh);
	MeshDivide::subDivideEqual(mesh);
}

