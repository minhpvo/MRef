#include "MeshRefine.h"
#include "GradCalcStereo.h"
#include "GradThinPlate.h"
#include "GradStraightEdges.h"
#include "../MeshClassify/MeshMRF.h"
#include "../LikelihoodImage/LikelihoodImage.h"
#include "../PRSTimer/PRSTimer.h"
#include "../MeshUtil/MeshDivide.h"
#include "../MeshUtil/MeshIO.h"
#include "../FileSystemUtil/FDUtil.h"

#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>

static void showUsage(std::string name) {
  std::cerr << "Usage: " << name << " <option(s)> SOURCES"
            << "Options:\n"
            << "\t-h,--help\t\tShow this help message\n"
            << "\t-b,--basepath \t\tSpecify the basepath \n"
            << "\t-st,--starttile \t\tSpecify the starttile id \n"
            << "\t-et,--endtile \t\tSpecify the endtile id \n";
}


static bool parseArguments(int argc, char *argv[],
                           std::string &basepath,
                           int &starttile,
                           int &endtile) {
  int nt = -1;
  basepath = "";
  starttile = -1;
  endtile = -1;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-h") || (arg == "--help")) {
      showUsage(argv[0]);
      return 0;
    } else if ((arg == "-b") || (arg == "--basepath")) {
      if (i + 1 < argc) { basepath = argv[++i]; }
      else {
        std::cerr << "--basepath option requires argument.\n";
        return 1;
      }
    } else if ((arg == "-st") || (arg == "--starttile")) {
      if (i + 1 < argc) { starttile = atoi(argv[++i]); }
      else {
        std::cerr << "--starttile option requires argument.\n";
        return 1;
      }
    } else if ((arg == "-et") || (arg == "--endtile")) {
      if (i + 1 < argc) { endtile = atoi(argv[++i]); }
      else {
        std::cerr << "--starttile option requires argument.\n";
        return 1;
      }
    }
  }

  //Verify
  if (basepath == "") {
    std::cout << "\nBasepath required.";
    showUsage(argv[0]);
    exit(1);
  }

  return true;
}

void verifyInput(const IOList &imglist, const IOList &orilist, const IOList &likelilist, const IOList &meshlist,
                 const ControlRefine &controlrefine) {

  // Check proper sizes
  if (imglist.size() <= 1) {
    std::cout << "\nImglist: size=1/0.";
    exit(1);
  }
  if (orilist.size() <= 1) {
    std::cout << "\nOrilist: size=1/0.";
    exit(1);
  }
  if (likelilist.size() <= 1) {
    std::cout << "\nLiklilist: size=1/0.";
    exit(1);
  }
  if (meshlist.size() < 1) {
    std::cout << "\nMeshlist: size=1/0.";
    exit(1);
  }

  // Check sizes
  int imgnum = imglist.size();
  if (orilist.size() != imgnum) {
    std::cout << "Number of oris different from number of imgs";
    exit(1);
  }
  if (controlrefine._usesemanticdata || controlrefine._usesemanticsmooth) {
    if (likelilist.size() != imgnum) {
      std::cout << "Number of likeliimgs different from number of imgs";
      exit(1);
    }
  }
}

void assembleTileList(std::vector<int> &tilestoprocess, const int starttile, const int endtile, const int numalltiles) {
  // Update meshlist
  if (starttile == -1 || endtile == -1) {
    tilestoprocess.resize(numalltiles);
    for (int i = 0; i < numalltiles; i++) {
      tilestoprocess[i] = i;
    }
  } else // Verify if all in range
  {
    if (starttile > endtile) {
      std::cout << "\nError: startile>endtile";
      exit(1);
    }
    else if (starttile < 0) {
      std::cout << "\nError: startile<0";
      exit(1);
    }
    else if (endtile >= numalltiles) {
      std::cout << "\nError: endtile>numalltiles";
      exit(1);
    }
    else {
      int count = 0;
      tilestoprocess.resize(endtile - starttile + 1);
      for (int i = starttile; i <= endtile; i++) {
        tilestoprocess[count++] = i;
      }
    }
  }
}

void prepareOutput(const std::string basepath, const IOList &tilelist,
                   const std::vector<int> &tilenum,
                   std::vector <std::string> &infilenames,
                   std::vector <std::string> &outfilenames) {
  boost::filesystem::path fsbasepath(basepath);
  if (!boost::filesystem::exists(fsbasepath)) {
    std::cout << "Basepath does not exist.";
    exit(1);
  }

  // Make the output folder
  std::string outdir(basepath);
  outdir.append("/out");
  boost::filesystem::path fsoutpath(outdir);
  if (!boost::filesystem::exists(fsoutpath)) {
    boost::filesystem::create_directory(fsoutpath);
  }

  // Make the new output files
  for (int i = 0; i < tilenum.size(); i++) {
    std::string outfilename = basepath;
    outfilename.append("/out/");
    outfilename.append(tilelist.getNameWithoutEnding(tilenum[i]));
    outfilename.append("_output.obj");
    outfilenames.push_back(outfilename);

    std::string infilename = basepath;
    infilename.append("/out/");
    infilename.append(tilelist.getNameWithoutEnding(tilenum[i]));
    infilename.append("_input.obj");
    infilenames.push_back(infilename);
  }
}


void printSummary(const std::vector<int> &tilestoprocess, const std::vector <std::string> &outfilenames,
                  const IOList &tilelist) {
  for (int t = 0; t < tilestoprocess.size(); t++) {
    std::cout << "\n Processing tile: " << tilestoprocess[t] << " tile"
              << tilelist.getNameWithoutEnding(tilestoprocess[t]) << std::endl;
    std::cout << " Savename is: " << outfilenames[t] << "\n" << std::endl;
  }
}


int main(int argc, char *argv[]) {
  // Parse Input
  std::string basepath;
  int starttile, endtile;
  parseArguments(argc, argv, basepath, starttile, endtile);

  std::string imglistname = basepath;
  imglistname.append("/imglist.txt");
  std::string likelilistname = basepath;
  likelilistname.append("/likelilist.txt"); // Can be blank, not doing semantic yet
  std::string orilistname = basepath;
  orilistname.append("/orilist.txt");
  std::string meshlistname = basepath;
  meshlistname.append("/meshlist.txt");
  std::string controlfilename = basepath;
  controlfilename.append("/ControlRefine.txt");
  IOList imglist(imglistname);
  IOList likelilist(likelilistname);
  IOList orilist(orilistname);
  IOList meshlist(meshlistname);

  // Read Controlfile
  ControlRefine ctr;
  if (!boost::filesystem::exists(controlfilename.c_str())) {
    ctr.writeFile(controlfilename);
    std::cout << "\n\nNo controlfile found, new one generated at:" << controlfilename << ". Modify and rerun.\n\n";
    return 1;
  } else {
    ctr.readFile(controlfilename);
    ctr.writeFile(controlfilename); // To cope with modified control file structure
  }

  // Should be enhanced at some point
  verifyInput(imglist, orilist, likelilist, meshlist, ctr);

  // Verify the list of tiles to be processed
  std::vector<int> tilestoprocess;
  assembleTileList(tilestoprocess, starttile, endtile, meshlist.size());

  // Preparing output folder and output tile names
  std::vector <std::string> outfilenames;
  std::vector <std::string> infilenames;
  prepareOutput(basepath, meshlist, tilestoprocess, infilenames, outfilenames);

  printSummary(tilestoprocess, outfilenames, meshlist);

  // note this script is only set up to run 1 mesh/tile at pyramid level 0
  int pyr_id = 0;

  MyMesh mesh; // mesh with original vertices
  std::string meshname(meshlist.getElement(pyr_id));
  MeshIO::readMesh(mesh, meshname, true, false, true, false, true);

  mesh.request_face_colors();
  mesh.request_vertex_colors();

  // takes the 4th channel of colors and turns it into the classification value
  MeshConv::vertexAlphaToVertexLabel(mesh);
  // sets face classification to the first vertex class
  MeshConv::vertexLabelToFaceLabel(mesh);
  // just to check color IO
//    MeshConv::vertexLabelToVertexColorICCV(mesh);
  MeshConv::faceLabelToFaceColorICCV(mesh);

  std::cout << " Saving input mesh face colors to file " << std::endl;
  MeshIO::writeMesh(mesh, infilenames[pyr_id], true, true, true, false);

//  int mac = 0;
//  for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
//    int vertexclass = mesh.data(*v_it).classification();
//    std::cout << "vertexclass " << vertexclass << " faceclass: (";
//    for (MyMesh::VertexFaceIter vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it) {
//      short faceclass = mesh.data(*vf_it).labelid();
//      std::cout << " " << faceclass;
//    }
//    std::cout << " )" << std::endl;
//    if (mac > 5) {
//      std::cout << " " << std::endl;
//      break;
//    }
//    mac++;
//  }

  int numverts = mesh.n_vertices();

  // Read mesh meta data
  FDUtil::exchangeExtension(meshname, "mmd");
  MeshMetaData mmd(meshname);
  //  *** Photometric and Semantic Consistency ***

  // Boostrap full adjancy matrix
  Eigen::MatrixXf adjacency(orilist.size(), orilist.size());
  adjacency << 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
      0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0;

  MeshRefine refmod(&mesh, &mmd, &ctr, &adjacency, &imglist, &likelilist, &orilist);
  refmod.process();

  std::cout << "\nSaving..." << outfilenames[pyr_id] << std::endl;
  mesh.request_face_colors();
  mesh.request_vertex_colors();
  std::cout << " Saving face labels to face colors " << std::endl;
  MeshConv::faceLabelToFaceColorICCV(mesh);
  std::cout << " Saving mesh face colors to file " << std::endl;
  MeshIO::writeMesh(mesh, outfilenames[pyr_id], true, true, true, false);

//  mac = 0;
//  for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
//    int vertexclass = mesh.data(*v_it).classification();
//    std::cout << "vertexclass " << vertexclass << " faceclass: (";
//    for (MyMesh::VertexFaceIter vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it) {
//      short faceclass = mesh.data(*vf_it).labelid();
//      std::cout << " " << faceclass;
//    }
//    std::cout << " )" << std::endl;
//    if (mac > 5) {
//      std::cout << " " << std::endl;
//      break;
//    }
//    mac++;
//  }

  std::cout << "..Done";
  return 0;
}