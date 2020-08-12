// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "MeshRefine.h"
#include "GradCalcStereo.h"
#include "GradThinPlate.h"
#include "GradStraightEdges.h"
#include "../MeshClassify/MeshMRF.h"
#include "../LikelihoodImage/LikelihoodImage.h"
#include "../PRSTimer/PRSTimer.h"
#include "../MeshUtil/MeshDivide.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// This should be the class controling the tile-wise processing
MeshRefine::MeshRefine
    (
        MyMesh *tile,
        MeshMetaData *mmd,
        ControlRefine *ctr,
        Eigen::MatrixXf *adjacency,
        IOList *imglistphoto,
        IOList *imglistsem,
        IOList *orilist
    ) :
    _mesh(tile),
    _mmd(mmd),
    _ctr(ctr),
    _adjacency(adjacency),
    _imglistphoto(imglistphoto),
    _imglistsem(imglistsem),
    _orilist(orilist),
    _verboselevel(ctr->_verboselevel) {
}

MeshRefine::~MeshRefine() {
}


void MeshRefine::computeAdaptedOrientation(std::vector <Orientation> &adaptedori) {
  // Get the connectivity matrix
  Eigen::Vector3d O;
  std::vector <Orientation> orivec;
  for (int i = 0; i < _orilist->size(); i++) {
    Orientation ori(_orilist->getElement(i));
    // Handle offset
    O << _mmd->_offsetx, _mmd->_offsety, _mmd->_offsetz;
    Eigen::Vector3d C = ori.getC();
    C -= O;
    ori.setC(C);
    adaptedori.push_back(ori);
  }
}

void MeshRefine::downscale(cv::Mat &img, int level) {
  for (unsigned int i = 0; i < level; i++) {
    cv::pyrDown(img, img);
  }
}

int MeshRefine::cleanGrad(const float avedgelength, Eigen::MatrixXf &grad, Eigen::VectorXi &counter, MyMesh &mesh) {
  double gl;
  double scale;
  double _MULTIPLICATOR = .5;
  double avel = avedgelength * _MULTIPLICATOR;

  // Some counters
  int clean = 0;
  int count = 0;
  int nancount = 0;
  int sizecount = 0;
  int newnancount = 0;

  for (int y = 0; y < grad.rows(); y++) {
    // Gradient length
    double sgl = pow(grad(y, 0), 2.0) + pow(grad(y, 1), 2.0) + pow(grad(y, 2), 2.0);
    if (sgl > 0) {
      gl = sqrt(sgl); // 2-norm of the gradient vec
      scale = avel / gl; // normalize gradient (1/gl) and multiply by "avel" 
    } else {
      gl = 0;
      scale = 0;
    }
    // Rubber out nan
    if (!(std::isfinite(grad(y, 0))) || !(std::isfinite(grad(y, 1))) || !(std::isfinite(grad(y, 2)))) {
      grad(y, 0) = 0.0f;
      grad(y, 1) = 0.0f;
      grad(y, 2) = 0.0f;
      nancount++;
    }
      // Rubber out to long
    else if ((gl > avel)) {
      // scale the gradient to be length "avel" if it's larger than avel (I think just capping magnitude)
      grad(y, 0) *= scale;
      grad(y, 1) *= scale;
      grad(y, 2) *= scale;
      sizecount++;
      counter(y)++;
    } else {
      clean++;
      counter(y)++;
    }
    count++;
  }
  if (_verboselevel > 1) {
    std::cout << "Invalidated NAN -> " << (float) nancount / (float) count * 100.0 << "%" << std::endl;
    std::cout << "Invalidated SIZE ->" << (float) sizecount / (float) count * 100.0 << "%" << std::endl;
  }

  for (int y = 0; y < grad.rows(); y++) {
    if (!(std::isfinite(grad(y, 0))) || !(std::isfinite(grad(y, 1))) || !(std::isfinite(grad(y, 2)))) {
      std::cout << "MeshRefine: Another Nan in grad! This is bad; What's going on?\n y= " << y << std::endl;
      std::cout << "y: " << y << "\t " << grad(y, 0) << "\t " << grad(y, 1) << "\t " << grad(y, 2) << "\t "
                << std::endl;
      std::cout << "avel: " << avel << "\t gl: " << gl << std::endl;
      exit(1);
    }
  }
  return clean;
}

double MeshRefine::avEdgeLength(MyMesh &mesh) {
  double avel = 0.0;
  int count = 0;
  for (MyMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
    avel += mesh.calc_edge_length(*e_it);
    count++;
  }
  return avel / (double) count;
}

void MeshRefine::updateMesh(MyMesh &mesh, Eigen::MatrixXf &grad) {
  int idx = 0;
  for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
    //	std::cout<<grad(idx,0);
//    if (v_it > 139729)

    if ((grad(idx, 0)) && (grad(idx, 1)) && (grad(idx, 2))) {
      mesh.point(*v_it)[0] += (grad(idx, 0));
      mesh.point(*v_it)[1] += (grad(idx, 1));
      mesh.point(*v_it)[2] += (grad(idx, 2));
    }
    idx++;
  }
}

void MeshRefine::scaleGradByCounter(Eigen::MatrixXf &grad, Eigen::VectorXi &counter) {

  for (int idx = 0; idx < grad.rows(); idx++) {
    if (counter(idx) > 0) {
      grad(idx, 0) /= static_cast<float>(counter(idx));
      grad(idx, 1) /= static_cast<float>(counter(idx));
      grad(idx, 2) /= static_cast<float>(counter(idx));
    }
  }
}

double MeshRefine::photoConsistency() {
  double energy = 0;

//  Orientation ori0{};
//  Orientation ori1{};
  std::vector <Orientation> orivec;
  PRSTimer timer;
  PRSTimer timer2;
  LikelihoodImage limg0;
  LikelihoodImage limg1;
  int image_pairs = 0;
//  cv::Mat img1;
//  cv::Mat img0;

  computeAdaptedOrientation(orivec);
  // Iterate over all images
  for (int i = 0; i < _adjacency->rows(); i++) {
    // Check if image is a master
//    if ((*_adjacency)(i, i) > -1.0) {
    // if(_verboselevel>=1) { std::cout <<"\nProcessing baseimage "<<i; }
    // Load base image stuff
    Orientation ori0 = orivec[i];
    // ori0.downscalePyr(pyr); PYR is pyramid level, we will just perform everything at full resolution (pyr=0) - MRC
    timer.start();
    // Read image from image list in greyscale - MRC
    auto img0 = cv::imread(_imglistphoto->getElement(i), cv::IMREAD_GRAYSCALE);
    // downscale(img0, pyr); No downscaling - MRC
    if (img0.cols == 0 || img0.rows == 0) {
      std::cout << "Problems Reading Image" << std::endl;
      exit(1);
    }
    if (img0.type() != CV_8UC1) { img0.convertTo(img0, CV_8UC1); }
    timer.stop();
//      LikelihoodImage limg0;
    // Let's not deal with semantic stuff for now... - MRC

    // if(_ctr->_usesemanticdata){
    //     limg0.loadImage(_imglistsem->getElement(i));
    //     limg0.downscale(pyr);
    // }

    // GradCalcStereo gradcomp(img0, &limg0, &ori0, _mesh,
    //                         _ctr->_usesemanticdata ? GradCalcStereo::SEMANTICMODE::ON : GradCalcStereo::SEMANTICMODE::OFF,
    //                         _ctr->_tnear, _ctr->_tfar, _ctr->_verboselevel);
    GradCalcStereo gradcomp(img0, &limg0, &ori0, _mesh, GradCalcStereo::SEMANTICMODE::OFF,
                            _ctr->_tnear, _ctr->_tfar, _ctr->_verboselevel);
    for (int j = 0; j < _adjacency->cols(); j++) {
      // avoid reference image
      if ((*_adjacency)(i, j) > 0.0 && j != i) {
        image_pairs += 1;
        // if(_verboselevel>=1) { std::cout <<"\nProcessing match image "<<j;}
        // Load slave img stuff
        timer2.start();
        Orientation ori1 = orivec[j];
        // ori1.downscalePyr(pyr); No downscaling - MRC
        auto img1 = cv::imread(_imglistphoto->getElement(j), cv::IMREAD_GRAYSCALE);
        //downscale(img1,pyr); No dowscaling - MRC
        if (img1.type() != CV_8UC1) {
          img1.convertTo(img1, CV_8UC1, 255);
        }
//          LikelihoodImage limg1;

        // if(_ctr->_usesemanticdata)
        // {
        //     limg1.loadImage(_imglistsem->getElement(j));
        //     limg1.downscale(pyr);
        // }
        timer2.stop();

        // Set the second view...
        gradcomp.setSecondView(img1, &limg1, &ori1);

        // This should compute Photometric consistency
        // and if available Semantic data terms
        auto img_energy = gradcomp.process();
        energy = energy + img_energy;
        std::cout << "IMG " << i << " " << j << " img_energy: " << img_energy;
        std::cout << " \t Energy: " << energy << std::endl;
        // pixsize += gradcomp.computeTriangleSizeInPix();

        // // Actually here base should be scaled

        // // Get the gradient, rubber nan and a
        // tempgrad = gradcomp.getPhotoGrad()*_ctr->_photoweightvec[pyr]*pow(gradcomp.getMeanDist(),2.0)/pow((ori1.getK())(0,0),2.0);

        // cleanGrad(avedgelength,tempgrad,counter,*_mesh);
        // grad += tempgrad;

        // // Get the gradient, rubber nan and a
        // tempgrad = gradcomp.getSemanticGrad()*_ctr->_semweightvec[pyr];
        // cleanGrad(avedgelength,tempgrad,counter,*_mesh);
        // grad += tempgrad;
        // nm++;
      }
    }
//    }
  }
  std::cout << "\t num image_pairs: " << image_pairs << std::endl;
  return energy / image_pairs;
}

// Main procesing function
void MeshRefine::process() {
  int numverts = _mesh->n_vertices();
  double avedgelength;
  double oldenergy = 0;
  double energy;
  double pixsize;
  int nm;
  int image_pairs;
  const int nrows = _ctr->_nrows; //600;
  const int ncols = _ctr->_ncols; //1066;
  const int nlabels = _ctr->_nlabels; //7;

  std::vector <Orientation> orivec;
  computeAdaptedOrientation(orivec);

  // Hirarchical loop
  for (int pyr = _ctr->_startlevel; pyr >= _ctr->_endlevel; pyr--) {
    avedgelength = avEdgeLength(*_mesh);
    if (_verboselevel >= 1) {
      std::cout << "avg. mesh edge length: " << avedgelength << std::endl;
    }

    Eigen::MatrixXf grad(numverts, 3);
    Eigen::VectorXi counter(numverts);
    Eigen::MatrixXf tempgrad(numverts, 3);

    // Mesh refinement
    for (int iter = 0; iter < _ctr->_numitervec[pyr]; iter++) {
      // Reset gradvec
      grad.setZero();
      tempgrad.setZero();
      counter.setZero();

      PRSTimer itertimer;
      itertimer.start();
      energy = 0;
      pixsize = 0.0;
      image_pairs = 0;
      nm = 0;
      //  *** Do the Relabeling ***
      if ((_ctr->_usemrflabelsmoothing && iter % _ctr->_skipiterations == 0) ||
          (_ctr->_usesemanticsmooth && iter == 0) ||
          (_ctr->_usestraightedgegrad && iter == 0)) {
        if (_verboselevel >= 0) {
          std::cout << "\n||> Semantic Optimization <||" << std::endl;
        }
        // TODO (MAC) Number of labels is hard-coded here -- this, and the
        //  likelihood class -- should be watched for specific row/column/label sizes
        MeshMRF meshmrf(_mesh, _mmd, nlabels);
        meshmrf.process(_imglistsem->getList(), _orilist->getList(), _ctr->_nummrfitervec[pyr]);
      }
      if (_verboselevel >= 0) {
        std::cout << "\n||> Geometric Optimization <||" << std::endl;
      }
      //  *** Photometric and Semantic Consistency ***
      // Iterate over all images
      if (_verboselevel >= 0) {
        std::cout << "Level: " << pyr << "\tIteration: " << (iter + 1) << " / " << _ctr->_numitervec[pyr] << std::endl;;
      }
      for (int i = 0; i < _adjacency->rows(); i++) {
        // Check if image is a master
        // if ((*_adjacency)(i,i)> 0.0 )
        if ((*_adjacency)(i, i) > -1.0) {

          if (_verboselevel >= 1) {
            std::cout << "Processing base image " << i << std::endl;
          }
          // Load base image stuff
          Orientation ori0 = orivec[i];
          ori0.downscalePyr(pyr);

          PRSTimer timer;
          timer.start();
          cv::Mat img0 = cv::imread(_imglistphoto->getElement(i), cv::IMREAD_GRAYSCALE);
          downscale(img0, pyr);
          if (img0.cols == 0 || img0.rows == 0) {
            std::cout << "Problems reading image" << std::endl;
            exit(1);
          }
          if (img0.type() != CV_8UC1) { img0.convertTo(img0, CV_8UC1); }
          timer.stop();

          LikelihoodImage limg0;
          if (_ctr->_usesemanticdata) {
            limg0.loadImage(_imglistsem->getElement(i), nrows, ncols, nlabels);
            limg0.downscale(pyr);
          }

          GradCalcStereo gradcomp(img0, &limg0, &ori0, _mesh,
                                  _ctr->_usesemanticdata ? GradCalcStereo::SEMANTICMODE::ON
                                                         : GradCalcStereo::SEMANTICMODE::OFF, _ctr->_tnear, _ctr->_tfar,
                                  _ctr->_verboselevel);
          // Get vector with all slave images
          for (int j = 0; j < _adjacency->cols(); j++) {
            // avoid reference image
            if ((*_adjacency)(i, j) > 0.0 && j != i) {
              image_pairs += 1;

              if (_verboselevel >= 1) {
                std::cout << "Processing match image " << j << std::endl;
              }
              // Load slave img stuff
              PRSTimer timer2;
              timer2.start();
              Orientation ori1 = orivec[j];
              ori1.downscalePyr(pyr);
              cv::Mat img1 = cv::imread(_imglistphoto->getElement(j), cv::IMREAD_GRAYSCALE);
              downscale(img1, pyr);
              if (img1.type() != CV_8UC1) { img1.convertTo(img1, CV_8UC1, 255); }
              LikelihoodImage limg1;
              if (_ctr->_usesemanticdata) {
                limg1.loadImage(_imglistsem->getElement(j), nrows, ncols, nlabels);
                limg1.downscale(pyr);
              }
              timer2.stop();

              // Set the second view...
              gradcomp.setSecondView(img1, &limg1, &ori1);

              // This should compute Potometric
              // and if available Semantic data terms
              double img_energy = gradcomp.process();
              energy += img_energy;
              std::cout << " IMG " << i << " " << j << " img_energy: " << img_energy;
              std::cout << " \t Energy: " << energy << std::endl;

              //energy+=gradcomp.process();
              pixsize += gradcomp.computeTriangleSizeInPix();

              // Actually here base should be scaled

              // Get the gradient, rubber nan and a
              tempgrad = gradcomp.getPhotoGrad() * _ctr->_photoweightvec[pyr] * pow(gradcomp.getMeanDist(), 2.0) /
                         pow((ori1.getK())(0, 0), 2.0);
              //	std::cout<<tempgrad;
              cleanGrad(avedgelength, tempgrad, counter, *_mesh);
              grad += tempgrad;

              // Get the gradient, rubber nan and a
              tempgrad = gradcomp.getSemanticGrad() * _ctr->_semweightvec[pyr];
              cleanGrad(avedgelength, tempgrad, counter, *_mesh);
              grad += tempgrad;
              nm++;
            }
          }
        }
      }
      std::cout << "\n\t num image_pairs: " << image_pairs << std::endl;
      std::cout << "\t energy/image_pairs: " << energy / image_pairs << std::endl;

      if (_verboselevel >= 0) {
        std::cout << "Energy=" << energy << " || Avg Delta energy=" << energy / image_pairs - oldenergy
                  << " || Av. face size="
                  << floor(pixsize / (float) nm * 10.0) * 0.1 << "[pix]\n" << std::endl;
      }
      oldenergy = energy / image_pairs;
      image_pairs = 0;

      scaleGradByCounter(grad, counter);

      // ********* Thin plate gradient ********
      PRSTimer tptime;
      tptime.start();
      GradThinPlate smoothgen(_mesh);
      smoothgen.assignGrad(GradThinPlate::LaplacianMode::KOBBELT);
      if (_ctr->_usesemanticsmooth) {
        // roof -> 2 | facade -> 0 | ground -> 1 | veg -> 3
        std::vector<float> penalties(nlabels);
        // Set the penalties for level
        penalties[0] = _ctr->_smoothweightvecunknown[pyr]; // unknown
        penalties[1] = _ctr->_smoothweightvecmobile[pyr]; //mobile
        penalties[2] = _ctr->_smoothweightvectrees[pyr]; // trees
        penalties[3] = _ctr->_smoothweightvecground[pyr]; // ground
        penalties[4] = _ctr->_smoothweightvecpavement[pyr]; // pavement
        penalties[5] = _ctr->_smoothweightvecbuilding[pyr]; // building
        penalties[6] = _ctr->_smoothweightvecwater[pyr]; // water
        // TODO: This is where gradient updates get set at each mesh vertex, based on 
        //    whether or not all the faces sharing that vertex have the same class 
        // (grad *= penalty, so I guess decreasing the magnitude since penalty <1),
        //    OR if they are NOT all the same label, set the gradient to 0 at that vertex.
        smoothgen.weightClassSpecificPenalties(penalties);
        tempgrad = smoothgen.getGrad() * (-1.0);
      } else {
        // so this _smoothweightvec value only matters if you're not using class-specific weights
        tempgrad = smoothgen.getGrad() * (-1.0) * _ctr->_smoothweightvec[pyr];
      }
      int clean = cleanGrad(avedgelength, tempgrad, counter, *_mesh);
      grad += tempgrad;

      tptime.stop();
      if (_verboselevel >= 1) {
        std::cout << " Thinplate took: " << tptime.getTimeSec()
                  << " sec. Valid:" << (float) clean / (float) numverts << std::endl;
      }

      //********* Edge Gradient *********
      if (_ctr->_usestraightedgegrad) {
        GradStraightEdges edgegen(*_mesh);
        edgegen.oneRingStraightEdge2();
        tempgrad = edgegen.getGrad();
        cleanGrad(avedgelength, tempgrad, counter, *_mesh);
        grad += tempgrad * _ctr->_straightedgeweightvec[pyr];
      }

      // Here do the overall scaling of the gradient
      updateMesh(*_mesh, grad);
      itertimer.stop();
      if (_verboselevel >= 1) {
        std::cout << " Iteration took: " << itertimer.getTimeMin() << " min" << std::endl;
      }

    } // end iterations

    // Here upscaling has to be done, densify mesh
    if (pyr != _ctr->_endlevel) {
      MeshDivide::subDivideEqual(*_mesh);
      numverts = _mesh->n_vertices();
    }
  } //end level
}


