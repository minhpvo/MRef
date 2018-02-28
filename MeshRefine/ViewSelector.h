// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#pragma once
#include "../Ori/Orientation.h"
#include "../MeshMetaData/MeshMetaData.h"
#include "../IOList/IOList.h"
#include <vector>
#include <Eigen/Dense>

class ViewSelector
{

	public:

	    	ViewSelector( const IOList& orilist, const MeshMetaData& mmd);
		~ViewSelector();

		void computeOriCors(	const Orientation& baseori,
					const std::vector<Orientation>& orivec,
					const double maxangle,
					const double maxnumcams,
					std::vector<int>& indexvec);

		void computeConnectivity(	const std::vector<Orientation>& orivec,
						const double maxangle,
						const double maxnumcams,
						Eigen::MatrixXf& connectivity);
		void buildClusterVec();
		std::vector<std::vector<int>> nearestImagesFromCluster(const int& maxnumviews);

		std::vector<std::vector<int>> nearestMidPointImagesFromCluster(const int& maxnumviews);

		std::vector<int> nearestNeighborsInCluster(const int imgidx, const int numneighbors,							 				bool inflight, bool crossflight);

		// Identify closest 3d dists in imageid vec, set adjacency
		void setClosestNeighborsDistance( std::vector<int>& nadircluster,
						  Eigen::MatrixXf& adjacency,
						  int maxnummatchimages);

		// Identify id of nadir cluster
		int getNadirCluster(std::vector<std::vector<int>>& baseimages);

		// Debug output
		void printClusterVec(const IOList& namelist);

		// Main Processing function Oblique
		void getAdjacencyTileOblique(Eigen::MatrixXf& adjacency);

		// Main Processing function Nadir 
		void getAdjacencyTileNadir(Eigen::MatrixXf& adjacency);

		// Print Statistics
		void printSummary(Eigen::MatrixXf& adjacency);
		void invalidateAllExcept(int id, Eigen::MatrixXf& adjacency);
		void validate(int id, Eigen::MatrixXf& adjacency);

	private:

		MeshMetaData _mmd;
		IOList _orilist;

		void setViewingDirs(const std::vector<Orientation>& orivec);
		void setPoints();

		Eigen::MatrixXd _points;

		std::vector<Eigen::Vector3d> _viewdirvec;
		std::vector<float> _areaimgvec;
		std::vector<Orientation> _orivec;
		std::vector<std::vector<int>> _clustervec;


};
