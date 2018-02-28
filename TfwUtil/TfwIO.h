// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch

#pragma once

#include <string>
#include <fstream>

namespace TfwIO
{

    	// Metrics are all center pixel coordinates
	void readTfw(	const std::string &name,
			const int rows, const int cols,
			double& minx, double& maxx,
			double& miny, double& maxy,
			double& gsd );

	void writeTfw(	const std::string &name,
			const double minx, const double maxx,
			const double miny, const double maxy,
			const double gsd);

}
