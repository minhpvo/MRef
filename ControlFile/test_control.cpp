// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "ControlRefine.h"
#include <string>

int main()
{
    	// Make control refine
    	ControlRefine cr;

	// Save it
  std::string readname("../data/ControlRefine.txt");
	std::string savename("./ControlRefine.txt");
	cr.readFile(readname);
	cr.writeFile(savename);
	return 1;

}
