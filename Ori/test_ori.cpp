// Copyright ETHZ 2017
#include "Orientation.h"

int main()
{
  Orientation myfirstori;
	std::string name("../data/ori/1.or");
	std::string outname("./lastori.or");
	//myfirstori.writeFile(name);
	myfirstori.readFile(name);
	Eigen::Vector3d vec(3,1); vec<< 1,2,3;
	myfirstori.setC(vec);
	myfirstori.writeFile(outname);

}
