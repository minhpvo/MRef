// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rotheremel@geod.baug.ethz.ch
#include "DataCopy.h"
#include <cstdlib>
#include <iostream>
#include <boost/filesystem.hpp>

DataCopy::DataCopy()
{
}

DataCopy::~DataCopy()
{
}

void DataCopy::setTempDir()
{
     	if(const char* env_p = std::getenv("TMPDIR"))
     	{
     		std::cout << "Tempdir is: " << env_p << '\n';
		_tempdir=env_p;
	}
	else
	{
	    	std::cout<< "Couldnt find Tempdir:"<<_tempdir;
	    	exit(1);
	}
}

void DataCopy::copyData(IOList& list, const std::vector<bool>& cpvec)
{
    	boost::filesystem::path fsnewpath(_tempdir);
	for(int l=0;l<list.size(); l++)
	{
		if(cpvec[l])
		{
			// Get filename
	    		boost::filesystem::path fsoldfile(list.getElement(l));
			boost::filesystem::path fsnewfile=fsnewpath/fsoldfile.filename();

			// Now Copy
			if(!boost::filesystem::exists(fsnewfile))
			{
				boost::filesystem::copy_file(fsoldfile,fsnewfile);
			
				// Update IOList
				list.setElement(fsnewfile.string(),l);
				std::cout<<"\nCopied " << fsoldfile.string() << " to " << fsnewfile.string(); 
			}
		}
	}
}

