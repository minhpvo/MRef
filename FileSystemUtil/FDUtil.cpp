// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@googlemail.com
#include <iostream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

namespace FDUtil
{
	// Exists if folder does not exist
    	void folderExist(std::string rootdir)
	{
		boost::filesystem::path fsrootdir(rootdir);
		if( !boost::filesystem::is_directory(fsrootdir))
		{
			std::cout<<"\nProblems finding directory:"<< rootdir;
			exit(1);
		}
	}

	// Get a list with all directories 
	void dirsFromDir(const std::string rootdir, std::vector<std::string>& dirlist)
	{
		// root folder exist
		folderExist(rootdir);

		boost::filesystem::path fsrootdir(rootdir);
		for ( boost::filesystem::directory_iterator itr(fsrootdir); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			if(boost::filesystem::is_directory(itr->path()) )
			{
				dirlist.push_back(itr->path().string());
			}
	    	}
	}

	// Get a list with all files with specific extension
	void filesWithDirFromDir(const std::string rootpath, std::vector<std::string>& filelist)
	{
		folderExist(rootpath);

		boost::filesystem::path fsrootdir(rootpath);
		for ( boost::filesystem::directory_iterator itr(fsrootdir); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			if(	boost::filesystem::is_regular_file(itr->path()))
			{
				filelist.push_back(itr->path().string());
			}
	    	}
	}

	// Get a list with all files with specific extension
	void filesWithDirFromDir(const std::string rootpath, const std::string extension, std::vector<std::string>& filelist)
	{
		folderExist(rootpath);
		std::string expluspt(".");
		expluspt.append(extension);

		boost::filesystem::path fsrootdir(rootpath);
		for ( boost::filesystem::directory_iterator itr(fsrootdir); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			if(	boost::filesystem::is_regular_file(itr->path()) &&
				(!itr->path().extension().string().compare(extension) ||
				!itr->path().extension().string().compare(expluspt)))
			{
				filelist.push_back(itr->path().string());
			}
	    	}
	}

	void filesFromDir(const std::string rootpath, std::vector<std::string>& filelist)
	{
		filesWithDirFromDir(rootpath, filelist);

		for(int i=0; i<filelist.size(); i++)
		{
			boost::filesystem::path cur(filelist[i]);
		    	filelist[i]=cur.filename().string();
		}
	}

	void filesFromDir(const std::string rootpath, const std::string extension, std::vector<std::string>& filelist)
	{
		filesWithDirFromDir(rootpath, extension, filelist);

		for(int i=0; i<filelist.size(); i++)
		{
			boost::filesystem::path cur(filelist[i]);
		    	filelist[i]=cur.filename().string();
		}
	}

	void filesWithoutExtFromDir(const std::string rootpath, std::vector<std::string>& filelist)
	{
		filesWithDirFromDir(rootpath, filelist);

		for(int i=0; i<filelist.size(); i++)
		{
			boost::filesystem::path cur(filelist[i]);
		    	filelist[i]=cur.stem().string();
		}
	}

	void filesWithoutExtFromDir(const std::string rootpath, const std::string extension, std::vector<std::string>& filelist)
	{
		filesWithDirFromDir(rootpath, extension, filelist);

		for(int i=0; i<filelist.size(); i++)
		{
			boost::filesystem::path cur(filelist[i]);
		    	filelist[i]=cur.stem().string();
		}
	}


	void exchangeExtension(std::string& file, std::string newext)
	{
		boost::filesystem::path fsfile(file);
		boost::filesystem::path fspath(fsfile.parent_path());
		fsfile=fsfile.stem();
		std::string newfile=fsfile.string()+"."+newext;
		fsfile=fspath/(boost::filesystem::path(newfile));
		file=fsfile.string();
	}

}

