// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#pragma once

#include <string>
#include <vector>

namespace FDUtil
{

	void folderExist(std::string path);

	void dirsFromDir(const std::string rootdir, std::vector<std::string>& dirlist);

	void filesFromDir(const std::string rootpath, std::vector<std::string>& files);

	void filesFromDir(const std::string rootpath, const std::string extension, std::vector<std::string>& files);
	
	void filesWithDirFromDir(const std::string rootpath, std::vector<std::string>& files);

	void filesWithDirFromDir(const std::string rootpath, const std::string extension, std::vector<std::string>& files);

	void filesWithoutExtFromDir(const std::string rootpath, std::vector<std::string>& files);

	void filesWithoutExtFromDir(const std::string rootpath, const std::string extension, std::vector<std::string>& files);
	void exchangeExtension(std::string& file, std::string newext);
}
