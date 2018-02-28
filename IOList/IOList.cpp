// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#include "IOList.h"

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

namespace bfs=boost::filesystem;


IOList::IOList()
{
}

IOList::IOList(const std::string &loadfile )
{
	readFile(loadfile);
}

IOList::IOList(int elements)
{
    _list.resize(elements);
}

IOList::~IOList()
{
}

void IOList::resize( const int newsize)
{
    _list.resize(newsize);
}

int IOList::size(void) const
{
	return _list.size();
}

std::string IOList::getElement( const int i) const
{
    return _list[i];
}

void  IOList::setElement( const std::string value, const int i)
{
   _list[i]=value;
}

std::string IOList::getName(const int idx ) const
{
    boost::filesystem::path p(_list[idx]);
    return p.filename().string();
}

std::string IOList::getNameWithoutEnding(const int idx ) const
{
    boost::filesystem::path p(_list[idx]);
    std::string helper=boost::filesystem::change_extension(p.filename(), "").string();
    return helper;
    
}

std::vector<std::string> IOList::getList() const
{
    return _list;
}

// This actually stores the list...
bool IOList::readFile ( const std::string &inputFile )
{

    std::string buffer;

    std::ifstream filestr(inputFile.c_str(), std::fstream::in);
    if (filestr.is_open())
    {
        unsigned int i = 0;
        while(std::getline(filestr,buffer))
        {
            if ( buffer.compare("") == 0 )
            {
                break;
            }
            std::string filepath = buffer;
            std::replace(filepath.begin(),filepath.end(), '\\', '/');

            int filename_pos = buffer.find_last_of ( "/\\" ) + 1;
            if ( filename_pos != (int)std::string::npos )
            {
                std::string after_dir = buffer.substr ( filename_pos );
                int first_point = after_dir.find_first_of ( "." );

                if ( first_point != (int)std::string::npos )
                {
                    after_dir = after_dir.substr ( first_point );
                    int first_whitespace = after_dir.find_first_of ( " \t" );
                    if ( first_whitespace != (int)std::string::npos )
                        filepath = buffer.substr ( 0, filename_pos+first_point+first_whitespace);
                }
            }

            _list.push_back(bfs::absolute(boost::filesystem::path(filepath), boost::filesystem::path(inputFile).parent_path()).string());
            std::string::size_type pos = 0;
            while ( ( pos = _list.back().find ("\r",pos) ) != std::string::npos )
            {
                _list.back().erase ( pos, 2 );
            } 
            i++;

        }
        filestr.close();
    }
    else
    {
    	std::cout<<"\nCould not open "<< inputFile;
    }
}
