// Copyright ETHZ 2017
// author: mathias, 2017, mathias.rothermel@geod.baug.ethz.ch
#pragma once
#include <vector>
#include <string>


class IOList
{
    public:

	IOList();
	IOList(int elements);
	IOList(const std::string &loadfile);
	~IOList();

	bool readFile(const std::string &filetoload);
	std::string getElement(const int i) const;
	int size(void) const;
	void resize(const int newsize);
	std::string getName(const int idx ) const;
	std::string getNameWithoutEnding(const int idx ) const;
 
	std::vector<std::string> getList() const;
	void setElement( const std::string value, const int i);

    private:

	std::vector<std::string> _list;
};
