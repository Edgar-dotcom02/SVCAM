#ifndef TLIVE_WANTED_H
#define TLIVE_WANTED_H

#include <map>

namespace tlive {

class Wanted
{
public:
	std::map<std::string, int> map;
	Wanted() {};
	Wanted(char** list, int n) {support(list, n);}
	~Wanted(){};
	void support(std::string name) {map[name] = 0;}
	void support(char** list, int n)
	{
		for(int i = 0; i < n; i++)
			support(list[i]);
	}
	void want(std::string name, int val) 
	{
		std::map<std::string, int>::iterator it = map.find(name);
		if(it != map.end())
			if(it->second < val)
				it->second = val;
	}
        void operator--(int)
	{
		for(std::map<std::string, int>::iterator it = map.begin(); it != map.end(); it++)
		{
			if(it->second > 0)
				it->second--;
		}
	}
	void operator=(int val)
	{
		for(std::map<std::string, int>::iterator it = map.begin(); it != map.end(); it++)
		{
			if(it->second > 0)
				it->second = val;
		}
	}
	bool operator[](std::string name) {return map[name] > 0;}
};

};

#endif
