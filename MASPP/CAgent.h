#pragma once
#include"CGridMap.h"
class CAgent
{
	int id_int;
	stPoint orig;
	stPoint desk;
	std::string path;
public:
	CAgent();
	~CAgent();
};

