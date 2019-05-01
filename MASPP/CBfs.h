#pragma once
#include <iostream>
#include "GridMap.h"
struct stNode {
	stPoint p_stPoint;
	int dir_int;
	int depth_int;
};

class CBfs
{
	int solncost_int;
	CGridMap* gridmap_CGridMapp;
	stPoint* orig_stPoint;
	stPoint* dest_stPoint;
	bool* visited_boolp;
	int len_int;
	stPoint dim_stPoint;
public:
	CBfs();
	CBfs(stPoint* o,stPoint* d,CGridMap* g);
	~CBfs();

	void search();
	int cost();
};

