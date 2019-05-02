#pragma once
#include "CBfs.h"
#include "CGridMap.h"
#include <unordered_map>

struct Key {
	stPoint init_stPoint;	// Initial state
	stPoint goal_stPoint;	// Goal state
	Key(stPoint i, stPoint g) : init_stPoint(i), goal_stPoint(g) {};
};

class CDistance
{
	CGridMap* gridmap_CGridMap;
	int** table_intpp;//������ľ����

	void populate();//��ʼ��table_intpp
	void search(stPoint& o, stPoint& d);

public:
	int getDistance(const stPoint& init, const stPoint& goal);
	CDistance(CGridMap* gridmap);
	~CDistance();
};

