/*
program file No.3
配合CBfs.h优先遍历计算点之间的距离
这个距离可以理解为广度优先遍历的深度
Written by Geoganlle Goo
**/
#pragma once
#include "CBfs.h"
#include "CGridMap.h"
#include <direct.h>

class CDistance
{
	CGridMap* gridmap_CGridMap;
	int** table_intpp;//任意两点间的距离表

	void populate();//初始化table_intpp 用广度优先遍历的方式
	void search(stPoint& o, stPoint& d);

public:
	int getDistance(const stPoint& init, const stPoint& goal);
	CDistance(CGridMap* gridmap);
	~CDistance();
	void printDistanceTable();	//输出table_intpp中的值,仅用于调试输出

};

inline
int CDistance::getDistance(const stPoint& init, const stPoint& goal)
{
	int dimX = gridmap_CGridMap->getDim().x;
	int dimY = gridmap_CGridMap->getDim().y;
	return table_intpp[init.y * dimX + init.x][goal.y * dimX + goal.x];
}
