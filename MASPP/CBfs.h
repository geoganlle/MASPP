/*
广度优先遍历
计算地图中两点间的距离
主要用于确认路径是否存在
**/
#pragma once
#include <iostream>
#include <queue>
#include "CGridMap.h"
struct stNode {
	stPoint p_stPoint;
	int dir_int;//父节点方向
	int depth_int;//深度
	stNode(stPoint c, int dir, int depth) : p_stPoint(c), dir_int(dir), depth_int(depth) { };
};

class CBfs
{
	int solncost_int;//消耗
	CGridMap* gridmap_CGridMapp;//地图
	stPoint dim_stPoint;//地图尺寸
	stPoint* orig_stPoint;//起点
	stPoint* dest_stPoint;//终点

	bool* visited_boolp;
	int len_int;

public:
	CBfs();
	CBfs(stPoint* o,stPoint* d,CGridMap* g);
	~CBfs();

	void search();
	int getsolncost_int();
};

inline
int CBfs::getsolncost_int()
{
	return solncost_int;
}
