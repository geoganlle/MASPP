/*
������ȱ���
�����ͼ�������ľ���
��Ҫ����ȷ��·���Ƿ����
**/
#pragma once
#include <iostream>
#include <queue>
#include "CGridMap.h"
struct stNode {
	stPoint p_stPoint;
	int dir_int;//���ڵ㷽��
	int depth_int;//���
	stNode(stPoint c, int dir, int depth) : p_stPoint(c), dir_int(dir), depth_int(depth) { };
};

class CBfs
{
	int solncost_int;//����
	CGridMap* gridmap_CGridMapp;//��ͼ
	stPoint dim_stPoint;//��ͼ�ߴ�
	stPoint* orig_stPoint;//���
	stPoint* dest_stPoint;//�յ�

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
