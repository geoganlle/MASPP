/*
program file No.3
���CBfs.h���ȱ��������֮��ľ���
�������������Ϊ������ȱ��������
Written by Geoganlle Goo
**/
#pragma once
#include "CBfs.h"
#include "CGridMap.h"
#include <direct.h>

class CDistance
{
	CGridMap* gridmap_CGridMap;
	int** table_intpp;//���������ľ����

	void populate();//��ʼ��table_intpp �ù�����ȱ����ķ�ʽ
	void search(stPoint& o, stPoint& d);

public:
	int getDistance(const stPoint& init, const stPoint& goal);
	CDistance(CGridMap* gridmap);
	~CDistance();
	void printDistanceTable();	//���table_intpp�е�ֵ,�����ڵ������

};

inline
int CDistance::getDistance(const stPoint& init, const stPoint& goal)
{
	int dimX = gridmap_CGridMap->getDim().x;
	int dimY = gridmap_CGridMap->getDim().y;
	return table_intpp[init.y * dimX + init.x][goal.y * dimX + goal.x];
}
