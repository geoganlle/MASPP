/*
���CBfs.h������ȱ��������֮��ľ���
**/
#pragma once
#include "CBfs.h"
#include "CGridMap.h"
#include <direct.h>

class CDistance
{
	CGridMap* gridmap_CGridMap;
	int** table_intpp;//������ľ����

	void populate();//��ʼ��table_intpp �ù�����ȱ����ķ�ʽ
	void search(stPoint& o, stPoint& d);

public:
	int getDistance(const stPoint& init, const stPoint& goal);
	CDistance(CGridMap* gridmap);
	~CDistance();

	//���table_intpp�е�ֵ//�����ڵ������
	public:	void printDistanceTable() {
		int n = gridmap_CGridMap->getDim().x * gridmap_CGridMap->getDim().y;
		std::ofstream __file;
		if (_mkdir("../test"))std::cout << "mkdir failes: Folder \"test\" already exists" << std::endl;//Ŀ¼�Ѵ���
		__file.open("../test/table.txt", std::ios::out | std::ios::trunc);
		if (!__file.is_open()) {
			std::cout << "file open failed" << std::endl;
			__file.close();
			return;
		}
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				__file << table_intpp[i][j]<<"\t";
			}
			__file << std::endl;
		}
		__file.close();
	};
};

inline
int CDistance::getDistance(const stPoint& init, const stPoint& goal)
{
	int dimX = gridmap_CGridMap->getDim().x;
	int dimY = gridmap_CGridMap->getDim().y;
	return table_intpp[init.y * dimX + init.x][goal.y * dimX + goal.x];
}
