/*
gridmap�� ��������ͼ��ر����ͷ���
	
	˵����
	1.��ά���顰map_intpp����ʽ��
		x-> x����
	  y	0123456789
	  |	1234567898
	  V	2345678987
		3456789876
	  y	4567898765
	 ��	5678987654
	 ��	6789876543
		7898765432
		8987654321
		9876543210
	eg:map_intpp[3][2];ָy����ȡ���ĸ�λ�ã�x����ȡ������λ��

	2.�����ϱ����������Ҷ��������������Ұ�����������ʽ�涨
					��
		y-1,x-1		y-1,x	y-1,x+1
	��	y,x-1		y,x		y,x+1		��
		y+1,x-1		y+1,x	y+1,x+1
					��

Written by Geoganlle Goo
Last change:2019.4.30
**/

#pragma once
#include<iostream>
#include<fstream>
constexpr auto DIM = 8;//8������;

struct stPoint {
	int x;
	int y;
	stPoint(int inputx, int inputy) :x(inputx), y(inputy) {};
	stPoint() :x(0), y(0) {};
};
inline bool pointEquals(stPoint* p1, int i, int j) {
	return (p1->x == i && p1->y == j);
}
enum eDirection{EAST,SOUTH,WEST,NORTH,EN,ES,WS,WN,WAIT};//9������

class CGridMap
{
private:
	bool** map_boolpp;
	int dimX_int;
	int dimY_int;
public:
	void printGridMap();
	bool* getNeighbor(const stPoint& pos) const;//pos��Χ8��λ���Ƿ��е�
	bool hasNode(const stPoint& pos)const;//posλ���Ƿ��е�
	 
	int hashpt(stPoint* p);//����ͼ�е�һ������ӳ�䵽��һλ������±�
	stPoint unhash(int hash);//��һ�������±�ӳ��ɵ�ͼ�ϵ�

	CGridMap();
	CGridMap(std::string pathname);//���ļ��ж�ȡ��ͼ
	CGridMap(int dimX, int dimY, stPoint** blocklist, int listlen);//����һ��հ׵㴴����ͼ
	~CGridMap();
	
};

