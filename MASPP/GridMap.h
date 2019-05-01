/*
gridmap类 处理环境地图相关变量和方法
	
	说明：
	1.二维数组“map_intpp”格式：
		x-> x方向
	  y	0123456789
	  |	1234567898
	  V	2345678987
		3456789876
	  y	4567898765
	 方	5678987654
	 向	6789876543
		7898765432
		8987654321
		9876543210
	eg:map_intpp[3][2];指y方向取第四个位置，x方向取第三个位置

	2.方向：上北下南左西右东，其中上下左右按下面的数组格式规定
					北
		y-1,x-1		y-1,x	y-1,x+1
	西	y,x-1		y,x		y,x+1		东
		y+1,x-1		y+1,x	y+1,x+1
					南

Written by Geoganlle Goo
Last change:2019.4.30
**/

#pragma once
#include<iostream>
#include<fstream>
constexpr auto DIM = 8;//8个方向;

struct stPoint {
	int x;
	int y;
	stPoint(int inputx, int inputy) :x(inputx), y(inputy) {};
	stPoint() :x(0), y(0) {};
};
inline bool pointEquals(stPoint* p1, int i, int j) {
	return (p1->x == i && p1->y == j);
}
enum eDirection{EAST,SOUTH,WEST,NORTH,EN,ES,WS,WN,WAIT};//9个方向

class CGridMap
{
private:
	bool** map_boolpp;
	int dimX_int;
	int dimY_int;
public:
	void printGridMap();
	bool* getNeighbor(const stPoint& pos) const;//pos周围8个位置是否有点
	bool hasNode(const stPoint& pos)const;//pos位置是否有点
	 
	int hashpt(stPoint* p);//将地图中的一个坐标映射到成一位数组的下标
	stPoint unhash(int hash);//将一个数组下标映射成地图上的

	CGridMap();
	CGridMap(std::string pathname);//从文件中读取地图
	CGridMap(int dimX, int dimY, stPoint** blocklist, int listlen);//根据一组空白点创建地图
	~CGridMap();
	
};

