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

/*得到反方向*/
inline
int reverse_dir(int dir) {
	switch (dir) {
	case EAST:
		return WEST;
	case SOUTH:
		return NORTH;
	case WEST:
		return EAST;
	case NORTH:
		return SOUTH;
	case EN:
		return WS;
	case ES:
		return WN;
	case WS:
		return EN;
	case WN:
		return ES;
	default:
		return dir;
	}
}

/*得到某一方向的点*/
inline
stPoint getPoint_move_dir(const stPoint* from, int dir) {

	/*						北
			y - 1, x - 1	y - 1, x	y - 1, x + 1
		西	y, x - 1		y, x		y, x + 1		东
			y + 1, x - 1	y + 1, x	y + 1, x + 1
							南
	**/
	
	stPoint point(from->x, from->y);
	switch (dir) {
	case EAST:
		point.x++; break;
	case SOUTH:
		point.y++; break;
	case WEST:
		point.x--; break;
	case NORTH:
		point.y--; break;
	case EN:
		point.x++;
		point.y--;
		break;
	case ES:
		point.x++;
		point.y++;
		break;
	case WS:
		point.x--;
		point.y--;
		break;
	case WN:
		point.x--;
		point.y++;
		break;
	default:
		return point;
	}
	return point;
}

inline
int get_direction(stPoint* from,stPoint* to) {
	int dx, dy, diff;
	if (!(from && to)) return -1;	// Error
	dx = to->x - from->x;
	dy = to->y - from->y;
	diff = 3 * dy + dx;
	/*	
		dx = | -1	0	1 |	3 * dy = | -3  -3  -3 | 
			 | -1	0	1 |			 |  0	0	0 |
			 | -1	0	1 |		     |  3	3	3 |
		3 * dy + dx = | -4	-3	-2 |	WN	N	EN
					  | -1	 0	 1 |	W		E
					  |  2	 3	 4 |	WS	S	ES
	**/
	switch (diff) {
	case 4:
		return ES;
	case 3:
		return SOUTH;
	case 2:
		return WS;
	case 1:
		return EAST;
	case 0:
		return WAIT;
	case -1:
		return WEST;
	case -2:
		return EN;
	case -3:
		return NORTH;
	case -4:
		return WN;
	default:
		return -1;
	}

}

inline
std::string dir_to_string(int dir) {
	switch (dir) {
	case NORTH:
		return "North";
	case SOUTH:
		return "South";
	case EAST:
		return "East";
	case WEST:
		return "West";
	case EN:
		return "Northeast";
	case WN:
		return "Northwest";
	case ES:
		return "Southeast";
	case WS:
		return "Southwest";
	case WAIT:
		return "Wait";
	default:
		return "";
	}
}

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
	stPoint getDim();//得到地图尺寸
	int hashpt(stPoint* p);//将地图中的一个坐标映射到成一位数组的下标
	stPoint unhash(int hash);//将一个数组下标映射成地图上的

	CGridMap();
	CGridMap(std::string pathname);//从文件中读取地图
	CGridMap(int dimX, int dimY, stPoint** blocklist, int listlen);//根据一组空白点创建地图
	~CGridMap();
	
};
inline bool CGridMap::hasNode(const stPoint& pos) const
{
	return (pos.x >= 0 && pos.x < dimX_int &&
		pos.y >= 0 && pos.y < dimY_int
		&& map_boolpp[pos.x][pos.y]);
}
inline
stPoint CGridMap::getDim()
{
	return stPoint(dimX_int, dimY_int);
}
inline
int CGridMap::hashpt(stPoint * p)
{
	return (p->y * dimX_int + p->x);
}
inline
stPoint CGridMap::unhash(int hash)
{
	return stPoint(hash / dimX_int, hash % dimX_int);
}