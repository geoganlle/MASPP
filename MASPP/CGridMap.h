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

/*�õ�������*/
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

/*�õ�ĳһ����ĵ�*/
inline
stPoint getPoint_move_dir(const stPoint* from, int dir) {

	/*						��
			y - 1, x - 1	y - 1, x	y - 1, x + 1
		��	y, x - 1		y, x		y, x + 1		��
			y + 1, x - 1	y + 1, x	y + 1, x + 1
							��
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
	bool* getNeighbor(const stPoint& pos) const;//pos��Χ8��λ���Ƿ��е�
	bool hasNode(const stPoint& pos)const;//posλ���Ƿ��е�
	stPoint getDim();//�õ���ͼ�ߴ�
	int hashpt(stPoint* p);//����ͼ�е�һ������ӳ�䵽��һλ������±�
	stPoint unhash(int hash);//��һ�������±�ӳ��ɵ�ͼ�ϵ�

	CGridMap();
	CGridMap(std::string pathname);//���ļ��ж�ȡ��ͼ
	CGridMap(int dimX, int dimY, stPoint** blocklist, int listlen);//����һ��հ׵㴴����ͼ
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