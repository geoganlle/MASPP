#pragma once
struct stPoint {
	int x;
	int y;
};
enum eDirection{EAST,SOUTH,WEST,NORTH,EN,ES,WS,WN,WAIT};//9个方向
class CGridMap
{
private:
	int** map_intpp;
	int dimX_int;
	int dimY_int;
public:
	int* getNeighbor(const stPoint& pos) const;//pos周围8个位置是否有点
	int hasNode(const stPoint& pos)const;//pos位置是否有点

	int hashpt(stPoint* p);
	CGridMap();
	~CGridMap();
};

