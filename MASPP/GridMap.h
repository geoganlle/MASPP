#pragma once
struct stPoint {
	int x;
	int y;
};
enum eDirection{EAST,SOUTH,WEST,NORTH,EN,ES,WS,WN,WAIT};//9������
class CGridMap
{
private:
	int** map_intpp;
	int dimX_int;
	int dimY_int;
public:
	int* getNeighbor(const stPoint& pos) const;//pos��Χ8��λ���Ƿ��е�
	int hasNode(const stPoint& pos)const;//posλ���Ƿ��е�

	int hashpt(stPoint* p);
	CGridMap();
	~CGridMap();
};

