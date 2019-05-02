#include "CDistance.h"

void CDistance::populate()
{
	int dimX = gridmap_CGridMap->getDim().x;
	int dimY = gridmap_CGridMap->getDim().y;
	for (int i = 0; i < dimX; i++) {
		for (int j = 0; j < dimY; j++) {
			stPoint orig(i, j);
			for (int k = 0; k < dimX; k++) {
				for (int l = 0; l < dimY; l++) {
					stPoint dest(k, l);
					search(orig, dest);
				}
			}
		}
	}
}

void CDistance::search(stPoint& o, stPoint& d)
{
	int dimX = gridmap_CGridMap->getDim().x;
	int dimY = gridmap_CGridMap->getDim().y;
	CBfs bfs(&o, &d, this->gridmap_CGridMap);
	table_intpp[o.y * dimX + o.x][d.y * dimX + d.x] = bfs.getsolncost_int();
}

inline
int CDistance::getDistance(const stPoint& init, const stPoint& goal)
{
	int dimX = gridmap_CGridMap->getDim().x;
	int dimY = gridmap_CGridMap->getDim().y;
	return table_intpp[init.y * dimX + init.x][goal.y * dimX + goal.x];
}

CDistance::CDistance(CGridMap* gridmap):gridmap_CGridMap(gridmap)
{
	stPoint dim = gridmap_CGridMap->getDim();
	table_intpp = new int* [dim.x * dim.y];
	for (int i = 0; i < dim.x * dim.y; i++)
		table_intpp[i] = new int[dim.y * dim.x];
	populate();
}

CDistance::~CDistance()
{
	int dimX = gridmap_CGridMap->getDim().x;
	int dimY = gridmap_CGridMap->getDim().y;
	for (int i = 0; i < dimX * dimY; i++)
		delete[] table_intpp[i];
	delete[] table_intpp;
}
