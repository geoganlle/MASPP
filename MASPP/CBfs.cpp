#include "CBfs.h"



CBfs::CBfs()
{
	this->orig_stPoint = nullptr;
	this->dest_stPoint = nullptr;
	this->gridmap_CGridMapp = nullptr;
	this->len_int = 0;
	this->visited_boolp = nullptr;
	this->solncost_int = 0;
}

CBfs::CBfs(stPoint* o, stPoint* d, CGridMap* g):orig_stPoint(o),dest_stPoint(d),gridmap_CGridMapp(g)
{
	solncost_int = std::numeric_limits<int>::max();
	this->dim_stPoint = g->getDim();
	len_int = dim_stPoint.x * dim_stPoint.y;
	visited_boolp = new bool[len_int]();//默认初始化为false(0)
	if (gridmap_CGridMapp->hasNode(*orig_stPoint) && gridmap_CGridMapp->hasNode(*dest_stPoint) ){
		search();
	}

}


CBfs::~CBfs()
{
	delete[] visited_boolp;
}

void CBfs::search()
{
	std::queue<stNode> node_queue;
	node_queue.push(stNode(*orig_stPoint, WAIT, 0));
	while (!node_queue.empty()) {
		stNode curNode = node_queue.front();
		node_queue.pop();
		if (curNode.p_stPoint.x == dest_stPoint->x && 
			curNode.p_stPoint.y == dest_stPoint->y) {
			solncost_int = curNode.depth_int;
			return;
		}

		bool* eighborhood = gridmap_CGridMapp->getNeighbor(curNode.p_stPoint);
		
		for (int i = 0; i < DIM; i++) {//处理8个邻居
			stPoint child = getPoint_move_dir(&(curNode.p_stPoint), i);
			int hash = child.y * dim_stPoint.x + child.x;
			if (gridmap_CGridMapp->hasNode(child) &&
				!visited_boolp[hash] && i != curNode.dir_int && //未访问过且不为来时的方向
				eighborhood && eighborhood[i]) {//eighborhood!=NULL 邻居点无障碍物
				int to_parent_dir = reverse_dir(i);
				int depth = curNode.depth_int + 1;
				node_queue.push(stNode(child, to_parent_dir, depth));
				visited_boolp[hash] = true;	//初始化时默认是false
			}
		}
		delete[] eighborhood;
	}
}
