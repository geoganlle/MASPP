/*
program file No.5
搜索类

Written by Geoganlle Goo
**/
#pragma once
#include "CGridMap.h"
#include "CState.h"
#include "CDistance.h"
#include <vector>
#include <unordered_map>

constexpr auto EXPLIM = 2000000;	// 最大节点处理数;参考:最大int为2147483647

/*
针对每个智能体的节点结构体
封装了A*算法的启发式估值f(n)
对state进行了封装，方便生成逻辑组织结构
->node(0)<-node(1)<-node(2)  
A->B表示A的父指针指向B
**/
struct stNode_Search {
	int f;//成本估值 f(n) = g(n) + h(n)
	short agentid_short;//要移动智能体的id
	short dir_from_parent;//从父节点来时的方向
	CState* state_CStatep;//该节点的状态
	stNode_Search* parent_stNodep;//父节点
	bool operator<(const stNode_Search& stNode_input) const { return stNode_input.f < f; };
};

class CSearch
{
//variate:
	int n_agentnumber_int;//智能体的数量
	int	expansions_node_number_int;	//拓展节点的数量
	int	cost_path_int;//路径的成本

	CGridMap* gridmap;//地图
	CDistance* distance_CDistance;//距离表
	stPoint* goal_stPointp;	// 目标状态集
	stPoint* init_stPointp;	// 初始状态集

	stNode_Search* last_stNode;//如果找到目标节点则设置为当前节点

	std::unordered_map<int,stAgentPosition>* CAT;//碰撞避免表 Collision avoidance table
	/*
	int timestep 用路径长度上的开销做主键索引智能体的位置结构体
	stAgentPosition 智能体的位置
	*/

	std::priority_queue<stNode_Search*> open_priority_queue;//待完成的搜索点 优先队列 大顶堆
	std::vector<stNode_Search*> closed_vector;//已完成的搜索点

//function:

	stNode_Search* generate_childnode(stNode_Search* p, int dir);//生成一个节点的子节点
	bool is_goal_stNode(stNode_Search* node);//检查目标节点是否合法
	std::vector<int>* getpath(stNode_Search* n);	//输出从根节点到n节点的路径(方向)
	int* get_hashPoint_of_path(int agent, const std::vector<int>& tr);//得到路径的hash点序列

public:
	int	expand();//选择一个f(n)最小的点	返回值说明：1目标节点 2处理节点数过多
	int	get_num_expansions_int();//计算扩展节点数
	int get_cost_int();	//搜索成本

	std::vector<int>* path(bool print);	//从目标中检索找到的路径

	CSearch(int n, stPoint* init, stPoint* goal, CGridMap* g, CDistance* d = NULL,
		std::unordered_map<int,stAgentPosition>* cat = NULL);
	~CSearch();
};

inline
int CSearch::get_num_expansions_int() { 
	return expansions_node_number_int; 
}

inline int CSearch::get_cost_int()
{
	return cost_path_int;
}
