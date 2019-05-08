#pragma once
#include "CGridMap.h"
#include "CState.h"
#include "CDistance.h"
#include <vector>
#include <unordered_map>
#define EXPLIM 20000000	// 最大节点处理数
//#define MAXINT 2147483647 //最大int为2147483647
struct stNode {
	int f;//成本估值 f(n) = g(n) + h(n)
	short agentid_short;//要移动智能体的id
	short dir_from_parent;//从父节点来时的方向
	CState* state_CStatep;//该节点的状态
	stNode* parent_stNodep;//下一个节点
	bool operator<(const stNode& stNode_input) const { return stNode_input.f < f; };
};

////////////////////////////////////////////////////////////////
struct stNode_t {
	stNode* n;
	stNode_t(stNode* ptr) : n(ptr) {};
	bool operator<(const stNode_t& stNode_t_input) const {
		return n->operator<(*(stNode_t_input.n));
	}
};

class CSearch
{
	int n_agentnumber_int;//智能体的数量
	int	expansions_node_number_int;	//拓展节点的数量
	int	cost_path_int;//路径的成本
	CGridMap* gridmap;
	stPoint* goal_stPointp;	// 目标状态集
	stPoint* init_stPointp;	// 初始状态集

	stNode* current_stNode;//如果找到目标节点则设置为当前节点
	CDistance* distance_CDistance;//距离表
	std::unordered_map<int,stAgentPosition>* cat;//碰撞避免表 Collision avoidance table

	std::priority_queue<stNode*> open_priority_queue;	//优先队列 大顶堆结构
	std::vector<stNode*> closed_vector;	// Closed list

	stNode* generate(stNode* p, int dir);	//生成一个节点的子节点
	bool is_goal_stNode(stNode* node);
	std::vector<int>* backtrace(stNode* n);	//输出从根节点到n节点的路径

	int* reconstruct_path(int agent, const std::vector<int>& tr);

public:
	int	expand();//选择一个f(n)最小的点	1目标节点 2处理节点数过多
	int	num_expansions();//计算扩展节点数
	int cost();	// Solution cost

	std::vector<int>* path(bool print);	//从目标中检索找到的路径

	CSearch(int n, stPoint* init, stPoint* goal, CGridMap* g, CDistance* d = NULL,
		std::unordered_map<int,stAgentPosition>* cat = NULL);
	~CSearch();
};

inline
int CSearch::num_expansions(void) { 
	return expansions_node_number_int; 
}

inline int CSearch::cost()
{
	return cost_path_int;
}
