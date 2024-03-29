/*
program file No.6

Written by Geoganlle Goo
**/

#pragma once
#include "CGridMap.h"
#include "CDistance.h"
#include "CState.h"
#include "CSearch.h"
#include <time.h>
#include <unordered_map>

/*预处理检测路径是否存在时的开销限制
后期可优化*为依据地图特征计算的开销估值
*/
constexpr auto MAX_TOUR = 100000;

struct stAgent
{
	int id;//智能体编号
	stPoint init;//出发点
	stPoint goal;//终点
	std::vector<int>* path;//智能体路径
	stAgent(int id, stPoint init, stPoint goal) :
			id(id), init(init), goal(goal),	path(NULL) {};
};

struct stMultiAgentSystem {	// 一个多智能体系统的整体信息 
	int num_agents_int;	//智能体数量
	int	num_expansions_int;	//扩展节点的数量
	int	num_collisions_int;	//冲突的数量
	int	system_cost_int;	//解决方案总开销
	time_t	time_timet;	//解决方案总耗时
	bool 	canbesolved_bool;//问题是否可以解决
	stPoint	dim;	//地图尺寸
	/*智能体数量 扩展节点的数量 冲突的数量 解决方案总开销 解决方案总耗时 问题是否可以解决 地图尺寸*/
	stMultiAgentSystem(int na, int ne, int nc, int cost, time_t t, bool s, stPoint d) :
		num_agents_int(na), num_expansions_int(ne), num_collisions_int(nc), system_cost_int(cost), time_timet(t), canbesolved_bool(s), dim(d) {};
};

/*预检查路径是否存在*/
inline
static bool chksolution(int init[], int goal[], int len, CGridMap* g) {//预检查路径是否存在
	for (int i = 0; i < len; i += 2) {
		CBfs bfs((stPoint*)& init[i], (stPoint*)& goal[i], g);
		if (bfs.get_soln_cost_int() > MAX_TOUR) return false;
		//cout << "Found path for agent" << i << endl;
	}
	//cout << "Checked solutions OK\n";
	return true;
}

class CMultiAgentSystem 
{
private:
	int n_agent_number_int;//智能体数量
	int e_expansions_int;//拓展节点的数量
	int c_collisions_int;//碰撞的数量
	int max_cost_int;

	clock_t	start_t;	//开始时间 毫秒级计算
	clock_t	elapse_t;	//已用总时间

	CDistance* distance_CDistance;
	CGridMap* gridmap_CGridMap;
	stAgentPosition lastPosition_AgentPosition;

	std::unordered_map<int, stAgentPosition>* cat;	//避免碰撞表
	std::vector<stAgent> agentlist;	
	std::vector<std::vector<int>> groups;

	int group_conflict(std::vector<int>* g1, std::vector<int>* g2, int len1, int len2);
	int path_conflict(std::vector<int>* p1, std::vector<int>* p2, int len);

public:
	CMultiAgentSystem(int n, stPoint* s_init, stPoint* s_goal, CGridMap* gd);
	~CMultiAgentSystem();
	int resolve_conflicts(void);//解决冲突

	int	num_expansions(void);
	clock_t	get_time(void);
	int	get_collisions(void);
	int	cost(void);	// Solution Cost

};

inline
int CMultiAgentSystem::num_expansions(void) {
	return e_expansions_int;
}

inline
clock_t CMultiAgentSystem::get_time(void) {
	return elapse_t;
}

inline
int CMultiAgentSystem::get_collisions(void) { 
	return c_collisions_int; 
}

inline
int CMultiAgentSystem::cost(void) { 
	return max_cost_int; 
}