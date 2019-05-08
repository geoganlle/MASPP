#pragma once
#include "CGridMap.h"
#include "CDistance.h"
#include "CState.h"
#include "CSearch.h"
#include <time.h>
#include <unordered_map>
struct stAgent
{
	int id;//��������
	stPoint init;//������
	stPoint goal;//�յ�
	std::vector<int>* path;//������·��
		stAgent(int id, stPoint init, stPoint goal) :
			id(id), init(init), goal(goal),	path(NULL) {};
};
class CAgent
{
public:
	CAgent();
	~CAgent();
};

struct stMapf {	// MAPF Test Node 
	int num_agents;	// # of agents in the puzzle
	int	num_exp;	// Total # of node expansions
	int	collisions;	// Num of group collisions
	int	cost;	// Solution cost;
	time_t	time;	// Total time taken
	bool 	solved;	// Whether MAPF could solve the puzzle
	stPoint	dim;	// Dimensions of the grid
	stMapf(int na, int ne, time_t t, bool s, stPoint d) :
		num_agents(na), num_exp(ne), time(t), solved(s), dim(d) {}
};

class Mapf 
{
private:
	int n_agent_number_int;//����������
	int e_expansions_int;//��չ�ڵ������
	int c_collisions_int;//��ײ������
	int max_cost_int;

	time_t	start_t;
	time_t	elapse_t;	//������ʱ�� Total time elapsed

	CDistance* dlt;	//Distance lookup table
	CGridMap* grid;
	stAgentPosition lastConflict;

	std::unordered_map<int, stAgentPosition>* cat;	//������ײ��
	std::vector<stAgent> agentlist;	//�������б�
	std::vector<std::vector<int>> groups;//������

	int group_conflict(std::vector<int>* g1, std::vector<int>* g2, int len1, int len2);
	int path_conflict(std::vector<int>* p1, std::vector<int>* p2, int len);

public:
	Mapf(int n, stPoint* s_init, stPoint* s_goal, CGridMap* gd);
	~Mapf();
	int resolve_conflicts(void);

	int	num_expansions(void);
	time_t	get_time(void);
	int	get_collisions(void);
	int	cost(void);	// Solution Cost
};

inline
int Mapf::num_expansions(void) {
	return e_expansions_int;
}

inline
time_t Mapf::get_time(void) {
	return elapse_t;
}

inline
int Mapf::get_collisions(void) { 
	return c_collisions_int; 
}

inline
int Mapf::cost(void) { 
	return max_cost_int; 
}