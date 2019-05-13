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
struct stAgent
{
	int id;//��������
	stPoint init;//������
	stPoint goal;//�յ�
	std::vector<int>* path;//������·��
	stAgent(int id, stPoint init, stPoint goal) :
			id(id), init(init), goal(goal),	path(NULL) {};
};

struct stMultiAgentSystem {	// һ����������ϵͳ��������Ϣ 
	int num_agents_int;	//����������
	int	num_expansions_int;	//��չ�ڵ������
	int	num_collisions_int;	//��ͻ������
	int	system_cost_int;	//��������ܿ���
	time_t	time_timet;	//��������ܺ�ʱ
	bool 	canbesolved_bool;//�����Ƿ���Խ��
	stPoint	dim;	//��ͼ�ߴ�
	stMultiAgentSystem(int na, int ne, int nc, int cost, time_t t, bool s, stPoint d) :
		num_agents_int(na), num_expansions_int(ne), num_collisions_int(nc), system_cost_int(cost), time_timet(t), canbesolved_bool(s), dim(d) {};
};

class CMultiAgentSystem 
{
private:
	int n_agent_number_int;//����������
	int e_expansions_int;//��չ�ڵ������
	int c_collisions_int;//��ײ������
	int max_cost_int;

	time_t	start_t;	//��ʼʱ��
	time_t	elapse_t;	//������ʱ��

	CDistance* distance_CDistance;
	CGridMap* gridmap_CGridMap;
	stAgentPosition lastPosition_AgentPosition;

	std::unordered_map<int, stAgentPosition>* cat;	//������ײ��
	std::vector<stAgent> agentlist;	
	std::vector<std::vector<int>> groups;

	int group_conflict(std::vector<int>* g1, std::vector<int>* g2, int len1, int len2);
	int path_conflict(std::vector<int>* p1, std::vector<int>* p2, int len);

public:
	CMultiAgentSystem(int n, stPoint* s_init, stPoint* s_goal, CGridMap* gd);
	~CMultiAgentSystem();
	int resolve_conflicts(void);//�����ͻ

	int	num_expansions(void);
	time_t	get_time(void);
	int	get_collisions(void);
	int	cost(void);	// Solution Cost
};

inline
int CMultiAgentSystem::num_expansions(void) {
	return e_expansions_int;
}

inline
time_t CMultiAgentSystem::get_time(void) {
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