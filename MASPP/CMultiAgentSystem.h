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

/*Ԥ������·���Ƿ����ʱ�Ŀ�������
���ڿ��Ż�*Ϊ���ݵ�ͼ��������Ŀ�����ֵ
*/
constexpr auto MAX_TOUR = 100000;

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
	/*���������� ��չ�ڵ������ ��ͻ������ ��������ܿ��� ��������ܺ�ʱ �����Ƿ���Խ�� ��ͼ�ߴ�*/
	stMultiAgentSystem(int na, int ne, int nc, int cost, time_t t, bool s, stPoint d) :
		num_agents_int(na), num_expansions_int(ne), num_collisions_int(nc), system_cost_int(cost), time_timet(t), canbesolved_bool(s), dim(d) {};
};

/*Ԥ���·���Ƿ����*/
inline
static bool chksolution(int init[], int goal[], int len, CGridMap* g) {//Ԥ���·���Ƿ����
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
	int n_agent_number_int;//����������
	int e_expansions_int;//��չ�ڵ������
	int c_collisions_int;//��ײ������
	int max_cost_int;

	clock_t	start_t;	//��ʼʱ�� ���뼶����
	clock_t	elapse_t;	//������ʱ��

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